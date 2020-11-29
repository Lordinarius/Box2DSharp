using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// <summary>
    /// Linear constraint (point-to-line)
    /// d = p2 - p1 = x2 + r2 - x1 - r1
    /// C = dot(perp, d)
    /// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    ///      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    /// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    ///
    /// Angular constraint
    /// C = a2 - a1 + a_initial
    /// Cdot = w2 - w1
    /// J = [0 0 -1 0 0 1]
    ///
    /// K = J * invM * JT
    ///
    /// J = [-a -s1 a s2]
    ///     [0  -1  0  1]
    /// a = perp
    /// s1 = cross(d + r1, a) = cross(p2 - x1, a)
    /// s2 = cross(r2, a) = cross(p2 - x2, a)
    ///
    /// Motor/Limit linear constraint
    /// C = dot(ax1, d)
    /// Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    /// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
    ///
    /// Predictive limit is applied even when the limit is not active.
    /// Prevents a constraint speed that can lead to a constraint error in one time step.
    /// Want C2 = C1 + h * Cdot >= 0
    /// Or:
    /// Cdot + C1/h >= 0
    /// I do not apply a negative constraint error because that is handled in position correction.
    /// So:
    /// Cdot + max(C1, 0)/h >= 0
    ///
    /// Block Solver
    /// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
    ///
    /// The Jacobian has 2 rows:
    /// J = [-uT -s1 uT s2] /// linear
    ///     [0   -1   0  1] /// angular
    ///
    /// u = perp
    /// s1 = cross(d + r1, u), s2 = cross(r2, u)
    /// a1 = cross(d + r1, v), a2 = cross(r2, v)
    /// </summary>
    public class PrismaticJoint : Joint
    {
        internal readonly V2 LocalAnchorA;

        internal readonly V2 LocalAnchorB;

        internal readonly V2 LocalXAxisA;

        internal readonly V2 LocalYAxisA;

        internal readonly F ReferenceAngle;

        private V2 _impulse;

        private F _motorImpulse;

        private F _lowerImpulse;

        private F _upperImpulse;

        private F _lowerTranslation;

        private F _upperTranslation;

        private F _maxMotorForce;

        private F _motorSpeed;

        private bool _enableLimit;

        private bool _enableMotor;

        #region Solver temp

        private int _indexA;

        private int _indexB;

        private V2 _localCenterA;

        private V2 _localCenterB;

        private F _invMassA;

        private F _invMassB;

        private F _invIA;

        private F _invIB;

        private V2 _axis, _perp;

        private F _s1, _s2;

        private F _a1, _a2;

        private Matrix2x2 _k;

        private F _translation;

        private F _axialMass;

        #endregion

        internal PrismaticJoint(PrismaticJointDef def)
            : base(def)
        {
            LocalAnchorA = def.LocalAnchorA;
            LocalAnchorB = def.LocalAnchorB;
            LocalXAxisA = def.LocalAxisA;
            LocalXAxisA.Normalize();
            LocalYAxisA = MathUtils.Cross(F.One, LocalXAxisA);
            ReferenceAngle = def.ReferenceAngle;

            _impulse.SetZero();
            _axialMass = F.Zero;
            _motorImpulse = F.Zero;
            _lowerImpulse = F.Zero;
            _upperImpulse = F.Zero;

            _lowerTranslation = def.LowerTranslation;
            _upperTranslation = def.UpperTranslation;

            Debug.Assert(_lowerTranslation <= _upperTranslation);

            _maxMotorForce = def.MaxMotorForce;
            _motorSpeed = def.MotorSpeed;
            _enableLimit = def.EnableLimit;
            _enableMotor = def.EnableMotor;

            _axis.SetZero();
            _perp.SetZero();
        }

        /// The local anchor point relative to bodyA's origin.
        public V2 GetLocalAnchorA()
        {
            return LocalAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        public V2 GetLocalAnchorB()
        {
            return LocalAnchorB;
        }

        /// The local joint axis relative to bodyA.
        public V2 GetLocalAxisA()
        {
            return LocalXAxisA;
        }

        /// Get the reference angle.
        public F GetReferenceAngle()
        {
            return ReferenceAngle;
        }

        /// Get the current joint translation, usually in meters.
        public F GetJointTranslation()
        {
            var pA = BodyA.GetWorldPoint(LocalAnchorA);
            var pB = BodyB.GetWorldPoint(LocalAnchorB);
            var d = pB - pA;
            var axis = BodyA.GetWorldVector(LocalXAxisA);

            var translation = V2.Dot(d, axis);
            return translation;
        }

        /// Get the current joint translation speed, usually in meters per second.
        public F GetJointSpeed()
        {
            var bA = BodyA;
            var bB = BodyB;

            var rA = MathUtils.Mul(bA.Transform.Rotation, LocalAnchorA - bA.Sweep.LocalCenter);
            var rB = MathUtils.Mul(bB.Transform.Rotation, LocalAnchorB - bB.Sweep.LocalCenter);
            var p1 = bA.Sweep.C + rA;
            var p2 = bB.Sweep.C + rB;
            var d = p2 - p1;
            var axis = MathUtils.Mul(bA.Transform.Rotation, LocalXAxisA);

            var vA = bA.LinearVelocity;
            var vB = bB.LinearVelocity;
            var wA = bA.AngularVelocity;
            var wB = bB.AngularVelocity;

            var speed = V2.Dot(d, MathUtils.Cross(wA, axis))
                      + V2.Dot(axis, vB + MathUtils.Cross(wB, rB) - vA - MathUtils.Cross(wA, rA));
            return speed;
        }

        /// Is the joint limit enabled?
        public bool IsLimitEnabled()
        {
            return _enableLimit;
        }

        /// Enable/disable the joint limit.
        public void EnableLimit(bool flag)
        {
            if (flag != _enableLimit)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                _enableLimit = flag;
                _lowerImpulse = F.Zero;
                _upperImpulse = F.Zero;
            }
        }

        /// Get the lower joint limit, usually in meters.
        public F GetLowerLimit()
        {
            return _lowerTranslation;
        }

        /// Get the upper joint limit, usually in meters.
        public F GetUpperLimit()
        {
            return _upperTranslation;
        }

        /// Set the joint limits, usually in meters.
        public void SetLimits(F lower, F upper)
        {
            Debug.Assert(lower <= upper);
            if (!lower.Equals(_lowerTranslation) || !upper.Equals(_upperTranslation))
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                _lowerTranslation = lower;
                _upperTranslation = upper;
                _lowerImpulse = F.Zero;
                _upperImpulse = F.Zero;
            }
        }

        /// Is the joint motor enabled?
        public bool IsMotorEnabled()
        {
            return _enableMotor;
        }

        /// Enable/disable the joint motor.
        public void EnableMotor(bool flag)
        {
            if (flag != _enableMotor)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                _enableMotor = flag;
            }
        }

        /// Set the motor speed, usually in meters per second.
        public void SetMotorSpeed(F speed)
        {
            if (speed != _motorSpeed)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                _motorSpeed = speed;
            }
        }

        /// Get the motor speed, usually in meters per second.
        public F GetMotorSpeed()
        {
            return _motorSpeed;
        }

        /// Set the maximum motor force, usually in N.
        public void SetMaxMotorForce(F force)
        {
            if (F.Abs(force - _maxMotorForce) > 0.000001f)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                _maxMotorForce = force;
            }
        }

        public F GetMaxMotorForce()
        {
            return _maxMotorForce;
        }

        /// Get the current motor force given the inverse time step, usually in N.
        public F GetMotorForce(F inv_dt)
        {
            return inv_dt * _motorImpulse;
        }

        /// <inheritdoc />
        public override V2 GetAnchorA()
        {
            return BodyA.GetWorldPoint(LocalAnchorA);
        }

        /// <inheritdoc />
        public override V2 GetAnchorB()
        {
            return BodyB.GetWorldPoint(LocalAnchorB);
        }

        /// <inheritdoc />
        public override V2 GetReactionForce(F inv_dt)
        {
            return inv_dt * (_impulse.X * _perp + (_motorImpulse + _lowerImpulse + _upperImpulse) * _axis);
        }

        /// <inheritdoc />
        public override F GetReactionTorque(F inv_dt)
        {
            return inv_dt * _impulse.Y;
        }

        /// Dump to b2Log
        public override void Dump()
        { }

        internal override void InitVelocityConstraints(in SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _indexB = BodyB.IslandIndex;
            _localCenterA = BodyA.Sweep.LocalCenter;
            _localCenterB = BodyB.Sweep.LocalCenter;
            _invMassA = BodyA.InvMass;
            _invMassB = BodyB.InvMass;
            _invIA = BodyA.InverseInertia;
            _invIB = BodyB.InverseInertia;

            var cA = data.Positions[_indexA].Center;
            var aA = data.Positions[_indexA].Angle;
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;

            var cB = data.Positions[_indexB].Center;
            var aB = data.Positions[_indexB].Angle;
            var vB = data.Velocities[_indexB].V;
            var wB = data.Velocities[_indexB].W;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            // Compute the effective masses.
            var rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            var rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            var d = cB - cA + rB - rA;

            F mA = _invMassA, mB = _invMassB;
            F iA = _invIA, iB = _invIB;

            // Compute motor Jacobian and effective mass.
            {
                _axis = MathUtils.Mul(qA, LocalXAxisA);
                _a1 = MathUtils.Cross(d + rA, _axis);
                _a2 = MathUtils.Cross(rB, _axis);

                _axialMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
                if (_axialMass > F.Zero)
                {
                    _axialMass = F.One / _axialMass;
                }
            }

            // Prismatic constraint.
            {
                _perp = MathUtils.Mul(qA, LocalYAxisA);

                _s1 = MathUtils.Cross(d + rA, _perp);
                _s2 = MathUtils.Cross(rB, _perp);

                var k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
                var k12 = iA * _s1 + iB * _s2;
                var k22 = iA + iB;
                if (k22.Equals(F.Zero))
                {
                    // For bodies with fixed rotation.
                    k22 = F.One;
                }

                _k.Ex.Set(k11, k12);
                _k.Ey.Set(k12, k22);
            }

            if (_enableLimit)
            {
                _translation = V2.Dot(_axis, d);
            }
            else
            {
                _lowerImpulse = F.Zero;
                _upperImpulse = F.Zero;
            }

            if (_enableMotor == false)
            {
                _motorImpulse = F.Zero;
            }

            if (data.Step.WarmStarting)
            {
                // Account for variable time step.
                _impulse *= data.Step.DtRatio;
                _motorImpulse *= data.Step.DtRatio;
                _lowerImpulse = data.Step.DtRatio;
                _upperImpulse = data.Step.DtRatio;

                var axialImpulse = _motorImpulse + _lowerImpulse - _upperImpulse;
                var P = _impulse.X * _perp + axialImpulse * _axis;
                var LA = _impulse.X * _s1 + _impulse.Y + axialImpulse * _a1;
                var LB = _impulse.X * _s2 + _impulse.Y + axialImpulse * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                _impulse.SetZero();
                _motorImpulse = F.Zero;
                _lowerImpulse = F.Zero;
                _upperImpulse = F.Zero;
            }

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
            data.Velocities[_indexB].V = vB;
            data.Velocities[_indexB].W = wB;
        }

        internal override void SolveVelocityConstraints(in SolverData data)
        {
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;
            var vB = data.Velocities[_indexB].V;
            var wB = data.Velocities[_indexB].W;

            F mA = _invMassA, mB = _invMassB;
            F iA = _invIA, iB = _invIB;

            // Solve linear motor constraint.
            if (_enableMotor)
            {
                var Cdot = V2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                var impulse = _axialMass * (_motorSpeed - Cdot);
                var oldImpulse = _motorImpulse;
                var maxImpulse = data.Step.Dt * _maxMotorForce;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                var P = impulse * _axis;
                var LA = impulse * _a1;
                var LB = impulse * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            V2 Cdot1;
            Cdot1.X = V2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA;
            Cdot1.Y = wB - wA;

            if (_enableLimit)
            {
                // Lower limit
                {
                    var C = _translation - _lowerTranslation;
                    var Cdot = V2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                    var impulse = -_axialMass * (Cdot + F.Max(C, F.Zero) * data.Step.InvDt);
                    var oldImpulse = _lowerImpulse;
                    _lowerImpulse = F.Max(_lowerImpulse + impulse, F.Zero);
                    impulse = _lowerImpulse - oldImpulse;

                    var P = impulse * _axis;
                    var LA = impulse * _a1;
                    var LB = impulse * _a2;

                    vA -= mA * P;
                    wA -= iA * LA;
                    vB += mB * P;
                    wB += iB * LB;
                }

                // Upper limit
                // Note: signs are flipped to keep C positive when the constraint is satisfied.
                // This also keeps the impulse positive when the limit is active.
                {
                    var C = _upperTranslation - _translation;
                    var Cdot = V2.Dot(_axis, vA - vB) + _a1 * wA - _a2 * wB;
                    var impulse = -_axialMass * (Cdot + F.Max(C, F.Zero) * data.Step.InvDt);
                    var oldImpulse = _upperImpulse;
                    _upperImpulse = F.Max(_upperImpulse + impulse, F.Zero);
                    impulse = _upperImpulse - oldImpulse;

                    var P = impulse * _axis;
                    var LA = impulse * _a1;
                    var LB = impulse * _a2;

                    vA += mA * P;
                    wA += iA * LA;
                    vB -= mB * P;
                    wB -= iB * LB;
                }
            }

            // Solve the prismatic constraint in block form.
            {
                var Cdot = new V2
                {
                    X = V2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA,
                    Y = wB - wA
                };

                var df = _k.Solve(-Cdot);
                _impulse += df;

                var P = df.X * _perp;
                var LA = df.X * _s1 + df.Y;
                var LB = df.X * _s2 + df.Y;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
            data.Velocities[_indexB].V = vB;
            data.Velocities[_indexB].W = wB;
        }

        // A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
        // the position solver is not there to resolve forces.It is only there to cope with integration error.
        //
        // Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
        //
        // We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
        // solver indicates the limit is inactive.
        internal override bool SolvePositionConstraints(in SolverData data)
        {
            var cA = data.Positions[_indexA].Center;
            var aA = data.Positions[_indexA].Angle;
            var cB = data.Positions[_indexB].Center;
            var aB = data.Positions[_indexB].Angle;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            F mA = _invMassA, mB = _invMassB;
            F iA = _invIA, iB = _invIB;

            // Compute fresh Jacobians
            var rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            var rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            var d = cB + rB - cA - rA;

            var axis = MathUtils.Mul(qA, LocalXAxisA);
            var a1 = MathUtils.Cross(d + rA, axis);
            var a2 = MathUtils.Cross(rB, axis);
            var perp = MathUtils.Mul(qA, LocalYAxisA);

            var s1 = MathUtils.Cross(d + rA, perp);
            var s2 = MathUtils.Cross(rB, perp);

            var impulse = new V3();
            var C1 = new V2();
            C1.X = V2.Dot(perp, d);
            C1.Y = aB - aA - ReferenceAngle;

            var linearError = F.Abs(C1.X);
            var angularError = F.Abs(C1.Y);

            var active = false;
            var C2 = F.Zero;
            if (_enableLimit)
            {
                var translation = V2.Dot(axis, d);
                if (F.Abs(_upperTranslation - _lowerTranslation) < F.Two * Settings.LinearSlop)
                {
                    C2 = translation;
                    linearError = F.Max(linearError, F.Abs(translation));
                    active = true;
                }
                else if (translation <= _lowerTranslation)
                {
                    C2 = F.Min(translation - _lowerTranslation, F.Zero);
                    linearError = F.Max(linearError, _lowerTranslation - translation);
                    active = true;
                }
                else if (translation >= _upperTranslation)
                {
                    C2 = F.Max(translation - _upperTranslation, F.Zero);
                    linearError = F.Max(linearError, translation - _upperTranslation);
                    active = true;
                }
            }

            if (active)
            {
                var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                var k12 = iA * s1 + iB * s2;
                var k13 = iA * s1 * a1 + iB * s2 * a2;
                var k22 = iA + iB;
                if (k22.Equals(F.Zero))
                {
                    // For fixed rotation
                    k22 = F.One;
                }

                var k23 = iA * a1 + iB * a2;
                var k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                var K = new Matrix3x3();
                K.Ex.Set(k11, k12, k13);
                K.Ey.Set(k12, k22, k23);
                K.Ez.Set(k13, k23, k33);

                var C = new V3();
                C.X = C1.X;
                C.Y = C1.Y;
                C.Z = C2;

                impulse = K.Solve33(-C);
            }
            else
            {
                var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                var k12 = iA * s1 + iB * s2;
                var k22 = iA + iB;
                if (k22.Equals(F.Zero))
                {
                    k22 = F.One;
                }

                var K = new Matrix2x2();
                K.Ex.Set(k11, k12);
                K.Ey.Set(k12, k22);

                var impulse1 = K.Solve(-C1);
                impulse.X = impulse1.X;
                impulse.Y = impulse1.Y;
                impulse.Z = F.Zero;
            }

            var P = impulse.X * perp + impulse.Z * axis;
            var LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
            var LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.Positions[_indexA].Center = cA;
            data.Positions[_indexA].Angle = aA;
            data.Positions[_indexB].Center = cB;
            data.Positions[_indexB].Angle = aB;

            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }

        /// <inheritdoc />
        public override void Draw(IDrawer drawer)
        {
            var xfA = BodyA.GetTransform();
            var xfB = BodyB.GetTransform();
            var pA = MathUtils.Mul(xfA, LocalAnchorA);
            var pB = MathUtils.Mul(xfB, LocalAnchorB);

            var axis = MathUtils.Mul(xfA.Rotation, LocalXAxisA);

            var c1 = Color.FromArgb(0.7f, 0.7f, 0.7f);
            var c2 = Color.FromArgb(0.3f, 0.9f, 0.3f);
            var c3 = Color.FromArgb(0.9f, 0.3f, 0.3f);
            var c4 = Color.FromArgb(0.3f, 0.3f, 0.9f);
            var c5 = Color.FromArgb(0.4f, 0.4f, 0.4f);

            drawer.DrawSegment(pA, pB, c5);

            if (_enableLimit)
            {
                var lower = pA + _lowerTranslation * axis;
                var upper = pA + _upperTranslation * axis;
                var perp = MathUtils.Mul(xfA.Rotation, LocalYAxisA);
                drawer.DrawSegment(lower, upper, c1);
                drawer.DrawSegment(lower - F.Half * perp, lower + F.Half * perp, c2);
                drawer.DrawSegment(upper - F.Half * perp, upper + F.Half * perp, c3);
            }
            else
            {
                drawer.DrawSegment(pA - F.One * axis, pA + F.One * axis, c1);
            }

            drawer.DrawPoint(pA, 5, c1);
            drawer.DrawPoint(pB, 5, c4);
        }
    }
}