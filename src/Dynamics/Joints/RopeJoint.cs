using System;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A rope joint enforces a maximum distance between two points
    /// on two bodies. It has no other effect.
    /// Warning: if you attempt to change the maximum length during
    /// the simulation you will get some non-physical behavior.
    /// A model that would allow you to dynamically modify the length
    /// would have some sponginess, so I chose not to implement it
    /// that way. See b2DistanceJoint if you want to dynamically
    /// control length.
    public class RopeJoint : Joint
    {
        // Solver shared
        private readonly V2 _localAnchorA;

        private readonly V2 _localAnchorB;

        private F _impulse;

        // Solver temp
        private int _indexA;

        private int _indexB;

        private F _invIa;

        private F _invIb;

        private F _invMassA;

        private F _invMassB;

        private F _length;

        private V2 _localCenterA;

        private V2 _localCenterB;

        private F _mass;

        private F _maxLength;

        private V2 _rA;

        private V2 _rB;

        private V2 _u;

        internal RopeJoint(RopeJointDef def)
            : base(def)
        {
            _localAnchorA = def.LocalAnchorA;
            _localAnchorB = def.LocalAnchorB;

            _maxLength = def.MaxLength;

            _mass = F.Zero;
            _impulse = F.Zero;
            _length = F.Zero;
        }

        /// The local anchor point relative to bodyA's origin.
        public V2 GetLocalAnchorA()
        {
            return _localAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        public V2 GetLocalAnchorB()
        {
            return _localAnchorB;
        }

        /// Set/Get the maximum length of the rope.
        public void SetMaxLength(F length)
        {
            _maxLength = length;
        }

        public F GetMaxLength()
        {
            return _maxLength;
        }

        public F GetLength()
        {
            return _length;
        }

        /// <inheritdoc />
        public override V2 GetAnchorA()
        {
            return BodyA.GetWorldPoint(_localAnchorA);
        }

        /// <inheritdoc />
        public override V2 GetAnchorB()
        {
            return BodyB.GetWorldPoint(_localAnchorB);
        }

        /// <inheritdoc />
        public override V2 GetReactionForce(F inv_dt)
        {
            var F = inv_dt * _impulse * _u;
            return F;
        }

        /// <inheritdoc />
        public override F GetReactionTorque(F inv_dt)
        {
            return F.Zero;
        }

        /// Dump joint to dmLog
        public override void Dump()
        {
            throw new NotImplementedException();
        }

        /// <inheritdoc />
        internal override void InitVelocityConstraints(in SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _indexB = BodyB.IslandIndex;
            _localCenterA = BodyA.Sweep.LocalCenter;
            _localCenterB = BodyB.Sweep.LocalCenter;
            _invMassA = BodyA.InvMass;
            _invMassB = BodyB.InvMass;
            _invIa = BodyA.InverseInertia;
            _invIb = BodyB.InverseInertia;

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

            _rA = MathUtils.Mul(qA, _localAnchorA - _localCenterA);
            _rB = MathUtils.Mul(qB, _localAnchorB - _localCenterB);
            _u = cB + _rB - cA - _rA;

            _length = _u.Length();

            if (_length > Settings.LinearSlop)
            {
                _u *= F.One / _length;
            }
            else
            {
                _u.SetZero();
                _mass = F.Zero;
                _impulse = F.Zero;
                return;
            }

            // Compute effective mass.
            var crA = MathUtils.Cross(_rA, _u);
            var crB = MathUtils.Cross(_rB, _u);
            var invMass = _invMassA + _invIa * crA * crA + _invMassB + _invIb * crB * crB;

            _mass = !invMass.Equals(F.Zero) ? F.One / invMass : F.Zero;

            if (data.Step.WarmStarting)
            {
                // Scale the impulse to support a variable time step.
                _impulse *= data.Step.DtRatio;

                var P = _impulse * _u;
                vA -= _invMassA * P;
                wA -= _invIa * MathUtils.Cross(_rA, P);
                vB += _invMassB * P;
                wB += _invIb * MathUtils.Cross(_rB, P);
            }
            else
            {
                _impulse = F.Zero;
            }

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
            data.Velocities[_indexB].V = vB;
            data.Velocities[_indexB].W = wB;
        }

        /// <inheritdoc />
        internal override void SolveVelocityConstraints(in SolverData data)
        {
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;
            var vB = data.Velocities[_indexB].V;
            var wB = data.Velocities[_indexB].W;

            // Cdot = dot(u, v + cross(w, r))
            var vpA = vA + MathUtils.Cross(wA, _rA);
            var vpB = vB + MathUtils.Cross(wB, _rB);
            var C = _length - _maxLength;
            var cdot = V2.Dot(_u, vpB - vpA);

            // Predictive constraint.
            if (C < F.Zero)
            {
                cdot += data.Step.InvDt * C;
            }

            var impulse = -_mass * cdot;
            var oldImpulse = _impulse;
            _impulse = F.Min(F.Zero, _impulse + impulse);
            impulse = _impulse - oldImpulse;

            var P = impulse * _u;
            vA -= _invMassA * P;
            wA -= _invIa * MathUtils.Cross(_rA, P);
            vB += _invMassB * P;
            wB += _invIb * MathUtils.Cross(_rB, P);

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
            data.Velocities[_indexB].V = vB;
            data.Velocities[_indexB].W = wB;
        }

        /// <inheritdoc />
        internal override bool SolvePositionConstraints(in SolverData data)
        {
            var cA = data.Positions[_indexA].Center;
            var aA = data.Positions[_indexA].Angle;
            var cB = data.Positions[_indexB].Center;
            var aB = data.Positions[_indexB].Angle;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            var rA = MathUtils.Mul(qA, _localAnchorA - _localCenterA);
            var rB = MathUtils.Mul(qB, _localAnchorB - _localCenterB);
            var u = cB + rB - cA - rA;

            _length = u.Normalize();
            var C = _length - _maxLength;

            C = MathUtils.Clamp(C, F.Zero, Settings.MaxLinearCorrection);

            var impulse = -_mass * C;
            var P = impulse * u;

            cA -= _invMassA * P;
            aA -= _invIa * MathUtils.Cross(rA, P);
            cB += _invMassB * P;
            aB += _invIb * MathUtils.Cross(rB, P);

            data.Positions[_indexA].Center = cA;
            data.Positions[_indexA].Angle = aA;
            data.Positions[_indexB].Center = cB;
            data.Positions[_indexB].Angle = aB;

            return _length - _maxLength < Settings.LinearSlop;
        }
    }
}