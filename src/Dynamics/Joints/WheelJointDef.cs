using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Wheel joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
    public class WheelJointDef : JointDef
    {
        /// The local anchor point relative to bodyA's origin.
        public V2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public V2 LocalAnchorB;

        /// The local translation axis in bodyA.
        public V2 LocalAxisA;

        /// Enable/disable the joint limit.
        public bool EnableLimit;

        /// The lower translation limit, usually in meters.
        public F LowerTranslation;

        /// The upper translation limit, usually in meters.
        public F UpperTranslation;

        /// Enable/disable the joint motor.
        public bool EnableMotor;

        /// The maximum motor torque, usually in N-m.
        public F MaxMotorTorque;

        /// The desired motor speed in radians per second.
        public F MotorSpeed;

        /// Suspension stiffness. Typically in units N/m.
        public F Stiffness;

        /// Suspension damping. Typically in units of N*s/m.
        public F Damping;

        public WheelJointDef()
        {
            JointType = JointType.WheelJoint;
            LocalAnchorA.SetZero();
            LocalAnchorB.SetZero();
            LocalAxisA.Set(F.One, F.Zero);
            EnableLimit = false;
            LowerTranslation = F.Zero;
            UpperTranslation = F.Zero;
            EnableMotor = false;
            MaxMotorTorque = F.Zero;
            MotorSpeed = F.Zero;
            Stiffness = F.Zero;
            Damping = F.Zero;
        }

        /// Initialize the bodies, anchors, axis, and reference angle using the world
        /// anchor and world axis.
        public void Initialize(Body bA, Body bB, in V2 anchor, in V2 axis)
        {
            BodyA = bA;
            BodyB = bB;
            LocalAnchorA = BodyA.GetLocalPoint(anchor);
            LocalAnchorB = BodyB.GetLocalPoint(anchor);
            LocalAxisA = BodyA.GetLocalVector(axis);
        }
    }
}