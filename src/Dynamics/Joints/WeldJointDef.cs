using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Weld joint definition. You need to specify local anchor points
    /// where they are attached and the relative body angle. The position
    /// of the anchor points is important for computing the reaction torque.
    public class WeldJointDef : JointDef
    {
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        public F DampingRatio;

        /// The mass-spring-damper frequency in Hertz. Rotation only.
        /// Disable softness with a value of 0.
        public F FrequencyHz;

        /// The local anchor point relative to bodyA's origin.
        public V2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public V2 LocalAnchorB;

        /// The bodyB angle minus bodyA angle in the reference state (radians).
        public F ReferenceAngle;

        public WeldJointDef()
        {
            JointType = JointType.WeldJoint;
            LocalAnchorA.Set(F.Zero, F.Zero);
            LocalAnchorB.Set(F.Zero, F.Zero);
            ReferenceAngle = F.Zero;
            FrequencyHz = F.Zero;
            DampingRatio = F.Zero;
        }

        /// Initialize the bodies, anchors, and reference angle using a world
        /// anchor point.
        public void Initialize(Body bA, Body bB, in V2 anchor)
        {
            BodyA = bA;
            BodyB = bB;
            LocalAnchorA = BodyA.GetLocalPoint(anchor);
            LocalAnchorB = BodyB.GetLocalPoint(anchor);
            ReferenceAngle = BodyB.GetAngle() - BodyA.GetAngle();
        }
    }
}