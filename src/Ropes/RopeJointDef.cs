using System.Numerics;
using Box2DSharp.Common;
using Box2DSharp.Dynamics.Joints;

namespace Box2DSharp.Ropes
{
    /// Rope joint definition. This requires two body anchor points and
    /// a maximum lengths.
    /// Note: by default the connected objects will not collide.
    /// see collideConnected in b2JointDef.
    public class RopeJointDef : JointDef
    {
        public RopeJointDef()
        {
            JointType = JointType.RopeJoint;
            LocalAnchorA.Set(-F.One, F.Zero);
            LocalAnchorB.Set(F.One, F.Zero);
            MaxLength = F.Zero;
        }

        /// The local anchor point relative to bodyA's origin.
        public V2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public V2 LocalAnchorB;

        /// The maximum length of the rope.
        /// Warning: this must be larger than b2_linearSlop or
        /// the joint will have no effect.
        public F MaxLength;
    };
}