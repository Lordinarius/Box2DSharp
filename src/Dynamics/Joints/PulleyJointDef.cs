using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Pulley joint definition. This requires two ground anchors,
    /// two dynamic body anchor points, and a pulley ratio.
    public class PulleyJointDef : JointDef
    {
        /// The first ground anchor in world coordinates. This point never moves.
        public V2 GroundAnchorA;

        /// The second ground anchor in world coordinates. This point never moves.
        public V2 GroundAnchorB;

        /// The a reference length for the segment attached to bodyA.
        public F LengthA;

        /// The a reference length for the segment attached to bodyB.
        public F LengthB;

        /// The local anchor point relative to bodyA's origin.
        public V2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public V2 LocalAnchorB;

        /// The pulley ratio, used to simulate a block-and-tackle.
        public F Ratio;

        public PulleyJointDef()
        {
            JointType = JointType.PulleyJoint;

            GroundAnchorA.Set(-F.One, F.One);

            GroundAnchorB.Set(F.One, F.One);

            LocalAnchorA.Set(-F.One, F.Zero);

            LocalAnchorB.Set(F.One, F.Zero);

            LengthA = F.Zero;

            LengthB = F.Zero;

            Ratio = F.One;

            CollideConnected = true;
        }

        /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
        public void Initialize(
            Body bA,
            Body bB,
            in V2 groundA,
            in V2 groundB,
            in V2 anchorA,
            in V2 anchorB,
            F r)
        {
            BodyA = bA;
            BodyB = bB;
            GroundAnchorA = groundA;
            GroundAnchorB = groundB;
            LocalAnchorA = BodyA.GetLocalPoint(anchorA);
            LocalAnchorB = BodyB.GetLocalPoint(anchorB);
            var dA = anchorA - groundA;
            LengthA = dA.Length();
            var dB = anchorB - groundB;
            LengthB = dB.Length();
            Ratio = r;
            Debug.Assert(Ratio > Settings.Epsilon);
        }
    }
}