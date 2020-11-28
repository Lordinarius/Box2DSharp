using System.Numerics;

namespace Box2DSharp.Collision
{
    /// Output for b2Distance.
    public struct DistanceOutput
    {
        /// closest point on shapeA
        public V2 PointA;

        /// closest point on shapeB
        public V2 PointB;

        public F Distance;

        /// number of GJK iterations used
        public int Iterations;
    }
}