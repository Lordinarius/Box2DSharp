using System.Numerics;

namespace Box2DSharp.Collision
{
    /// Output results for b2ShapeCast
    public struct ShapeCastOutput
    {
        public V2 Point;

        public V2 Normal;

        public F Lambda;

        public int Iterations;
    }
}