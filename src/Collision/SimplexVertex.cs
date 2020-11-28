using System.Numerics;

namespace Box2DSharp.Collision
{
    public struct SimplexVertex
    {
        public V2 Wa; // support point in proxyA

        public V2 Wb; // support point in proxyB

        public V2 W; // wB - wA

        public F A; // barycentric coordinate for closest point

        public int IndexA; // wA index

        public int IndexB; // wB index
    }
}