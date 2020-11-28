using System.Numerics;

namespace Box2DSharp.Ropes
{
    /// 
    public struct RopeDef
    {
        public V2 Position;

        public V2[] Vertices;

        public int Count;

        public F[] Masses;

        public V2 Gravity;

        public RopeTuning Tuning;
    };
}