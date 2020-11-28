namespace Box2DSharp.Ropes
{
    public struct RopeBend
    {
        public int i1;

        public int i2;

        public int i3;

        public F invMass1;

        public F invMass2;

        public F invMass3;

        public F invEffectiveMass;

        public F lambda;

        public F L1, L2;

        public F alpha1;

        public F alpha2;

        public F spring;

        public F damper;
    };
}