namespace Box2DSharp.Ropes
{
    ///
    public class RopeTuning
    {
        public RopeTuning()
        {
            StretchingModel = StretchingModel.PbdStretchingModel;
            BendingModel = BendingModel.PbdAngleBendingModel;
            Damping = F.Zero;
            StretchStiffness = F.One;
            BendStiffness = F.Half;
            BendHertz = F.One;
            BendDamping = F.Zero;
            Isometric = false;
            FixedEffectiveMass = false;
            WarmStart = false;
        }

        public StretchingModel StretchingModel;

        public BendingModel BendingModel;

        public F Damping;

        public F StretchStiffness;

        public F StretchHertz;

        public F StretchDamping;

        public F BendStiffness;

        public F BendHertz;

        public F BendDamping;

        public bool Isometric;

        public bool FixedEffectiveMass;

        public bool WarmStart;
    };
}