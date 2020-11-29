namespace Box2DSharp.Common
{
    public static class Settings
    {
        public readonly static F MaxFloat = F.MaxValue;

        public readonly static F Epsilon = F.Epsilon;

        public readonly static F Pi = F.Pi;

        // @file
        // Global tuning constants based on meters-kilograms-seconds (MKS) units.

        // Collision

        /// The maximum number of contact points between two convex shapes. Do
        /// not change this value.
        public const int MaxManifoldPoints = 2;

        /// The maximum number of vertices on a convex polygon. You cannot increase
        /// this too much because b2BlockAllocator has a maximum object size.
        public const int MaxPolygonVertices = 8;

        /// This is used to fatten AABBs in the dynamic tree. This allows proxies
        /// to move by a small amount without triggering a tree adjustment.
        /// This is in meters.
        public readonly static F AABBExtension = new F(429496736L);//0.1f;

        /// This is used to fatten AABBs in the dynamic tree. This is used to predict
        /// the future position based on the current displacement.
        /// This is a dimensionless multiplier.
        public readonly static F AABBMultiplier = 4;

        /// A small length used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        public readonly static F LinearSlop = new F(21474836L); //0.005f;

        /// A small angle used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        public readonly static F AngularSlop = F.Two / ((F)180) * Pi;

        /// The radius of the polygon/edge shape skin. This should not be modified. Making
        /// this smaller means polygons will have an insufficient buffer for continuous collision.
        /// Making it larger may create artifacts for vertex collision.
        public readonly static F PolygonRadius = F.Two * LinearSlop;

        /// Maximum number of sub-steps per contact in continuous physics simulation.
        public const int MaxSubSteps = 8;

        // Dynamics

        /// Maximum number of contacts to be handled to solve a TOI impact.
        public const int MaxToiContacts = 32;

        /// A velocity threshold for elastic collisions. Any collision with a relative linear
        /// velocity below this threshold will be treated as inelastic.
        public readonly static F VelocityThreshold = F.One;

        /// The maximum linear position correction used when solving constraints. This helps to
        /// prevent overshoot.
        public readonly static F MaxLinearCorrection = new F(858993472L); //0.2f;

        /// The maximum angular position correction used when solving constraints. This helps to
        /// prevent overshoot.
        public readonly static F MaxAngularCorrection = ((F)8) / ((F)180) * Pi;

        /// The maximum linear velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        public readonly static F MaxTranslation = F.Two;

        public readonly static F MaxTranslationSquared = MaxTranslation * MaxTranslation;

        /// The maximum angular velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        public readonly static F MaxRotation = F.Half * Pi;

        public readonly static F MaxRotationSquared = MaxRotation * MaxRotation;

        /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
        /// that overlap is removed in one time step. However using values close to 1 often lead
        /// to overshoot.
        public readonly static F Baumgarte = new F(858993472L); //0.2f;

        public readonly static F ToiBaumgarte = new F(3221225472L); //0.75f;

        // Sleep

        /// The time that a body must be still before it will go to sleep.
        public readonly static F TimeToSleep = F.Half;

        /// A body cannot sleep if its linear velocity is above this tolerance.
        public readonly static F LinearSleepTolerance = new F(429496736L); //0.01f;

        /// A body cannot sleep if its angular velocity is above this tolerance.
        public readonly static F AngularSleepTolerance = F.Two / ((F)180) * Pi;
    }
}