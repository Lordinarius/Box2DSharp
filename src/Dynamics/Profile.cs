namespace Box2DSharp.Dynamics
{
    /// Profiling data. Times are in milliseconds.
    public struct Profile
    {
        public F Step;

        public F Collide;

        public F Solve;

        public F SolveInit;

        public F SolveVelocity;

        public F SolvePosition;

        public F Broadphase;

        public F SolveTOI;
    }
}