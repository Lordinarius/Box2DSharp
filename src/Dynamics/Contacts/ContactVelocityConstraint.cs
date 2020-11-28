using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public struct ContactVelocityConstraint
    {
        /// <summary>
        /// Size <see cref="Settings.MaxManifoldPoints"/>
        /// </summary>
        public FixedArray2<VelocityConstraintPoint> Points;

        public int ContactIndex;

        public F Friction;

        public int IndexA;

        public int IndexB;

        public F InvIa, InvIb;

        public F InvMassA, InvMassB;

        public Matrix2x2 K;

        public V2 Normal;

        public Matrix2x2 NormalMass;

        public int PointCount;

        public F Restitution;

        public F TangentSpeed;
    }
}