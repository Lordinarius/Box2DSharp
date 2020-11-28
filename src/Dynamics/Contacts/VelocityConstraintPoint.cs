using System.Numerics;

namespace Box2DSharp.Dynamics.Contacts
{
    public struct VelocityConstraintPoint
    {
        public F NormalImpulse;

        public F NormalMass;

        public V2 Ra;

        public V2 Rb;

        public F TangentImpulse;

        public F TangentMass;

        public F VelocityBias;
    }
}