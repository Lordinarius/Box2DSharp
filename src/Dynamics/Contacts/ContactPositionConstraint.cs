using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public struct ContactPositionConstraint
    {
        /// <summary>
        /// Size <see cref="Settings.MaxManifoldPoints"/>
        /// </summary>
        public FixedArray2<V2> LocalPoints;

        public int IndexA;

        public int IndexB;

        public F InvIa, InvIb;

        public F InvMassA, InvMassB;

        public V2 LocalCenterA, LocalCenterB;

        public V2 LocalNormal;

        public V2 LocalPoint;

        public int PointCount;

        public F RadiusA, RadiusB;

        public ManifoldType Type;
    }
}