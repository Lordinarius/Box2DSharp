using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2DSharp.Common
{
    /// Rotation
    public struct Rotation
    {
        /// Sine and cosine
        public F Sin;

        public F Cos;

        public Rotation(F sin, F cos)
        {
            Sin = sin;
            Cos = cos;
        }

        /// Initialize from an angle in radians
        public Rotation(F angle)
        {
            // TODO_ERIN optimize
            Sin = F.Sin(angle);
            Cos = F.Cos(angle);
        }

        /// Set using an angle in radians.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Set(F angle)
        {
            // TODO_ERIN optimize
            Sin = F.Sin(angle);
            Cos = F.Cos(angle);
        }

        /// Set to the identity rotation
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetIdentity()
        {
            Sin = F.Zero;
            Cos = F.One;
        }

        /// Get the angle in radians
        public F Angle => F.Atan2(Sin, Cos);

        /// Get the x-axis
        public V2 GetXAxis()
        {
            return new V2(Cos, Sin);
        }

        /// Get the u-axis
        public V2 GetYAxis()
        {
            return new V2(-Sin, Cos);
        }
    }
}