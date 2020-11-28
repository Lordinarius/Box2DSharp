using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2DSharp.Common
{
    public static class MathExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsValid(in this V2 vector2)
        {
            return true;
            //return !F.IsInfinity(vector2.X) && !F.IsInfinity(vector2.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsValid(this F x)
        {
            return true;
            //return !F.IsInfinity(x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetZero(ref this V2 vector2)
        {
            vector2.X = F.Zero;
            vector2.Y = F.Zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Set(ref this V2 vector2, F x, F y)
        {
            vector2.X = x;
            vector2.Y = y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetZero(ref this Vector3 vector3)
        {
            vector3.X = F.Zero;
            vector3.Y = F.Zero;
            vector3.Z = F.Zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Set(ref this Vector3 vector3, F x, F y, F z)
        {
            vector3.X = x;
            vector3.Y = y;
            vector3.Z = z;
        }

        /// Convert this vector into a unit vector. Returns the length.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static F Normalize(ref this V2 vector2)
        {
            var length = vector2.Length();
            if (length < Settings.Epsilon)
            {
                return F.Zero;
            }

            var invLength = F.One / length;
            vector2.X *= invLength;
            vector2.Y *= invLength;

            return length;
        }

        /// <summary>
        ///  Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
        /// </summary>
        /// <param name="vector2"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V2 Skew(ref this V2 vector2)
        {
            return new V2(-vector2.Y, vector2.X);
        }
    }
}