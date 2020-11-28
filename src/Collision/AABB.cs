using System;
using System.Diagnostics.Contracts;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    /// <summary>
    ///     An axis aligned bounding box.
    /// </summary>
    public struct AABB
    {
        /// <summary>
        ///     the lower vertex
        /// </summary>
        public V2 LowerBound;

        /// <summary>
        ///     the upper vertex
        /// </summary>
        public V2 UpperBound;

        public AABB(in V2 lowerBound, in V2 upperBound)
        {
            LowerBound = lowerBound;
            UpperBound = upperBound;
        }

        /// <summary>
        ///     Verify that the bounds are sorted.
        /// </summary>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool IsValid()
        {
            var d = UpperBound - LowerBound;
            var valid = d.X >= F.Zero && d.Y >= F.Zero;
            valid = valid && LowerBound.IsValid() && UpperBound.IsValid();
            return valid;
        }

        /// <summary>
        ///     Get the center of the AABB.
        /// </summary>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public V2 GetCenter()
        {
            return F.Half * (LowerBound + UpperBound);
        }

        /// <summary>
        ///     Get the extents of the AABB (half-widths).
        /// </summary>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public V2 GetExtents()
        {
            return F.Half * (UpperBound - LowerBound);
        }

        /// <summary>
        ///     Get the perimeter length
        /// </summary>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public F GetPerimeter()
        {
            var wx = UpperBound.X - LowerBound.X;
            var wy = UpperBound.Y - LowerBound.Y;
            return wx + wx + wy + wy;
        }

        public bool RayCast(out RayCastOutput output, in RayCastInput input)
        {
            output = default;
            var tmin = -Settings.MaxFloat;
            var tmax = Settings.MaxFloat;

            var p = input.P1;
            var d = input.P2 - input.P1;
            var absD = V2.Abs(d);

            var normal = new V2();

            {
                if (absD.X < Settings.Epsilon)
                {
                    // Parallel.
                    if (p.X < LowerBound.X || UpperBound.X < p.X)
                    {
                        return false;
                    }
                }
                else
                {
                    var invD = F.One / d.X;
                    var t1 = (LowerBound.X - p.X) * invD;
                    var t2 = (UpperBound.X - p.X) * invD;

                    // Sign of the normal vector.
                    var s = -F.One;

                    if (t1 > t2)
                    {
                        MathUtils.Swap(ref t1, ref t2);
                        s = F.One;
                    }

                    // Push the min up
                    if (t1 > tmin)
                    {
                        normal.SetZero();
                        normal.X = s;
                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = F.Min(tmax, t2);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }
            {
                if (absD.Y < Settings.Epsilon)
                {
                    // Parallel.
                    if (p.Y < LowerBound.Y || UpperBound.Y < p.Y)
                    {
                        return false;
                    }
                }
                else
                {
                    var invD = F.One / d.Y;
                    var t1 = (LowerBound.Y - p.Y) * invD;
                    var t2 = (UpperBound.Y - p.Y) * invD;

                    // Sign of the normal vector.
                    var s = -F.One;

                    if (t1 > t2)
                    {
                        MathUtils.Swap(ref t1, ref t2);
                        s = F.One;
                    }

                    // Push the min up
                    if (t1 > tmin)
                    {
                        normal.SetZero();
                        normal.Y = s;
                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = F.Min(tmax, t2);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin < F.Zero || input.MaxFraction < tmin)
            {
                return false;
            }

            // Intersection.
            output = new RayCastOutput {Fraction = tmin, Normal = normal};

            return true;
        }

        public static void Combine(in AABB left, in AABB right, out AABB aabb)
        {
            aabb = new AABB(
                V2.Min(left.LowerBound, right.LowerBound),
                V2.Max(left.UpperBound, right.UpperBound));
        }

        /// <summary>
        ///     Combine an AABB into this one.
        /// </summary>
        /// <param name="aabb"></param>
        public void Combine(in AABB aabb)
        {
            LowerBound = V2.Min(LowerBound, aabb.LowerBound);
            UpperBound = V2.Max(UpperBound, aabb.UpperBound);
        }

        /// <summary>
        ///     Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1"></param>
        /// <param name="aabb2"></param>
        public void Combine(in AABB aabb1, in AABB aabb2)
        {
            LowerBound = V2.Min(aabb1.LowerBound, aabb2.LowerBound);
            UpperBound = V2.Max(aabb1.UpperBound, aabb2.UpperBound);
        }

        /// <summary>
        ///     Does this aabb contain the provided AABB.
        /// </summary>
        /// <param name="aabb">the provided AABB</param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool Contains(in AABB aabb)
        {
            return LowerBound.X <= aabb.LowerBound.X
                && LowerBound.Y <= aabb.LowerBound.Y
                && aabb.UpperBound.X <= UpperBound.X
                && aabb.UpperBound.Y <= UpperBound.Y;
        }
    }
}