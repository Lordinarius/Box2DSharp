using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2DSharp.Common
{
    public struct Matrix2x2
    {
        public V2 Ex;

        public V2 Ey;

        /// The default constructor does nothing (for performance).
        /// Construct this matrix using columns.
        public Matrix2x2(in V2 c1, in V2 c2)
        {
            Ex = c1;
            Ey = c2;
        }

        /// Construct this matrix using scalars.
        public Matrix2x2(F a11, F a12, F a21, F a22)
        {
            Ex.X = a11;
            Ex.Y = a21;
            Ey.X = a12;
            Ey.Y = a22;
        }

        /// Initialize this matrix using columns.
        public void Set(in V2 c1, in V2 c2)
        {
            Ex = c1;
            Ey = c2;
        }

        /// Set this to the identity matrix.
        public void SetIdentity()
        {
            Ex.X = F.One;
            Ey.X = F.Zero;
            Ex.Y = F.Zero;
            Ey.Y = F.One;
        }

        /// Set this matrix to all zeros.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetZero()
        {
            Ex.X = F.Zero;
            Ey.X = F.Zero;
            Ex.Y = F.Zero;
            Ey.Y = F.Zero;
        }

        public Matrix2x2 GetInverse()
        {
            var a = Ex.X;
            var b = Ey.X;
            var c = Ex.Y;
            var d = Ey.Y;

            var det = a * d - b * c;
            if (!det.Equals(F.Zero))
            {
                det = F.One / det;
            }

            var B = new Matrix2x2();
            B.Ex.X = det * d;
            B.Ey.X = -det * b;
            B.Ex.Y = -det * c;
            B.Ey.Y = det * a;
            return B;
        }

        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        public V2 Solve(in V2 b)
        {
            var a11 = Ex.X;
            var a12 = Ey.X;
            var a21 = Ex.Y;
            var a22 = Ey.Y;
            var det = a11 * a22 - a12 * a21;
            if (!det.Equals(F.Zero))
            {
                det = F.One / det;
            }

            var x = new V2 {X = det * (a22 * b.X - a12 * b.Y), Y = det * (a11 * b.Y - a21 * b.X)};
            return x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix2x2 operator +(in Matrix2x2 A, in Matrix2x2 B)
        {
            return new Matrix2x2(A.Ex + B.Ex, A.Ey + B.Ey);
        }
    }
}