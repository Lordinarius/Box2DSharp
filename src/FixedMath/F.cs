using System;
using System.IO;
using System.Runtime.CompilerServices;

/// <summary>
/// Represents a Q31.32 fixed-point number.
/// </summary>
public partial struct F : IEquatable<F>, IComparable<F>
{
    readonly long m_rawValue;

    // Precision of this type is 2^-32, that is 2,3283064365386962890625E-10
    public static readonly decimal Precision = (decimal)(new F(1L));//0.00000000023283064365386962890625m;
    public static readonly F MaxValue = new F(MAX_VALUE);
    public static readonly F MinValue = new F(MIN_VALUE);
    public static readonly F Two = new F(TWO);
    public static readonly F One = new F(ONE);
    public static readonly F Half = new F(HALF);
    public static readonly F Epsilon = new F(EPSILON);
    public static readonly F Zero = new F();
    /// <summary>
    /// The value of Pi
    /// </summary>
    public static readonly F Pi = new F(PI);
    public static readonly F PiOver2 = new F(PI_OVER_2);
    public static readonly F PiTimes2 = new F(PI_TIMES_2);
    public static readonly F PiInv = new F(1367130496L); //0.3183099f
    public static readonly F PiOver2Inv = new F(2734260992L); //0.6366197f
    public static readonly F Rad2Deg = new F(246083502080L); //57.29578f
    public static readonly F Deg2Rad = new F(74961320L); //0.01745329f

    static readonly F LutInterval = (F)(LUT_SIZE - 1) / PiOver2;
    const long MAX_VALUE = long.MaxValue;
    const long MIN_VALUE = long.MinValue;
    const int NUM_BITS = 64;
    const int FRACTIONAL_PLACES = 32;
    public const long ONE = 1L << FRACTIONAL_PLACES;
    const long HALF = 1L << (FRACTIONAL_PLACES - 1);
    const long TWO = 1L << (FRACTIONAL_PLACES + 1);
    const long PI_TIMES_2 = 0x6487ED511;
    const long PI = 0x3243F6A88;
    const long PI_OVER_2 = 0x1921FB544;
    const long EPSILON = 429496L;
    const int LUT_SIZE = (int)(PI_OVER_2 >> 15);
    const int LUT_SIZE_ASIN = (int)(ONE >> 16);

    /// <summary>
    /// Returns a number indicating the sign of a Fix64 number.
    /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
    /// </summary>
    public static int Sign(F value)
    {
        return
            value.m_rawValue < 0 ? -1 :
            value.m_rawValue > 0 ? 1 :
            0;
    }


    /// <summary>
    /// Returns the absolute value of a Fix64 number.
    /// Note: Abs(Fix64.MinValue) == Fix64.MaxValue.
    /// </summary>
    public static F Abs(F value)
    {
        if (value.m_rawValue == MIN_VALUE)
        {
            return MaxValue;
        }

        // branchless implementation, see http://www.strchr.com/optimized_abs_function
        var mask = value.m_rawValue >> 63;
        return new F((value.m_rawValue + mask) ^ mask);
    }

    /// <summary>
    /// Returns the absolute value of a Fix64 number.
    /// FastAbs(Fix64.MinValue) is undefined.
    /// </summary>
    public static F FastAbs(F value)
    {
        // branchless implementation, see http://www.strchr.com/optimized_abs_function
        var mask = value.m_rawValue >> 63;
        return new F((value.m_rawValue + mask) ^ mask);
    }


    /// <summary>
    /// Returns the largest integer less than or equal to the specified number.
    /// </summary>
    public static F Floor(F value)
    {
        // Just zero out the fractional part
        return new F((long)((ulong)value.m_rawValue & 0xFFFFFFFF00000000));
    }


    /// <summary>
    /// Returns the largest integer less than or equal to the specified number.
    /// </summary>
    public static int FloorToInt(F value)
    {
        // Just zero out the fractional part
        return (int)Floor(value);
    }

    /// <summary>
    /// Returns the smallest integral value that is greater than or equal to the specified number.
    /// </summary>
    public static F Ceiling(F value)
    {
        var hasFractionalPart = (value.m_rawValue & 0x00000000FFFFFFFF) != 0;
        return hasFractionalPart ? Floor(value) + One : value;
    }

    /// <summary>
    /// Rounds a value to the nearest integral value.
    /// If the value is halfway between an even and an uneven value, returns the even value.
    /// </summary>
    public static F Round(F value)
    {
        var fractionalPart = value.m_rawValue & 0x00000000FFFFFFFF;
        var integralPart = Floor(value);
        if (fractionalPart < 0x80000000)
        {
            return integralPart;
        }
        if (fractionalPart > 0x80000000)
        {
            return integralPart + One;
        }
        // if number is halfway between two values, round to the nearest even number
        // this is the method used by System.Math.Round().
        return (integralPart.m_rawValue & ONE) == 0
                   ? integralPart
                   : integralPart + One;
    }

    /// <summary>
    /// Adds x and y. Performs saturating addition, i.e. in case of overflow, 
    /// rounds to MinValue or MaxValue depending on sign of operands.
    /// </summary>
    public static F operator +(F x, F y)
    {
        var xl = x.m_rawValue;
        var yl = y.m_rawValue;
        var sum = xl + yl;
        // if signs of operands are equal and signs of sum and x are different
        if (((~(xl ^ yl) & (xl ^ sum)) & MIN_VALUE) != 0)
        {
            sum = xl > 0 ? MAX_VALUE : MIN_VALUE;
        }
        return new F(sum);
    }

    /// <summary>
    /// Adds x and y witout performing overflow checking. Should be inlined by the CLR.
    /// </summary>
    public static F FastAdd(F x, F y)
    {
        return new F(x.m_rawValue + y.m_rawValue);
    }

    /// <summary>
    /// Subtracts y from x. Performs saturating substraction, i.e. in case of overflow, 
    /// rounds to MinValue or MaxValue depending on sign of operands.
    /// </summary>
    public static F operator -(F x, F y)
    {
        var xl = x.m_rawValue;
        var yl = y.m_rawValue;
        var diff = xl - yl;
        // if signs of operands are different and signs of sum and x are different
        if ((((xl ^ yl) & (xl ^ diff)) & MIN_VALUE) != 0)
        {
            diff = xl < 0 ? MIN_VALUE : MAX_VALUE;
        }
        return new F(diff);
    }

    /// <summary>
    /// Subtracts y from x witout performing overflow checking. Should be inlined by the CLR.
    /// </summary>
    public static F FastSub(F x, F y)
    {
        return new F(x.m_rawValue - y.m_rawValue);
    }

    static long AddOverflowHelper(long x, long y, ref bool overflow)
    {
        var sum = x + y;
        // x + y overflows if sign(x) ^ sign(y) != sign(sum)
        overflow |= ((x ^ y ^ sum) & MIN_VALUE) != 0;
        return sum;
    }

    public static F operator *(F x, F y)
    {

        var xl = x.m_rawValue;
        var yl = y.m_rawValue;

        var xlo = (ulong)(xl & 0x00000000FFFFFFFF);
        var xhi = xl >> FRACTIONAL_PLACES;
        var ylo = (ulong)(yl & 0x00000000FFFFFFFF);
        var yhi = yl >> FRACTIONAL_PLACES;

        var lolo = xlo * ylo;
        var lohi = (long)xlo * yhi;
        var hilo = xhi * (long)ylo;
        var hihi = xhi * yhi;

        var loResult = lolo >> FRACTIONAL_PLACES;
        var midResult1 = lohi;
        var midResult2 = hilo;
        var hiResult = hihi << FRACTIONAL_PLACES;

        bool overflow = false;
        var sum = AddOverflowHelper((long)loResult, midResult1, ref overflow);
        sum = AddOverflowHelper(sum, midResult2, ref overflow);
        sum = AddOverflowHelper(sum, hiResult, ref overflow);

        bool opSignsEqual = ((xl ^ yl) & MIN_VALUE) == 0;

        // if signs of operands are equal and sign of result is negative,
        // then multiplication overflowed positively
        // the reverse is also true
        if (opSignsEqual)
        {
            if (sum < 0 || (overflow && xl > 0))
            {
                return MaxValue;
            }
        }
        else
        {
            if (sum > 0)
            {
                return MinValue;
            }
        }

        // if the top 32 bits of hihi (unused in the result) are neither all 0s or 1s,
        // then this means the result overflowed.
        var topCarry = hihi >> FRACTIONAL_PLACES;
        if (topCarry != 0 && topCarry != -1 /*&& xl != -17 && yl != -17*/)
        {
            return opSignsEqual ? MaxValue : MinValue;
        }

        // If signs differ, both operands' magnitudes are greater than 1,
        // and the result is greater than the negative operand, then there was negative overflow.
        if (!opSignsEqual)
        {
            long posOp, negOp;
            if (xl > yl)
            {
                posOp = xl;
                negOp = yl;
            }
            else
            {
                posOp = yl;
                negOp = xl;
            }
            if (sum > negOp && negOp < -ONE && posOp > ONE)
            {
                return MinValue;
            }
        }

        return new F(sum);
    }

    /// <summary>
    /// Performs multiplication without checking for overflow.
    /// Useful for performance-critical code where the values are guaranteed not to cause overflow
    /// </summary>
    public static F FastMul(F x, F y)
    {

        var xl = x.m_rawValue;
        var yl = y.m_rawValue;

        var xlo = (ulong)(xl & 0x00000000FFFFFFFF);
        var xhi = xl >> FRACTIONAL_PLACES;
        var ylo = (ulong)(yl & 0x00000000FFFFFFFF);
        var yhi = yl >> FRACTIONAL_PLACES;

        var lolo = xlo * ylo;
        var lohi = (long)xlo * yhi;
        var hilo = xhi * (long)ylo;
        var hihi = xhi * yhi;

        var loResult = lolo >> FRACTIONAL_PLACES;
        var midResult1 = lohi;
        var midResult2 = hilo;
        var hiResult = hihi << FRACTIONAL_PLACES;

        var sum = (long)loResult + midResult1 + midResult2 + hiResult;
        return new F(sum);
    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)] 
    static int CountLeadingZeroes(ulong x)
    {
        int result = 0;
        while ((x & 0xF000000000000000) == 0) { result += 4; x <<= 4; }
        while ((x & 0x8000000000000000) == 0) { result += 1; x <<= 1; }
        return result;
    }

    public static F operator /(F x, F y)
    {
        var xl = x.m_rawValue;
        var yl = y.m_rawValue;

        if (yl == 0)
        {
            throw new DivideByZeroException();
        }

        var remainder = (ulong)(xl >= 0 ? xl : -xl);
        var divider = (ulong)(yl >= 0 ? yl : -yl);
        var quotient = 0UL;
        var bitPos = NUM_BITS / 2 + 1;


        // If the divider is divisible by 2^n, take advantage of it.
        while ((divider & 0xF) == 0 && bitPos >= 4)
        {
            divider >>= 4;
            bitPos -= 4;
        }

        while (remainder != 0 && bitPos >= 0)
        {
            int shift = CountLeadingZeroes(remainder);
            if (shift > bitPos)
            {
                shift = bitPos;
            }
            remainder <<= shift;
            bitPos -= shift;

            var div = remainder / divider;
            remainder = remainder % divider;
            quotient += div << bitPos;

            // Detect overflow
            if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0)
            {
                return ((xl ^ yl) & MIN_VALUE) == 0 ? MaxValue : MinValue;
            }

            remainder <<= 1;
            --bitPos;
        }

        // rounding
        ++quotient;
        var result = (long)(quotient >> 1);
        if (((xl ^ yl) & MIN_VALUE) != 0)
        {
            result = -result;
        }

        return new F(result);
    }

    public static F operator %(F x, F y)
    {
        return new F(
            x.m_rawValue == MIN_VALUE & y.m_rawValue == -1 ?
            0 :
            x.m_rawValue % y.m_rawValue);
    }

    /// <summary>
    /// Performs modulo as fast as possible; throws if x == MinValue and y == -1.
    /// Use the operator (%) for a more reliable but slower modulo.
    /// </summary>
    public static F FastMod(F x, F y)
    {
        return new F(x.m_rawValue % y.m_rawValue);
    }

    public static F operator -(F x)
    {
        return x.m_rawValue == MIN_VALUE ? MaxValue : new F(-x.m_rawValue);
    }

    public static bool operator ==(F x, F y)
    {
        return x.m_rawValue == y.m_rawValue;
    }

    public static bool operator !=(F x, F y)
    {
        return x.m_rawValue != y.m_rawValue;
    }

    public static bool operator >(F x, F y)
    {
        return x.m_rawValue > y.m_rawValue;
    }

    public static bool operator <(F x, F y)
    {
        return x.m_rawValue < y.m_rawValue;
    }

    public static bool operator >=(F x, F y)
    {
        return x.m_rawValue >= y.m_rawValue;
    }

    public static bool operator <=(F x, F y)
    {
        return x.m_rawValue <= y.m_rawValue;
    }


    /// <summary>
    /// Returns the square root of a specified number.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// The argument was negative.
    /// </exception>
    public static F Sqrt(F x)
    {
        var xl = x.m_rawValue;
        if (xl < 0)
        {
            // We cannot represent infinities like Single and Double, and Sqrt is
            // mathematically undefined for x < 0. So we just throw an exception.
            throw new ArgumentOutOfRangeException("Negative value passed to Sqrt", "x");
        }

        var num = (ulong)xl;
        var result = 0UL;

        // second-to-top bit
        var bit = 1UL << (NUM_BITS - 2);

        while (bit > num)
        {
            bit >>= 2;
        }

        // The main part is executed twice, in order to avoid
        // using 128 bit values in computations.
        for (var i = 0; i < 2; ++i)
        {
            // First we get the top 48 bits of the answer.
            while (bit != 0)
            {
                if (num >= result + bit)
                {
                    num -= result + bit;
                    result = (result >> 1) + bit;
                }
                else
                {
                    result = result >> 1;
                }
                bit >>= 2;
            }

            if (i == 0)
            {
                // Then process it again to get the lowest 16 bits.
                if (num > (1UL << (NUM_BITS / 2)) - 1)
                {
                    // The remainder 'num' is too large to be shifted left
                    // by 32, so we have to add 1 to result manually and
                    // adjust 'num' accordingly.
                    // num = a - (result + 0.5)^2
                    //       = num + result^2 - (result + 0.5)^2
                    //       = num - result - 0.5
                    num -= result;
                    num = (num << (NUM_BITS / 2)) - 0x80000000UL;
                    result = (result << (NUM_BITS / 2)) + 0x80000000UL;
                }
                else
                {
                    num <<= (NUM_BITS / 2);
                    result <<= (NUM_BITS / 2);
                }

                bit = 1UL << (NUM_BITS / 2 - 2);
            }
        }
        // Finally, if next bit would have been 1, round the result upwards.
        if (num > result)
        {
            ++result;
        }
        return new F((long)result);
    }

    /// <summary>
    /// Returns the Sine of x.
    /// This function has about 9 decimals of accuracy for small values of x.
    /// It may lose accuracy as the value of x grows.
    /// Performance: about 25% slower than Math.Sin() in x64, and 200% slower in x86.
    /// </summary>
    public static F Sin(F x)
    {
        bool flipHorizontal, flipVertical;
        var clampedL = ClampSinValue(x.m_rawValue, out flipHorizontal, out flipVertical);
        var clamped = new F(clampedL);

        // Find the two closest values in the LUT and perform linear interpolation
        // This is what kills the performance of this function on x86 - x64 is fine though
        var rawIndex = FastMul(clamped, LutInterval);
        var roundedIndex = Round(rawIndex);
        var indexError = FastSub(rawIndex, roundedIndex);

        var nearestValue = new F(SinLut[flipHorizontal ?
            SinLut.Length - 1 - (int)roundedIndex :
            (int)roundedIndex]);
        var secondNearestValue = new F(SinLut[flipHorizontal ?
            SinLut.Length - 1 - (int)roundedIndex - Sign(indexError) :
            (int)roundedIndex + Sign(indexError)]);

        var delta = FastMul(indexError, FastAbs(FastSub(nearestValue, secondNearestValue))).m_rawValue;
        var interpolatedValue = nearestValue.m_rawValue + (flipHorizontal ? -delta : delta);
        var finalValue = flipVertical ? -interpolatedValue : interpolatedValue;
        return new F(finalValue);
    }

    /// <summary>
    /// Returns a rough approximation of the Sine of x.
    /// This is at least 3 times faster than Sin() on x86 and slightly faster than Math.Sin(),
    /// however its accuracy is limited to 4-5 decimals, for small enough values of x.
    /// </summary>
    public static F FastSin(F x)
    {
        bool flipHorizontal, flipVertical;
        var clampedL = ClampSinValue(x.m_rawValue, out flipHorizontal, out flipVertical);

        // Here we use the fact that the SinLut table has a number of entries
        // equal to (PI_OVER_2 >> 15) to use the angle to index directly into it
        var rawIndex = (uint)(clampedL >> 15);
        if (rawIndex >= LUT_SIZE)
        {
            rawIndex = LUT_SIZE - 1;
        }
        var nearestValue = SinLut[flipHorizontal ?
            SinLut.Length - 1 - (int)rawIndex :
            (int)rawIndex];
        return new F(flipVertical ? -nearestValue : nearestValue);
    }



    // [MethodImplAttribute(MethodImplOptions.AggressiveInlining)] 
    static long ClampSinValue(long angle, out bool flipHorizontal, out bool flipVertical)
    {
        // Clamp value to 0 - 2*PI using modulo; this is very slow but there's no better way AFAIK
        var clamped2Pi = angle % PI_TIMES_2;
        if (angle < 0)
        {
            clamped2Pi += PI_TIMES_2;
        }

        // The LUT contains values for 0 - PiOver2; every other value must be obtained by
        // vertical or horizontal mirroring
        flipVertical = clamped2Pi >= PI;
        // obtain (angle % PI) from (angle % 2PI) - much faster than doing another modulo
        var clampedPi = clamped2Pi;
        while (clampedPi >= PI)
        {
            clampedPi -= PI;
        }
        flipHorizontal = clampedPi >= PI_OVER_2;
        // obtain (angle % PI_OVER_2) from (angle % PI) - much faster than doing another modulo
        var clampedPiOver2 = clampedPi;
        if (clampedPiOver2 >= PI_OVER_2)
        {
            clampedPiOver2 -= PI_OVER_2;
        }
        return clampedPiOver2;
    }

    public static long ClampASinValue(long val, out bool flipVertical)
    {
        flipVertical = false;
        if (val < 0)
        {
            val += ONE;
            flipVertical = true;
        }
        return val;
    }

    /// <summary>
    /// Returns the cosine of x.
    /// See Sin() for more details.
    /// </summary>
    public static F Cos(F x)
    {
        var xl = x.m_rawValue;
        var rawAngle = xl + (xl > 0 ? -PI - PI_OVER_2 : PI_OVER_2);
        return Sin(new F(rawAngle));
    }

    /// <summary>
    /// Returns a rough approximation of the cosine of x.
    /// See FastSin for more details.
    /// </summary>
    public static F FastCos(F x)
    {
        var xl = x.m_rawValue;
        var rawAngle = xl + (xl > 0 ? -PI - PI_OVER_2 : PI_OVER_2);
        return FastSin(new F(rawAngle));
    }

    /// <summary>
    /// Returns the tangent of x.
    /// </summary>
    /// <remarks>
    /// This function is not well-tested. It may be wildly inaccurate.
    /// </remarks>
    public static F Tan(F x)
    {
        var clampedPi = x.m_rawValue % PI;
        var flip = false;
        if (clampedPi < 0)
        {
            clampedPi = -clampedPi;
            flip = true;
        }
        if (clampedPi > PI_OVER_2)
        {
            flip = !flip;
            clampedPi = PI_OVER_2 - (clampedPi - PI_OVER_2);
        }

        var clamped = new F(clampedPi);

        // Find the two closest values in the LUT and perform linear interpolation
        var rawIndex = FastMul(clamped, LutInterval);
        var roundedIndex = Round(rawIndex);
        var indexError = FastSub(rawIndex, roundedIndex);

        var nearestValue = new F(TanLut[(int)roundedIndex]);
        var secondNearestValue = new F(TanLut[(int)roundedIndex + Sign(indexError)]);

        var delta = FastMul(indexError, FastAbs(FastSub(nearestValue, secondNearestValue))).m_rawValue;
        var interpolatedValue = nearestValue.m_rawValue + delta;
        var finalValue = flip ? -interpolatedValue : interpolatedValue;
        return new F(finalValue);
    }

    public static F Atan2(F y, F x)
    {
        var yl = y.m_rawValue;
        var xl = x.m_rawValue;
        if (xl == 0)
        {
            if (yl > 0)
            {
                return PiOver2;
            }
            if (yl == 0)
            {
                return Zero;
            }
            return -PiOver2;
        }
        F atan;
        var z = y / x;

        // Deal with overflow
        if (One + (F)0.28M * z * z == MaxValue)
        {
            return y < Zero ? -PiOver2 : PiOver2;
        }

        if (Abs(z) < One)
        {
            atan = z / (One + (F)0.28M * z * z);
            if (xl < 0)
            {
                if (yl < 0)
                {
                    return atan - Pi;
                }
                return atan + Pi;
            }
        }
        else
        {
            atan = PiOver2 - z / (z * z + (F)0.28M);
            if (yl < 0)
            {
                return atan - Pi;
            }
        }
        return atan;
    }

    public static F Clamp(F value, F min, F max)
    {
        if (value < min) value = min;
        if (value > max) value = max;
        return value;
    }

    public static F Clamp01(F value)
    {
        if (value < 0) value = 0;
        if (value > 1) value = 1;
        return value;
    }

    public static implicit operator F(long value)
    {
        return new F(value * ONE);
    }
    public static implicit operator long(F value)
    {
        return value.m_rawValue >> FRACTIONAL_PLACES;
    }
    public static implicit operator F(float value)
    {
        return new F((long)(value * ONE));
    }
    public static implicit operator float(F value)
    {
        return (float)value.m_rawValue / ONE;
    }
    public static implicit operator F(double value)
    {
        return new F((long)(value * ONE));
    }
    public static implicit operator double(F value)
    {
        return (double)value.m_rawValue / ONE;
    }
    public static implicit operator F(decimal value)
    {
        return new F((long)(value * ONE));
    }
    public static implicit operator decimal(F value)
    {
        return (decimal)value.m_rawValue / ONE;
    }

    public override bool Equals(object obj)
    {
        return obj is F && ((F)obj).m_rawValue == m_rawValue;
    }

    public override int GetHashCode()
    {
        return m_rawValue.GetHashCode();
    }

    public bool Equals(F other)
    {
        return m_rawValue == other.m_rawValue;
    }

    public int CompareTo(F other)
    {
        return m_rawValue.CompareTo(other.m_rawValue);
    }

    public override string ToString()
    {
        return ((decimal)this).ToString();
    }

    public string ToString(string provider)
    {
        return ((decimal)this).ToString(provider);
    }


    public static F FromRaw(long rawValue)
    {
        return new F(rawValue);
    }

    public static void GenerateASinLut()
    {
        using (var writer = new StreamWriter("Fix64ASinLut.cs"))
        {
            writer.Write(
@"partial struct F {
        public static readonly long[] ASinLut = new[] {");
            int lineCounter = 0;
            for (int i = 0; i < LUT_SIZE_ASIN; ++i)
            {
                var angle = i * 1.0 / (LUT_SIZE_ASIN - 1);
                if (lineCounter++ % 8 == 0)
                {
                    writer.WriteLine();
                    writer.Write("            ");
                }
                var sin = Math.Asin(angle);
                var rawValue = ((F)sin).m_rawValue;
                writer.Write(string.Format("0x{0:X}L, ", rawValue));
            }
            writer.Write(
@"
        };
    }");
        }
    }

    internal static void GenerateSinLut()
    {
        using (var writer = new StreamWriter("Fix64SinLut.cs"))
        {
            writer.Write(
@"namespace FixMath.NET {
    partial struct Fix64 {
        public static readonly long[] SinLut = new[] {");
            int lineCounter = 0;
            for (int i = 0; i < LUT_SIZE; ++i)
            {
                var angle = i * Math.PI * 0.5 / (LUT_SIZE - 1);
                if (lineCounter++ % 8 == 0)
                {
                    writer.WriteLine();
                    writer.Write("            ");
                }
                var sin = Math.Sin(angle);
                var rawValue = ((F)sin).m_rawValue;
                writer.Write(string.Format("0x{0:X}L, ", rawValue));
            }
            writer.Write(
@"
        };
    }
}");
        }
    }

    internal static void GenerateTanLut()
    {
        using (var writer = new StreamWriter("Fix64TanLut.cs"))
        {
            writer.Write(
@"namespace FixMath.NET {
    partial struct F {
        public static readonly long[] TanLut = new[] {");
            int lineCounter = 0;
            for (int i = 0; i < LUT_SIZE; ++i)
            {
                var angle = i * Math.PI * 0.5 / (LUT_SIZE - 1);
                if (lineCounter++ % 8 == 0)
                {
                    writer.WriteLine();
                    writer.Write("            ");
                }
                var tan = Math.Tan(angle);
                if (tan > (double)MaxValue || tan < 0.0)
                {
                    tan = (double)MaxValue;
                }
                var rawValue = (((decimal)tan > (decimal)MaxValue || tan < 0.0) ? MaxValue : (F)tan).m_rawValue;
                writer.Write(string.Format("0x{0:X}L, ", rawValue));
            }
            writer.Write(
@"
        };
    }
}");
        }
    }

    /// <summary>
    /// The underlying integer representation
    /// </summary>

    public long RawValue { get { return m_rawValue; } }

    /// <summary>
    /// This is the constructor from raw value; it can only be used interally.
    /// </summary>
    /// <param name="rawValue"></param>
    public F(long rawValue)
    {
        m_rawValue = rawValue;
    }

    public F(int value)
    {
        m_rawValue = value * ONE;
    }

    public F(int decimalPart, int fractionPart)
    {
        var fraction = (fractionPart * ONE) / 10000;
        m_rawValue = decimalPart * ONE + fraction;
    }

    public static int Min(int f1, int f2)
    {
        if (f1 <= f2) return f1;
        return f2;
    }

    public static int Max(int f1, int f2)
    {
        if (f1 >= f2) return f1;
        return f2;
    }

    public static F Min(F f1, F f2)
    {
        if (f1 <= f2) return f1;
        return f2;
    }

    public static F Max(F f1, F f2)
    {
        if (f1 >= f2) return f1;
        return f2;
    }

    public static F Pow(F f, uint pow)
    {
        F result = f;
        if (pow == 0) return 1;
        for (int i = 1; i < pow; i++)
        {
            result *= f;
        }
        return result;
    }

    public static bool IsAngleBetween(F mid, F start, F end)
    {
        end = (end - start) < 0 ? end - start + PiTimes2 : end - start;
        mid = (mid - start) < 0 ? mid - start + PiTimes2 : mid - start;
        return (mid < end);
    }

    public static F Parse(string value)
    {
        const char sep = '.';
        var split = value.Split(sep);

        var decimalPart = int.Parse(split[0]) * ONE;
        var fractionPart = 0L;
        if (split.Length == 2)
        {
            var fp = split[1];
            var fractionDivider = Pow(10, fp.Length);
            fractionPart = (int.Parse(fp) * ONE) / fractionDivider;
        }

        return new F(decimalPart + fractionPart);
    }

    private static int Pow(int b, int p)
    {
        var res = 1;
        for (int i = 0; i < p; i++)
        {
            res *= b;
        }
        return res;
    }
}