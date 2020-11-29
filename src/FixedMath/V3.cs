using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;

public struct V3
{
    /// <summary>
    /// The X component of the vector.
    /// </summary>
    public F X;
    /// <summary>
    /// The Y component of the vector.
    /// </summary>
    public F Y;
    /// <summary>
    /// The Z component of the vector.
    /// </summary>
    public F Z;

    #region Constructors
    /// <summary>
    /// Constructs a vector whose elements are all the single specified value.
    /// </summary>
    /// <param name="value">The element to fill the vector with.</param>
    ///
    public V3(F value) : this(value, value, value) { }

    /// <summary>
    /// Constructs a V3 from the given V2 and a third value.
    /// </summary>
    /// <param name="value">The Vector to extract X and Y components from.</param>
    /// <param name="z">The Z component.</param>
    public V3(V2 value, F z) : this(value.X, value.Y, z) { }

    /// <summary>
    /// Constructs a vector with the given individual elements.
    /// </summary>
    /// <param name="x">The X component.</param>
    /// <param name="y">The Y component.</param>
    /// <param name="z">The Z component.</param>
    ///
    public V3(F x, F y, F z)
    {
        X = x;
        Y = y;
        Z = z;
    }
    #endregion Constructors

    #region Public Instance Methods
    /// <summary>
    /// Copies the contents of the vector into the given array.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CopyTo(F[] array)
    {
        CopyTo(array, 0);
    }

    /// <summary>
    /// Copies the contents of the vector into the given array, starting from index.
    /// </summary>
    /// <exception cref="ArgumentNullException">If array is null.</exception>
    /// <exception cref="RankException">If array is multidimensional.</exception>
    /// <exception cref="ArgumentOutOfRangeException">If index is greater than end of the array or index is less than zero.</exception>
    /// <exception cref="ArgumentException">If number of elements in source vector is greater than those available in destination array.</exception>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CopyTo(F[] array, int index)
    {
        array[index] = X;
        array[index + 1] = Y;
        array[index + 2] = Z;
    }

    /// <summary>
    /// Returns a boolean indicating whether the given V3 is equal to this V3 instance.
    /// </summary>
    /// <param name="other">The V3 to compare this instance to.</param>
    /// <returns>True if the other V3 is equal to this instance; False otherwise.</returns>
    ///
    public bool Equals(V3 other)
    {
        return X == other.X &&
               Y == other.Y &&
               Z == other.Z;
    }
    #endregion Public Instance Methods

    #region Public Static Methods
    /// <summary>
    /// Returns the dot product of two vectors.
    /// </summary>
    /// <param name="vector1">The first vector.</param>
    /// <param name="vector2">The second vector.</param>
    /// <returns>The dot product.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F Dot(V3 vector1, V3 vector2)
    {
        return vector1.X * vector2.X +
               vector1.Y * vector2.Y +
               vector1.Z * vector2.Z;
    }

    /// <summary>
    /// Returns a vector whose elements are the minimum of each of the pairs of elements in the two source vectors.
    /// </summary>
    /// <param name="value1">The first source vector.</param>
    /// <param name="value2">The second source vector.</param>
    /// <returns>The minimized vector.</returns>
    ///
    public static V3 Min(V3 value1, V3 value2)
    {
        return new V3(
            (value1.X < value2.X) ? value1.X : value2.X,
            (value1.Y < value2.Y) ? value1.Y : value2.Y,
            (value1.Z < value2.Z) ? value1.Z : value2.Z);
    }

    /// <summary>
    /// Returns a vector whose elements are the maximum of each of the pairs of elements in the two source vectors.
    /// </summary>
    /// <param name="value1">The first source vector.</param>
    /// <param name="value2">The second source vector.</param>
    /// <returns>The maximized vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Max(V3 value1, V3 value2)
    {
        return new V3(
            (value1.X > value2.X) ? value1.X : value2.X,
            (value1.Y > value2.Y) ? value1.Y : value2.Y,
            (value1.Z > value2.Z) ? value1.Z : value2.Z);
    }

    /// <summary>
    /// Returns a vector whose elements are the absolute values of each of the source vector's elements.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The absolute value vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Abs(V3 value)
    {
        return new V3(F.Abs(value.X), F.Abs(value.Y), F.Abs(value.Z));
    }

    /// <summary>
    /// Returns a vector whose elements are the square root of each of the source vector's elements.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The square root vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 SquareRoot(V3 value)
    {
        return new V3(F.Sqrt(value.X), F.Sqrt(value.Y), F.Sqrt(value.Z));
    }
    #endregion Public Static Methods

    #region Public Static Operators
    /// <summary>
    /// Adds two vectors together.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The summed vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator +(V3 left, V3 right)
    {
        return new V3(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
    }

    /// <summary>
    /// Subtracts the second vector from the first.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The difference vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator -(V3 left, V3 right)
    {
        return new V3(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
    }

    /// <summary>
    /// Multiplies two vectors together.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The product vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator *(V3 left, V3 right)
    {
        return new V3(left.X * right.X, left.Y * right.Y, left.Z * right.Z);
    }

    /// <summary>
    /// Multiplies a vector by the given scalar.
    /// </summary>
    /// <param name="left">The source vector.</param>
    /// <param name="right">The scalar value.</param>
    /// <returns>The scaled vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator *(V3 left, F right)
    {
        return left * new V3(right);
    }

    /// <summary>
    /// Multiplies a vector by the given scalar.
    /// </summary>
    /// <param name="left">The scalar value.</param>
    /// <param name="right">The source vector.</param>
    /// <returns>The scaled vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator *(F left, V3 right)
    {
        return new V3(left) * right;
    }

    /// <summary>
    /// Divides the first vector by the second.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The vector resulting from the division.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator /(V3 left, V3 right)
    {
        return new V3(left.X / right.X, left.Y / right.Y, left.Z / right.Z);
    }

    /// <summary>
    /// Divides the vector by the given scalar.
    /// </summary>
    /// <param name="value1">The source vector.</param>
    /// <param name="value2">The scalar value.</param>
    /// <returns>The result of the division.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator /(V3 value1, F value2)
    {
        F invDiv = F.One / value2;

        return new V3(
            value1.X * invDiv,
            value1.Y * invDiv,
            value1.Z * invDiv);
    }

    /// <summary>
    /// Negates a given vector.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The negated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 operator -(V3 value)
    {
        return Zero - value;
    }

    /// <summary>
    /// Returns a boolean indicating whether the two given vectors are equal.
    /// </summary>
    /// <param name="left">The first vector to compare.</param>
    /// <param name="right">The second vector to compare.</param>
    /// <returns>True if the vectors are equal; False otherwise.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(V3 left, V3 right)
    {
        return (left.X == right.X &&
                left.Y == right.Y &&
                left.Z == right.Z);
    }

    /// <summary>
    /// Returns a boolean indicating whether the two given vectors are not equal.
    /// </summary>
    /// <param name="left">The first vector to compare.</param>
    /// <param name="right">The second vector to compare.</param>
    /// <returns>True if the vectors are not equal; False if they are equal.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(V3 left, V3 right)
    {
        return (left.X != right.X ||
                left.Y != right.Y ||
                left.Z != right.Z);
    }
    #endregion Public Static Operators
    #region Public Static Properties
    /// <summary>
    /// Returns the vector (0,0,0).
    /// </summary>
    public static V3 Zero { get { return new V3(); } }
    /// <summary>
    /// Returns the vector (1,1,1).
    /// </summary>
    public static V3 One { get { return new V3(F.One, F.One, F.One); } }
    /// <summary>
    /// Returns the vector (1,0,0).
    /// </summary>
    public static V3 UnitX { get { return new V3(F.One, F.Zero, F.Zero); } }
    /// <summary>
    /// Returns the vector (0,1,0).
    /// </summary>
    public static V3 UnitY { get { return new V3(F.Zero, F.One, F.Zero); } }
    /// <summary>
    /// Returns the vector (0,0,1).
    /// </summary>
    public static V3 UnitZ { get { return new V3(F.Zero, F.Zero, F.One); } }
    #endregion Public Static Properties

    #region Public Instance Methods

    /// <summary>
    /// Returns the hash code for this instance.
    /// </summary>
    /// <returns>The hash code.</returns>
    public override int GetHashCode()
    {
        int hash = this.X.GetHashCode();
        return hash;
    }

    /// <summary>
    /// Returns a boolean indicating whether the given Object is equal to this V3 instance.
    /// </summary>
    /// <param name="obj">The Object to compare against.</param>
    /// <returns>True if the Object is equal to this V3; False otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object obj)
    {
        if (!(obj is V3))
            return false;
        return Equals((V3)obj);
    }

    /// <summary>
    /// Returns a String representing this V3 instance.
    /// </summary>
    /// <returns>The string representation.</returns>
    public override string ToString()
    {
        return ToString("G", CultureInfo.CurrentCulture);
    }

    /// <summary>
    /// Returns a String representing this V3 instance, using the specified format to format individual elements.
    /// </summary>
    /// <param name="format">The format of individual elements.</param>
    /// <returns>The string representation.</returns>
    public string ToString(string format)
    {
        return ToString(format, CultureInfo.CurrentCulture);
    }

    /// <summary>
    /// Returns a String representing this V3 instance, using the specified format to format individual elements 
    /// and the given IFormatProvider.
    /// </summary>
    /// <param name="format">The format of individual elements.</param>
    /// <param name="formatProvider">The format provider to use when formatting elements.</param>
    /// <returns>The string representation.</returns>
    public string ToString(string format, IFormatProvider formatProvider)
    {
        StringBuilder sb = new StringBuilder();
        string separator = NumberFormatInfo.GetInstance(formatProvider).NumberGroupSeparator;
        sb.Append('<');
        sb.Append((this.X).ToString());
        sb.Append(separator);
        sb.Append(' ');
        sb.Append((this.Y).ToString());
        sb.Append(separator);
        sb.Append(' ');
        sb.Append((this.Z).ToString());
        sb.Append('>');
        return sb.ToString();
    }

    /// <summary>
    /// Returns the length of the vector.
    /// </summary>
    /// <returns>The vector's length.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public F Length()
    {
        F ls = X * X + Y * Y + Z * Z;
        return F.Sqrt(ls);
    }

    /// <summary>
    /// Returns the length of the vector squared. This operation is cheaper than Length().
    /// </summary>
    /// <returns>The vector's length squared.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public F LengthSquared()
    {
        return X * X + Y * Y + Z * Z;
    }
    #endregion Public Instance Methods

    #region Public Static Methods
    /// <summary>
    /// Returns the Euclidean distance between the two given points.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <returns>The distance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F Distance(V3 value1, V3 value2)
    {
        F dx = value1.X - value2.X;
        F dy = value1.Y - value2.Y;
        F dz = value1.Z - value2.Z;

        F ls = dx * dx + dy * dy + dz * dz;

        return F.Sqrt(ls);
    }

    /// <summary>
    /// Returns the Euclidean distance squared between the two given points.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <returns>The distance squared.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F DistanceSquared(V3 value1, V3 value2)
    {
        F dx = value1.X - value2.X;
        F dy = value1.Y - value2.Y;
        F dz = value1.Z - value2.Z;

        return dx * dx + dy * dy + dz * dz;
    }

    /// <summary>
    /// Returns a vector with the same direction as the given vector, but with a length of 1.
    /// </summary>
    /// <param name="value">The vector to normalize.</param>
    /// <returns>The normalized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Normalize(V3 value)
    {
        F ls = value.X * value.X + value.Y * value.Y + value.Z * value.Z;
        F length = F.Sqrt(ls);
        return new V3(value.X / length, value.Y / length, value.Z / length);
    }

    /// <summary>
    /// Computes the cross product of two vectors.
    /// </summary>
    /// <param name="vector1">The first vector.</param>
    /// <param name="vector2">The second vector.</param>
    /// <returns>The cross product.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Cross(V3 vector1, V3 vector2)
    {
        return new V3(
            vector1.Y * vector2.Z - vector1.Z * vector2.Y,
            vector1.Z * vector2.X - vector1.X * vector2.Z,
            vector1.X * vector2.Y - vector1.Y * vector2.X);
    }

    /// <summary>
    /// Returns the reflection of a vector off a surface that has the specified normal.
    /// </summary>
    /// <param name="vector">The source vector.</param>
    /// <param name="normal">The normal of the surface being reflected off.</param>
    /// <returns>The reflected vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Reflect(V3 vector, V3 normal)
    {
        F dot = vector.X * normal.X + vector.Y * normal.Y + vector.Z * normal.Z;
        F tempX = normal.X * dot * F.Two;
        F tempY = normal.Y * dot * F.Two;
        F tempZ = normal.Z * dot * F.Two;
        return new V3(vector.X - tempX, vector.Y - tempY, vector.Z - tempZ);
    }

    /// <summary>
    /// Restricts a vector between a min and max value.
    /// </summary>
    /// <param name="value1">The source vector.</param>
    /// <param name="min">The minimum value.</param>
    /// <param name="max">The maximum value.</param>
    /// <returns>The restricted vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Clamp(V3 value1, V3 min, V3 max)
    {
        // This compare order is very important!!!
        // We must follow HLSL behavior in the case user specified min value is bigger than max value.

        F x = value1.X;
        x = (x > max.X) ? max.X : x;
        x = (x < min.X) ? min.X : x;

        F y = value1.Y;
        y = (y > max.Y) ? max.Y : y;
        y = (y < min.Y) ? min.Y : y;

        F z = value1.Z;
        z = (z > max.Z) ? max.Z : z;
        z = (z < min.Z) ? min.Z : z;

        return new V3(x, y, z);
    }

    /// <summary>
    /// Linearly interpolates between two vectors based on the given weighting.
    /// </summary>
    /// <param name="value1">The first source vector.</param>
    /// <param name="value2">The second source vector.</param>
    /// <param name="amount">Value between 0 and 1 indicating the weight of the second source vector.</param>
    /// <returns>The interpolated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Lerp(V3 value1, V3 value2, F amount)
    {
        return new V3(
            value1.X + (value2.X - value1.X) * amount,
            value1.Y + (value2.Y - value1.Y) * amount,
            value1.Z + (value2.Z - value1.Z) * amount);
    }
    #endregion Public Static Methods

    #region Public operator methods

    // All these methods should be inlined as they are implemented
    // over JIT intrinsics

    /// <summary>
    /// Adds two vectors together.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The summed vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Add(V3 left, V3 right)
    {
        return left + right;
    }

    /// <summary>
    /// Subtracts the second vector from the first.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The difference vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Subtract(V3 left, V3 right)
    {
        return left - right;
    }

    /// <summary>
    /// Multiplies two vectors together.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The product vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Multiply(V3 left, V3 right)
    {
        return left * right;
    }

    /// <summary>
    /// Multiplies a vector by the given scalar.
    /// </summary>
    /// <param name="left">The source vector.</param>
    /// <param name="right">The scalar value.</param>
    /// <returns>The scaled vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Multiply(V3 left, F right)
    {
        return left * right;
    }

    /// <summary>
    /// Multiplies a vector by the given scalar.
    /// </summary>
    /// <param name="left">The scalar value.</param>
    /// <param name="right">The source vector.</param>
    /// <returns>The scaled vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Multiply(F left, V3 right)
    {
        return left * right;
    }

    /// <summary>
    /// Divides the first vector by the second.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The vector resulting from the division.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Divide(V3 left, V3 right)
    {
        return left / right;
    }

    /// <summary>
    /// Divides the vector by the given scalar.
    /// </summary>
    /// <param name="left">The source vector.</param>
    /// <param name="divisor">The scalar value.</param>
    /// <returns>The result of the division.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Divide(V3 left, F divisor)
    {
        return left / divisor;
    }

    /// <summary>
    /// Negates a given vector.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The negated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V3 Negate(V3 value)
    {
        return -value;
    }
    #endregion Public operator methods
}
