using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;

public struct V2
{
    /// <summary>
    /// The X component of the vector.
    /// </summary>
    public F X;
    /// <summary>
    /// The Y component of the vector.
    /// </summary>
    public F Y;

    #region Constructors
    /// <summary>
    /// Constructs a vector whose elements are all the single specified value.
    /// </summary>
    /// <param name="value">The element to fill the vector with.</param>
    ///
    public V2(F value) : this(value, value) { }

    /// <summary>
    /// Constructs a vector with the given individual elements.
    /// </summary>
    /// <param name="x">The X component.</param>
    /// <param name="y">The Y component.</param>
    ///
    public V2(F x, F y)
    {
        X = x;
        Y = y;
    }
    #endregion Constructors

    #region Public Instance Methods
    /// <summary>
    /// Copies the contents of the vector into the given array.
    /// </summary>
    /// <param name="array">The destination array.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void CopyTo(F[] array)
    {
        CopyTo(array, 0);
    }

    /// <summary>
    /// Copies the contents of the vector into the given array, starting from the given index.
    /// </summary>
    /// <exception cref="ArgumentNullException">If array is null.</exception>
    /// <exception cref="RankException">If array is multidimensional.</exception>
    /// <exception cref="ArgumentOutOfRangeException">If index is greater than end of the array or index is less than zero.</exception>
    /// <exception cref="ArgumentException">If number of elements in source vector is greater than those available in destination array
    /// or if there are not enough elements to copy.</exception>
    public void CopyTo(F[] array, int index)
    {
        array[index] = X;
        array[index + 1] = Y;
    }

    /// <summary>
    /// Returns a boolean indicating whether the given V2 is equal to this V2 instance.
    /// </summary>
    /// <param name="other">The V2 to compare this instance to.</param>
    /// <returns>True if the other V2 is equal to this instance; False otherwise.</returns>
    ///
    public bool Equals(V2 other)
    {
        return this.X == other.X && this.Y == other.Y;
    }
    #endregion Public Instance Methods

    #region Public Static Methods
    /// <summary>
    /// Returns the dot product of two vectors.
    /// </summary>
    /// <param name="value1">The first vector.</param>
    /// <param name="value2">The second vector.</param>
    /// <returns>The dot product.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F Dot(V2 value1, V2 value2)
    {
        return value1.X * value2.X +
               value1.Y * value2.Y;
    }

    /// <summary>
    /// Returns a vector whose elements are the minimum of each of the pairs of elements in the two source vectors.
    /// </summary>
    /// <param name="value1">The first source vector.</param>
    /// <param name="value2">The second source vector.</param>
    /// <returns>The minimized vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Min(V2 value1, V2 value2)
    {
        return new V2(
            (value1.X < value2.X) ? value1.X : value2.X,
            (value1.Y < value2.Y) ? value1.Y : value2.Y);
    }

    /// <summary>
    /// Returns a vector whose elements are the maximum of each of the pairs of elements in the two source vectors
    /// </summary>
    /// <param name="value1">The first source vector</param>
    /// <param name="value2">The second source vector</param>
    /// <returns>The maximized vector</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Max(V2 value1, V2 value2)
    {
        return new V2(
            (value1.X > value2.X) ? value1.X : value2.X,
            (value1.Y > value2.Y) ? value1.Y : value2.Y);
    }

    /// <summary>
    /// Returns a vector whose elements are the absolute values of each of the source vector's elements.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The absolute value vector.</returns>        
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Abs(V2 value)
    {
        return new V2(F.Abs(value.X), F.Abs(value.Y));
    }

    /// <summary>
    /// Returns a vector whose elements are the square root of each of the source vector's elements.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The square root vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 SquareRoot(V2 value)
    {
        return new V2(F.Sqrt(value.X), F.Sqrt(value.Y));
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
    public static V2 operator +(V2 left, V2 right)
    {
        return new V2(left.X + right.X, left.Y + right.Y);
    }

    /// <summary>
    /// Subtracts the second vector from the first.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The difference vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator -(V2 left, V2 right)
    {
        return new V2(left.X - right.X, left.Y - right.Y);
    }

    /// <summary>
    /// Multiplies two vectors together.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The product vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator *(V2 left, V2 right)
    {
        return new V2(left.X * right.X, left.Y * right.Y);
    }

    /// <summary>
    /// Multiplies a vector by the given scalar.
    /// </summary>
    /// <param name="left">The scalar value.</param>
    /// <param name="right">The source vector.</param>
    /// <returns>The scaled vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator *(F left, V2 right)
    {
        return new V2(left, left) * right;
    }

    /// <summary>
    /// Multiplies a vector by the given scalar.
    /// </summary>
    /// <param name="left">The source vector.</param>
    /// <param name="right">The scalar value.</param>
    /// <returns>The scaled vector.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator *(V2 left, F right)
    {
        return left * new V2(right, right);
    }

    /// <summary>
    /// Divides the first vector by the second.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The vector resulting from the division.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator /(V2 left, V2 right)
    {
        return new V2(left.X / right.X, left.Y / right.Y);
    }

    /// <summary>
    /// Divides the vector by the given scalar.
    /// </summary>
    /// <param name="value1">The source vector.</param>
    /// <param name="value2">The scalar value.</param>
    /// <returns>The result of the division.</returns>
    ///
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator /(V2 value1, F value2)
    {
        F invDiv = F.One / value2;
        return new V2(
            value1.X * invDiv,
            value1.Y * invDiv);
    }

    /// <summary>
    /// Negates a given vector.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The negated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 operator -(V2 value)
    {
        return Zero - value;
    }

    /// <summary>
    /// Returns a boolean indicating whether the two given vectors are equal.
    /// </summary>
    /// <param name="left">The first vector to compare.</param>
    /// <param name="right">The second vector to compare.</param>
    /// <returns>True if the vectors are equal; False otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(V2 left, V2 right)
    {
        return left.Equals(right);
    }

    /// <summary>
    /// Returns a boolean indicating whether the two given vectors are not equal.
    /// </summary>
    /// <param name="left">The first vector to compare.</param>
    /// <param name="right">The second vector to compare.</param>
    /// <returns>True if the vectors are not equal; False if they are equal.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(V2 left, V2 right)
    {
        return !(left == right);
    }
    #endregion Public Static Operators

    #region Public Static Properties
    /// <summary>
    /// Returns the vector (0,0).
    /// </summary>
    public static V2 Zero { get { return new V2(); } }
    /// <summary>
    /// Returns the vector (1,1).
    /// </summary>
    public static V2 One { get { return new V2(F.One, F.One); } }
    /// <summary>
    /// Returns the vector (1,0).
    /// </summary>
    public static V2 UnitX { get { return new V2(F.One, F.Zero); } }
    /// <summary>
    /// Returns the vector (0,1).
    /// </summary>
    public static V2 UnitY { get { return new V2(F.Zero, F.One); } }
    #endregion Public Static Properties

    #region Public instance methods
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
    /// Returns a boolean indicating whether the given Object is equal to this V2 instance.
    /// </summary>
    /// <param name="obj">The Object to compare against.</param>
    /// <returns>True if the Object is equal to this V2; False otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object obj)
    {
        if (!(obj is V2))
            return false;
        return Equals((V2)obj);
    }

    /// <summary>
    /// Returns a String representing this V2 instance.
    /// </summary>
    /// <returns>The string representation.</returns>
    public override string ToString()
    {
        return ToString("G", CultureInfo.CurrentCulture);
    }

    /// <summary>
    /// Returns a String representing this V2 instance, using the specified format to format individual elements.
    /// </summary>
    /// <param name="format">The format of individual elements.</param>
    /// <returns>The string representation.</returns>
    public string ToString(string format)
    {
        return ToString(format, CultureInfo.CurrentCulture);
    }

    /// <summary>
    /// Returns a String representing this V2 instance, using the specified format to format individual elements 
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
        sb.Append(this.X.ToString());
        sb.Append(separator);
        sb.Append(' ');
        sb.Append(this.Y.ToString());
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
        F ls = X * X + Y * Y;
        return F.Sqrt(ls);
    }

    /// <summary>
    /// Returns the length of the vector squared. This operation is cheaper than Length().
    /// </summary>
    /// <returns>The vector's length squared.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public F LengthSquared()
    {
        return X * X + Y * Y;
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
    public static F Distance(V2 value1, V2 value2)
    {
        F dx = value1.X - value2.X;
        F dy = value1.Y - value2.Y;

        F ls = dx * dx + dy * dy;

        return F.Sqrt(ls);
    }

    /// <summary>
    /// Returns the Euclidean distance squared between the two given points.
    /// </summary>
    /// <param name="value1">The first point.</param>
    /// <param name="value2">The second point.</param>
    /// <returns>The distance squared.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F DistanceSquared(V2 value1, V2 value2)
    {
        F dx = value1.X - value2.X;
        F dy = value1.Y - value2.Y;

        return dx * dx + dy * dy;
    }

    /// <summary>
    /// Returns a vector with the same direction as the given vector, but with a length of 1.
    /// </summary>
    /// <param name="value">The vector to normalize.</param>
    /// <returns>The normalized vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Normalize(V2 value)
    {
        F ls = value.X * value.X + value.Y * value.Y;
        F invNorm = F.One / F.Sqrt(ls);

        return new V2(
            value.X * invNorm,
            value.Y * invNorm);
    }

    /// <summary>
    /// Returns the reflection of a vector off a surface that has the specified normal.
    /// </summary>
    /// <param name="vector">The source vector.</param>
    /// <param name="normal">The normal of the surface being reflected off.</param>
    /// <returns>The reflected vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Reflect(V2 vector, V2 normal)
    {
        F dot = vector.X * normal.X + vector.Y * normal.Y;

        return new V2(
            vector.X - F.Two * dot * normal.X,
            vector.Y - F.Two * dot * normal.Y);
    }

    /// <summary>
    /// Restricts a vector between a min and max value.
    /// </summary>
    /// <param name="value1">The source vector.</param>
    /// <param name="min">The minimum value.</param>
    /// <param name="max">The maximum value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Clamp(V2 value1, V2 min, V2 max)
    {
        // This compare order is very important!!!
        // We must follow HLSL behavior in the case user specified min value is bigger than max value.
        F x = value1.X;
        x = (x > max.X) ? max.X : x;
        x = (x < min.X) ? min.X : x;

        F y = value1.Y;
        y = (y > max.Y) ? max.Y : y;
        y = (y < min.Y) ? min.Y : y;

        return new V2(x, y);
    }

    /// <summary>
    /// Linearly interpolates between two vectors based on the given weighting.
    /// </summary>
    /// <param name="value1">The first source vector.</param>
    /// <param name="value2">The second source vector.</param>
    /// <param name="amount">Value between 0 and 1 indicating the weight of the second source vector.</param>
    /// <returns>The interpolated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Lerp(V2 value1, V2 value2, F amount)
    {
        return new V2(
            value1.X + (value2.X - value1.X) * amount,
            value1.Y + (value2.Y - value1.Y) * amount);
    }
    #endregion Public Static Methods

    #region Public operator methods
    // all the below methods should be inlined as they are 
    // implemented over JIT intrinsics

    /// <summary>
    /// Adds two vectors together.
    /// </summary>
    /// <param name="left">The first source vector.</param>
    /// <param name="right">The second source vector.</param>
    /// <returns>The summed vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Add(V2 left, V2 right)
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
    public static V2 Subtract(V2 left, V2 right)
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
    public static V2 Multiply(V2 left, V2 right)
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
    public static V2 Multiply(V2 left, F right)
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
    public static V2 Multiply(F left, V2 right)
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
    public static V2 Divide(V2 left, V2 right)
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
    public static V2 Divide(V2 left, F divisor)
    {
        return left / divisor;
    }

    /// <summary>
    /// Negates a given vector.
    /// </summary>
    /// <param name="value">The source vector.</param>
    /// <returns>The negated vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static V2 Negate(V2 value)
    {
        return -value;
    }
    #endregion Public operator methods
}
