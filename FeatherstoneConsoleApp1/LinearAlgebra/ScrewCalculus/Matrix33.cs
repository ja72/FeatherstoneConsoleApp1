using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;


namespace JA.LinearAlgebra.ScrewCalculus
{
    using Vector3 = JA.LinearAlgebra.VectorCalculus.Vector3;
    using Matrix3 = JA.LinearAlgebra.VectorCalculus.Matrix3;

    /// <summary>
    /// Immutable 2x2 block matrix where each block is a 3x3 Matrix3.
    /// Logical layout:
    /// [ A11 A12 ]
    /// [ A21 A22 ]
    /// Each Aij is a 3x3 Matrix3, so the full matrix is 6x6.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Matrix33 : IEquatable<Matrix33>
    {
        internal const int Size = 36;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly Matrix3 a11;
        internal readonly Matrix3 a12;
        internal readonly Matrix3 a21;
        internal readonly Matrix3 a22;

        #region Factory
        public Matrix33(Matrix3 a11, Matrix3 a12, Matrix3 a21, Matrix3 a22)
        {
            this.a11=a11;
            this.a12=a12;
            this.a21=a21;
            this.a22=a22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 FromBlocks(Matrix3 a11, Matrix3 a12, Matrix3 a21, Matrix3 a22)
            => new Matrix33(a11, a12, a21, a22);
        public static Matrix33 Diagonal(Matrix3 a11, Matrix3 a22)
            => new Matrix33(a11, Matrix3.Zero, Matrix3.Zero, a22);
        public static Matrix33 Scalar(double scalar) => Diagonal( Matrix3.Scalar(scalar), Matrix3.Scalar(scalar));
        public static Matrix33 Zero { get; } = new Matrix33(Matrix3.Zero, Matrix3.Zero, Matrix3.Zero, Matrix3.Zero);

        public static Matrix33 Identity { get; } = new Matrix33(Matrix3.Identity, Matrix3.Zero, Matrix3.Zero, Matrix3.Identity);

        public static implicit operator Matrix33(double scalar) => Scalar(scalar);
        #endregion

        #region Properties
        public Matrix3 A11 => a11;
        public Matrix3 A12 => a12;
        public Matrix3 A21 => a21;
        public Matrix3 A22 => a22;

        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Negate(Matrix33 a) =>
            new Matrix33(
                Matrix3.Negate(a.a11),
                Matrix3.Negate(a.a12),
                Matrix3.Negate(a.a21),
                Matrix3.Negate(a.a22)
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Add(Matrix33 a, Matrix33 b) =>
            new Matrix33(
                Matrix3.Add(a.a11, b.a11),
                Matrix3.Add(a.a12, b.a12),
                Matrix3.Add(a.a21, b.a21),
                Matrix3.Add(a.a22, b.a22)
            );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Subtract(Matrix33 a, Matrix33 b) =>
            new Matrix33(
                Matrix3.Subtract(a.a11, b.a11),
                Matrix3.Subtract(a.a12, b.a12),
                Matrix3.Subtract(a.a21, b.a21),
                Matrix3.Subtract(a.a22, b.a22)
            );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Scale(Matrix33 a, double s) 
            => new Matrix33(
                Matrix3.Scale(a.a11, s),
                Matrix3.Scale(a.a12, s),
                Matrix3.Scale(a.a21, s),
                Matrix3.Scale(a.a22, s)
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Divide(Matrix33 a, double d) 
            => new Matrix33(
                Matrix3.Divide(a.a11, d),
                Matrix3.Divide(a.a12, d),
                Matrix3.Divide(a.a21, d),
                Matrix3.Divide(a.a22, d)
            );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Product(Matrix33 a, Vector33 b)
        {
            // Block multiplication:
            // [ a11 a12 ] * [ v1 ] = [ a11*v1 + a12*v2 ]
            // [ a21 a22 ]   [ v2 ]   [ a21*v1 + a22*v2 ]
            return new Vector33(
                Vector3.Add(Matrix3.Product(a.a11, b.linear), Matrix3.Product(a.a12, b.angular)),
                Vector3.Add(Matrix3.Product(a.a21, b.linear), Matrix3.Product(a.a22, b.angular))
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Product(Vector33 b, Matrix33 a)
        {
            // Block multiplication:
            //[ v1  v2 ] * [ a11 a12 ]  = [ a11*v1 + a21*v2  a12*v1 + a22*v2 ]
            //             [ a21 a22 ]     
            return new Vector33(
                Vector3.Add(Matrix3.Product(a.a11, b.linear), Matrix3.Product(a.a22, b.angular)),
                Vector3.Add(Matrix3.Product(a.a12, b.linear), Matrix3.Product(a.a22, b.angular))
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Product(Matrix33 a, Matrix33 b)
        {
            // Block multiplication:
            // [ a11 a12 ] * [ b11 b12 ] = [ a11*b11 + a12*b21, a11*b12 + a12*b22 ]
            // [ a21 a22 ]   [ b21 b22 ]   [ a21*b11 + a22*b21, a21*b12 + a22*b22 ]
            return new Matrix33(
                Matrix3.Add(Matrix3.Product(a.a11, b.a11), Matrix3.Product(a.a12, b.a21)),
                Matrix3.Add(Matrix3.Product(a.a11, b.a12), Matrix3.Product(a.a12, b.a22)),
                Matrix3.Add(Matrix3.Product(a.a21, b.a11), Matrix3.Product(a.a22, b.a21)),
                Matrix3.Add(Matrix3.Product(a.a21, b.a12), Matrix3.Product(a.a22, b.a22))
            );
        }
        #endregion

        #region Operators
        public static Matrix33 operator -(Matrix33 a) => Negate(a);
        public static Matrix33 operator +(Matrix33 a, Matrix33 b) => Add(a, b);
        public static Matrix33 operator -(Matrix33 a, Matrix33 b) => Subtract(a, b);
        public static Vector33 operator *(Matrix33 a, Vector33 b) => Product(a, b);
        public static Vector33 operator *(Vector33 a, Matrix33 b) => Product(a, b);
        public static Matrix33 operator *(Matrix33 a, Matrix33 b) => Product(a, b);
        public static Matrix33 operator *(Matrix33 a, double s) => Scale(a, s);
        public static Matrix33 operator *(double s, Matrix33 a) => Scale(a, s);
        public static Matrix33 operator /(Matrix33 a, double d) => Divide(a, d);

        #endregion

        #region Equality / Hash
        public bool Equals(Matrix33 other) =>
            a11.Equals(other.a11) && a12.Equals(other.a12) && a21.Equals(other.a21) && a22.Equals(other.a22);

        public override bool Equals(object obj) => obj is Matrix33 other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1521134295;
                hc = hc * -1521134295 + a11.GetHashCode();
                hc = hc * -1521134295 + a12.GetHashCode();
                hc = hc * -1521134295 + a21.GetHashCode();
                hc = hc * -1521134295 + a22.GetHashCode();
                return hc;
            }
        }

        public static bool operator ==(Matrix33 left, Matrix33 right) => left.Equals(right);
        public static bool operator !=(Matrix33 left, Matrix33 right) => !left.Equals(right);
        #endregion

        #region Formatting
        public override string ToString() =>
            $"[[{a11} {a12}]; [{a21} {a22}]]";
        #endregion

        #region  Collection
        public static implicit operator double[](Matrix33 @this) => @this.ToArray();

        /// <summary>
        /// Flatten to a 6x6 row-major array of 36 doubles.
        /// Order: block rows then inner rows.
        /// </summary>
        public double[] ToArray() => new double[] {
            a11.m11, a11.m12, a11.m13, a12.m11, a12.m12, a12.m13,
            a11.m21, a11.m22, a11.m23, a12.m21, a12.m22, a12.m23,
            a11.m31, a11.m32, a11.m33, a12.m31, a12.m32, a12.m33,
            a21.m11, a21.m12, a21.m13, a22.m11, a22.m12, a22.m13,
            a21.m21, a21.m22, a21.m23, a22.m21, a22.m22, a22.m23,
            a21.m31, a21.m32, a21.m33, a22.m31, a22.m32, a22.m33,
        };

        public double[,] ToArray2() => new double[,] {
            {a11.m11, a11.m12, a11.m13, a12.m11, a12.m12, a12.m13 },
            {a11.m21, a11.m22, a11.m23, a12.m21, a12.m22, a12.m23 },
            {a11.m31, a11.m32, a11.m33, a12.m31, a12.m32, a12.m33 },
            {a21.m11, a21.m12, a21.m13, a22.m11, a22.m12, a22.m13 },
            {a21.m21, a21.m22, a21.m23, a22.m21, a22.m22, a22.m23 },
            {a21.m31, a21.m32, a21.m33, a22.m31, a22.m32, a22.m33 },
        };

        public double[][] ToJaggedArray() => Factory.CreateArray2(6, 6, ToArray());
        #endregion

    }
}