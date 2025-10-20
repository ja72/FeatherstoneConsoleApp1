using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Featherstone.VectorCalculus
{
    /// <summary>
    /// Immutable 3x3 matrix using double precision.
    /// Row-major storage:
    /// [ M11 M12 M13 ]
    /// [ M21 M22 M23 ]
    /// [ M31 M32 M33 ]
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Matrix3 : IEquatable<Matrix3>
    {
        internal const int Size = 9;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly double m11;
        internal readonly double m12;
        internal readonly double m13;
        internal readonly double m21;
        internal readonly double m22;
        internal readonly double m23;
        internal readonly double m31;
        internal readonly double m32;
        internal readonly double m33;

        public Matrix3(
            double m11, double m12, double m13,
            double m21, double m22, double m23,
            double m31, double m32, double m33)
        {
            this.m11=m11; this.m12=m12; this.m13=m13;
            this.m21=m21; this.m22=m22; this.m23=m23;
            this.m31=m31; this.m32=m32; this.m33=m33;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Symmetric(
            double m11, double m12, double m13,
                        double m22, double m23, 
                                    double m33)
        {
            return new Matrix3(
                m11, m12, m13,
                m12, m22, m23,
                m13, m23, m33);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 SkewSymmetric(
            double m12, double m13,
                        double m23)
        {
            return new Matrix3(
                0, m12, m13,
                -m12, 0, m23,
                -m13, -m23, 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Scalar(double d) => new Matrix3(
            d, 0.0, 0.0,
            0.0, d, 0.0,
            0.0, 0.0, d);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Diagonal(Vector3 diag) => Diagonal(diag.X, diag.Y, diag.Z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Diagonal(double d1, double d2, double d3)
            => new Matrix3(
            d1, 0.0, 0.0,
            0.0, d2, 0.0,
            0.0, 0.0, d3);

        public static Matrix3 Zero { get; } = new Matrix3(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);

        public static Matrix3 Identity { get; } = new Matrix3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 CrossOp(Vector3 v) =>
            new Matrix3(
                0.0, -v.z, v.y,
                v.z, 0.0, -v.x,
                -v.y, v.x, 0.0);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 MomentTensor(Vector3 v) =>
            new Matrix3(
                v.y*v.y+v.z*v.z, -v.x*v.y, -v.x*v.z,
                -v.y*v.x, v.x*v.x+v.z*v.z, -v.y*v.z,
                -v.z*v.x, -v.z*v.y, v.x*v.x+v.y*v.y);

        public static Matrix3 Rotation(Vector3 axis, double angle, bool inverse = false)
        {
            if(axis.MagnitudeSquared==0.0)
            {
                throw new ArgumentException("Rotation axis must be non-zero.", nameof(axis));
            }
            if (angle==0.0)
            {
                return Identity;
            }
            if(inverse)
            {
                angle=-angle;
            }
            double cos = Math.Cos(angle), sin = Math.Sin(angle);
            Matrix3 vx = axis.CrossOp();
            Matrix3 vxx = axis.MomentTensor();
            return Identity+vx*sin+vxx*(cos-1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 FromQuaternion(Quaternion3 q)
        {
            // Convert quaternion (w, x, y, z) to a 3x3 rotation matrix.
            // Ensure quaternion is normalized; normalize if necessary.
            var norm2 = q.MagnitudeSquared;
            if (norm2==0.0)
            {
                throw new ArgumentException("Quaternion must be non-zero.", nameof(q));
            }
            var invlen = 1.0 / Math.Sqrt(norm2);

            double w = q.w * invlen;
            double x = q.x * invlen;
            double y = q.y * invlen;
            double z = q.z * invlen;

            double xx = x * x;
            double yy = y * y;
            double zz = z * z;
            double xy = x * y;
            double xz = x * z;
            double yz = y * z;
            double wx = w * x;
            double wy = w * y;
            double wz = w * z;

            return new Matrix3(
                1.0-2.0*( yy+zz ), 2.0*( xy-wz ), 2.0*( xz+wy ),
                2.0*( xy+wz ), 1.0-2.0*( xx+zz ), 2.0*( yz-wx ),
                2.0*( xz-wy ), 2.0*( yz+wx ), 1.0-2.0*( xx+yy )
            );
        }

        public Quaternion3 AsRotation()
        {
            double x = m32-m23, y = m13-m31, z = m21-m12;
            double r2 = x*x+y*y+z*z;
            double tr =  m11+m22+m33;
            double w = 0.5 * Math.Sqrt( r2/(3-tr));
            return new Quaternion3(w, x/( 4*w ), y/( 4*w ), z/( 4*w ));
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 FromRows(Vector3 r1, Vector3 r2, Vector3 r3) =>
            new Matrix3(r1.x, r1.y, r1.z, r2.x, r2.y, r2.z, r3.x, r3.y, r3.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 FromColumns(Vector3 c1, Vector3 c2, Vector3 c3) =>
            new Matrix3(c1.x, c2.x, c3.x, c1.y, c2.y, c3.y, c1.z, c2.z, c3.z);

        #region Properties
        public double M11 => m11;
        public double M12 => m12;
        public double M13 => m13;
        public double M21 => m21;
        public double M22 => m22;
        public double M23 => m23;
        public double M31 => m31;
        public double M32 => m32;
        public double M33 => m33; 
        #endregion

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3 Product(Vector3 v) => Product(this, v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Product(Matrix3 m, Vector3 v) =>
            new Vector3(
                m.m11*v.x+m.m12*v.y+m.m13*v.z,
                m.m21*v.x+m.m22*v.y+m.m23*v.z,
                m.m31*v.x+m.m32*v.y+m.m33*v.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Product(Vector3 v, Matrix3 m) =>
            new Vector3(
                m.m11*v.x+m.m21*v.y+m.m31*v.z,
                m.m12*v.x+m.m22*v.y+m.m32*v.z,
                m.m13*v.x+m.m23*v.y+m.m33*v.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Product(Matrix3 a, Matrix3 b)
        {
            return new Matrix3(
                a.m11*b.m11+a.m12*b.m21+a.m13*b.m31,
                a.m11*b.m12+a.m12*b.m22+a.m13*b.m32,
                a.m11*b.m13+a.m12*b.m23+a.m13*b.m33,

                a.m21*b.m11+a.m22*b.m21+a.m23*b.m31,
                a.m21*b.m12+a.m22*b.m22+a.m23*b.m32,
                a.m21*b.m13+a.m22*b.m23+a.m23*b.m33,

                a.m31*b.m11+a.m32*b.m21+a.m33*b.m31,
                a.m31*b.m12+a.m32*b.m22+a.m33*b.m32,
                a.m31*b.m13+a.m32*b.m23+a.m33*b.m33);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Trace(Matrix3 m) => m.m11 + m.m22 + m.m33;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Transpose(Matrix3 m) =>
            new Matrix3(
                m.m11, m.m21, m.m31,
                m.m12, m.m22, m.m32,
                m.m13, m.m23, m.m33);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Scale(Matrix3 m, double s) =>
            new Matrix3(
                m.m11*s, m.m12*s, m.m13*s,
                m.m21*s, m.m22*s, m.m23*s,
                m.m31*s, m.m32*s, m.m33*s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Divide(Matrix3 m, double d) =>
            new Matrix3(
                m.m11/d, m.m12/d, m.m13/d,
                m.m21/d, m.m22/d, m.m23/d,
                m.m31/d, m.m32/d, m.m33/d);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Negate(Matrix3 a) =>
            new Matrix3(
                -a.m11, -a.m12, -a.m13,
                -a.m21, -a.m22, -a.m23,
                -a.m31, -a.m32, -a.m33);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Add(Matrix3 a, Matrix3 b) =>
            new Matrix3(
                a.m11+b.m11, a.m12+b.m12, a.m13+b.m13,
                a.m21+b.m21, a.m22+b.m22, a.m23+b.m23,
                a.m31+b.m31, a.m32+b.m32, a.m33+b.m33);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Subtract(Matrix3 a, Matrix3 b) =>
            new Matrix3(
                a.m11-b.m11, a.m12-b.m12, a.m13-b.m13,
                a.m21-b.m21, a.m22-b.m22, a.m23-b.m23,
                a.m31-b.m31, a.m32-b.m32, a.m33-b.m33);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Determinant(Matrix3 m)
        {
            // Rule of Sarrus / cofactor expansion
            return
                m.m11*( m.m22*m.m33-m.m23*m.m32 )-
                m.m12*( m.m21*m.m33-m.m23*m.m31 )+
                m.m13*( m.m21*m.m32-m.m22*m.m31 );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryInvert(out Matrix3 result)
        {
            var det = Determinant(this);
            if (det==0.0)
            {
                result=Zero;
                return false;
            }

            var invDet = 1.0 / det;

            // Compute adjugate (cofactor matrix transposed) and multiply by 1/det
            result=new Matrix3(
                ( this.m22*this.m33-this.m23*this.m32 )*invDet,
                ( this.m13*this.m32-this.m12*this.m33 )*invDet,
                ( this.m12*this.m23-this.m13*this.m22 )*invDet,

                ( this.m23*this.m31-this.m21*this.m33 )*invDet,
                ( this.m11*this.m33-this.m13*this.m31 )*invDet,
                ( this.m13*this.m21-this.m11*this.m23 )*invDet,

                ( this.m21*this.m32-this.m22*this.m31 )*invDet,
                ( this.m12*this.m31-this.m11*this.m32 )*invDet,
                ( this.m11*this.m22-this.m12*this.m21 )*invDet
            );
            return true;
        }
        public bool TrySolve(Vector3 b, out Vector3 result)
        {
            if (!this.TryInvert(out var inv))
            {
                result=Vector3.Zero;
                return false;
            }
            result=inv.Product(b);
            return true;
        }
        public bool Equals(Matrix3 other) =>
            m11==other.m11&&m12==other.m12&&m13==other.m13&&
            m21==other.m21&&m22==other.m22&&m23==other.m23&&
            m31==other.m31&&m32==other.m32&&m33==other.m33;

        public override bool Equals(object obj) => obj is Matrix3 other&&Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc=( -1521134295 )*hc+m11.GetHashCode();
                hc=( -1521134295 )*hc+m12.GetHashCode();
                hc=( -1521134295 )*hc+m13.GetHashCode();
                hc=( -1521134295 )*hc+m21.GetHashCode();
                hc=( -1521134295 )*hc+m22.GetHashCode();
                hc=( -1521134295 )*hc+m23.GetHashCode();
                hc=( -1521134295 )*hc+m31.GetHashCode();
                hc=( -1521134295 )*hc+m32.GetHashCode();
                hc=( -1521134295 )*hc+m33.GetHashCode();
                return hc;
            }
        }

        public static Matrix3 operator +(Matrix3 a, Matrix3 b) => Add(a, b);
        public static Matrix3 operator -(Matrix3 a, Matrix3 b) => Subtract(a, b);
        public static Matrix3 operator *(Matrix3 a, Matrix3 b) => Product(a, b);
        public static Vector3 operator *(Matrix3 m, Vector3 v) => Product(m, v);
        public static Vector3 operator *(Vector3 v, Matrix3 m) => Product(v, m);
        public static Matrix3 operator *(Matrix3 m, double s) => Scale(m, s);
        public static Matrix3 operator *(double s, Matrix3 m) => Scale(m, s);
        public static Matrix3 operator /(Matrix3 m, double d) => Divide(m, d);
        public static bool operator ==(Matrix3 left, Matrix3 right) => left.Equals(right);
        public static bool operator !=(Matrix3 left, Matrix3 right) => !left.Equals(right);

        public override string ToString() =>
            $"[{(float)m11}, {(float)m12}, {(float)m13}; {(float)m21}, {(float)m22}, {(float)m23}; {(float)m31}, {(float)m32}, {(float)m33}]";

        public double this[int row, int column]
        {
            get
            {
                if (row<0||row>2) throw new ArgumentOutOfRangeException(nameof(row));
                if (column<0||column>2) throw new ArgumentOutOfRangeException(nameof(column));
                int index = 3*row+column;
                switch (index)
                {
                    case 0: return m11;
                    case 1: return m12;
                    case 2: return m13;
                    case 3: return m21;
                    case 4: return m22;
                    case 5: return m23;
                    case 6: return m31;
                    case 7: return m32;
                    case 8: return m33;
                    default: throw new ArgumentOutOfRangeException();
                }
            }
        }

        public unsafe ReadOnlySpan<double> AsSpan()
        {
            fixed (double* ptr = &m11)
            {
                return new ReadOnlySpan<double>(ptr, Size);
            }
        }

        /// <summary>
        /// Flatten to a 3x3 row-major array of 9 doubles.
        /// Order: block rows then inner rows.
        /// </summary>
        public double[] ToArray() => AsSpan().ToArray();
    }
}