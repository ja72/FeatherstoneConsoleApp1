using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Featherstone.VectorCalculus
{
    /// <summary>
    /// Immutable 3D vector using double precision.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Vector3 : IEquatable<Vector3>
    {
        internal const int Size = 3;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly double x;
        internal readonly double y;
        internal readonly double z;

        #region Factory
        public Vector3(double x, double y, double z)
        {
            this.x=x;
            this.y=y;
            this.z=z;
        }
        public static Vector3 Direction(Vector3 from, Vector3 to) => Normalized(to-from);
        public static Vector3 FromSpherical(double radius, double theta, double phi)
        {
            var cosTheta = Math.Cos(theta);
            var sinTheta = Math.Sin(theta);
            return new Vector3(
                radius*cosTheta*Math.Cos(phi),
                radius*cosTheta*Math.Sin(phi),
                radius*sinTheta
            );
        }
        public static Vector3 FromCylindrical(double radius, double theta, double z)
        {
            return new Vector3(
                radius*Math.Cos(theta),
                radius*Math.Sin(theta),
                z
            );
        }
        public static Vector3 Zero { get; } = new Vector3(0.0, 0.0, 0.0);
        public static Vector3 UnitX { get; } = new Vector3(1.0, 0.0, 0.0);
        public static Vector3 UnitY { get; } = new Vector3(0.0, 1.0, 0.0);
        public static Vector3 UnitZ { get; } = new Vector3(0.0, 0.0, 1.0);
        #endregion

        #region Properties
        public double MagnitudeSquared => x*x+y*y+z*z;
        public double Magnitude => Math.Sqrt(MagnitudeSquared);

        public double X => this.x;

        public double Y => this.y;

        public double Z => this.z;
        #endregion

        #region Algebra
        public Vector3 Normalize() => Normalized(this);

        public bool TryNormalize(out Vector3 result)
        {
            var len = this.MagnitudeSquared;
            if (len==0.0)
            {
                result=Zero;
                return false;
            }
            if (len==1.0)
            {
                result=this;
                return true;
            }
            result=this/Math.Sqrt(len);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Normalized(Vector3 v)
        {
            var len = v.Magnitude;
            if (len==0.0) return Zero;
            return v/len;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Vector3 a, Vector3 b) => a.X*b.X+a.Y*b.Y+a.Z*b.Z;
        public static Matrix3 Outer(Vector3 a, Vector3 b) =>
            new Matrix3(
                a.x*b.x, a.x*b.y, a.x*b.z,
                a.y*b.x, a.y*b.y, a.y*b.z,
                a.z*b.x, a.z*b.y, a.z*b.z
            );
        public static Vector3 Cross(Vector3 a, Vector3 b) =>
            new Vector3(
                a.y*b.z-a.z*b.y,
                a.z*b.x-a.x*b.z,
                a.x*b.y-a.y*b.x
            );
        /// <summary>
        /// Cross product operator matrix (skew-symmetric)
        /// </summary>
        /// <returns><code>[0,-z,y; z,0,-x; -y,x,0]</code></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 CrossOp() => Matrix3.CrossOp(this);
        /// <summary>
        /// Moment of inertia tensor (symmetric)
        /// </summary>
        /// <returns><code>[y^2+z^2,-x*y,-x*z; -x*y,x^2+z^2,-y*z; -x*z,y*z,x^2+y^2]</code></returns>
        /// <remarks>Equals to <code>-CrossOp()*CrossOp()</code></remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 MomentTensor() => Matrix3.MomentTensor(this);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Negate(Vector3 v) => new Vector3(-v.x, -v.y, -v.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Scale(Vector3 v, double s) => new Vector3(v.x*s, v.y*s, v.z*s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Divide(Vector3 v, double s) => new Vector3(v.x/s, v.y/s, v.z/s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Add(Vector3 a, Vector3 b) => new Vector3(a.x+b.x, a.y+b.y, a.z+b.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Subtract(Vector3 a, Vector3 b) => new Vector3(a.x-b.x, a.y-b.y, a.z-b.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Invert(Vector3 v) => new Vector3(1.0/v.x, 1.0/v.y, 1.0/v.z);
        #endregion

        #region Operators
        public static Vector3 operator -(Vector3 v) => Negate(v);
        public static Vector3 operator +(Vector3 a, Vector3 b) => Add(a, b);
        public static Vector3 operator -(Vector3 a, Vector3 b) => Subtract(a, b);
        public static Vector3 operator *(Vector3 v, double s) => Scale(v, s);
        public static Vector3 operator *(double s, Vector3 v) => v*s;
        public static Vector3 operator /(Vector3 v, double s) => Divide(v, s);
        #endregion

        #region IEquatable Members
        public bool Equals(Vector3 other) => x==other.x&&y==other.y&&z==other.z;
        public override bool Equals(object obj) => obj is Vector3 other&&Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc=( -1521134295 )*hc+X.GetHashCode();
                hc=( -1521134295 )*hc+Y.GetHashCode();
                hc=( -1521134295 )*hc+Z.GetHashCode();
                return hc;
            }
        }

        public static bool operator ==(Vector3 left, Vector3 right) => left.Equals(right);

        public static bool operator !=(Vector3 left, Vector3 right) => !left.Equals(right);
        #endregion

        #region Formatting
        public override string ToString() => $"({(float)X}, {(float)Y}, {(float)Z})";
        #endregion

        #region Collection
        public unsafe ReadOnlySpan<double> AsSpan()
        {
            fixed (double* ptr = &x)
            {
                return new ReadOnlySpan<double>(ptr, Size);
            }
        }

        public double[] ToArray() => AsSpan().ToArray();
        #endregion
    }
}