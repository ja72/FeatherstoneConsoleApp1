using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

using JA.LinearAlgebra.ScrewCalculus;

using static System.Math;

namespace JA.LinearAlgebra.VectorCalculus
{

    public enum Axis
    {
        [Description("x-axis")]
        X,
        [Description("y-axis")]
        Y,
        [Description("z-axis")]
        Z,
    }

    /// <summary>
    /// Immutable 3D vector using double precision.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Vector3 : IEquatable<Vector3>
    {
        static readonly Random rng = new Random();

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
        public static implicit operator Vector3(Axis axis) => FromAxis(axis);
        public static Vector3 FromAxis(Axis axis, double magnitude = 1)
        {
            switch (axis)
            {
                case Axis.X: return magnitude*UnitX;
                case Axis.Y: return magnitude*UnitY;
                case Axis.Z: return magnitude*UnitZ;
                default: throw new NotImplementedException(axis.ToString());
            }
        }
        public static Vector3 RandomAxis(double magnitude = 1)
        {
            return FromAxis((Axis)rng.Next(3), magnitude);
        }
        public static Vector3 RandomVector(double magnitude = 1)
        {
            double theta = PI*(rng.NextDouble()-0.5);
            double phi = 2*PI*rng.NextDouble();
            return FromSpherical(UnitZ, UnitX, magnitude, theta, phi);
        }
        public static Vector3 FromPolar(Vector3 axis, Vector3 reference, double radius, double theta)
            => FromCylindrical(axis, reference, radius, theta, 0);
        public static Vector3 FromCylindrical(Vector3 axis, Vector3 reference, double radius, double azimuth, double z)
        {
            Vector3 y = Cross(axis, reference).Normalize();
            Vector3 x = Cross(y, axis).Normalize();
            double a_sin = Sin(azimuth);
            double a_cos = Cos(azimuth);
            return radius*a_cos*x+radius*a_sin*y+z*axis;
        }
        public static Vector3 FromSpherical(Vector3 axis, Vector3 reference, double radius, double elevation, double azimuth)
        {
            Vector3 y = Cross(axis, reference).Normalize();
            Vector3 x = Cross(y, axis).Normalize();
            var e_sin = Sin(elevation);
            var e_cos = Cos(elevation);
            return radius*e_cos*Cos(azimuth)*x
                +radius*e_cos*Sin(azimuth)*y
                +radius*e_sin*axis;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Relative(Vector3 from, Vector3 to) => to-from;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Direction(Vector3 from, Vector3 to) => Normalized(to-from);

        public static Vector3 FromVector(System.Numerics.Vector3 v)
            => new Vector3(v.X, v.Y, v.Z);
        public static Vector3 Zero { get; } = new Vector3(0.0, 0.0, 0.0);
        public static Vector3 UnitX { get; } = new Vector3(1.0, 0.0, 0.0);
        public static Vector3 UnitY { get; } = new Vector3(0.0, 1.0, 0.0);
        public static Vector3 UnitZ { get; } = new Vector3(0.0, 0.0, 1.0);

        #endregion

        #region Properties
        public double MagnitudeSquared => x*x+y*y+z*z;
        public double Magnitude => Sqrt(MagnitudeSquared);

        public double X => this.x;

        public double Y => this.y;

        public double Z => this.z;

        public bool IsZero => x==0.0&&y==0.0&&z==0.0;
        public bool IsUnitX => x==1.0&&y==0.0&&z==0.0;
        public bool IsUnitY => x==0.0&&y==1.0&&z==0.0;
        public bool IsUnitZ => x==0.0&&y==0.0&&z==1.0;
        #endregion

        #region Algebra
        public Vector3 Normalize() => Normalized(this);

        public bool TryNormalize(out Vector3 result)
        {
            var len = MagnitudeSquared;
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
            result=this/Sqrt(len);
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
        public static double Dot(Vector3 a, Vector3 b)
            => a.X*b.X+a.Y*b.Y+a.Z*b.Z;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Outer(Vector3 a, Vector3 b)
            => new Matrix3(
                a.x*b.x, a.x*b.y, a.x*b.z,
                a.y*b.x, a.y*b.y, a.y*b.z,
                a.z*b.x, a.z*b.y, a.z*b.z
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Cross(Vector3 a, Vector3 b)
            => new Vector3(
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
        public Matrix3 MomentTensor(double scale = 1) => Matrix3.MomentTensor(this, scale);
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
        public static double operator *(Vector3 a, Vector3 b) => Dot(a, b);
        public static Matrix3 operator %(Vector3 a, Vector3 b) => Outer(a, b);
        #endregion

        #region IEquatable Members
        public bool Equals(Vector3 other) => x==other.x&&y==other.y&&z==other.z;
        public override bool Equals(object obj) => obj is Vector3 other&&Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc=-1521134295*hc+X.GetHashCode();
                hc=-1521134295*hc+Y.GetHashCode();
                hc=-1521134295*hc+Z.GetHashCode();
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
        public static implicit operator double[](Vector3 vector) => vector.ToArray();
        public double[] ToArray() => new[] { x, y, z };
        #endregion
    }
}