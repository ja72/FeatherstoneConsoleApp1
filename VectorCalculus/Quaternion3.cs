﻿using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Featherstone.VectorCalculus
{
    /// <summary>
    /// Immutable quaternion using double precision. 
    /// Representation: (W, X, Y, Z) where W is the scalar part.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Quaternion3 : IEquatable<Quaternion3>
    {
        internal const int Size = 4;
        internal const int ByteSize = Size * sizeof(double);
        internal readonly double w;
        internal readonly double x;
        internal readonly double y;
        internal readonly double z;
        public Quaternion3(double w, Vector3 vector)
             : this(w, vector.X, vector.Y, vector.Z) { }
        public Quaternion3(double w, double x, double y, double z)
        {
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public static Quaternion3 FromVector(Vector3 vector) => new Quaternion3(0.0, vector.X, vector.Y, vector.Z);

        public double W => w;
        public double X => x;
        public double Y => y;
        public double Z => z;

        public static Quaternion3 Zero { get; } = new Quaternion3(0.0, 0.0, 0.0, 0.0);
        public static Quaternion3 Identity { get; } = new Quaternion3(1.0, 0.0, 0.0, 0.0);
        public double ScalarPart { get => w; }
        public Vector3 VectorPart { get => new Vector3(x, y, z); }
        public double MagnitudeSquared => w * w + x * x + y * y + z * z;
        public double Magnitude => Math.Sqrt(MagnitudeSquared);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Quaternion3 Conjugate() => new Quaternion3(w, -x, -y, -z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion3 Conjugate(Quaternion3 q) => q.Conjugate();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryInverse(out Quaternion3 result)
        {
            var norm  = this.MagnitudeSquared;
            if (norm == 0.0)
            {
                result = Zero;
                return false;
            }
            var inv = 1.0 / norm;
            result = new Quaternion3(this.w * inv, -this.x * inv, -this.y * inv, -this.z * inv);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Quaternion3 Inverse()
        {
            if (!this.TryInverse(out var inv))
                throw new InvalidOperationException("Cannot invert a quaternion with zero magnitude.");
            return inv;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion3 Multiply(Quaternion3 a, Quaternion3 b)
            => new Quaternion3(
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion3 Scale(Quaternion3 q, double s) => new Quaternion3(q.w * s, q.x * s, q.y * s, q.z * s);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion3 Divide(Quaternion3 q, double d) => new Quaternion3(q.w / d, q.x / d, q.y / d, q.z / d);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion3 Add(Quaternion3 a, Quaternion3 b) => new Quaternion3(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion3 Subtract(Quaternion3 a, Quaternion3 b) => new Quaternion3(a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z);

        /// <summary>
        /// Create a rotation quaternion from axis and angle (radians).
        /// </summary>
        public static Quaternion3 FromAxisAngle(Vector3 axis, double angle, bool inverse = false)
        {
            if (axis.MagnitudeSquared == 0.0)
                throw new ArgumentException("Rotation axis must be non-zero.", nameof(axis));
            if (angle == 0.0)
                return Identity;
            if (inverse) angle = -angle;
            var half = 0.5 * angle;
            var s = Math.Sin(half);
            var c = Math.Cos(half);
            var naxis = Vector3.Normalized(axis);
            return new Quaternion3(c, naxis.X * s, naxis.Y * s, naxis.Z * s);
        }

        /// <summary>
        /// Rotate a vector by this quaternion.
        /// Uses optimized formula: v' = v + 2 * cross(qv, cross(qv, v) + w * v)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3 Rotate(Vector3 b, bool inverse = false)
        {
            var v = VectorPart;
            var w = ScalarPart;
            var vxb = Vector3.Cross(v, b);
            var vxb2 = Vector3.Scale(vxb, 2.0);
            if(inverse)
            {
                w=-w;
            }
            return b + vxb2 * w + Vector3.Cross(v, vxb2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Quaternion3 Normalize()
        {
            var len2 = MagnitudeSquared;
            if (len2 == 0.0) return Zero;
            var len = Math.Sqrt(len2);
            return Divide(this, len2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryNormalize(out Quaternion3 result)
        {
            var len2 = MagnitudeSquared;
            if (len2 == 0.0)
            {
                result = Zero;
                return false;
            }
            if (len2 == 1.0)
            {
                result = this;
                return true;
            }
            double len = Math.Sqrt(len2);
            result = Divide(this, len);
            return true;
        }

        public bool Equals(Quaternion3 other) =>
            w == other.w && x == other.x && y == other.y && z == other.z;


        public override bool Equals(object obj) => obj is Quaternion3 other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc=( -1521134295 )*hc+W.GetHashCode();
                hc=( -1521134295 )*hc+X.GetHashCode();
                hc=( -1521134295 )*hc+Y.GetHashCode();
                hc=( -1521134295 )*hc+Z.GetHashCode();
                return hc;
            }
        }
        public Matrix3 ToRotation() => Matrix3.FromQuaternion(this);
        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        public Quaternion3 Differentiate(Vector3 angularVelocity)
        {
            // dq/dt = 0.5 * omega * q
            var omegaQuat = new Quaternion3(0,
                angularVelocity.X/2, 
                angularVelocity.Y/2,                
                angularVelocity.Z/2);

            return Multiply(omegaQuat, this);
        }
        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        public Quaternion3 Integrate(Vector3 angularVelocity, double deltaTime)
        {
            // dq/dt = 0.5 * omega * q
            // Δq = [dq/dt * dt]
            var dq = Differentiate(angularVelocity);
            dq = Scale(dq, deltaTime);
            return Add(this, dq).Normalize();
        }
        public static Quaternion3 operator +(Quaternion3 a, Quaternion3 b) => Add(a, b);
        public static Quaternion3 operator -(Quaternion3 a, Quaternion3 b) => Subtract(a, b);
        public static Quaternion3 operator *(Quaternion3 a, Quaternion3 b) => Multiply(a, b);
        public static Quaternion3 operator *(Quaternion3 q, double s) => Scale(q, s);
        public static Quaternion3 operator *(double s, Quaternion3 q) => Scale(q, s);
        public static Quaternion3 operator /(Quaternion3 q, double d) => Divide(q, d);
        public static bool operator ==(Quaternion3 left, Quaternion3 right) => left.Equals(right);
        public static bool operator !=(Quaternion3 left, Quaternion3 right) => !left.Equals(right);

        public override string ToString() => $"({(float)W}, {(float)X}, {(float)Y}, {(float)Z})";

        public unsafe ReadOnlySpan<double> AsSpan()
        {
            fixed (double* ptr = &w)
            {
                return new ReadOnlySpan<double>(ptr, Size);
            }
        }

        public double[] ToArray() => AsSpan().ToArray();
    }
}