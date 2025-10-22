using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

using Featherstone.VectorCalculus;

namespace Featherstone.ScrewCalculus
{
    /// <summary>
    /// Immutable double 3D vector using double precision.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]

    public readonly struct Vector33 : IEquatable<Vector33>
    {
        internal const int Size = 6;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly Vector3 v1, v2;

        #region Factory
        public Vector33(Vector3 vTop, Vector3 vBottom)
        {
            this.v1=vTop;
            this.v2=vBottom;
        }

        public static Vector33 Zero { get; } = new Vector33(Vector3.Zero, Vector3.Zero);

        public static implicit operator Vector33(Twist33 twist)
            => new Vector33(twist.linear, twist.angular);   
        public static implicit operator Vector33(Wrench33 wrench)
            => new Vector33(wrench.linear, wrench.angular);   

        #endregion

        #region Properties
        public Vector3 Top => this.v1;
        public Vector3 Bottom => this.v2;
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Vector33 a, Vector33 b) 
            => Vector3.Dot(a.v1, b.v1)+Vector3.Dot(a.v2, b.v2);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Negate(Vector33 a)
            => new Vector33(
                -a.v1,
                -a.v2);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Scale(Vector33 a, double factor)
            => new Vector33(
                factor*a.v1,
                factor*a.v2);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Divide(Vector33 a, double factor)
            => new Vector33(
                a.v1/factor,
                a.v2/factor);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Add(Vector33 a, Vector33 b)
            => new Vector33(
                a.v1+b.v1,
                a.v2+b.v2);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Subtract(Vector33 a, Vector33 b)
            => new Vector33(
                a.v1-b.v1,
                a.v2-b.v2);

        #endregion

        #region Operators
        public static Vector33 operator -(Vector33 a) => Negate(a);
        public static Vector33 operator +(Vector33 a, Vector33 b) => Add(a, b);
        public static Vector33 operator -(Vector33 a, Vector33 b) => Subtract(a, b);
        public static Vector33 operator *(double f, Vector33 a) => Scale(a, f);
        public static Vector33 operator *(Vector33 a, double f) => Scale(a, f);
        public static Vector33 operator /(Vector33 a, double d) => Divide(a, d);
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Vector33)</code></returns>
        public override bool Equals(object obj)
        {
            return obj is Vector33 vector&&Equals(vector);
        }

        public static bool operator ==(Vector33 target, Vector33 other) { return target.Equals(other); }
        public static bool operator !=(Vector33 target, Vector33 other) { return !target.Equals(other); }


        /// <summary>
        /// Checks for equality among <see cref="Vector33"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Vector33"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Vector33 other) => v1.Equals(other.v1)&&v2.Equals(other.v2);

        /// <summary>
        /// Calculates the hash code for the <see cref="Vector33"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc=( -1521134295 )*hc+v1.GetHashCode();
                hc=( -1521134295 )*hc+v2.GetHashCode();
                return hc;
            }
        }

        #endregion

        #region Formatting
        public override string ToString() => $"S[{v1}|{v2}]";
        #endregion

        #region Collection
        public unsafe ReadOnlySpan<double> AsSpan()
        {
            fixed (double* ptr = &v1.x)
            {
                return new ReadOnlySpan<double>(ptr, Size);
            }
        }

        public double[] ToArray() => AsSpan().ToArray(); 
        #endregion

    }


}
