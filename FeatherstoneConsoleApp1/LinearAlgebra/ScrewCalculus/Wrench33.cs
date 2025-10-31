using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

using JA.LinearAlgebra.VectorCalculus;

namespace JA.LinearAlgebra.ScrewCalculus
{

    using Vector3 = VectorCalculus.Vector3;

    /// <summary>
    /// Immutable 3D screw to hold wrenches in (linear/angular) form.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Wrench33 : IEquatable<Wrench33>
    {
        internal const int Size = 6;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly Vector3 linear, angular;

        #region Factory
        Wrench33(Vector3 linear, Vector3 angular)
        {
            this.linear=linear;
            this.angular=angular;
        }
        public static Wrench33 Zero { get; } = new Wrench33(Vector3.Zero, Vector3.Zero);
        public static Wrench33 At(Vector3 linear, Vector3 position, double pitch = 0.0)
            => new Wrench33(
                linear,
                linear*pitch+Vector3.Cross(position, linear)
            );
        public static Wrench33 Pure(Vector3 angular)
            => new Wrench33(
                Vector3.Zero,
                angular
            );
        public static explicit operator Wrench33(Vector33 vector)
            => new Wrench33(vector.v1, vector.v2);
        #endregion

        #region Properties
        public ScrewCoordinates Coordinates { get => ScrewCoordinates.Ray; }
        public Vector3 Linear => this.linear;
        public Vector3 Angular => this.angular;
        public double Magnitude => this.linear.Magnitude;
        public Vector3 Direction => Vector3.Normalized(this.linear);
        public double Pitch => Vector3.Dot(this.linear, this.angular)/this.linear.MagnitudeSquared;
        public Vector3 Position => Vector3.Cross(this.linear, this.angular)/this.linear.MagnitudeSquared;
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Wrench33 @this, Twist33 other)
            => Vector3.Dot(@this.linear, other.linear)+Vector3.Dot(@this.angular, other.angular);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Outer(Wrench33 a, Twist33 b)
            => new Matrix33(
                Vector3.Outer(a.linear, b.linear), Vector3.Outer(a.linear, b.angular),
                Vector3.Outer(a.angular, b.linear), Vector3.Outer(a.angular, b.angular));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Wrench33 Cross(Twist33 @this, Twist33 other)
        {
            return new Wrench33(
                Vector3.Cross(@this.angular, other.angular),
                Vector3.Cross(@this.angular, other.linear) + Vector3.Cross(@this.linear, other.angular)
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Wrench33 Cross(Wrench33 @this, Twist33 other)
        {
            return new Wrench33(
                Vector3.Cross(@this.linear, other.angular), 
                Vector3.Cross(@this.linear, other.linear) + Vector3.Cross(@this.angular, other.angular)
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Wrench33 Cross(Twist33 @this, Wrench33 other)
        {
            return new Wrench33(
                Vector3.Cross(@this.angular, other.linear),
                Vector3.Cross(@this.angular, other.angular) + Vector3.Cross(@this.linear, other.linear)
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Wrench33 Cross(Wrench33 @this, Wrench33 other)
        {
            return new Wrench33(
                Vector3.Cross(@this.linear, other.linear),
                Vector3.Cross(@this.linear, other.angular) + Vector3.Cross(@this.angular, other.linear)
            );
        }

        #endregion

        #region Operetors
        public static Wrench33 operator -(Wrench33 a) => new Wrench33(-a.linear, -a.angular);
        public static Wrench33 operator + (Wrench33 a, Wrench33 b) 
            => new Wrench33(
                a.linear  + b.linear,
                a.angular + b.angular);
        public static Wrench33 operator - (Wrench33 a, Wrench33 b) 
            => new Wrench33(
                a.linear  - b.linear,
                a.angular - b.angular);
        public static Wrench33 operator * (Wrench33 a, double factor) 
            => new Wrench33(
                a.linear  * factor,
                a.angular * factor);
        public static Wrench33 operator *(double factor, Wrench33 a)
            => a*factor;
        public static Wrench33 operator / (Wrench33 a, double divisor) 
            => new Wrench33(
                a.linear  / divisor,
                a.angular / divisor);

        //public static Wrench33 operator ^(Twist33 a, Twist33 b)   => Cross(a, b);
        //public static Wrench33 operator ^(Twist33 a, Wrench33 b)  => Cross(a, b);
        public static Wrench33 operator ^(Wrench33 a, Twist33 b)  => Cross(a, b);
        public static Wrench33 operator ^(Wrench33 a, Wrench33 b) => Cross(a, b);

        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Twist33)</code></returns>
        public override bool Equals(object obj)
        {
            return obj is Wrench33 vector&&Equals(vector);
        }

        public static bool operator ==(Wrench33 target, Wrench33 other) { return target.Equals(other); }
        public static bool operator !=(Wrench33 target, Wrench33 other) { return !( target==other ); }


        /// <summary>
        /// Checks for equality among <see cref="Wrench33"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Wrench33"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Wrench33 other) 
            => linear.Equals(other.linear)&&angular.Equals(other.angular);

        /// <summary>
        /// Calculates the hash code for the <see cref="Wrench33"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc= -1521134295 *hc+linear.GetHashCode();
                hc= -1521134295 *hc+angular.GetHashCode();
                return hc;
            }
        }

        #endregion

        #region Formatting
        public override string ToString() => $"W[{linear}|{angular}]";
        #endregion

        #region Collection
        public static implicit operator double[](Wrench33 @this) => @this.ToArray();
        public double[] ToArray() => new double[] { linear.x, linear.y, linear.z, angular.x, angular.y, angular.z };
        #endregion

    }
}
