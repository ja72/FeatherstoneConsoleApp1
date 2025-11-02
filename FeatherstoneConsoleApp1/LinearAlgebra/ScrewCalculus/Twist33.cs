using System;
using System.Drawing;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;


namespace JA.LinearAlgebra.ScrewCalculus
{

    using Vector3 = VectorCalculus.Vector3;

    /// <summary>
    /// Immutable 3D screw to hold twists in (linear/angular) form.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]
    public readonly struct Twist33 : IEquatable<Twist33>
    {
        internal const int Size = 6;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly Vector3 linear, angular;

        #region Factory
        Twist33(Vector3 linear, Vector3 angular)
        {
            this.linear=linear;
            this.angular=angular;
        }
        public static Twist33 Zero { get; } = new Twist33(Vector3.Zero, Vector3.Zero);
        public static Twist33 At(Vector3 angular, Vector3 position, double pitch = 0.0)
            => new Twist33(
                angular*pitch+Vector3.Cross(position, angular),
                angular
            );
        public static Twist33 Pure(Vector3 linear)
            => new Twist33(
                linear, 
                Vector3.Zero
            );
        public static implicit operator Twist33(Vector33 vector)
            => new Twist33(vector.v1, vector.v2);   
        #endregion

        #region Properties
        public ScrewCoordinates Coordinates { get => ScrewCoordinates.Axis; }
        public Vector3 Linear => this.linear;
        public Vector3 Angular => this.angular;
        public double Magnitude => this.angular.Magnitude;
        public Vector3 Direction => Vector3.Normalized(this.angular);
        public double Pitch => Vector3.Dot(this.angular, this.linear)/this.angular.MagnitudeSquared;
        public Vector3 Position => Vector3.Cross(this.angular, this.linear)/this.angular.MagnitudeSquared;
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Twist33 @this, Wrench33 other)
            => Vector3.Dot(@this.linear, other.linear)+Vector3.Dot(@this.angular, other.angular);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Outer(Twist33 a, Wrench33 b)
            => new Matrix33(
                Vector3.Outer(a.linear, b.linear), Vector3.Outer(a.linear, b.angular),
                Vector3.Outer(a.angular, b.linear), Vector3.Outer(a.angular, b.angular));


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Twist33 Cross(Twist33 @this, Twist33 other)
        {
            return new Twist33(
                Vector3.Cross(@this.angular, other.linear) + Vector3.Cross(@this.linear, other.angular),
                Vector3.Cross(@this.angular, other.angular)
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Twist33 Cross(Wrench33 @this, Twist33 other)
        {
            return new Twist33(
                Vector3.Cross(@this.linear, other.linear) + Vector3.Cross(@this.angular, other.angular),
                Vector3.Cross(@this.linear, other.angular)
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Twist33 Cross(Twist33 @this, Wrench33 other)
        {
            return new Twist33(
                Vector3.Cross(@this.angular, other.angular) + Vector3.Cross(@this.linear, other.linear),
                Vector3.Cross(@this.angular, other.linear)
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Twist33 Cross(Wrench33 @this, Wrench33 other)
        {
            return new Twist33(
                Vector3.Cross(@this.linear, other.angular) + Vector3.Cross(@this.angular, other.linear),
                Vector3.Cross(@this.linear, other.linear)
            );
        }
        #endregion

        #region Operators
        public static Twist33 operator -(Twist33 a) => new Twist33(-a.linear, -a.angular);
        public static Twist33 operator +(Twist33 a, Twist33 b)
            => new Twist33(
                a.linear+b.linear,
                a.angular+b.angular);
        public static Twist33 operator -(Twist33 a, Twist33 b)
            => new Twist33(
                a.linear-b.linear,
                a.angular-b.angular);
        public static Twist33 operator *(Twist33 a, double factor)
            => new Twist33(
                a.linear*factor,
                a.angular*factor);
        public static Twist33 operator *(double factor, Twist33 a)
            => a*factor;
        public static Twist33 operator /(Twist33 a, double divisor)
            => new Twist33(
                a.linear/divisor,
                a.angular/divisor);

        public static Twist33 operator ^(Twist33 a, Twist33 b)   => Cross(a, b);
        public static Twist33 operator ^(Twist33 a, Wrench33 b)  => Cross(a, b);
        //public static Twist33 operator ^(Wrench33 a, Twist33 b)  => Cross(a, b);
        //public static Twist33 operator ^(Wrench33 a, Wrench33 b) => Cross(a, b); 
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Twist33)</code></returns>
        public override bool Equals(object obj)
        {
            return obj is Twist33 vector&&Equals(vector);
        }

        public static bool operator ==(Twist33 target, Twist33 other) { return target.Equals(other); }
        public static bool operator !=(Twist33 target, Twist33 other) { return !( target==other ); }


        /// <summary>
        /// Checks for equality among <see cref="Twist33"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Twist33"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Twist33 other) 
            => linear.Equals(other.linear)&&angular.Equals(other.angular);

        /// <summary>
        /// Calculates the hash code for the <see cref="Twist33"/>
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
        public override string ToString() => $"T[{linear}|{angular}]";
        #endregion

        #region Collection
        public static implicit operator double[](Twist33 @this) => @this.ToArray();
        public double[] ToArray() => new double[] { linear.x, linear.y, linear.z, angular.x, angular.y, angular.z };
        #endregion

    }
}
