using System;
using System.Runtime.CompilerServices;

using Featherstone.VectorCalculus;

namespace Featherstone.ScrewCalculus
{
    /// <summary>
    /// Immutable 3D screw to hold twists in (linear/angular) form.
    /// </summary>
    public readonly struct Twist33 : IEquatable<Twist33>
    {
        internal readonly Vector3 linear, angular;

        #region Factory
        public Twist33(Vector3 linear, Vector3 angular)
        {
            this.linear=linear;
            this.angular=angular;
        }
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
        public Vector3 Linear => this.linear;
        public Vector3 Angular => this.angular;
        public double Magnitude => this.angular.Magnitude;
        public Vector3 Direction => Vector3.Normalized(this.angular);
        public double Pitch => Vector3.Dot(this.angular, this.linear)/this.angular.MagnitudeSquared;
        public Vector3 Position => Vector3.Cross(this.angular, this.linear)/this.angular.MagnitudeSquared;
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Twist33 Cross(Twist33 @this, Twist33 other)
        {
            return new Twist33(
                Vector3.Cross(@this.angular, other.linear) + Vector3.Cross(@this.linear, other.angular),
                Vector3.Cross(@this.angular, other.angular)
            );
        }
        public static Twist33 operator -(Twist33 a) => new Twist33(-a.linear, -a.angular);
        public static Twist33 operator + (Twist33 a, Twist33 b) 
            => new Twist33(
                a.linear  + b.linear,
                a.angular + b.angular);
        public static Twist33 operator - (Twist33 a, Twist33 b) 
            => new Twist33(
                a.linear  - b.linear,
                a.angular - b.angular);
        public static Twist33 operator * (Twist33 a, double factor) 
            => new Twist33(
                a.linear  * factor,
                a.angular * factor);
        public static Twist33 operator *(double factor, Twist33 a)
            => a*factor;
        public static Twist33 operator / (Twist33 a, double divisor) 
            => new Twist33(
                a.linear  / divisor,
                a.angular / divisor);

        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
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
                hc=( -1521134295 )*hc+linear.GetHashCode();
                hc=( -1521134295 )*hc+angular.GetHashCode();
                return hc;
            }
        }

        #endregion

        #region Formatting
        public override string ToString() => $"T[{linear}|{angular}]";
        #endregion
    }
}
