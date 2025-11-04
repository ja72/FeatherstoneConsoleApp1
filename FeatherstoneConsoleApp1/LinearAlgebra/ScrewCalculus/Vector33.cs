using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

using JA.LinearAlgebra.VectorCalculus;


namespace JA.LinearAlgebra.ScrewCalculus
{
    
    using Vector3 = VectorCalculus.Vector3;

    public enum ScrewCoordinates
    {
        [Description("screw=(moment vector, direction vector)")]
        Axis,   
        [Description("screw=(direction vector, moment vector)")]
        Ray,    
    }

    /// <summary>
    /// Immutable double 3D vector using double precision.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = ByteSize)]

    public readonly struct Vector33 : IEquatable<Vector33>
    {
        internal const int Size = 6;
        internal const int ByteSize = Size * sizeof(double);

        internal readonly Vector3 linear, angular;

        #region Factory
        public Vector33(Vector3 linear, Vector3 angular)
        {
            this.linear=linear;
            this.angular=angular;
        }

        public static Vector33 Zero { get; } = new Vector33(Vector3.Zero, Vector3.Zero);

        public static Vector33 Twist(Vector3 value, Vector3 position, double pitch = 0)
            => Screws.TwistAt(value, position, pitch);
        public static Vector33 Twist(Vector3 value)
            => Screws.PureTwist(value);
        public static Vector33 Wrench(Vector3 value, Vector3 position, double pitch = 0)
            => Screws.WrenchAt(value, position, pitch);
        public static Vector33 Wrench(Vector3 value)
            => Screws.PureWrench(value);

        #endregion

        #region Properties
        public Vector3 Linear => this.linear;
        public Vector3 Angular => this.angular;
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Vector33 a, Vector33 b) 
            => Vector3.Dot(a.linear, b.linear)+Vector3.Dot(a.angular, b.angular);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 Outer(Vector33 a, Vector33 b)
            => new Matrix33(
                Vector3.Outer(a.linear, b.linear), Vector3.Outer(a.linear, b.angular),
                Vector3.Outer(a.angular, b.linear), Vector3.Outer(a.angular, b.angular));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Negate(Vector33 a)
            => new Vector33(
                -a.linear,
                -a.angular);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Scale(Vector33 a, double factor)
            => new Vector33(
                factor*a.linear,
                factor*a.angular);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Divide(Vector33 a, double factor)
            => new Vector33(
                a.linear/factor,
                a.angular/factor);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Add(Vector33 a, Vector33 b)
            => new Vector33(
                a.linear+b.linear,
                a.angular+b.angular);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Subtract(Vector33 a, Vector33 b)
            => new Vector33(
                a.linear-b.linear,
                a.angular-b.angular);

        #endregion

        #region Operators
        public static Vector33 operator -(Vector33 a) => Negate(a);
        public static Vector33 operator +(Vector33 a, Vector33 b) => Add(a, b);
        public static Vector33 operator -(Vector33 a, Vector33 b) => Subtract(a, b);
        public static Vector33 operator *(double f, Vector33 a) => Scale(a, f);
        public static Vector33 operator *(Vector33 a, double f) => Scale(a, f);
        public static Vector33 operator /(Vector33 a, double d) => Divide(a, d);
        public static double operator *(Vector33 a, Vector33 b) => Dot(a, b);
        public static Matrix33 operator %(Vector33 a, Vector33 b) => Outer(a, b);
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="object"/>
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
        public bool Equals(Vector33 other) => linear.Equals(other.linear)&&angular.Equals(other.angular);

        /// <summary>
        /// Calculates the hash code for the <see cref="Vector33"/>
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
        public override string ToString() => $"S[{linear}|{angular}]";
        #endregion

        #region Collection
        public static implicit operator double[](Vector33 @this) => @this.ToArray();
        public double[] ToArray() => new double[] { linear.x, linear.y, linear.z, angular.x, angular.y, angular.z };
        #endregion
    }


}
