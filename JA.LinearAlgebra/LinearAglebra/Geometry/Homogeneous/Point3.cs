using System;
using System.Globalization;
using System.Runtime.CompilerServices;

using static System.Math;

//using Vector4 = System.Numerics.Vector4;

namespace JA.LinearAlgebra.Geometry.Homogeneous
{
    using Vector3 = Vectors.Vector3;
    using Quaternion3 = Vectors.Quaternion3;
    public readonly struct Point3 : IEquatable<Point3>
    {
        private readonly (Vector3 position, double w) data;

        /// <summary>Creates a <see cref="T:System.Numerics.Point" /> object from the X, Y, and Z components of its normal, and its distance from the origin on that normal.</summary>
        /// <param name="x">The X component of the position.</param>
        /// <param name="y">The Y component of the position.</param>
        /// <param name="z">The Z component of the position.</param>
        /// <param name="w">The weight of the point.</param>
        public Point3(double x, double y, double z, double w)
        {
            if (w>0)
            {
                x /= w;
                y /= w;
                z /= w;
            }
            data =(new Vector3(x, y, z), w);
        }

        /// <summary>Creates a <see cref="T:System.Numerics.Point" /> object from a specified normal and the distance along the normal from the origin.</summary>
        /// <param name="position">The point's position vector.</param>
        /// <param name="w">The weight of the point.</param>
        public Point3(Vector3 position, double w)
        {
            if (w>0)
            {
                position /= w;
            }
            data = (position, w);
        }

        /// <summary>Creates a <see cref="T:System.Numerics.Point" /> object from a specified four-dimensional vector.</summary>
        /// <param name="value">A vector whose first three elements describe the position vector, and whose <see cref="F:System.Numerics.Vector4.W" /> defines the weight of the point.</param>
        public Point3(System.Numerics.Vector4 value)
        {
            float w = value.W;
            if (w>0)
            {
                value /= w;
            }
            data = (new Vector3(value.X, value.Y, value.Z), w);
        }

        public static Point3 Empty { get; } = new Point3(Vector3.Zero, 0);
        public static Point3 Origin { get; } = new Point3(Vector3.Zero, 1);
        public static Point3 InfX { get; } = new Point3(Vector3.UnitX, 0);
        public static Point3 InfY { get; } = new Point3(Vector3.UnitY, 0);
        public static Point3 InfZ { get; } = new Point3(Vector3.UnitZ, 0);

        /// <summary>The vector part of the point.</summary>
        public Vector3 Vector => data.position;
        /// <summary>The weight of the point.</summary>
        public double W => data.w;
        public bool IsFinite => data.w*data.w > 0;

        /// <summary>The position vector of the point.</summary>
        public Vector3 Position => Vector/W;

        /// <summary>Creates a new <see cref="T:System.Numerics.Point" /> object whose normal vector is the source point's normal vector normalized.</summary>
        /// <param name="value">The source point.</param>
        /// <returns>The normalized point.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3 Normalize(Point3 value)
        {
            const double tolsq = 1.1920929E-07f;
            double w2 = value.W*value.W;
            if (Math.Abs(w2 - 1f) < tolsq)
            {
                return value;
            }
            return new Point3(value.Vector / value.W, 1f);
        }

        /// <summary>Transforms a normalized point by a Quaternion rotation.</summary>
        /// <param name="point">The normalized point to transform.</param>
        /// <param name="rotation">The Quaternion rotation to apply to the point.</param>
        /// <returns>A new point that results from applying the Quaternion rotation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3 Transform(Point3 point, Quaternion3 rotation) 
            => new Point3(Vector3.Transform(point.Vector, rotation), point.W);

        #region Geometric Algebra

        /// <summary>
        /// Implements the join operator.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="line">The line.</param>
        /// <returns>The plane joining a point and a line.</returns>
        public static Plane3 operator &(Point3 point, Line3 line)
          => Plane3.FromLineAndPoint(line, point);

        /// <summary>
        /// Implements the join operator.
        /// </summary>
        public static Line3 operator &(Point3 point1, Point3 point2)
            => Line3.FromTwoPoints(point1, point2);

        public static double operator *(Point3 point, Plane3 plane)
            => Vector3.Dot(point.Vector, plane.Normal) + point.W*plane.D;

        public static double operator *(Point3 point, Line3 line)
            => line * point;

        #endregion

        #region Formatting
        /// <summary>Returns the string representation of this point object.</summary>
        public override string ToString()
        {
            return $"{{Vector:{Vector} W:{W}}}";
        }
        #endregion

        #region Comparisons
        /// <summary>Returns a value that indicates whether two planes are equal.</summary>
        /// <param name="value1">The first point to compare.</param>
        /// <param name="value2">The second point to compare.</param>
        /// <returns>
        ///   <see langword="true" /> if <paramref name="value1" /> and <paramref name="value2" /> are equal; otherwise, <see langword="false" />.</returns>
        public static bool operator ==(Point3 value1, Point3 value2)
        {
            if (value1.Vector.X == value2.Vector.X && value1.Vector.Y == value2.Vector.Y && value1.Vector.Z == value2.Vector.Z)
            {
                return value1.W == value2.W;
            }
            return false;
        }

        /// <summary>Returns a value that indicates whether two planes are not equal.</summary>
        /// <param name="value1">The first point to compare.</param>
        /// <param name="value2">The second point to compare.</param>
        /// <returns>
        ///   <see langword="true" /> if <paramref name="value1" /> and <paramref name="value2" /> are not equal; otherwise, <see langword="false" />.</returns>
        public static bool operator !=(Point3 value1, Point3 value2)
        {
            if (value1.Vector.X == value2.Vector.X && value1.Vector.Y == value2.Vector.Y && value1.Vector.Z == value2.Vector.Z)
            {
                return value1.W != value2.W;
            }
            return true;
        }

        /// <summary>Returns a value that indicates whether this instance and another point object are equal.</summary>
        /// <param name="other">The other point.</param>
        /// <returns>
        ///   <see langword="true" /> if the two planes are equal; otherwise, <see langword="false" />.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Point3 other)
        {
            return data.Equals(other.data);
        }

        /// <summary>Returns a value that indicates whether this instance and a specified object are equal.</summary>
        /// <param name="obj">The object to compare with the current instance.</param>
        /// <returns>
        ///   <see langword="true" /> if the current instance and <paramref name="obj" /> are equal; otherwise, <see langword="false" />. If <paramref name="obj" /> is <see langword="null" />, the method returns <see langword="false" />.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            if (obj is Point3 point)
            {
                return Equals(point);
            }
            return false;
        }

        /// <summary>Returns the hash code for this instance.</summary>
        /// <returns>The hash code.</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc = (-1521134295)*hc + data.GetHashCode();
                return hc;
            }
        }

        #endregion

        #region Algebra
        public static Point3 Negate(Point3 a)
            => new Point3(
                -a.Vector,
                -a.W);
        public static Point3 Scale(double factor, Point3 a)
            => new Point3(
                factor*a.Vector,
                factor*a.W);
        public static Point3 Add(Point3 a, Point3 b)
            => new Point3(
                a.Vector+b.Vector,
                a.W+b.W);
        public static Point3 Subtract(Point3 a, Point3 b)
            => new Point3(
                a.Vector-b.Vector,
                a.W-b.W);

        public static Point3 operator +(Point3 a, Point3 b) => Add(a, b);
        public static Point3 operator -(Point3 a) => Negate(a);
        public static Point3 operator -(Point3 a, Point3 b) => Subtract(a, b);
        public static Point3 operator *(double f, Point3 a) => Scale(f, a);
        public static Point3 operator *(Point3 a, double f) => Scale(f, a);
        public static Point3 operator /(Point3 a, double d) => Scale(1/d, a);
        #endregion

        #region Geometry        
        /// <summary>Normalize the point and return it as the geometric center.</summary>
        public Point3 Center => Normalize(this);

        public static Point3 FromThreePlanes(Plane3 A, Plane3 B, Plane3 C)
        {
            return A ^ B ^ C;
        }

        public double Distance
            => Vector.Magnitude/Abs(W);

        public static Point3 FromLineAndPlane(Line3 line, Plane3 plane)
            => new Point3(
                Vector3.Cross(plane.Normal, line.Moment) - plane.D * line.Vector, 
                Vector3.Dot(plane.Normal, line.Vector));

        public double DistanceTo(Plane3 plane)
            => Abs(Vector3.Dot(plane.Normal, Vector) + plane.D*W)
            /( W*plane.Normal ).Magnitude;

        public double DistanceTo(Line3 line) => line.DistanceTo(this);

        #endregion

    }
}
