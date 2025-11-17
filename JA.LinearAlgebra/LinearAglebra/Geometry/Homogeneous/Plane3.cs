using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

using static System.Math;

//using Vector4 = System.Numerics.Vector4;

namespace JA.LinearAlgebra.Geometry.Homogeneous
{
    using Vector3 = Vectors.Vector3;
    using Quaternion3 = Vectors.Quaternion3;

    /// <summary>Represents a plane in three-dimensional space.</summary>
    public struct Plane3 : IEquatable<Plane3>
    {
        private readonly (Vector3 normal, double d) data;

        /// <summary>Creates a <see cref="T:System.Numerics.Plane" /> object from the X, Y, and Z components of its normal, and its distance from the origin on that normal.</summary>
        /// <param name="x">The X component of the normal.</param>
        /// <param name="y">The Y component of the normal.</param>
        /// <param name="z">The Z component of the normal.</param>
        /// <param name="d">The distance of the plane along its normal from the origin.</param>		
        public Plane3(double x, double y, double z, double d)
        {

            var normal = new Vector3(x, y, z);
            double mag = normal.Magnitude;
            if (mag>0)
            {
                double factor = 1/mag;
                normal *= factor;
                d *= factor;
            }
            data =(normal, d);
        }

        /// <summary>Creates a <see cref="T:System.Numerics.Plane" /> object from a specified normal and the distance along the normal from the origin.</summary>
        /// <param name="normal">The plane's normal vector.</param>
        /// <param name="d">The plane's distance from the origin along its normal vector.</param>
        public Plane3(Vector3 normal, double d)
        {
            double mag = normal.Magnitude;
            if (mag>0)
            {
                double factor = 1/mag;
                normal *= factor;
                d *= factor;
            }
            data =(normal, d);
        }

        /// <summary>Creates a <see cref="T:System.Numerics.Plane" /> object from a specified four-dimensional vector.</summary>
        /// <param name="value">A vector whose first three elements describe the normal vector, and whose <see cref="F:System.Numerics.Vector4.W" /> defines the distance along that normal from the origin.</param>
        public Plane3(System.Numerics.Vector4 value)
        {
            var normal = new System.Numerics.Vector3(value.X, value.Y, value.Z);
            var d = value.W; 
            float mag = normal.Length();
            if (mag>0)
            {
                float factor = 1f/mag;
                normal *= factor;
                d *= factor;
            }
            data =(new Vector3(normal.X, normal.Y, normal.Z), value.W);
        }

        public static Plane3 Empty { get; } = new Plane3(Vector3.Zero, 0);
        public static Plane3 Infinity { get; } = new Plane3(Vector3.Zero, 1);
        public static Plane3 AlongYZ { get; } = new Plane3(Vector3.UnitX, 0);
        public static Plane3 AlongZX { get; } = new Plane3(Vector3.UnitY, 0);
        public static Plane3 AlongXY { get; } = new Plane3(Vector3.UnitZ, 0);

        /// <summary>The normal vector of the plane.</summary>
        public Vector3 Normal => data.normal;
        /// <summary>The distance of the plane along its normal from the origin.</summary>
        public double D => data.d;

        public bool IsFinite => data.normal.MagnitudeSquared>0;

        /// <summary>The position vector of the point on the plane closest to the origin.</summary>
        public Vector3 Position => data.d * data.normal/data.normal.MagnitudeSquared;

        /// <summary>Creates a <see cref="T:System.Numerics.Plane" /> object that contains three specified points.</summary>
        /// <param name="point1">The first point defining the plane.</param>
        /// <param name="point2">The second point defining the plane.</param>
        /// <param name="point3">The third point defining the plane.</param>
        /// <returns>The plane containing the three points.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane3 CreateFromVertices(Vector3 point1, Vector3 point2, Vector3 point3)
        {
            Vector3 vector = point2 - point1;
            Vector3 vector2 = point3 - point1;
            Vector3 value = Vector3.Cross(vector, vector2);
            Vector3 vector3 = Vector3.Normalized(value);
            double d = 0 - Vector3.Dot(vector3, point1);
            return new Plane3(vector3, d);
        }

        /// <summary>Creates a new <see cref="T:JA.Sng.Plane3" /> object whose normal vector is the source plane's normal vector normalized.</summary>
        /// <param name="value">The source plane.</param>
        /// <returns>The normalized plane.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane3 Normalize(Plane3 value)
        {
            double num = value.Normal.MagnitudeSquared;
            double num2 = (double)Math.Sqrt(num);
            return new Plane3(value.Normal / num2, value.D / num2);
        }

        /// <summary>Transforms a normalized plane by a Quaternion rotation.</summary>
        /// <param name="plane">The normalized plane to transform.</param>
        /// <param name="rotation">The Quaternion rotation to apply to the plane.</param>
        /// <returns>A new plane that results from applying the Quaternion rotation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane3 Transform(Plane3 plane, Quaternion3 rotation) 
            => new Plane3(Vector3.Transform(plane.Normal, rotation), plane.D);

        /// <summary>Calculates the dot product of a plane and a 4-dimensional vector.</summary>
        /// <param name="plane">The plane.</param>
        /// <param name="value">The four-dimensional vector.</param>
        /// <returns>The dot product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Plane3 plane, System.Numerics.Vector4 value)
        {
            return plane.Normal.X * value.X + plane.Normal.Y * value.Y + plane.Normal.Z * value.Z + plane.D * value.W;
        }

        /// <summary>Returns the dot product of a specified three-dimensional vector and the normal vector of this plane plus the distance (<see cref="F:System.Numerics.Plane.D" />) value of the plane.</summary>
        /// <param name="plane">The plane.</param>
        /// <param name="value">The 3-dimensional vector.</param>
        /// <returns>The dot product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double DotCoordinate(Plane3 plane, Vector3 value)
        {
            return Vector3.Dot(plane.Normal, value) + plane.D;
        }

        /// <summary>Returns the dot product of a specified three-dimensional vector and the <see cref="F:System.Numerics.Plane.Normal" /> vector of this plane.</summary>
        /// <param name="plane">The plane.</param>
        /// <param name="value">The three-dimensional vector.</param>
        /// <returns>The dot product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double DotNormal(Plane3 plane, Vector3 value)
        {
            return Vector3.Dot(plane.Normal, value);
        }

        #region Formatting
        /// <summary>Returns the string representation of this plane object.</summary>
        public override string ToString()
        {
            return $"{{Normal:{Normal} D:{D}}}";
        }
        #endregion

        #region Comparisons

        /// <summary>Returns a value that indicates whether two planes are equal.</summary>
        /// <param name="value1">The first plane to compare.</param>
        /// <param name="value2">The second plane to compare.</param>
        /// <returns>
        ///   <see langword="true" /> if <paramref name="value1" /> and <paramref name="value2" /> are equal; otherwise, <see langword="false" />.</returns>
        public static bool operator ==(Plane3 value1, Plane3 value2)
        {
            if (value1.Normal.X == value2.Normal.X && value1.Normal.Y == value2.Normal.Y && value1.Normal.Z == value2.Normal.Z)
            {
                return value1.D == value2.D;
            }
            return false;
        }

        /// <summary>Returns a value that indicates whether two planes are not equal.</summary>
        /// <param name="value1">The first plane to compare.</param>
        /// <param name="value2">The second plane to compare.</param>
        /// <returns>
        ///   <see langword="true" /> if <paramref name="value1" /> and <paramref name="value2" /> are not equal; otherwise, <see langword="false" />.</returns>
        public static bool operator !=(Plane3 value1, Plane3 value2)
        {
            if (value1.Normal.X == value2.Normal.X && value1.Normal.Y == value2.Normal.Y && value1.Normal.Z == value2.Normal.Z)
            {
                return value1.D != value2.D;
            }
            return true;
        }

        /// <summary>Returns a value that indicates whether this instance and another plane object are equal.</summary>
        /// <param name="other">The other plane.</param>
        /// <returns>
        ///   <see langword="true" /> if the two planes are equal; otherwise, <see langword="false" />.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Plane3 other)
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
            if (obj is Plane3 item)
            {
                return Equals(item);
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
        public static Plane3 Negate(Plane3 a)
            => new Plane3(
                -a.Normal,
                -a.D);
        public static Plane3 Scale(double factor, Plane3 a)
            => new Plane3(
                factor*a.Normal,
                factor*a.D);
        public static Plane3 Add(Plane3 a, Plane3 b)
            => new Plane3(
                a.Normal+b.Normal,
                a.D+b.D);
        public static Plane3 Subtract(Plane3 a, Plane3 b)
            => new Plane3(
                a.Normal-b.Normal,
                a.D-b.D);

        public static Plane3 operator +(Plane3 a, Plane3 b) => Add(a, b);
        public static Plane3 operator -(Plane3 a) => Negate(a);
        public static Plane3 operator -(Plane3 a, Plane3 b) => Subtract(a, b);
        public static Plane3 operator *(double f, Plane3 a) => Scale(f, a);
        public static Plane3 operator *(Plane3 a, double f) => Scale(f, a);
        public static Plane3 operator /(Plane3 a, double d) => Scale(1/d, a);
        #endregion

        #region Geometry
        public Point3 Center 
            => new Point3(
                -data.d*data.normal, 
                data.normal.MagnitudeSquared);

        public double Distance
            => Abs(D)/Normal.Magnitude;

        public static Plane3 FromThreePoints(Point3 A, Point3 B, Point3 C)
        {
            return A & B & C;
        }

        public static Plane3 FromLineAndDirection(Line3 line, Vector3 direction)
            => new Plane3(
                Vector3.Cross(line.Vector, direction),
                -Vector3.Dot(line.Moment, direction));

        public static Plane3 FromLineAndPoint(Line3 line, Point3 point)
            => new Plane3(
                Vector3.Cross(point.Vector, line.Vector) - point.W * line.Moment,
                Vector3.Dot(point.Vector, line.Moment));

        public static Plane3 FromLineThroughOrigin(Line3 line)
            => new Plane3(line.Moment, 0);

        public static Plane3 FromLineAwayFromOrigin(Line3 line)
            => new Plane3(
                Vector3.Cross(line.Moment, line.Vector), 
                line.Moment.MagnitudeSquared);

        public static Plane3 FromPointAwayFromOrigin(Point3 point)
            => new Plane3(
                -point.W*point.Vector, point.W*point.W);

        public double DistanceTo(Point3 point)
            => Abs(Vector3.Dot(Normal, point.Vector) + D*point.W)
            /(point.W*Normal).Magnitude;

        public double DistanceTo(Line3 line) => line.DistanceTo(this);

        #endregion

        #region Geometric Algebra        
        /// <summary>
        /// Implements the meet operator.
        /// </summary>
        /// <param name="plane">The plane.</param>
        /// <param name="line">The line.</param>
        /// <returns>The point where the line meets the plane.</returns>
        public static Point3 operator ^(Plane3 plane, Line3 line)
            => Point3.FromLineAndPlane(line, plane);

        /// <summary>
        /// Implements the meet operator.
        /// </summary>
        public static Line3 operator ^(Plane3 plane1, Plane3 plane2)
            => Line3.FromTwoPlanes(plane1, plane2);

        public static double operator *(Plane3 plane, Point3 point)
            => Vector3.Dot(point.Vector, plane.Normal) + point.W*plane.D;

        public static double operator *(Plane3 plane, Line3 line)
            => line * plane;


        #endregion

    }
}
