using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Xml;

using static System.Math;

namespace JA.LinearAlgebra.Geometry.Homogeneous
{
    using Quaternion3 = Vectors.Quaternion3;
    using Vector3 = Vectors.Vector3;

    public struct Line3 : IEquatable<Line3>
    {
        private readonly (Vector3 direction, Vector3 moment) data;

        public Line3(Vector3 direction, Vector3 moment)
        {
            double mag = direction.Magnitude;
            if(mag > 0)
            {
                double factor = 1/mag;
                direction *= factor;
                moment *= factor;
            }
            this.data=(direction, moment);
        }

        public static Line3 Empty { get; } = new Line3(Vector3.Zero, Vector3.Zero);
        public static Line3 AlongX { get; } = new Line3(Vector3.UnitX, Vector3.Zero);
        public static Line3 AlongY { get; } = new Line3(Vector3.UnitY, Vector3.Zero);
        public static Line3 AlongZ { get; } = new Line3(Vector3.UnitZ, Vector3.Zero);
        public static Line3 AboutYZ { get; } = new Line3(Vector3.Zero, Vector3.UnitX);
        public static Line3 AboutZX { get; } = new Line3(Vector3.Zero, Vector3.UnitY);
        public static Line3 AboutXY { get; } = new Line3(Vector3.Zero, Vector3.UnitZ);

        public Vector3 Vector => data.direction;
        public Vector3 Moment => data.moment;
        public bool IsFinite => Vector.MagnitudeSquared > 0;

        public Vector3 Direction => Vector3.Normalized(Vector);

        /// <summary>Creates a new <see cref="Sng.Line3" /> object whose normal vector is the source plane's normal vector normalized.</summary>
        /// <param name="value">The source line.</param>
        /// <returns>The normalized line.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Line3 Normalize(Line3 value)
        {
            var num = value.Vector.MagnitudeSquared;
            if (num>0)
            {
                var num2 = Math.Sqrt(num);
                return new Line3(value.Vector / num2, value.Moment / num2);
            }
            var mom = value.Moment.MagnitudeSquared;
            if(mom>0)
            {
                var mom2 = Math.Sqrt(mom);
                return new Line3(value.Vector / mom2, value.Moment / mom2);
            }
            return value;
        }

        /// <summary>Transforms a normalized line by a Quaternion rotation.</summary>
        /// <param name="line">The normalized line to transform.</param>
        /// <param name="rotation">The Quaternion rotation to apply to the line.</param>
        /// <returns>A new plane that results from applying the Quaternion rotation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Line3 Transform(Line3 line, Quaternion3 rotation, bool inverse = false)
        => new Line3(
                Vector3.Transform(line.Vector, rotation, inverse),
                Vector3.Transform(line.Moment, rotation, inverse));        

        #region Formatting
        /// <summary>Returns the string representation of this point object.</summary>
        public override string ToString()
        {
            return $"{{Vector:{Vector} Moment:{Moment}}}";
        }

        #endregion

        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Line)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Line3 item)
            {
                return Equals(item);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="Line3"/> classes
        /// </summary>
        /// <returns>True if equal</returns>
        public bool Equals(Line3 other)
        {
            return data.Equals(other.data);
        }
        /// <summary>
        /// Calculates the hash code for the <see cref="Line3"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc = (-1521134295)*hc + data.GetHashCode();
                return hc;
            }
        }
        public static bool operator ==(Line3 target, Line3 other)
        {
            return target.Equals(other);
        }
        public static bool operator !=(Line3 target, Line3 other)
        {
            return !target.Equals(other);
        }

        #endregion

        #region Algebra
        public static Line3 Negate(Line3 a)
            => new Line3(
                -a.Vector,
                -a.Moment);
        public static Line3 Scale(double factor, Line3 a)
            => new Line3(
                factor*a.Vector,
                factor*a.Moment);
        public static Line3 Add(Line3 a, Line3 b)
            => new Line3(
                a.Vector+b.Vector,
                a.Moment+b.Moment);
        public static Line3 Subtract(Line3 a, Line3 b)
            => new Line3(
                a.Vector-b.Vector,
                a.Moment-b.Moment);

        public static Line3 operator +(Line3 a, Line3 b) => Add(a, b);
        public static Line3 operator -(Line3 a) => Negate(a);
        public static Line3 operator -(Line3 a, Line3 b) => Subtract(a, b);
        public static Line3 operator *(double f, Line3 a) => Scale(f, a);
        public static Line3 operator *(Line3 a, double f) => Scale(f, a);
        public static Line3 operator /(Line3 a, double d) => Scale(1/d, a);
        #endregion

        #region Geometry
        public Point3 Center 
            => new Point3(
                Vector3.Cross(Vector, Moment), 
                Vector.MagnitudeSquared);

        public double Distance
            => Moment.Magnitude/ Vector.Magnitude;

        public static Line3 FromPointAndDirection(Point3 point, Vector3 direction)
            => new Line3(
                direction * point.W, Vector3.Cross(point.Vector, 
                direction));

        public static Line3 FromTwoPoints(Point3 point1, Point3 point2)
            => new Line3(
                point2.Vector * point1.W - point1.Vector * point2.W, 
                Vector3.Cross(point1.Vector, point2.Vector));

        public static Line3 FromOriginTo(Point3 point)
            => FromTwoPoints(Point3.Origin, point);

        public static Line3 FromTwoPlanes(Plane3 plane1, Plane3 plane2)
            => new Line3(
                Vector3.Cross(plane1.Normal, plane2.Normal),
                plane1.D*plane2.Normal - plane2.D * plane1.Normal);

        public double DistanceTo(Point3 point)
            => (Vector3.Cross(Vector, point.Vector)+point.W*Moment).Magnitude
            /(point.W*Vector).Magnitude;

        public double DistanceTo(Plane3 plane)
            => ( Vector3.Cross(Vector, plane.Normal) + plane.D * Moment ).Magnitude
            /(plane.D*Vector).Magnitude;

        public double DistanceTo(Line3 line)
            => Abs(Vector3.Dot(Vector, line.Moment) + Vector3.Dot(Moment, line.Vector))
            /Vector3.Cross(Vector, line.Vector).Magnitude;

        #endregion

        #region Geometric Algebra

        /// <summary>
        /// Implements the meet operator.
        /// </summary>
        /// <param name="line">The line.</param>
        /// <param name="plane">The plane.</param>
        /// <returns>The point where the line meets the plane.</returns>
        public static Point3 operator ^(Line3 line, Plane3 plane)
            => -(plane ^ line);

        /// <summary>
        /// Implements the join operator.
        /// </summary>
        /// <param name="line">The line.</param>
        /// <param name="point">The point.</param>
        /// <returns>The plane where the line joins the point.</returns>
        public static Plane3 operator &(Line3 line, Point3 point)
            => -(point & line);

        public static Plane3 operator &(Line3 line, Vector3 direction)
            => Plane3.FromLineAndDirection(line, direction);

        public static double operator * (Line3 line, Point3 point)
            => (Vector3.Cross(line.Vector, point.Vector)+point.W*line.Moment).Magnitude;

        public static double operator * (Line3 line, Plane3 plane)
            => (Vector3.Cross(line.Vector, plane.Normal)+plane.D*line.Moment).Magnitude;

        #endregion

    }
}
