using System;
using System.Numerics;


namespace JA.LinearAlgebra.Geometry
{
    public readonly struct Circle3 : ICanConvertUnits<Circle3>
    {
        public const float ZERO_TOL = 1e-6f;
        #region Factory
        public Circle3(Vector3 center, Vector3 normal, float radius) : this()
        {
            Center=center;
            Normal= Vector3.Normalize(normal);
            Radius=radius;
        }
        public static Circle3 Circumscribed(Vector3 A, Vector3 B, Vector3 C)
            => Circumscribed(new Triangle3(A, B, C));
        public static Circle3 Circumscribed(Triangle3 triangle)
        {
            var t = triangle.B - triangle.A;
            var u = triangle.C - triangle.A;
            var v = triangle.C - triangle.B;

            var n = Vector3.Cross(t, u);
            var n_sq = Vector3.Dot(n, n);

            if (n_sq < 1e-11f)
            {

                throw new ArgumentException("Three points are colinear.", nameof(triangle));
            }

            var insq2 = 1/(2*n_sq);
            var tt = Vector3.Dot(t, t);
            var uu = Vector3.Dot(u, u);

            var cen = triangle.A + (u*tt*Vector3.Dot(u, v) - t*uu*Vector3.Dot(t, v))*insq2;
            var r = (float)Math.Sqrt(tt*uu*Vector3.Dot(v, v)*insq2*0.5f);

            return new Circle3(cen, n, r);
        }

        #endregion
        #region Peoperties
        public Vector3 Center { get; }
        public Vector3 Normal { get; }
        public float Radius { get; }
        public float Area { get => (float)( Math.PI*Radius*Radius ); }
        #endregion

        #region Geometry
        public bool Contains(Vector3 point)
        {
            float z = Vector3.Dot(Normal, point-Center);
            if (Math.Abs(z)<=1e-11f)
            {
                // Point is on plane
                float d = Vector3.Distance(point, Center);

                return d<=Radius;
            }
            return false;
        }
        public Vector3 ClosestPointTo(Vector3 point)
        {
            float projNorm = Vector3.Dot(Normal, point - Center);
            Vector3 projPoint = point - projNorm * Normal;
            Vector3 delta = projPoint - Center;
            if (delta.Length() == 0)
            {
                // point is at center
                var temp = Vector3.Cross(Vector3.UnitX, Normal);
                if(temp.Length() == 0)
                {
                    temp = Vector3.Cross(Vector3.UnitY, Normal);
                }
                if (temp.Length()==0)
                {
                    temp = Vector3.Cross(Vector3.UnitZ, Normal);
                }
                delta = Vector3.Cross(Normal, temp);
            }
            Vector3 dir = Vector3.Normalize(delta);
            return Center + Radius * dir;
        }

        public float DistanceTo(Vector3 point)
        {
            return Vector3.Distance(ClosestPointTo(point), point);
        } 
        #endregion

        #region Algebra
        public Circle3 Scale(float factor)
        {
            return new Circle3(factor*Center, Normal, factor*Radius);
        }
        public Circle3 Offset(Vector3 offset)
        {
            return new Circle3(Center+offset, Normal, Radius);
        }
        public Circle3 Transform(Matrix4x4 transform)
        {
            return new Circle3(
                Vector3.Transform(Center, transform), 
                Vector3.TransformNormal(Normal, transform), 
                Radius);
        }
        public Circle3 Rotate(Quaternion rotation)
        {
            return new Circle3(
                Vector3.Transform(Center, rotation), 
                Vector3.Transform(Normal, rotation), 
                Radius);
        }
        public Circle3 Rotate(Quaternion rotation, Vector3 pivot)
        {
            return new Circle3(
                pivot + Vector3.Transform(Center - pivot, rotation), 
                Vector3.Transform(Normal, rotation), 
                Radius);
        }
        public static Circle3 operator +(Circle3 Circle, Vector3 offset) => Circle.Offset(offset);
        public static Circle3 operator +(Vector3 offset, Circle3 Circle) => Circle.Offset(offset);
        public static Circle3 operator -(Circle3 Circle, Vector3 offset) => Circle.Offset(-offset);
        public static Circle3 operator -(Vector3 offset, Circle3 Circle) => Circle.Scale(-1f).Offset(offset);

        public static Circle3 operator *(float factor, Circle3 Circle) => Circle.Scale(factor);
        public static Circle3 operator *(Circle3 Circle, float factor) => Circle.Scale(factor);
        public static Circle3 operator /(Circle3 Circle, float divisor) => Circle.Scale(1/divisor);

        #endregion

        public Circle3 ToConvertedFrom(UnitSystem units, UnitSystem target)
        {
            var fl = Unit.Length.Convert(units, target);

            return new Circle3(fl*Center, Normal, fl*Radius);
        }
    }
}
