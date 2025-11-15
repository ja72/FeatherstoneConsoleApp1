using System;
using System.Drawing;
using System.Numerics;
using System.Security.Cryptography;
using System.Linq;
using JA.LinearAlgebra.Geometry;

namespace JA.LinearAlgebra.Geometry
{
    public readonly struct Triangle3 : ICanConvertUnits<Triangle3>
    {
        public const float ZERO_TOL = 1e-6f;
        public Triangle3(params Vector3[] nodes) : this()
        {
            if (nodes.Length != 3)
            {
                throw new ArgumentException("Triangle must have exactly three nodes.", nameof(nodes));
            }
            A = nodes[0];
            B = nodes[1];
            C = nodes[2];
        }
        public Triangle3(Vector3 a, Vector3 b, Vector3 c) : this()
        {
            A = a;
            B = b;
            C = c;
        }

        public Vector3 A { get; }
        public Vector3 B { get; }
        public Vector3 C { get; }
        public Vector3 Center { get => (A + B + C) / 3; }
        public Vector3 Normal { get => Vector3.Normalize(AreaVector); }
        public float Area { get => AreaVector.Length(); }

        public LineSegment SideAB { get => new LineSegment(A, B); }
        public LineSegment SideBC { get => new LineSegment(B, C); }
        public LineSegment SideCA { get => new LineSegment(C, A); }

        private Vector3 AreaVector
            => (Vector3.Cross(A, B)+ Vector3.Cross(B, C)+ Vector3.Cross(C, A)) / 2;

        public Vector3 ClosestPointTo(Vector3 point)
        {
            float d_A = Vector3.Distance(A, point);
            float d_B = Vector3.Distance(B, point);
            float d_C = Vector3.Distance(C, point);

            Vector3 p_AB = SideAB.ClosestPointTo(point);
            Vector3 p_BC = SideBC.ClosestPointTo(point);
            Vector3 p_CA = SideCA.ClosestPointTo(point);

            float d_AB = Vector3.Distance(p_AB, point);
            float d_BC = Vector3.Distance(p_BC, point);
            float d_CA = Vector3.Distance(p_CA, point);

            float d = Math.Min(d_A, Math.Min(d_B, d_C));
            d = Math.Min(d, Math.Min(d_AB, Math.Min(d_BC, d_CA)));

            if (d==d_A) return A;
            if (d==d_B) return B;
            if (d==d_C) return C;
            if (d==d_CA) return p_CA;
            if (d==d_BC) return p_BC;
            if (d==d_AB) return p_AB;

            throw new ArgumentException("Invalid Point", nameof(point));
        }

        public float DistanceTo(Vector3 point)
        {
            return Vector3.Distance(ClosestPointTo(point), point);
        }

        public Vector3 ProjectOnPlane(Vector3 point)
        {
            var n = Normal;
            var d = Vector3.Dot(n, point-A);
            return point - n*d;
        }
        public bool Barycentric(Vector3 P, out (float w_A, float w_B, float w_C) coord)
        {
            Vector3 n = Vector3.Cross(A, B)+ Vector3.Cross(B, C)+ Vector3.Cross(C, A);
            float w_A = Vector3.Dot(n, Vector3.Cross(P, B)+ Vector3.Cross(B, C)+ Vector3.Cross(C, P));
            float w_B = Vector3.Dot(n, Vector3.Cross(A, P)+ Vector3.Cross(P, C)+ Vector3.Cross(C, A));
            float w_C = Vector3.Dot(n, Vector3.Cross(A, B)+ Vector3.Cross(B, P)+ Vector3.Cross(P, A));
            float sum = w_A + w_B + w_C;
            if (sum != 0)
            {
                coord = (w_A/sum, w_B/sum, w_C/sum);
                return true;
            }
            coord  = (w_A, w_B, w_C);
            return false;
        }

        public Vector3 GetPoint(float w_A, float w_B, float w_C)
            => GetPoint((w_A, w_B, w_C));
        public Vector3 GetPoint((float w_A, float w_B, float w_C) coord)
        {
            return coord.w_A*A + coord.w_B*B + coord.w_C*C;
        }

        public bool Contains(Vector3 point, float distanceTolerance = ZERO_TOL)
        {
            if (Math.Abs(Vector3.Dot(Normal, point-A))<= distanceTolerance)
            {
                Barycentric(point, out var coord);
                return coord.w_A>=0 && coord.w_A<=1
                    && coord.w_B>=0 && coord.w_B<=1
                    && coord.w_C>=0 && coord.w_C<=1;
            }
            return false;
        }

        #region Geometry
        public Vector3[] GetNodes() => new Vector3[] { A, B, C };
        public Triangle3 Scale(float factor)
        {
            return new Triangle3(GetNodes().Select((n) => factor*n).ToArray());
        }
        public Triangle3 Offset(Vector3 offset)
        {
            return new Triangle3(GetNodes().Select((n) => n + offset).ToArray());
        }
        public Triangle3 Transform(Matrix4x4 transform, bool inverse = false)
        {
            if (inverse)
            {
                Matrix4x4.Invert(transform, out var inv);
                return new Triangle3(GetNodes().Select((n) => Vector3.Transform(n, inv)).ToArray());
            }
            return new Triangle3(GetNodes().Select((n) => Vector3.Transform(n, transform)).ToArray());
        }
        public Triangle3 Rotate(Quaternion rotation)
        {
            return new Triangle3(
                Vector3.Transform( A, rotation), 
                Vector3.Transform( B, rotation),
                Vector3.Transform( C, rotation));
        }
        public Triangle3 Rotate(Quaternion rotation, Vector3 pivot)
        {
            return new Triangle3(
                pivot + Vector3.Transform( A - pivot, rotation), 
                pivot + Vector3.Transform( B - pivot, rotation),
                pivot + Vector3.Transform( C - pivot, rotation));
        }
        public Triangle3 Reflect(Plane plane)
            => Reflect(plane.Normal, -plane.Normal * plane.D);
        public Triangle3 Reflect(Vector3 normal, Vector3 origin)
            => new Triangle3(
                origin + Vector3.Reflect(A - origin, normal),
                origin + Vector3.Reflect(B - origin, normal),
                origin + Vector3.Reflect(C - origin, normal));

        #endregion

        #region Algebra
        #endregion

        #region Operators
        public static Triangle3 operator +(Triangle3 triangle, Vector3 delta) => triangle.Offset(delta);
        public static Triangle3 operator +(Vector3 delta, Triangle3 triangle) => triangle.Offset(delta);
        public static Triangle3 operator -(Triangle3 triangle, Vector3 delta) => triangle.Offset(-delta);
        public static Triangle3 operator -(Triangle3 triangle) => triangle.Scale(-1);
        public static Triangle3 operator -(Vector3 delta, Triangle3 triangle) => triangle.Scale(-1f).Offset(delta);

        public static Triangle3 operator *(float factor, Triangle3 triangle) => triangle.Scale(factor);
        public static Triangle3 operator *(Triangle3 triangle, float factor) => triangle.Scale(factor);
        public static Triangle3 operator /(Triangle3 triangle, float divisor) => triangle.Scale(1/divisor);

        #endregion

        public Triangle3 ToConvertedFrom(UnitSystem units, UnitSystem target)
        {
            var f_len = Unit.Length.Convert(units, target);
            return new Triangle3(
                f_len * A,
                f_len * B,
                f_len * C);
        }
    }
}
