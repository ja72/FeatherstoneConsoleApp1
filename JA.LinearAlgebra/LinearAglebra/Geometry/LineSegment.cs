using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

using static System.Math;

namespace JA.LinearAlgebra.Geometry
{
    public readonly struct LineSegment
    {
        public LineSegment(Vector3 a, Vector3 b)
        {
            A=a;
            B=b;            
        }

        public Vector3 A { get; }
        public Vector3 B { get; }
        public float Length { get => Vector3.Distance(A, B); }

        public Vector3 Direction { get => Vector3.Normalize(B-A); }

        public LineSegment Flip() => new LineSegment(B, A);

        public Vector3 GetPointAlong(float along)
            => Vector3.Lerp(A, B, along);


        public float ProjectPointAlong(Vector3 point)
        {
            float aa = Vector3.Dot(A, A), bb=Vector3.Dot(B,B), ab = Vector3.Dot(A,B);

            return Min(1,Max(0,(aa-ab+Vector3.Dot(B-A, point))/(aa+bb-2*ab)));
        }

        public Vector3 ClosestPointTo(Vector3 point)=> GetPointAlong(ProjectPointAlong(point));

        public float DistanceTo(Vector3 point) => Vector3.Distance(point, ClosestPointTo(point));

        #region Algebra
        public LineSegment Offset(Vector3 delta)
        {
            return new LineSegment(A+delta, B+delta);
        }

        public LineSegment Scale(float factor)
        {
            return new LineSegment(factor*A, factor*B);
        }

        public LineSegment Transform(Matrix4x4 transform)
        {
            return new LineSegment(
                Vector3.Transform(A, transform), 
                Vector3.Transform(B, transform));
        }
        public LineSegment Rotate(Quaternion rotation)
        {
            return new LineSegment(
                Vector3.Transform( A, rotation), 
                Vector3.Transform( B, rotation));
        }
        public LineSegment Rotate(Quaternion rotation, Vector3 pivot)
        {
            return new LineSegment(
                pivot + Vector3.Transform( A - pivot, rotation), 
                pivot + Vector3.Transform( B - pivot, rotation));
        }
        public LineSegment Reflect(Vector3 normal)
        {
            return new LineSegment(
                Vector3.Reflect(A, normal),
                Vector3.Reflect(B, normal));
        }
        public LineSegment Reflect(Vector3 normal, Vector3 origin)
            => new LineSegment(
                origin + Vector3.Reflect(A - origin, normal),
                origin + Vector3.Reflect(B - origin, normal));
        #endregion

        #region Operators
        public static LineSegment operator +(LineSegment a, Vector3 b) => a.Offset(b);
        public static LineSegment operator -(LineSegment a) => a.Scale(-1);
        public static LineSegment operator -(LineSegment a, Vector3 b) => a.Offset(-b);
        public static LineSegment operator *(float a, LineSegment b) => b.Scale(a);
        public static LineSegment operator *(LineSegment a, float b) => a.Scale(b);
        public static LineSegment operator /(LineSegment a, float b) => a.Scale(1 / b);
        #endregion



    }
}
