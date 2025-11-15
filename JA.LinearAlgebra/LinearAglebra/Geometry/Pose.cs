using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JA.LinearAlgebra.Geometry
{
    public class Pose
    {
        public Pose(Vector3 position, Quaternion orientation)
        {
            Position=position;
            Orientation=orientation;
        }

        public static Pose Origin { get; } = new Pose(Vector3.Zero, Quaternion.Identity);
        public static implicit operator Pose(Vector3 position)=>new Pose(position, Quaternion.Identity);
        public static implicit operator Pose(Quaternion quaternion)=>new Pose(Vector3.Zero, quaternion);
        public static explicit operator Vector3(Pose pose)=>pose.Position;
        public static explicit operator Quaternion(Pose pose)=>pose.Orientation;
        public Vector3 Position { get; }
        public Quaternion Orientation { get; }
        public Matrix4x4 ToRotation() => Matrix4x4.CreateFromQuaternion(Orientation);
        public Matrix4x4 ToTranslation() => Matrix4x4.CreateTranslation(Position);
        public Matrix4x4 ToTransformation() => Matrix4x4.CreateFromQuaternion(Orientation) * Matrix4x4.CreateTranslation(Position);
        public Vector3 TransformedPoint(Vector3 point)
        {
            Vector3 rotatedPoint = Vector3.Transform(point, Orientation);
            return Position + rotatedPoint;
        }
        public Vector3 TransformedNormal(Vector3 point)
        {
            Vector3 rotatedPoint = Vector3.Transform(point, Orientation);
            return rotatedPoint;
        }
        public Vector3[] TransformedPoints(Vector3[] points)
        {
            Matrix4x4 transformation = ToTransformation();
            Vector3[] result = new Vector3[points.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i] = Vector3.Transform(points[i], transformation);
            }
            return result;
        }
        public Vector3[] TransformedNormals(Vector3[] points)
        {
            Matrix4x4 transformation = ToRotation();
            Vector3[] result = new Vector3[points.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i] = Vector3.TransformNormal(points[i], transformation);
            }
            return result;
        }
        public void TransformPoints(Vector3[] points)
        {
            Matrix4x4 transformation = ToTransformation();
            for (int i = 0; i<points.Length; i++)
            {
                points[i] = Vector3.Transform(points[i], transformation);
            }
        }
        public void TransformNormals(Vector3[] points)
        {
            Matrix4x4 transformation = ToRotation();
            for (int i = 0; i<points.Length; i++)
            {
                points[i] = Vector3.TransformNormal(points[i], transformation);
            }
        }
    }
}
