using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace JA.LinearAlgebra.Geometry
{
    public readonly struct Polygon3 
    {
        public Polygon3(params Vector3[] nodes)
        {
            Nodes=nodes;

            if (nodes.Length>=3)
            {
                Normal = new Triangle3(nodes[0], nodes[1], nodes[2]).Normal;
                IsConvex = nodes.Length==3 || CheckConvex(nodes);
            }
            else
            {
                IsConvex = true;
                Normal = Vector3.Zero;
            }
        }

        public Vector3[] Nodes { get; }
        public bool IsConvex { get; }

        public Vector3 Center
        {
            get
            {
                int count = Nodes.Length;
                return Nodes.Aggregate(Vector3.Zero, (cen, node) => cen+node/count);
            }
        }

        public Vector3 Normal { get; }

        public Triangle3[] GetTriangles()
        {
            if (Nodes.Length<2)
            {
                return Array.Empty<Triangle3>();
            }
            if (Nodes.Length==2)
            {
                return new Triangle3[] { new Triangle3( Nodes[0], Nodes[1], Nodes[1]) };
            }
            if (Nodes.Length==3)
            {
                return new Triangle3[] { new Triangle3( Nodes[0], Nodes[1], Nodes[2]) };
            }
            if (Nodes.Length==4)
            {
                return new Triangle3[] { 
                    new Triangle3( Nodes[0], Nodes[1], Nodes[2]),
                    new Triangle3( Nodes[2], Nodes[3], Nodes[0]),
                    };
            }
            var list = new List<Triangle3>();
            if (IsConvex)
            {
                // use fan method 
                var P = Center;
                for (int i = 0; i < Nodes.Length; i++)
                {
                    int j = (i+1)%Nodes.Length;
                    list.Add(new Triangle3(P, Nodes[i], Nodes[j]));
                }
            }
            else
            {
                throw new NotImplementedException();
            }
            return list.ToArray();
        }

        static bool CheckConvex(Vector3[] nodes)
        {
            for (int i = 0; i < nodes.Length; i++)
            {
                int j = (i+1)%nodes.Length;
                int k = (i+2)%nodes.Length;

                var trig = new Triangle3(nodes[i], nodes[j], nodes[k]);
                for (int r = 3; r < nodes.Length; r++)
                {
                    var P = nodes[(r+i)%nodes.Length];
                    if (trig.Contains(P))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        public Polygon3 Scale(float factor)
        {
            return new Polygon3(Nodes.Select((n) => factor*n).ToArray());
        }
        public Polygon3 Offset(Vector3 offset)
        {
            return new Polygon3(Nodes.Select((n) => n + offset).ToArray());
        }
        public Polygon3 Transform(Matrix4x4 transform, bool inverse = false)
        {
            if (inverse)
            {
                Matrix4x4.Invert(transform, out var inv);
                return new Polygon3(Nodes.Select((n) => Vector3.Transform(n, inv)).ToArray());
            }
            return new Polygon3(Nodes.Select((n) => Vector3.Transform(n, transform)).ToArray());
        }
        public Polygon3 Rotate(Quaternion rotation)
        {
            return new Polygon3(Nodes.Select((n) => Vector3.Transform(n, rotation)).ToArray());
        }
        public Polygon3 Rotate(Quaternion rotation, Vector3 pivot)
        {
            return new Polygon3(Nodes.Select((n) => pivot + Vector3.Transform(n - pivot, rotation)).ToArray());
        }
        public static Polygon3 operator +(Polygon3 polygon, Vector3 offset) => polygon.Offset(offset);
        public static Polygon3 operator +(Vector3 offset, Polygon3 polygon) => polygon.Offset(offset);
        public static Polygon3 operator -(Polygon3 polygon, Vector3 offset) => polygon.Offset(-offset);
        public static Polygon3 operator -(Vector3 offset, Polygon3 polygon) => polygon.Scale(-1f).Offset(offset);

        public static Polygon3 operator *(float factor, Polygon3 polygon) => polygon.Scale(factor);
        public static Polygon3 operator *(Polygon3 polygon, float factor) => polygon.Scale(factor);
        public static Polygon3 operator /(Polygon3 polygon, float divisor) => polygon.Scale(1/divisor);

    }
}
