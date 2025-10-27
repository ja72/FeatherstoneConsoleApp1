using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.ComponentModel;
using System.Numerics;

using static System.Math;

namespace JA.Geometry
{    

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Element
    {
        public Element(Color color, params int[] face)
        {
            Color = color;
            Face = face;
        }

        public int[] Face { get; }
        public Color Color { get; set; }

        public void Flip()
        {
            Array.Copy(Face.Reverse().ToArray(), Face, Face.Length);
        }
        public override string ToString()
        {
            return $"Face(Color={Color}, Nodes=[{string.Join(",", Face)}])";
        }
    }
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Mesh : ICanChangeUnits<Mesh>
    {
        readonly List<Vector3> nodeList;
        readonly List<Element> elementList;

        public Mesh(UnitSystem units)
        {
            this.Units = units;
            nodeList = new List<Vector3>();
            elementList = new List<Element>();
        }

        public Mesh(Mesh copy)
        {
            Units = copy.Units;
            nodeList = new List<Vector3>(copy.nodeList);
            elementList = new List<Element>(copy.elementList);
        }

        Mesh(UnitSystem units, IEnumerable<Vector3> nodeList, IEnumerable<Element> elementList) : this(units)
        {
            this.Units = units;
            this.nodeList= new List<Vector3>(nodeList);
            this.elementList= new List<Element>(elementList);
        }

        public UnitSystem Units { get; }
        public Mesh ConvertTo(UnitSystem target)
        {
            if (Units != target)
            {
                float fl = Unit.Length.Convert(Units, target);
                return new Mesh(target,
                    nodeList.Select((n)=>fl*n),
                    elementList);
            }
            return this;
        }

        [Browsable(false)] public List<Vector3> NodeList => nodeList;
        [Browsable(false)] public List<Element> ElementList => elementList;

        public Vector3[] Nodes { get => nodeList.ToArray(); }
        [Browsable(false)]
        public Element[] Elements { get => elementList.ToArray(); }

        public void ApplyTranform(Vector3 origin)
        {
            for (int i = 0; i < nodeList.Count; i++)
            {
                nodeList[i] = origin + nodeList[i];
            }
        }
        public void ReverseTranform(Vector3 origin)
        {
            for (int i = 0; i < nodeList.Count; i++)
            {
                nodeList[i] = nodeList[i] = origin;
            }
        }
        public void Tesselate(int level = 1)
        {
            for (int c = 0; c < level; c++)
            {
                for (int i = elementList.Count - 1; i >= 0; i--)
                {
                    TesselateFace(i);
                }
            }
        }
        public void TesselateFace(int index)
        {
            var face = elementList[index].Face;
            var color = elementList[index].Color;
            var nodes = face.Select(ni => nodeList[ni]).ToArray();
            if (nodes.Length == 4)
            {
                elementList.RemoveAt(index);
                // assume quadrelateral and split all four sides.
                // [f2]---[ 1]---[f1]
                // |    C   |   B   |
                // [ 2]---[ 4]---[ 0]
                // |    D   |   A   |
                // [f3]---[ 3]---[f0]
                int k = nodeList.Count;
                for (int i = 0; i < nodes.Length; i++)
                {
                    int j = (i+1) % nodes.Length;
                    nodeList.Add((nodes[i] + nodes[j])/2);
                }
                var center = nodes.Aggregate(Vector3.Zero, (a, b) => a + b) / nodes.Length;
                nodeList.Add(center);

                AddFace(color, face[0], k+0, k+4, k+3); // A
                AddFace(color, k+0, face[1], k+1, k+4); // B
                AddFace(color, k+4, k+1, face[2], k+2); // C
                AddFace(color, k+3, k+4, k+2, face[3]); // D
            }
            else if (nodes.Length >= 3)
            {
                elementList.RemoveAt(index);
                int k = nodeList.Count;
                var center = nodes.Aggregate(Vector3.Zero, (a, b) => a + b) / nodes.Length;
                nodeList.Add(center);

                for (int i = 0; i < nodes.Length; i++)
                {
                    int j = (i+1) % nodes.Length;

                    AddFace(color, k, face[i], face[j]);
                }
            }
        }

        /// <summary>
        /// Gets the local coordinates of the nodes of a face.
        /// </summary>
        /// <param name="faceIndex">The face index.</param>
        public Vector3[] GetFaceNodes(int faceIndex)
        {
            return elementList[faceIndex].Face.Select(ni => nodeList[ni]).ToArray();
        }
        public Triangle[] GetTriangles(int faceIndex) => GetPolygon(faceIndex).GetTriangles();
        public Polygon GetPolygon(int index) => new Polygon(GetFaceNodes(index));

        /// <summary>
        /// Gets the normal vector of a face, applying the mesh transformation.
        /// </summary>
        /// <param name="faceIndex">The face index.</param>
        public Vector3[] GetNormals(int faceIndex)
        {
            return GetNormals(GetFaceNodes(faceIndex));
        }
        /// <summary>
        /// Gets the normal vectors of a face at each node, applying the mesh transformation.
        /// </summary>
        /// <param name="nodes">The nodes of the face.</param>
        public Vector3[] GetNormals(Vector3[] nodes)
        {
            var normals = new Vector3[nodes.Length];
            for (int i = 0; i < nodes.Length; i++)
            {
                int j = (i + 1) % nodes.Length;
                int k = (i - 1 + nodes.Length) % nodes.Length;

                Vector3 A = nodes[i], B = nodes[j], C = nodes[k];

                normals[i] = Vector3.Normalize(
                    Vector3.Cross(A, B)
                    + Vector3.Cross(B, C)
                    + Vector3.Cross(C, A)
                    );
            }

            return normals;
        }
        /// <summary>
        /// Gets the average normal vector of a face, applying the mesh transformation.
        /// </summary>
        /// <param name="nodes">The nodes of the face.</param>
        public Vector3 GetNormal(Vector3[] nodes)
        {
            var list = GetNormals(nodes);
            Vector3 n = Vector3.Zero;
            for (int i = 0; i < list.Length; i++)
            {
                n += list[i];
            }
            return Vector3.Normalize(n);
        }

        /// <summary>
        /// Adds the face from a list of nodes.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="nodes">The local face nodes.</param>
        public void AddFace(Color color, params Vector3[] nodes)
        {
            var nodeIndex = new int[nodes.Length];
            for (int i = 0; i < nodes.Length; i++)
            {
                if (nodeList.Contains(nodes[i]))
                {
                    nodeIndex[i] = nodeList.IndexOf(nodes[i]);
                }
                else
                {
                    nodeIndex[i] = nodeList.Count;
                    nodeList.Add(nodes[i]);
                }
            }
            elementList.Add(new Element(color, nodeIndex));
        }
        public void AddFace(Color color, params int[] nodeIndex)
        {
            elementList.Add(new Element(color, nodeIndex));
        }
        /// <summary>
        /// Adds a square panel as a face.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="center">The center of the panel.</param>
        /// <param name="x_axis">The x-axis defining the direction of length.</param>
        /// <param name="length">The panel length.</param>
        /// <param name="width">The panel width.</param>
        public void AddPanel(Color color,
            Vector3 center,
            Vector3 x_axis,
            float length,
            float width)
        {
            x_axis = Vector3.Normalize(x_axis);
            Vector3 z_axis = center == Vector3.Zero ? Vector3.UnitZ : Vector3.Normalize(center);
            Vector3 y_axis = Vector3.Cross(z_axis, x_axis);

            AddFace(color,
                center - length / 2 * x_axis - width / 2 * y_axis,
                center + length / 2 * x_axis - width / 2 * y_axis,
                center + length / 2 * x_axis + width / 2 * y_axis,
                center - length / 2 * x_axis + width / 2 * y_axis);
        }
        /// <summary>
        /// Creates a cube mesh from 6 panels.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="sizeX">The size of the cube in the x-axis.</param>
        /// <param name="sizeY">The size of the cube in the y-axis.</param>
        /// <param name="sizeZ">The size of the cube in the z-axis.</param>
        public static Mesh CreateCube(UnitSystem units, Color color, float sizeX, float sizeY, float sizeZ)
        {
            var mesh = new Mesh(units);
            mesh.AddPanel(
                color,
                new Vector3(0, sizeY/2, 0),
                Vector3.UnitX,
                sizeX, sizeZ);
            mesh.AddPanel(
                color,
                new Vector3(0, -sizeY/2, 0),
                Vector3.UnitX,
                sizeX, sizeZ);
            mesh.AddPanel(
                color,
                new Vector3(sizeX/2, 0, 0),
                Vector3.UnitZ,
                sizeZ, sizeY);
            mesh.AddPanel(
                color,
                new Vector3(-sizeX/2, 0, 0),
                Vector3.UnitZ,
                sizeZ, sizeY);
            mesh.AddPanel(
                color,
                new Vector3(0, 0, sizeZ/2),
                Vector3.UnitX,
                sizeX, sizeY);
            mesh.AddPanel(
                color,
                new Vector3(0, 0, -sizeZ/2),
                Vector3.UnitX,
                sizeX, sizeY);
            return mesh;
        }

        public static Mesh CreateCube(UnitSystem units, Color color, float size)
            => CreateCube(units, color, size, size, size);

        public static Mesh CreateSphere(UnitSystem units, Color color, float radius)
        {
            var mesh = CreateCube(units, color, 1f);
            mesh.Tesselate(3);
            for (int i = 0; i < mesh.nodeList.Count; i++)
            {
                var n = mesh.nodeList[i];
                float d = mesh.nodeList[i].Length();
                (float x, float y, float z) = (n.X, n.Y, n.Z);
                x = x.CapAbs(0, 1);
                y = y.CapAbs(0, 1);
                z = z.CapAbs(0, 1);
                n = new Vector3(x, y, z);
                mesh.nodeList[i] = radius * Vector3.Normalize(n);
            }
            return mesh;
        }

        /// <summary>
        /// Creates a square pyramid mesh from 5 panels.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="base">The size of the base.</param>
        /// <param name="height">The height of the pyramid.</param>
        public static Mesh CreatePyramid(UnitSystem units, Color color, float @base, float height)
        {
            var mesh = new Mesh(units);
            mesh.nodeList.Add(new Vector3(-@base/2, -@base/2, 0));
            mesh.nodeList.Add(new Vector3(@base/2, -@base/2, 0));
            mesh.nodeList.Add(new Vector3(@base/2, @base/2, 0));
            mesh.nodeList.Add(new Vector3(-@base/2, @base/2, 0));
            mesh.elementList.Add(new Element(color, 3, 2, 1, 0));
            mesh.nodeList.Add(height*Vector3.UnitZ);
            mesh.elementList.Add(new Element(color, 4, 0, 1));
            mesh.elementList.Add(new Element(color, 4, 1, 2));
            mesh.elementList.Add(new Element(color, 4, 2, 3));
            mesh.elementList.Add(new Element(color, 4, 3, 0));

            return mesh;
        }

        #region Transformations
        //public Vector3[] GetNodes() => nodeList.ToArray();
        public Mesh Scale(float factor)
        {
            return new Mesh(Units, nodeList.Select((n) => factor*n), elementList);
        }
        public Mesh Offset(Vector3 offset)
        {
            return new Mesh(Units, nodeList.Select((n) => n + offset), elementList);
        }
        public Mesh Transform(Matrix4x4 transform, bool inverse = false)
        {
            if (inverse)
            {
                Matrix4x4.Invert(transform, out var inv);
                return new Mesh(Units, nodeList.Select((n) => Vector3.Transform(n, inv)), elementList);
            }
            return new Mesh(Units, nodeList.Select((n) => Vector3.Transform(n, transform)), elementList);
        }
        public Mesh Rotate(Quaternion rotation)
        {
            return new Mesh(Units,
                nodeList.Select((n) =>Vector3.Transform(n, rotation)),
                elementList);
        }
        public Mesh Rotate(Quaternion rotation, Vector3 pivot)
        {
            return new Mesh(Units,
                nodeList.Select((n) => pivot + Vector3.Transform(n - pivot, rotation)),
                elementList);
        }

        #endregion

        public override string ToString()
        {
            return $"Mesh(Nodes={nodeList.Count}, Elements={elementList.Count})";
        }
    }
}
