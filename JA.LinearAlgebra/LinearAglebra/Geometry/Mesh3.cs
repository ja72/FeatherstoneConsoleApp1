using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using System.ComponentModel;
using System.Numerics;

using static System.Math;

namespace JA.LinearAlgebra.Geometry
{

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Face
    {
        public Face(Color color, params int[] face)
        {
            Color=color;
            NodeIndex=face;
        }

        public int[] NodeIndex { get; }
        public Color Color { get; set; }

        public void Flip()
        {
            Array.Copy(NodeIndex.Reverse().ToArray(), NodeIndex, NodeIndex.Length);
        }
        public override string ToString()
        {
            return $"Face(Color={Color}, Nodes=[{string.Join(",", NodeIndex)}])";
        }
    }
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Mesh3 : ICanChangeUnits<Mesh3>
    {
        readonly List<Vector3> nodeList;
        readonly List<Face> faceList;

        #region Factory
        public Mesh3(UnitSystem units)
        {
            Units=units;
            nodeList=new List<Vector3>();
            faceList=new List<Face>();
        }

        public Mesh3(Mesh3 copy)
        {
            Units=copy.Units;
            nodeList=new List<Vector3>(copy.nodeList);
            faceList=new List<Face>(copy.faceList);
        }

        Mesh3(UnitSystem units, IEnumerable<Vector3> nodeList, IEnumerable<Face> elementList) : this(units)
        {
            Units=units;
            this.nodeList=new List<Vector3>(nodeList);
            this.faceList=new List<Face>(elementList);
        }
        #endregion

        #region Properties
        public UnitSystem Units { get; }
        public Mesh3 ToConverted(UnitSystem target)
        {
            if (Units!=target)
            {
                float fl = Unit.Length.Convert(Units, target);
                return new Mesh3(target,
                    nodeList.Select((n) => fl*n),
                    faceList);
            }
            return this;
        }

        [Browsable(false)] public List<Vector3> NodeList => nodeList;
        [Browsable(false)] public List<Face> ElementList => faceList;

        public Vector3[] Nodes { get => nodeList.ToArray(); }
        [Browsable(false)]
        public Face[] Elements { get => faceList.ToArray(); }
        #endregion

        #region Structure
        public void ApplyTranform(Vector3 origin)
        {
            for (int i = 0; i<nodeList.Count; i++)
            {
                nodeList[i]=origin+nodeList[i];
            }
        }
        public void ReverseTranform(Vector3 origin)
        {
            for (int i = 0; i<nodeList.Count; i++)
            {
                nodeList[i]=nodeList[i]=origin;
            }
        }
        public void Tesselate(int level = 1)
        {
            for (int c = 0; c<level; c++)
            {
                for (int i = faceList.Count-1; i>=0; i--)
                {
                    TesselateFace(i);
                }
            }
        }
        public void TesselateFace(int index)
        {
            var face = faceList[index].NodeIndex;
            var color = faceList[index].Color;
            var nodes = face.Select(ni => nodeList[ni]).ToArray();
            if (nodes.Length==4)
            {
                faceList.RemoveAt(index);
                // assume quadrelateral and split all four sides.
                // [f2]---[ 1]---[f1]
                // |    C   |   B   |
                // [ 2]---[ 4]---[ 0]
                // |    D   |   A   |
                // [f3]---[ 3]---[f0]
                int k = nodeList.Count;
                for (int i = 0; i<nodes.Length; i++)
                {
                    int j = (i+1) % nodes.Length;
                    nodeList.Add(( nodes[i]+nodes[j] )/2);
                }
                var center = nodes.Aggregate(Vector3.Zero, (a, b) => a + b) / nodes.Length;
                nodeList.Add(center);

                AddFace(color, face[0], k+0, k+4, k+3); // A
                AddFace(color, k+0, face[1], k+1, k+4); // B
                AddFace(color, k+4, k+1, face[2], k+2); // C
                AddFace(color, k+3, k+4, k+2, face[3]); // D
            }
            else if (nodes.Length>=3)
            {
                faceList.RemoveAt(index);
                int k = nodeList.Count;
                var center = nodes.Aggregate(Vector3.Zero, (a, b) => a + b) / nodes.Length;
                nodeList.Add(center);

                for (int i = 0; i<nodes.Length; i++)
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
            return faceList[faceIndex].NodeIndex.Select(ni => nodeList[ni]).ToArray();
        }
        public Triangle3[] GetTriangles(int faceIndex) => GetPolygon(faceIndex).GetTriangles();
        public Polygon3 GetPolygon(int index) => new Polygon3(GetFaceNodes(index));

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
            for (int i = 0; i<nodes.Length; i++)
            {
                int j = (i + 1) % nodes.Length;
                int k = (i - 1 + nodes.Length) % nodes.Length;

                Vector3 A = nodes[i], B = nodes[j], C = nodes[k];

                normals[i]=Vector3.Normalize(
                    Vector3.Cross(A, B)
                    +Vector3.Cross(B, C)
                    +Vector3.Cross(C, A)
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
            for (int i = 0; i<list.Length; i++)
            {
                n+=list[i];
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
            for (int i = 0; i<nodes.Length; i++)
            {
                if (nodeList.Contains(nodes[i]))
                {
                    nodeIndex[i]=nodeList.IndexOf(nodes[i]);
                }
                else
                {
                    nodeIndex[i]=nodeList.Count;
                    nodeList.Add(nodes[i]);
                }
            }
            faceList.Add(new Face(color, nodeIndex));
        }
        public void AddFace(Color color, params int[] nodeIndex)
        {
            faceList.Add(new Face(color, nodeIndex));
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
            x_axis=Vector3.Normalize(x_axis);
            Vector3 z_axis = center == Vector3.Zero ? Vector3.UnitZ : Vector3.Normalize(center);
            var y_axis = Vector3.Cross(z_axis, x_axis);

            AddFace(color,
                center-length/2*x_axis-width/2*y_axis,
                center+length/2*x_axis-width/2*y_axis,
                center+length/2*x_axis+width/2*y_axis,
                center-length/2*x_axis+width/2*y_axis);
        }
        #endregion

        #region Transformations
        //public Vector3[] GetNodes() => nodeList.ToArray();
        public Mesh3 Scale(float factor)
        {
            return new Mesh3(Units, nodeList.Select((n) => factor*n), faceList);
        }
        public Mesh3 Offset(Vector3 offset)
        {
            return new Mesh3(Units, nodeList.Select((n) => n+offset), faceList);
        }
        public Mesh3 Transform(Matrix4x4 transform, bool inverse = false)
        {
            if (inverse)
            {
                Matrix4x4.Invert(transform, out var inv);
                return new Mesh3(Units, nodeList.Select((n) => Vector3.Transform(n, inv)), faceList);
            }
            return new Mesh3(Units, nodeList.Select((n) => Vector3.Transform(n, transform)), faceList);
        }
        public Mesh3 Rotate(Quaternion rotation)
        {
            return new Mesh3(Units,
                nodeList.Select((n) => Vector3.Transform(n, rotation)),
                faceList);
        }
        public Mesh3 Rotate(Quaternion rotation, Vector3 pivot)
        {
            return new Mesh3(Units,
                nodeList.Select((n) => pivot+Vector3.Transform(n-pivot, rotation)),
                faceList);
        }

        #endregion

        #region Formatting
        public override string ToString()
        {
            return $"Mesh(Nodes={nodeList.Count}, Elements={faceList.Count})";
        }
        #endregion

        #region Basic Shapes
        /// <summary>
        /// Creates a cube mesh from 6 panels.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="sizeX">The size of the cube in the x-axis.</param>
        /// <param name="sizeY">The size of the cube in the y-axis.</param>
        /// <param name="sizeZ">The size of the cube in the z-axis.</param>
        public static Mesh3 CreateCube(UnitSystem units, Color color, float sizeX, float sizeY, float sizeZ)
        {
            var mesh = new Mesh3(units);
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

        public static Mesh3 CreateCube(UnitSystem units, Color color, float size)
            => CreateCube(units, color, size, size, size);

        public static Mesh3 CreateSphere(UnitSystem units, Color color, float radius)
        {
            var mesh = CreateCube(units, color, 1f);
            mesh.Tesselate(3);
            for (int i = 0; i<mesh.nodeList.Count; i++)
            {
                var n = mesh.nodeList[i];
                float d = mesh.nodeList[i].Length();
                (float x, float y, float z)=(n.X, n.Y, n.Z);
                x=Min(1, Max(0, x));
                y=Min(1, Max(0, y));
                z=Min(1, Max(0, z));
                n=new Vector3(x, y, z);
                mesh.nodeList[i]=radius*Vector3.Normalize(n);
            }
            return mesh;
        }

        /// <summary>
        /// Creates a square pyramid mesh from 5 panels.
        /// </summary>
        /// <param name="color">The face color.</param>
        /// <param name="base">The size of the base.</param>
        /// <param name="height">The height of the pyramid.</param>
        public static Mesh3 CreatePyramid(UnitSystem units, Color color, float @base, float height)
        {
            var mesh = new Mesh3(units);
            mesh.nodeList.Add(new Vector3(-@base/2, -@base/2, 0));
            mesh.nodeList.Add(new Vector3(@base/2, -@base/2, 0));
            mesh.nodeList.Add(new Vector3(@base/2, @base/2, 0));
            mesh.nodeList.Add(new Vector3(-@base/2, @base/2, 0));
            mesh.faceList.Add(new Face(color, 3, 2, 1, 0));
            mesh.nodeList.Add(height*Vector3.UnitZ);
            mesh.faceList.Add(new Face(color, 4, 0, 1));
            mesh.faceList.Add(new Face(color, 4, 1, 2));
            mesh.faceList.Add(new Face(color, 4, 2, 3));
            mesh.faceList.Add(new Face(color, 4, 3, 0));

            return mesh;
        }
        #endregion

        #region Imports
        public static bool ImportSTL(UnitSystem unit, string filePath, Color color, out Mesh3 mesh)
            => ImportSTL(unit, filePath, color, out mesh, Vector3.Zero);
        public static bool ImportSTL(UnitSystem unit, string filePath, Color color, out Mesh3 mesh, Vector3 localOrigin)
        {
            Vector3 Text_ReadVector3(string lineString)
            {
                string[] lineData=lineString.Split(' ');
                Vector3 vert;
                vert.X=float.Parse(lineData[1]); // x1
                vert.Y=float.Parse(lineData[2]); // y1
                vert.Z=float.Parse(lineData[3]); // z1
                return vert;
            }
            Vector3 Binary_ReadVector3(byte[] fileBytes, ref int byteIndex)
            {
                Vector3 vert;
                vert.X=BitConverter.ToSingle(fileBytes, byteIndex);
                byteIndex+=4;
                vert.Y=BitConverter.ToSingle(fileBytes, byteIndex);
                byteIndex+=4;
                vert.Z=BitConverter.ToSingle(fileBytes, byteIndex);
                byteIndex+=4;
                return vert;
            }

            mesh=new Mesh3(unit);
            bool isBinary;

            if (File.Exists(filePath))
            {
                int lineCount = 0;
                var allLines = File.ReadLines(filePath).ToList();
                lineCount=allLines.Count(); // number of lines in the file

                string firstLine = allLines.First();

                string endLines = allLines.Skip(lineCount - 1).Take(1).First() +
                                  allLines.Skip(lineCount - 2).Take(1).First();

                /* check the file is ascii or not */
                if (( firstLine.IndexOf("solid")!=-1 )&
                    ( endLines.IndexOf("endsolid")!=-1 ))
                {
                    isBinary=false;
                }
                else
                {
                    isBinary=true;
                }
            }
            else
            {
                return false;
            }

            if (isBinary)
            {
                int numOfMesh = 0;
                int i = 0;
                int byteIndex = 0;
                byte[] fileBytes = File.ReadAllBytes(filePath);


                /* 80 bytes title + 4 byte num of triangles + 50 bytes (1 of triangular mesh)  */
                if (fileBytes.Length>120)
                {
                    byteIndex=80;
                    numOfMesh=BitConverter.ToInt32(fileBytes, byteIndex);
                    byteIndex+=4;

                    for (i=0; i<numOfMesh; i++)
                    {
                        /* this try-catch block will be reviewed */
                        try
                        {

                            Vector3 normal1, normal2, normal3;
                            Vector3 vert1, vert2, vert3;

                            /* face normal */
                            normal1=Binary_ReadVector3(fileBytes, ref byteIndex);

                            /* normals of vertex 2 and 3 equals to vertex 1's normals */
                            normal2=normal1;
                            normal3=normal1;

                            /* vertex 1 */
                            vert1=Binary_ReadVector3(fileBytes, ref byteIndex);

                            /* vertex 2 */
                            vert2=Binary_ReadVector3(fileBytes, ref byteIndex);

                            /* vertex 3 */
                            vert3=Binary_ReadVector3(fileBytes, ref byteIndex);

                            byteIndex+=2; // Attribute byte count

                            // eat the surface normals
                            mesh.AddFace(color, vert1 - localOrigin, vert2 - localOrigin, vert3 - localOrigin);
                        }
                        catch
                        {
                            return false;
                        }
                    }

                }
                else
                {
                    // nitentionally left blank
                }
                return true;
            }
            else
            {
                using (var txtReader = new StreamReader(filePath))
                {
                    string lineString;

                    while (!txtReader.EndOfStream)
                    {
                        lineString=txtReader.ReadLine().Trim(); /* delete whitespace in front and tail of the string */
                        string[] lineData = lineString.Split(' ');

                        if (lineData[0]=="solid")
                        {
                            while (lineData[0]!="endsolid")
                            {
                                lineString=txtReader.ReadLine().Trim(); // facetnormal
                                lineData=lineString.Split(' ');

                                if (lineData[0]=="endsolid") // check if we reach at the end of file
                                {
                                    break;
                                }


                                /* this try-catch block will be reviewed */
                                try
                                {
                                    Vector3 normal1, normal2, normal3;
                                    Vector3 vert1, vert2, vert3;

                                    // FaceNormal 
                                    normal1.X=float.Parse(lineData[2]);
                                    normal1.Y=float.Parse(lineData[3]);
                                    normal1.Z=float.Parse(lineData[4]);

                                    /* normals of vertex 2 and 3 equals to vertex 1's normals */
                                    normal2=normal1;
                                    normal3=normal1;

                                    //----------------------------------------------------------------------
                                    lineString=txtReader.ReadLine(); // Just skip the OuterLoop line
                                                                     //----------------------------------------------------------------------

                                    // Vertex1
                                    lineString=txtReader.ReadLine().Trim();
                                    /* reduce spaces until string has proper format for split */
                                    while (lineString.IndexOf("  ")!=-1) lineString=lineString.Replace("  ", " ");
                                    vert1=Text_ReadVector3(lineString);

                                    // Vertex2
                                    lineString=txtReader.ReadLine().Trim();
                                    /* reduce spaces until string has proper format for split */
                                    while (lineString.IndexOf("  ")!=-1) lineString=lineString.Replace("  ", " ");
                                    vert2=Text_ReadVector3(lineString);

                                    // Vertex3
                                    lineString=txtReader.ReadLine().Trim();
                                    /* reduce spaces until string has proper format for split */
                                    while (lineString.IndexOf("  ")!=-1) lineString=lineString.Replace("  ", " ");
                                    vert3=Text_ReadVector3(lineString);

                                    // eat the surface normals
                                    mesh.AddFace(color, vert1 - localOrigin, vert2 - localOrigin, vert3 - localOrigin);

                                }
                                catch
                                {
                                    return false;
                                }

                                //----------------------------------------------------------------------
                                lineString=txtReader.ReadLine(); // Just skip the endloop
                                                                 //----------------------------------------------------------------------
                                lineString=txtReader.ReadLine(); // Just skip the endfacet

                                // add mesh to meshList
                                // NOTE: not supported by Mesh object

                            } // while linedata[0]
                        } // if solid
                    } // while !endofstream

                }
                return true;
            }
        }
        #endregion
    }
}
