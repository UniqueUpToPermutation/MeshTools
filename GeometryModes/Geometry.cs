using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using Linalg = MathNet.Numerics.LinearAlgebra;

using Mat = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vec = MathNet.Numerics.LinearAlgebra.Vector<double>;
using Clr = OpenTK.Graphics.Color4;
using Assimp;

namespace GeometryModes
{
    namespace Geometry
    {
        public interface IGeometrySource
        {
            List<Assimp.Vector3D> Vertices { get; }
            List<Assimp.Vector3D> Normals { get; }
            List<Assimp.Vector3D> TexCoords { get; }
            List<Assimp.Vector3D> Tangents { get; }
            List<Assimp.Face> Faces { get; }
        }

        public class AssimpMeshGeometrySource : IGeometrySource
        {
            public Assimp.Mesh mesh;

            public AssimpMeshGeometrySource(Assimp.Mesh mesh)
            {
                this.mesh = mesh;
            }

            public List<Vector3D> Vertices {
                get { return mesh.HasVertices ? mesh.Vertices : null; }
            }
            public List<Vector3D> Normals {
                get { return mesh.HasNormals ? mesh.Normals : null; }
            }
            public List<Vector3D> TexCoords {
                get { return mesh.HasTextureCoords(0) ? mesh.TextureCoordinateChannels[0] : null; }
            }
            public List<Vector3D> Tangents {
                get { return mesh.HasTangentBasis ? mesh.Tangents : null; }
            }
            public List<Assimp.Face> Faces {
                get { return mesh.HasFaces ? mesh.Faces : null; }
            }
        }

        class RawGeometrySource : IGeometrySource
        {
            public List<Vector3D> Vertices { get; set; } = new List<Vector3D>();
            public List<Vector3D> Normals { get; set; } = new List<Vector3D>();
            public List<Vector3D> TexCoords { get; set; } = new List<Vector3D>();
            public List<Vector3D> Tangents { get; set; } = new List<Vector3D>();
            public List<Assimp.Face> Faces { get; set; } = new List<Assimp.Face>();
        }

        public struct RawEdge
        {
            public int Opposite;
            public int Head;
            public int Face;
            public int Next;
        }

        public struct RawFace
        {
            public int Edge;
        }

        public struct RawVertex
        {
            public int Edge;
            public OpenTK.Vector3 Position;
            public Clr Color;
            public OpenTK.Vector2 UV;
            public OpenTK.Vector3 Normal;
            public OpenTK.Vector3 Tangent;
        }

        public struct Edge
        {
            public readonly Geometry parent;
            public readonly int id;

            public Edge(Geometry parent, int id)
            {
                this.parent = parent;
                this.id = id;
            }

            public RawEdge Data
            {
                get { return parent.edges[id]; }
                set { parent.edges[id] = value; }
            }

            public OpenTK.Vector3 Direction
            {
                get
                {
                    return Head.Position - Tail.Position;
                }
            }

            public static bool operator ==(Edge e1, Edge e2)
            {
                return e1.id == e2.id;
            }

            public static bool operator !=(Edge e1, Edge e2)
            {
                return e1.id != e2.id;
            }

            public static Edge None
            {
                get { return new Edge(null, -1); }
            }

            public Edge Opposite
            {
                get { return new Edge(parent, Data.Opposite); }
                set
                {
                    var data = Data;
                    data.Opposite = value.id;
                    Data = data;
                }
            }
            public Vertex Head
            {
                get { return new Vertex(parent, Data.Head); }
                set
                {
                    var data = Data;
                    data.Head = value.id;
                    Data = data;
                }
            }    
            public Face Face
            {
                get { return new Face(parent, Data.Face); }
                set
                {
                    var data = Data;
                    data.Face = value.id;
                    Data = data;
                }
            }
            public Edge Next
            {
                get { return new Edge(parent, Data.Next); }
                set
                {
                    var data = Data;
                    data.Next = value.id;
                    Data = data;
                }
            }
            public Vertex Tail
            {
                get { return Opposite.Head; }
            }
            public int ID { get { return id; } }

            public override bool Equals(object obj)
            {
                if (obj is Edge)
                    return (((Edge)obj) == this);
                return false;
            }

            public override int GetHashCode()
            {
                return id;
            }

            public override string ToString()
            {
                return $"ID: {id}, HEAD: {Head.id}, TAIL: {Tail.id}, NEXT: {Next.id}, FACE: {Face.id}";
            }
        }

        public struct Face
        {
            public readonly Geometry parent;
            public readonly int id;

            public Face(Geometry parent, int id)
            {
                this.parent = parent;
                this.id = id;
            }

            public RawFace Data
            {
                get { return parent.faces[id]; }
                set { parent.faces[id] = value; }
            }

            public static bool operator ==(Face f1, Face f2)
            {
                return f1.id == f2.id;
            }

            public static bool operator !=(Face f1, Face f2)
            {
                return f1.id != f2.id;
            }

            public float TriangleArea
            {
                get
                {
                    var v1 = Edge.Head;
                    var v2 = Edge.Next.Head;
                    var v3 = Edge.Next.Next.Head;
                    return Math.Abs(OpenTK.Vector3.Cross(v1.Data.Position - v2.Data.Position, 
                        v2.Data.Position - v3.Data.Position).Length / 2.0f);
                }
            }

            public static Face None
            {
                get { return new Face(null, -1); }
            }

            public Edge Edge
            {
                get { return new Edge(parent, Data.Edge); }
                set
                {
                    var data = Data;
                    data.Edge = value.id;
                    Data = data;
                }
            }
            public int ID { get { return id; } }

            public IEnumerable<Edge> Edges
            {
                get
                {
                    for (Edge start = Edge, current = start; ;)
                    {
                        yield return current;
                        current = current.Next;
                        if (current == start)
                            break;
                    }
                }
            }

            public IEnumerable<Face> AdjacentFaces
            {
                get
                {
                    return from e in Edges select e.Opposite.Face;
                }
            }

            public IEnumerable<Vertex> Vertices
            {
                get
                {
                    return from e in Edges select e.Head;
                }
            }

            public override bool Equals(object obj)
            {
                if (obj is Face)
                    return (((Face)obj) == this);
                return false;
            }

            public override int GetHashCode()
            {
                return id;
            }

            public override string ToString()
            {
                return $"ID: {id}, EDGE: {Edge.id}";
            }
        }

        public struct Vertex
        {
            public readonly Geometry parent;
            public readonly int id;

            public Vertex(Geometry parent, int id)
            {
                this.parent = parent;
                this.id = id;
            }

            public RawVertex Data
            {
                get { return parent.vertices[id]; }
                set { parent.vertices[id] = value; }
            }

            public OpenTK.Vector3 Position
            {
                get { return parent.vertices[id].Position; }
                set
                {
                    var data = parent.vertices[id];
                    data.Position = value;
                    parent.vertices[id] = data;
                }
            }

            public OpenTK.Vector2 UV
            {
                get { return parent.vertices[id].UV; }
                set
                {
                    var data = parent.vertices[id];
                    data.UV = value;
                    parent.vertices[id] = data;
                }
            }

            public OpenTK.Vector3 Normal
            {
                get { return parent.vertices[id].Normal; }
                set
                {
                    var data = parent.vertices[id];
                    data.Normal = value;
                    parent.vertices[id] = data;
                }
            }

            public OpenTK.Vector3 Tangent
            {
                get { return parent.vertices[id].Tangent; }
                set
                {
                    var data = parent.vertices[id];
                    data.Tangent = value;
                    parent.vertices[id] = data;
                }
            }

            public Clr Color
            {
                get { return parent.vertices[id].Color; }
                set
                {
                    var data = parent.vertices[id];
                    data.Color = value;
                    parent.vertices[id] = data;
                }
            }

            public static bool operator ==(Vertex v1, Vertex v2)
            {
                return v1.id == v2.id;
            }

            public static bool operator !=(Vertex v1, Vertex v2)
            {
                return v1.id != v2.id;
            }

            public static Vertex None
            {
                get { return new Vertex(null, -1); }
            }

            public Edge Edge
            {
                get { return new Edge(parent, Data.Edge); }
                set
                {
                    var data = Data;
                    data.Edge = value.id;
                    Data = data;
                }
            }

            public int ID { get { return id; } }

            public IEnumerable<Edge> OutgoingEdges
            {
                get
                {
                    for (Edge start = Edge, current = start; ;)
                    {
                        yield return current;
                        current = current.Opposite.Next;
                        if (current == start)
                            break;
                    }
                }
            }

            public IEnumerable<Edge> IngoingEdges
            {
                get
                {
                    return from e in OutgoingEdges select e.Opposite;
                }
            }

            public IEnumerable<Vertex> AdjacentVertices
            {
                get
                {
                    return from e in OutgoingEdges select e.Head;
                }
            }

            public IEnumerable<Face> AdjacentFaces
            {
                get
                {
                    return from e in OutgoingEdges select e.Face;
                }
            }

            public override bool Equals(object obj)
            {
                if (obj is Vertex)
                    return (((Vertex)obj) == this);
                return false;
            }

            public override int GetHashCode()
            {
                return id;
            }

            public override string ToString()
            {
                return $"ID: {id}, POSITION: {Data.Position}";
            }
        }

        public struct BoundingBox
        {
            public OpenTK.Vector3 Lower;
            public OpenTK.Vector3 Upper;
        }

        public class Geometry
        {
            public List<RawEdge> edges = new List<RawEdge>();
            public List<RawFace> faces = new List<RawFace>();
            public List<RawVertex> vertices = new List<RawVertex>();
            protected BoundingBox aabb = new BoundingBox();
            protected bool bHasBoundary = false;

            public bool HasVertices { get; protected set; } = false;
            public bool HasUVs { get; protected set; } = false;
            public bool HasNormals { get; protected set; } = false;
            public bool HasTangents { get; protected set; } = false;
            public bool HasColor { get; protected set; } = false;

            public Edge GetEdge(int id)
            {
                return new Edge(this, id);
            }

            public Face GetFace(int id)
            {
                return new Face(this, id);
            }

            public Vertex GetVertex(int id)
            {
                return new Vertex(this, id);
            }

            public BoundingBox BoundingBox
            {
                get { return aabb; }
            }

            public IEnumerable<Edge> Edges
            {
                get
                {
                    for (int i = 0; i < edges.Count; ++i)
                        yield return new Edge(this, i);
                }
            }

            public IEnumerable<Face> Faces
            {
                get
                {
                    for (int i = 0; i < faces.Count; ++i)
                        yield return new Face(this, i);
                }
            }

            public IEnumerable<Vertex> Vertices
            {
                get
                {
                    for (int i = 0; i < vertices.Count; ++i)
                        yield return new Vertex(this, i);
                }
            }

            public int EdgeCount
            {
                get { return edges.Count; }
            }

            public int VertexCount
            {
                get { return vertices.Count; }
            }

            public int FaceCount
            {
                get { return faces.Count; }
            }

            public Edge CreateEdge()
            {
                edges.Add(new RawEdge() { Face = -1, Head = -1, Next = -1, Opposite = -1 } );
                return LastEdge;
            }

            public Face CreateFace()
            {
                faces.Add(new RawFace() { Edge = -1 });
                return LastFace;
            }

            public Vertex CreateVertex()
            {
                vertices.Add(new RawVertex() { Edge = -1, Position = OpenTK.Vector3.Zero });
                return LastVertex;
            }

            public Edge LastEdge
            {
                get
                {
                    return GetEdge(edges.Count - 1);
                }
            }

            public Face LastFace
            {
                get
                {
                    return GetFace(faces.Count - 1);
                }
            }

            public Vertex LastVertex
            {
                get
                {
                    return GetVertex(vertices.Count - 1);
                }
            }

            public void UpdateBoundingBox()
            {
                var lower = new OpenTK.Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
                var upper = new OpenTK.Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

                foreach (var v in vertices)
                {
                    upper = OpenTK.Vector3.ComponentMax(upper, v.Position);
                    lower = OpenTK.Vector3.ComponentMin(lower, v.Position);
                }

                aabb.Lower = lower;
                aabb.Upper = upper;
            }

            public void UpdateHasBoundary()
            {
                bHasBoundary = false;
                foreach (var e in edges)
                {
                    if (e.Face == -1)
                        bHasBoundary = true;
                }
            }

            public float[] CreateIndexedVertexData(int stride,
                bool bUsePosition, int positionOffset,
                bool bUseColor, int colorOffset,
                bool bUseUV, int uvOffset,
                bool bUseNormal, int normalOffset,
                bool bUseTangent, int tangentOffset)
            {
                var buf = new float[stride * vertices.Count];
                int index = 0;
                foreach (var v in vertices)
                {
                    if (bUsePosition)
                    {
                        var pPosition = index * stride + positionOffset;
                        buf[pPosition] = v.Position.X;
                        buf[pPosition + 1] = v.Position.Y;
                        buf[pPosition + 2] = v.Position.Z;
                    }

                    if (bUseColor)
                    {
                        var pPosition = index * stride + colorOffset;
                        buf[pPosition] = v.Color.R;
                        buf[pPosition + 1] = v.Color.G;
                        buf[pPosition + 2] = v.Color.B;
                    }

                    if (bUseUV)
                    {
                        var pPosition = index * stride + uvOffset;
                        buf[pPosition] = v.UV.X;
                        buf[pPosition + 1] = v.UV.Y;
                    }

                    if (bUseNormal)
                    {
                        var pPosition = index * stride + normalOffset;
                        buf[pPosition] = v.Normal.X;
                        buf[pPosition + 1] = v.Normal.Y;
                        buf[pPosition + 2] = v.Normal.Z;
                    }

                    if (bUseTangent)
                    {
                        var pPosition = index * stride + tangentOffset;
                        buf[pPosition] = v.Tangent.X;
                        buf[pPosition + 1] = v.Tangent.Y;
                        buf[pPosition + 2] = v.Tangent.Z;
                    }
                }
                return buf;
            }

            public short[] CreateIndexData()
            {
                List<short> retValue = new List<short>();
                short[] faceVerts = new short[3];
                foreach (var face in Faces)
                {
                    int vCount = 0;
                    foreach (var v in face.Vertices)
                    {
                        if (vCount > 2)
                            throw new Exception("ERROR: Mesh has non-triangular faces! Triangulate first!");

                        retValue.Add((short)v.id);
                        vCount++;
                    }
                }
                return retValue.ToArray();
            }

            public float[] CreateUnindexedVertexData(int stride,
                bool bUsePosition, int positionOffset, 
                bool bUseColor, int colorOffset,
                bool bUseUV, int uvOffset,
                bool bUseNormal, int normalOffset,
                bool bUseTangent, int tangentOffset)
            {
                var buf = new float[stride * faces.Count * 3];
                int index = 0;
                foreach (var face in Faces)
                {
                    int vCount = 0;
                    foreach (var v in face.Vertices)
                    {
                        if (vCount > 2)
                            throw new Exception("ERROR: Mesh has non-triangular faces! Triangulate first!");

                        if (bUsePosition)
                        {
                            var pPosition = index * stride + positionOffset;
                            buf[pPosition] = v.Data.Position.X;
                            buf[pPosition + 1] = v.Data.Position.Y;
                            buf[pPosition + 2] = v.Data.Position.Z;
                        }

                        if (bUseColor)
                        {
                            var pPosition = index * stride + colorOffset;
                            buf[pPosition] = v.Data.Color.R;
                            buf[pPosition + 1] = v.Data.Color.G;
                            buf[pPosition + 2] = v.Data.Color.B;
                        }

                        if (bUseUV)
                        {
                            var pPosition = index * stride + uvOffset;
                            buf[pPosition] = v.Data.UV.X;
                            buf[pPosition + 1] = v.Data.UV.Y;
                        }

                        if (bUseNormal)
                        {
                            var pPosition = index * stride + normalOffset;
                            buf[pPosition] = v.Data.Normal.X;
                            buf[pPosition + 1] = v.Data.Normal.Y;
                            buf[pPosition + 2] = v.Data.Normal.Z;
                        }

                        if (bUseTangent)
                        {
                            var pPosition = index * stride + tangentOffset;
                            buf[pPosition] = v.Data.Tangent.X;
                            buf[pPosition + 1] = v.Data.Tangent.Y;
                            buf[pPosition + 2] = v.Data.Tangent.Z;
                        }

                        index += 1;
                        ++vCount;
                    }
                }
                return buf;
            }

            public static Geometry MakeGrid(OpenTK.Vector3 origin, OpenTK.Vector3 unitX,
                OpenTK.Vector3 unitY, int startX, int endX, int startY, int endY)
            {
                var aOrigin = new Assimp.Vector3D(origin.X, origin.Y, origin.Z);
                var aUnitX = new Assimp.Vector3D(unitX.X, unitX.Y, unitX.Z);
                var aUnitY = new Assimp.Vector3D(unitY.X, unitY.Y, unitY.Z);

                RawGeometrySource geo = new RawGeometrySource();
                var normal = Assimp.Vector3D.Cross(aUnitX, aUnitY);
                normal.Normalize();

                for (int x = startX; x <= endX; ++x)
                {
                    for (int y = startY; y <= endY; ++y)
                    {
                        geo.Vertices.Add(aOrigin + x * aUnitX + y * aUnitY);
                        geo.Normals.Add(normal);
                        geo.Tangents.Add(aUnitX);
                        geo.TexCoords.Add(new Vector3D(((float)x - (float)startX) / ((float)endX - (float)startX),
                            ((float)y - (float)startY) / ((float)endY - (float)startY),
                            0.0f));
                    }
                }

                for (int x = startX; x <= endX - 1; ++x)
                {
                    for (int y = startY; y <= endY - 1; ++y)
                    {
                        Func<int, int, int> conv = (i, j) => j - startY + ((i - startX) * (endY - startY + 1));

                        int[] top = { conv(x, y), conv(x + 1, y), conv(x, y + 1) };
                        int[] bottom = { conv(x, y + 1), conv(x + 1, y), conv(x + 1, y + 1) };

                        geo.Faces.Add(new Assimp.Face(top));
                        geo.Faces.Add(new Assimp.Face(bottom));
                    }
                }

                return From(geo);
            }

            public static Geometry FromAssimp(Assimp.Mesh mesh, 
                bool bGroupVertices = false, 
                bool bMergeVertices = false, 
                float relativeGroupTolerance = 1E-7f)
            {
                return From(new AssimpMeshGeometrySource(mesh), bGroupVertices, bMergeVertices, relativeGroupTolerance);
            }

            public void Save(string filename)
            {
                using (var fHandle = new FileStream(filename, FileMode.Create))
                    Save(fHandle);
            }

            public void Save(Stream s)
            {
                using (var writer = new BinaryWriter(s))
                {
                    writer.Write(HasVertices);
                    writer.Write(HasUVs);
                    writer.Write(HasNormals);
                    writer.Write(HasTangents);

                    writer.Write(vertices.Count);
                    writer.Write(edges.Count);
                    writer.Write(faces.Count);

                    // Write vertex data
                    if (HasVertices)
                    {
                        foreach (var v in vertices)
                        {
                            writer.Write(v.Position.X);
                            writer.Write(v.Position.Y);
                            writer.Write(v.Position.Z);
                        }
                    }

                    if (HasUVs)
                    {
                        foreach (var v in vertices)
                        {
                            writer.Write(v.UV.X);
                            writer.Write(v.UV.Y);
                        }
                    }

                    if (HasNormals)
                    {
                        foreach (var v in vertices)
                        {
                            writer.Write(v.Normal.X);
                            writer.Write(v.Normal.Y);
                            writer.Write(v.Normal.Z);
                        }
                    }

                    if (HasTangents)
                    {
                        foreach (var v in vertices)
                        {
                            writer.Write(v.Tangent.X);
                            writer.Write(v.Tangent.Y);
                            writer.Write(v.Tangent.Z);
                        }
                    }

                    // Write vertex connectivity data
                    foreach (var v in vertices)
                        writer.Write(v.Edge);

                    // Write edge connectivity data
                    foreach (var e in edges)
                    {
                        writer.Write(e.Face);
                        writer.Write(e.Head);
                        writer.Write(e.Next);
                        writer.Write(e.Opposite);
                    }

                    // Write face connectivity data
                    foreach (var f in faces)
                        writer.Write(f.Edge);
                }
            }

            public static Geometry Load(string filename)
            {
                using (var fHandle = new FileStream(filename, FileMode.Open))
                    return Load(fHandle);
            }

            public static Geometry Load(Stream s)
            {
                Geometry geo = new Geometry();

                using (var reader = new BinaryReader(s))
                {
                    geo.HasVertices = reader.ReadBoolean();
                    geo.HasUVs = reader.ReadBoolean();
                    geo.HasNormals = reader.ReadBoolean();
                    geo.HasTangents = reader.ReadBoolean();
                    geo.HasColor = true;

                    var vertexCount = reader.ReadInt32();
                    var edgeCount = reader.ReadInt32();
                    var faceCount = reader.ReadInt32();

                    geo.vertices = (from i in Enumerable.Range(0, vertexCount) select new RawVertex()).ToList();
                    geo.edges = (from i in Enumerable.Range(0, edgeCount) select new RawEdge()).ToList();
                    geo.faces = (from i in Enumerable.Range(0, faceCount) select new RawFace()).ToList();

                    var a = new List<Vertex?>();

                    if (geo.HasVertices)
                    {
                        byte[] byteBuf = reader.ReadBytes(sizeof(float) * 3 * geo.VertexCount);
                        float[] floatBuf = new float[3 * geo.VertexCount];
                        Buffer.BlockCopy(byteBuf, 0, floatBuf, 0, byteBuf.Length);

                        int i = 0;
                        foreach (var v in geo.Vertices)
                        {
                            Vertex vP = v;
                            vP.Position = new OpenTK.Vector3(floatBuf[i++], floatBuf[i++], floatBuf[i++]);
                        }
                    }

                    if (geo.HasUVs)
                    {
                        byte[] byteBuf = reader.ReadBytes(sizeof(float) * 2 * geo.VertexCount);
                        float[] floatBuf = new float[2 * geo.VertexCount];
                        Buffer.BlockCopy(byteBuf, 0, floatBuf, 0, byteBuf.Length);

                        int i = 0;
                        foreach (var v in geo.Vertices)
                        {
                            Vertex vP = v;
                            vP.UV = new OpenTK.Vector2(floatBuf[i++], floatBuf[i++]);
                        }
                    }

                    if (geo.HasNormals)
                    {
                        byte[] byteBuf = reader.ReadBytes(sizeof(float) * 3 * geo.VertexCount);
                        float[] floatBuf = new float[3 * geo.VertexCount];
                        Buffer.BlockCopy(byteBuf, 0, floatBuf, 0, byteBuf.Length);

                        int i = 0;
                        foreach (var v in geo.Vertices)
                        {
                            Vertex vP = v;
                            vP.Normal = new OpenTK.Vector3(floatBuf[i++], floatBuf[i++], floatBuf[i++]);
                        }
                    }

                    if (geo.HasTangents)
                    {
                        byte[] byteBuf = reader.ReadBytes(sizeof(float) * 3 * geo.VertexCount);
                        float[] floatBuf = new float[3 * geo.VertexCount];
                        Buffer.BlockCopy(byteBuf, 0, floatBuf, 0, byteBuf.Length);

                        int i = 0;

                        foreach (var v in geo.Vertices)
                        {
                            Vertex vP = v;
                            vP.Tangent = new OpenTK.Vector3(floatBuf[i++], floatBuf[i++], floatBuf[i++]);
                        }
                    }

                    {
                        byte[] byteBuf = reader.ReadBytes(sizeof(int) * geo.VertexCount);
                        int[] intBuf = new int[geo.VertexCount];
                        Buffer.BlockCopy(byteBuf, 0, intBuf, 0, byteBuf.Length);

                        int j = 0;
                        foreach (var v in geo.Vertices)
                        {
                            Vertex vP = v;
                            vP.Edge = geo.GetEdge(intBuf[j++]);
                            vP.Color = Clr.White;
                        }

                        byteBuf = reader.ReadBytes(4 * sizeof(int) * geo.EdgeCount);
                        intBuf = new int[4 * geo.EdgeCount];
                        Buffer.BlockCopy(byteBuf, 0, intBuf, 0, byteBuf.Length);
                        j = 0;
                        foreach (var e in geo.Edges)
                        {
                            Edge eP = e;
                            eP.Face = geo.GetFace(intBuf[j++]);
                            eP.Head = geo.GetVertex(intBuf[j++]);
                            eP.Next = geo.GetEdge(intBuf[j++]);
                            eP.Opposite = geo.GetEdge(intBuf[j++]);
                        }

                        byteBuf = reader.ReadBytes(sizeof(int) * geo.FaceCount);
                        intBuf = new int[geo.FaceCount];
                        Buffer.BlockCopy(byteBuf, 0, intBuf, 0, byteBuf.Length);
                        j = 0;
                        foreach (var f in geo.Faces)
                        {
                            Face fP = f;
                            fP.Edge = geo.GetEdge(intBuf[j++]);
                        }
                    }
                }

                geo.UpdateBoundingBox();
                geo.UpdateHasBoundary();

                return geo;
            }

            public void VisualizeVertexFunction(Vec vertexFunc, ColorScheme scheme)
            {
                var smallest = vertexFunc.Min();
                var largest = vertexFunc.Max();

                for (int i = 0; i < vertexFunc.Count; ++i)
                {
                    var data = vertices[i];
                    data.Color = scheme.GetColor((double)vertexFunc[i], (double)smallest, (double)largest);
                    vertices[i] = data;
                }
            }

            public static Geometry From(IGeometrySource source, 
                bool bGroupVertices = false, 
                bool bMergeVertices = false, 
                float relativeGroupTolerance = 1E-7f)
            {
                Geometry geo = new Geometry();

                var meshVertices = source.Vertices;
                var meshNormals = source.Normals;
                var meshUvs = source.TexCoords;
                var meshTangents = source.Tangents;
                var meshFaces = source.Faces;

                geo.HasVertices = true;
                geo.HasColor = true;

                for (int i = 0; i < meshVertices.Count; ++i)
                {
                    Assimp.Vector3D v = meshVertices[i];
                    Assimp.Vector3D n = new Assimp.Vector3D(0.0f);
                    if (meshNormals != null)
                    {
                        geo.HasNormals = true;
                        n = meshNormals[i];
                    }
                    Assimp.Vector3D t = new Assimp.Vector3D(0.0f);
                    if (meshTangents != null)
                    {
                        geo.HasTangents = true;
                        t = meshTangents[i];
                    }
                    Assimp.Vector3D uv = new Assimp.Vector3D(0.0f);
                    if (meshUvs != null)
                    {
                        geo.HasUVs = true;
                        uv = meshUvs[i];
                    }

                    geo.vertices.Add(new RawVertex()
                    {
                        Position = new OpenTK.Vector3(v.X, v.Y, v.Z),
                        Normal = new OpenTK.Vector3(n.X, n.Y, n.Z),
                        Tangent = new OpenTK.Vector3(t.X, t.Y, t.Z),
                        UV = new OpenTK.Vector2(uv.X, uv.Y),
                        Edge = -1
                    });
                }

                // Compute bounding box
                geo.UpdateBoundingBox();

                var groupTolerance = relativeGroupTolerance * (geo.aabb.Upper - geo.aabb.Lower).Length;

                var vertexVertexToEdgeMap = new Dictionary<Tuple<int, int>, int>();

                // Compute group map forest for link up duplicate vertices
                var groupMap = Enumerable.Range(0, geo.vertices.Count).ToArray();
                (Vertex, int)[] vertexCollection = null;

                if (bGroupVertices)
                {
                    Console.WriteLine("Grouping vertices...");
                    bool bCollisions = false;

                    vertexCollection = (from i in groupMap
                                            orderby geo.vertices[i].Position.X
                                            select (geo.GetVertex(i), i)).ToArray();

                    for (int i = 0; i < vertexCollection.Length; ++i)
                    {
                        for (int j = i + 1; j < vertexCollection.Length &&
                            Math.Abs(vertexCollection[j].Item1.Data.Position.X - vertexCollection[i].Item1.Data.Position.X) < groupTolerance;
                            ++j)
                        {
                            if ((vertexCollection[j].Item1.Data.Position - vertexCollection[i].Item1.Data.Position).Length < groupTolerance)
                            {
                                groupMap[i] = j;
                                bCollisions = true;
                                break;
                            }
                        }
                    }

                    // Flatten group map forest
                    for (int i = 0; i < groupMap.Length; ++i)
                    {
                        int j = i;
                        while (groupMap[j] != j)
                            j = groupMap[j];
                        groupMap[i] = j;
                    }

                    // Order the group map correctly (undo sorting)
                    var groupMapOrdered = new int[groupMap.Length];
                    for (int i = 0; i < groupMap.Length; ++i)
                        groupMapOrdered[vertexCollection[i].Item2] = vertexCollection[groupMap[i]].Item2;

                    if (bMergeVertices && bCollisions)
                    {
                        Console.WriteLine("Merging vertices...");
                        var extract = groupMapOrdered.Distinct().ToList();

                        var newVertexSet = (from i in extract
                                            select geo.vertices[i]).ToList();

                        var newGroupMap = new int[groupMap.Length];

                        var renameMap = new Dictionary<int, int>();
                        for (int i = 0; i < extract.Count; ++i)
                            renameMap[extract[i]] = i;
                        for (int i = 0; i < groupMap.Length; ++i)
                            newGroupMap[i] = renameMap[groupMapOrdered[i]];

                        // Merge normals and tangents
                        if (geo.HasNormals || geo.HasTangents)
                        {
                            // Zero data
                            for (int i = 0; i < newVertexSet.Count; ++i)
                            {
                                var data = newVertexSet[i];
                                data.Normal = OpenTK.Vector3.Zero;
                                data.Tangent = OpenTK.Vector3.Zero;
                                newVertexSet[i] = data;
                            }
                            // Add normals from old vertices to new vertices
                            for (int i = 0; i < newGroupMap.Length; ++i)
                            {
                                var data = newVertexSet[newGroupMap[i]];
                                data.Normal += geo.vertices[i].Normal;
                                data.Tangent += geo.vertices[i].Tangent;
                                newVertexSet[newGroupMap[i]] = data;
                            }
                            // Normalize result
                            for (int i = 0; i < newVertexSet.Count; ++i)
                            {
                                var data = newVertexSet[i];
                                data.Normal.Normalize();
                                data.Tangent.Normalize();
                                newVertexSet[i] = data;
                            }
                        }

                        geo.vertices = newVertexSet;
                        groupMap = newGroupMap;
                    }
                    else
                        groupMap = groupMapOrdered;
                }

                // Count edges
                int totalEdges = 0;
                foreach (var f in meshFaces)
                    totalEdges += f.IndexCount;

                var edgeToVertexVertexMap = new Tuple<int, int>[totalEdges];
                var edgeLinkedFlags = new bool[totalEdges];

                // Link up everything except for opposite edges
                Console.WriteLine("Performing initial link...");
                foreach (var f in meshFaces)
                {
                    var currentFace = geo.CreateFace();

                    Edge lastEdge = Edge.None;
                    Edge currentEdge = Edge.None;

                    for (int faceIndex = 1, lastFaceIndex = 0; ;)
                    {
                        currentEdge = geo.CreateEdge();
                        if (currentFace.Data.Edge == -1)
                            currentFace.Edge = currentEdge; // Set to first edge created

                        var vert = bMergeVertices ? geo.GetVertex(groupMap[f.Indices[faceIndex]]) : geo.GetVertex(f.Indices[faceIndex]);

                        currentEdge.Head = vert;
                        currentEdge.Face = currentFace;

                        var lastVert = bMergeVertices ? geo.GetVertex(groupMap[f.Indices[lastFaceIndex]]) : geo.GetVertex(f.Indices[lastFaceIndex]);
                        lastVert.Edge = currentEdge;

                        if (lastEdge.ID != -1) 
                            lastEdge.Next = currentEdge;

                        lastEdge = currentEdge;

                        // Add the edge to the vertex x vertex to edge map
                        vertexVertexToEdgeMap.Add(new Tuple<int, int>(groupMap[f.Indices[lastFaceIndex]], 
                            groupMap[f.Indices[faceIndex]]), currentEdge.id);
                        edgeToVertexVertexMap[currentEdge.id] = new Tuple<int, int>(groupMap[f.Indices[lastFaceIndex]],
                            groupMap[f.Indices[faceIndex]]);

                        lastFaceIndex = faceIndex;
                        faceIndex = (faceIndex + 1) % f.Indices.Count;

                        if (faceIndex == 1)
                        {
                            currentEdge.Next = currentFace.Edge; // Link up last edge with first edge to form loop
                            break;
                        }
                    }
                }

                var unmatchedEdges = new Stack<Tuple<int, int, int>>();
                Console.WriteLine("Performing opposte edge link...");
                // Link up opposite edges
                for (int edgeId = 0; edgeId < geo.edges.Count; ++edgeId)
                {
                    if (edgeLinkedFlags[edgeId])
                        continue;
                    edgeLinkedFlags[edgeId] = true;

                    var firstEdgePair = new KeyValuePair<Tuple<int, int>, int>(edgeToVertexVertexMap[edgeId], edgeId);
                    var oppositeKey = new Tuple<int, int>(firstEdgePair.Key.Item2, firstEdgePair.Key.Item1);

                    if (vertexVertexToEdgeMap.TryGetValue(oppositeKey, out int oppositeEdge))
                    { 
                        var edge1 = geo.GetEdge(firstEdgePair.Value);
                        var edge2 = geo.GetEdge(oppositeEdge);

                        edge1.Opposite = edge2;
                        edge2.Opposite = edge1;

                        edgeLinkedFlags[edge2.id] = true;
                    }
                    else
                    {
                        // This edge is unmatched! 
                        unmatchedEdges.Push(new Tuple<int, int, int>(firstEdgePair.Key.Item1,
                            firstEdgePair.Key.Item2,
                            firstEdgePair.Value));
                    }
                }

                var dummyEdges = new Stack<Edge>();
                Console.WriteLine("Creating dummy edges for holes...");
                // Create dummy opposite edges for unmatched edges
                while (unmatchedEdges.Count > 0)
                {
                    var e = unmatchedEdges.Pop();
                    var unmatchedEdge = geo.GetEdge(e.Item3);
                    var newEdge = geo.CreateEdge();
                    newEdge.Opposite = unmatchedEdge;
                    unmatchedEdge.Opposite = newEdge;
                    newEdge.Head = geo.GetVertex(e.Item1);

                    dummyEdges.Push(newEdge);
                }


                Console.WriteLine("Linking dummy edges...");
                // Link the dummy edges up correctly (note a vertex cannot be adjacent to multiple holes!)
                foreach (var edge in dummyEdges)
                {
                    var current = edge;

                    while (current.Opposite.Next.id != -1)
                    {
                        current = current.Opposite;
                        current = current.Next;
                    }

                    current = current.Opposite;
                    current.Next = edge;
                }

                // Check if mesh has a boundary
                geo.UpdateHasBoundary();

                return geo;
            }
        }

        public struct ColorScheme
        {
            public List<Clr> positiveColors;
            public List<Clr> negativeColors;
            public bool bUseNegatives;

            public static ColorScheme Default
            {
                get
                {
                    return new ColorScheme()
                    {
                        positiveColors =
                        new List<Clr>() {
                            Clr.Blue,
                            Clr.DodgerBlue,
                            Clr.DarkSeaGreen,
                            Clr.Orange,
                            Clr.Yellow,
                            Clr.White
                        },
                        negativeColors =
                        new List<Clr>() {
                            Clr.Blue,
                            Clr.Purple,
                            Clr.DarkRed,
                            Clr.Red,
                            Clr.PaleVioletRed,
                            Clr.White
                        },
                        bUseNegatives = true
                    };
                }
            }

            public static ColorScheme BlueRed
            {
                get
                {
                    return new ColorScheme()
                    {
                        positiveColors =
                        new List<Clr>()
                        {
                            Clr.White,
                            Clr.DarkRed
                        },
                        negativeColors =
                        new List<Clr>()
                        {
                            Clr.White,
                            Clr.DarkBlue
                        },
                        bUseNegatives = true
                    };
                }
            }

            public Clr GetColor(double value, double smallest, double largest)
            {
                if (largest - smallest < float.Epsilon)
                    return positiveColors[0];

                if (bUseNegatives)
                {
                    var largestMagnitude = Math.Max(Math.Abs(smallest), Math.Abs(largest));
                    value /= largestMagnitude;
                }
                else
                {
                    value = (value - smallest) / (largest - smallest);
                }

                var colors = positiveColors;
                if (value < 0.0f)
                {
                    value = -value;
                    colors = negativeColors;
                }
                value = Math.Max(0.0, Math.Min(1.0, value));

                value = value * (colors.Count - 1);
                var indx = (int)value;
                if (indx >= colors.Count - 1)
                    indx = colors.Count - 2;
                float interp = (float)Math.Min(value - (double)indx, 1.0f);

                var c1 = colors[indx];
                var c2 = colors[indx + 1];
                var cv1 = new OpenTK.Vector4(c1.R, c1.G, c1.B, c1.A);
                var cv2 = new OpenTK.Vector4(c2.R, c2.G, c2.B, c2.A);
                var cv = cv1 * (1 - interp) + interp * cv2;
                var c = new Clr(cv.X, cv.Y, cv.Z, cv.W);
                return c;
            }
        }
    }
}
