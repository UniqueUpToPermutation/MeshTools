using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using Newtonsoft.Json;

namespace GeoView
{
    namespace Geometry
    {
        struct GeometryRaw
        {
            public float[] vertexPositions;
            public float[] vertexUVs;
            public float[] vertexNormals;
            public float[] vertexTangents;
            public int[] vertexAttrEdge;
            public int[] edgeAttrNext;
            public int[] edgeAttrOpposite;
            public int[] edgeAttrHead;
            public int[] edgeAttrFace;
            public int[] faceAttrEdge;
        }

        public partial class Geometry
        {
            public void SaveToJson(string path)
            {
                using (var fileStream = new FileStream(path, FileMode.Create))
                    SaveToJson(fileStream);
            }

            public void SaveToJson(Stream s)
            {
                GeometryRaw raw = new GeometryRaw();

                using (var writer = new StreamWriter(s))
                { 
                    if (HasVertices)
                    {
                        raw.vertexPositions = new float[vertices.Count * 3];
                        for (int i = 0, j = 0; i < vertices.Count; ++i)
                        {
                            raw.vertexPositions[j++] = vertices[i].Position.X;
                            raw.vertexPositions[j++] = vertices[i].Position.Y;
                            raw.vertexPositions[j++] = vertices[i].Position.Z;
                        }
                    }

                    if (HasUVs)
                    {
                        raw.vertexUVs = new float[vertices.Count * 2];
                        for (int i = 0, j = 0; i < vertices.Count; ++i)
                        {
                            raw.vertexUVs[j++] = vertices[i].UV.X;
                            raw.vertexUVs[j++] = vertices[i].UV.Y;
                        }
                    }

                    if (HasNormals)
                    {
                        raw.vertexNormals = new float[vertices.Count * 3];
                        for (int i = 0, j = 0; i < vertices.Count; ++i)
                        {
                            raw.vertexNormals[j++] = vertices[i].Normal.X;
                            raw.vertexNormals[j++] = vertices[i].Normal.Y;
                            raw.vertexNormals[j++] = vertices[i].Normal.Z;
                        }
                    }

                    if (HasTangents)
                    {
                        raw.vertexTangents = new float[vertices.Count * 3];
                        for (int i = 0, j = 0; i < vertices.Count; ++i)
                        {
                            raw.vertexTangents[j++] = vertices[i].Tangent.X;
                            raw.vertexTangents[j++] = vertices[i].Tangent.Y;
                            raw.vertexTangents[j++] = vertices[i].Tangent.Z;
                        }
                    }

                    // Write vertex connectivity data
                    raw.vertexAttrEdge = new int[vertices.Count];
                    for (int i = 0; i < vertices.Count; ++i)
                        raw.vertexAttrEdge[i] = vertices[i].Edge;

                    // Write edge connectivity data
                    raw.edgeAttrFace = new int[edges.Count];
                    raw.edgeAttrHead = new int[edges.Count];
                    raw.edgeAttrNext = new int[edges.Count];
                    raw.edgeAttrOpposite = new int[edges.Count];
                    for (int i = 0; i < edges.Count; ++i)
                    {
                        raw.edgeAttrFace[i] = edges[i].Face;
                        raw.edgeAttrHead[i] = edges[i].Head;
                        raw.edgeAttrNext[i] = edges[i].Next;
                        raw.edgeAttrOpposite[i] = edges[i].Opposite;
                    }

                    // Write face connectivity data
                    raw.faceAttrEdge = new int[faces.Count];
                    for (int i = 0; i < faces.Count; ++i)
                        raw.faceAttrEdge[i] = faces[i].Edge;

                    var serializer = new JsonSerializer();
                    serializer.Serialize(writer, raw);
                }
            }

            public static Geometry LoadFromJson(Stream s)
            {
                Geometry geo = new Geometry();

                using (var reader = new StreamReader(s))
                {
                    var txt = reader.ReadToEnd();

                    var raw = JsonConvert.DeserializeObject<GeometryRaw>(txt);

                    RawVertex[] verts = new RawVertex[raw.vertexAttrEdge.Length];
                    RawEdge[] edges = new RawEdge[raw.edgeAttrFace.Length];
                    RawFace[] faces = new RawFace[raw.faceAttrEdge.Length];

                    geo.HasVertices = raw.vertexPositions != null;
                    geo.HasUVs = raw.vertexUVs != null;
                    geo.HasNormals = raw.vertexNormals != null;
                    geo.HasTangents = raw.vertexTangents != null;

                    if (geo.HasVertices)
                        for (int i = 0, j = 0; i < verts.Length; ++i)
                            verts[i].Position = new OpenTK.Vector3(raw.vertexPositions[j++], raw.vertexPositions[j++], raw.vertexPositions[j++]);
                    if (geo.HasUVs)
                        for (int i = 0, j = 0; i < verts.Length; ++i)
                            verts[i].UV = new OpenTK.Vector2(raw.vertexUVs[j++], raw.vertexUVs[j++]);
                    if (geo.HasNormals)
                        for (int i = 0, j = 0; i < verts.Length; ++i)
                            verts[i].Normal = new OpenTK.Vector3(raw.vertexNormals[j++], raw.vertexNormals[j++], raw.vertexNormals[j++]);
                    if (geo.HasTangents)
                        for (int i = 0, j = 0; i < verts.Length; ++i)
                            verts[i].Tangent = new OpenTK.Vector3(raw.vertexTangents[j++], raw.vertexTangents[j++], raw.vertexTangents[j++]);

                    for (int i = 0; i < verts.Length; ++i)
                        verts[i].Edge = raw.vertexAttrEdge[i];
                    for (int i = 0; i < edges.Length; ++i)
                    {
                        edges[i].Next = raw.edgeAttrNext[i];
                        edges[i].Head = raw.edgeAttrHead[i];
                        edges[i].Opposite = raw.edgeAttrOpposite[i];
                        edges[i].Face = raw.edgeAttrFace[i];
                    }
                    for (int i = 0; i < faces.Length; ++i)
                        faces[i].Edge = raw.faceAttrEdge[i];

                    geo.vertices = verts.ToList();
                    geo.edges = edges.ToList();
                    geo.faces = faces.ToList();
                }

                geo.UpdateBoundingBox();
                geo.UpdateHasBoundary();

                return geo;
            }

            public static Geometry LoadFromJson(string path)
            {
                using (var s = new FileStream(path, FileMode.Open))
                    return LoadFromJson(s);
            }
        }
    }
}
