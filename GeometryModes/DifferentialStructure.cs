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

using np = Numpy.np;

namespace GeoView
{
    namespace Geometry
    {
        public class DifferentialStructure
        {
            protected Geometry geometry;

            public int EdgeDimension
            {
                get { return geometry.EdgeCount; }
            }

            public int FaceDimension
            {
                get { return geometry.FaceCount; }
            }

            public int VertexDimension
            {
                get { return geometry.VertexCount; }
            }

            public int InteriorVertexDimension
            {
                get { return geometry.VertexCount - geometry.BoundaryVertexCount; }
            }

            public Mat PositionMatrix
            {
                get
                {
                    var mat = Mat.Build.Dense(VertexDimension, 3);
                    for (int i = 0; i < VertexDimension; ++i)
                    {
                        mat[i, 0] = geometry.vertices[i].Position.X;
                        mat[i, 1] = geometry.vertices[i].Position.Y;
                        mat[i, 2] = geometry.vertices[i].Position.Z;
                    }
                    return mat;
                }
            }

            public Mat NormalMatrix
            {
                get
                {
                    var mat = Mat.Build.Dense(VertexDimension, 3);
                    for (int i = 0; i < VertexDimension; ++i)
                    {
                        mat[i, 0] = geometry.vertices[i].Normal.X;
                        mat[i, 1] = geometry.vertices[i].Normal.Y;
                        mat[i, 2] = geometry.vertices[i].Normal.Z;
                    }
                    return mat;
                }
            }

            public Mat TangentMatrix
            {
                get
                {
                    var mat = Mat.Build.Dense(VertexDimension, 3);
                    for (int i = 0; i < VertexDimension; ++i)
                    {
                        mat[i, 0] = geometry.vertices[i].Tangent.X;
                        mat[i, 1] = geometry.vertices[i].Tangent.Y;
                        mat[i, 2] = geometry.vertices[i].Tangent.Z;
                    }
                    return mat;
                }
            }

            public IEnumerable<Tuple<int, int, double>> EnumWeakLaplacian()
            {
                foreach (var v in geometry.Vertices)
                {
                    var totalWeight = 0.0d;
                    foreach (var e in v.OutgoingEdges)
                    {
                        // Calculate cot(alpha)
                        var cotAlpha = 0.0d;
                        if (e.Face.id != -1)
                        {
                            var v1 = e.Next.Direction;
                            var v2 = -e.Next.Next.Direction;
                            cotAlpha = OpenTK.Vector3.Dot(v1, v2) / Math.Abs(OpenTK.Vector3.Cross(v1, v2).Length);
                        }

                        // Calculate cot(beta)
                        var cotBeta = 0.0d;
                        if (e.Opposite.Face.id != -1)
                        {
                            var v1 = e.Opposite.Next.Direction;
                            var v2 = -e.Opposite.Next.Next.Direction;
                            cotBeta = OpenTK.Vector3.Dot(v1, v2) / Math.Abs(OpenTK.Vector3.Cross(v1, v2).Length);
                        }

                        var weight = (cotAlpha + cotAlpha) / 2.0d;
                        totalWeight += weight;

                        yield return new Tuple<int, int, double>(v.id, e.Head.id, weight);
                    }
                    yield return new Tuple<int, int, double>(v.id, v.id, -totalWeight);
                }
            }

            public Mat DenseLaplacian
            {
                get
                {
                    return Mat.Build.DenseOfMatrix(Laplacian);
                }
            }

            public Mat DenseWeakLaplacian
            {
                get
                {
                    return Mat.Build.DenseOfMatrix(WeakLaplacian);
                }
            }

            public IEnumerable<Tuple<int, int, double>> EnumMassMatrix()
            {
                foreach (var v in geometry.Vertices)
                {
                    var totalArea = 0.0d;
                    foreach (var f in v.AdjacentFaces)
                        totalArea += f.TriangleArea;
                    var mass = totalArea / 3.0d;
                    yield return new Tuple<int, int, double>(v.id, v.id, mass);
                }
            }

            public DifferentialStructure(Geometry geometry)
            {
                this.geometry = geometry;
            }

            public Mat WeakLaplacian
            {
                get
                {
                    return Mat.Build.SparseOfIndexed(VertexDimension, VertexDimension, EnumWeakLaplacian());
                }
            }

            public Mat MassMatrix
            {
                get
                {
                    return Mat.Build.SparseOfIndexed(VertexDimension, VertexDimension, EnumMassMatrix());
                }
            }

            public Mat InverseMassMatrix
            {
                get
                {
                    var invMass = EnumMassMatrix().Select(t => new Tuple<int, int, double>(t.Item1, t.Item2, 1.0d / t.Item3));
                    return Mat.Build.SparseOfIndexed(VertexDimension, VertexDimension, invMass);
                }
            }

            public Mat HalfInverseMassMatrix
            {
                get
                {
                    var invMass = EnumMassMatrix().Select(t => new Tuple<int, int, double>(t.Item1, t.Item2, 1.0d / Math.Sqrt(t.Item3)));
                    return Mat.Build.SparseOfIndexed(VertexDimension, VertexDimension, invMass);
                }
            }

            public Mat HalfMassMatrix
            {
                get
                {
                    var invMass = EnumMassMatrix().Select(t => new Tuple<int, int, double>(t.Item1, t.Item2, Math.Sqrt(t.Item3)));
                    return Mat.Build.SparseOfIndexed(VertexDimension, VertexDimension, invMass);
                }
            }

            public Mat Laplacian
            {
                get
                {
                    return InverseMassMatrix * WeakLaplacian;
                }
            }

            public Mat InteriorLaplacian
            {
                get
                {
                    if (geometry.HasBoundary)
                        return ClosureToInteriorMap * Laplacian * InteriorToClosureMap;
                    else
                        return Laplacian;
                }
            }

            public Mat SymmetrizedLaplacian
            {
                get
                {
                    var halfInvMass = HalfInverseMassMatrix;
                    return halfInvMass * WeakLaplacian * halfInvMass;
                }
            }

            public Mat InteriorSymmetrizedLaplacian
            {
                get
                {
                    if (geometry.HasBoundary)
                        return ClosureToInteriorMap * SymmetrizedLaplacian * InteriorToClosureMap;
                    else
                        return SymmetrizedLaplacian;
                }
            }

            public Vec MassVector
            {
                get
                {
                    var masses = EnumMassMatrix();
                    var massesTransformed = masses.Select(t => t.Item3).ToArray();
                    return Vec.Build.Dense(massesTransformed);
                }
            }

            public static void WriteSparseMatrix(Mat mat, string filename)
            {
                var sp = mat as Linalg.Double.SparseMatrix;

                if (sp != null)
                {
                    var nnzCount = sp.NonZerosCount;
                    var enumer = sp.EnumerateIndexed(Linalg.Zeros.AllowSkip);

                    var npDim = np.array(mat.RowCount, mat.ColumnCount);

                    var loc = new int[nnzCount, 2];
                    var val = new double[nnzCount];

                    int i = 0;
                    foreach (var entry in enumer)
                    {
                        loc[i, 0] = entry.Item1;
                        loc[i, 1] = entry.Item2;
                        val[i] = entry.Item3;
                        ++i;
                    }

                    var npLoc = np.array(loc);
                    var npVal = np.array(val);

                    var arr = new Numpy.NDarray[] { npDim, npLoc, npVal };
                    np.savez(filename, arr);
                }
            }

            public IEnumerable<Tuple<int, int, double>> EnumClosureToInteriorMap()
            {
                int interior_id = 0;
                for (int i = 0; i < geometry.vertices.Count; ++i)
                {
                    if (geometry.GetVertex(i).IsInInterior)
                    {
                        yield return new Tuple<int, int, double>(interior_id, i, 1.0d);
                        ++interior_id;
                    }
                }
            }

            public Mat ClosureToInteriorMap
            {
                get
                {
                    if (geometry.HasBoundary)
                        return Mat.Build.SparseOfIndexed(InteriorVertexDimension, VertexDimension, EnumClosureToInteriorMap());
                    else
                        return Mat.Build.SparseIdentity(VertexDimension);
                }
            }

            public Mat InteriorToClosureMap
            {
                get
                {
                    return ClosureToInteriorMap.Transpose();
                }
            }

            public static Mat ReadMatrix(string filename)
            {
                var arr = np.load(filename);
                var shape = arr.shape;
                var rows = shape.Dimensions[0];
                var cols = shape.Dimensions[1];

                Mat result = Mat.Build.Dense(rows, cols);

                for (int i = 0; i < rows; ++i)
                    for (int j = 0; j < cols; ++j)
                        result[i, j] = (double)arr[i, j];

                return result;
            }

            public static void ReadModeData(string filename, out Mat modes, out Vec eigs)
            {
                var arrz = np.load(filename);

                var shape = arrz.shape;
                var rows = shape.Dimensions[0] - 1;
                var cols = shape.Dimensions[1];
                modes = Mat.Build.Dense(rows, cols);

                var data = arrz.GetData<double>();

                for (int i = 1; i <= rows; ++i)
                    for (int j = 0; j < cols; ++j)
                        modes[i - 1, j] = data[i * cols + j];

                eigs = Vec.Build.Dense(cols);

                for (int i = 0; i < cols; ++i)
                    eigs[i] = data[i];
            }

            public static void ReadFunctionData(string filename, out Mat funcs)
            {
                var arrz = np.load(filename);

                var shape = arrz.shape;
                var rows = shape.Dimensions[0];
                var cols = shape.Dimensions[1];
                funcs = Mat.Build.Dense(rows, cols);

                var data = arrz.GetData<double>();

                for (int i = 0; i <= rows; ++i)
                    for (int j = 0; j < cols; ++j)
                        funcs[i, j] = data[i * cols + j];
            }
        }
    }
}
