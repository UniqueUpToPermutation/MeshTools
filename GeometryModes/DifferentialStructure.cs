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

namespace GeometryModes
{
    namespace Geometry
    {
        public class DifferentialStructure
        {
            protected Geometry geometry;
            protected Mat gradient;
            protected Mat divergence;
            protected Mat curl;
            protected Mat laplacian;

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

            public DifferentialStructure(Geometry geometry)
            {
                this.geometry = geometry;

                var gradPos = from e in geometry.Edges
                              select new Tuple<int, int, double>(e.id, e.Head.id,
                                    1.0d / (2.0d * (e.Head.Data.Position - e.Tail.Data.Position).Length));
                var gradNeg = from e in geometry.Edges
                              select new Tuple<int, int, double>(e.id, e.Tail.id,
                                    -1.0d / (2.0d * (e.Head.Data.Position - e.Tail.Data.Position).Length));
                var gradEnum = Enumerable.Union(gradPos, gradNeg);

                gradient = Mat.Build.SparseOfIndexed(EdgeDimension, VertexDimension, gradEnum);
                
                divergence = gradient.Transpose() * 2.0d;
                laplacian = divergence * gradient;

                var curlEnum = from f in geometry.Faces
                               from e in f.Edges
                               select new Tuple<int, int, double>(f.id, e.id,
                               (2.0d * (e.Head.Data.Position - e.Tail.Data.Position).Length) / f.TriangleArea);

                curl = Mat.Build.SparseOfIndexed(FaceDimension, EdgeDimension, curlEnum);
            }

            public Vec Gradient(Vec vertexFunc)
            {
                return gradient * vertexFunc;
            }

            public Mat GradientMatrix
            {
                get { return gradient; }
            }

            public Vec Divergence(Vec edgeFunc)
            {
                return divergence * edgeFunc;
            }

            public Mat DivergenceMatrix
            {
                get { return divergence; }
            }

            public Vec Laplacian(Vec vertexFunc)
            {
                return laplacian * vertexFunc;
            }

            public Mat LaplacianMatrix
            {
                get { return laplacian; }
            }

            public Vec Curl(Vec edgeFunc)
            {
                return curl * edgeFunc;
            }

            public Mat CurlMatrix
            {
                get { return curl; }
            }

            public static void WriteSparseMatrix(Mat mat, string filename)
            {
                var sp = (mat as MathNet.Numerics.LinearAlgebra.Double.SparseMatrix);

                if (sp != null)
                {
                    var nnzCount = sp.NonZerosCount;
                    var enumer = sp.EnumerateIndexed(Linalg.Zeros.AllowSkip);

                    var npDim = np.array<int>(mat.RowCount, mat.ColumnCount);

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

                for (int i = 1; i < rows; ++i)
                    for (int j = 0; j < cols; ++j)
                        modes[i, j] = data[cols * i + j];

                eigs = Vec.Build.Dense(cols);

                for (int i = 0; i < cols; ++i)
                    eigs[i] = data[i];
            }
        }
    }
}
