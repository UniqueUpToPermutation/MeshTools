using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Platform;
using OpenTK;

using System.IO;
using PrimType = OpenTK.Graphics.OpenGL.PrimitiveType;
using OpenTK.Input;
using Mat = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vec = MathNet.Numerics.LinearAlgebra.Vector<double>;

using GeometryModes.Geometry;

namespace GeometryModes
{
    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length == 0)
            {
                Console.WriteLine("No arguments!");
            }

            var objectFile = args[0];

            Console.WriteLine("Loading mesh...");

            Geometry.Geometry geo;
            if (Path.GetExtension(objectFile) == ".hfe")
            {
                // Loading an hfe file
                geo = Geometry.Geometry.Load(objectFile);
            }
            else
            {
                // Loading an external file format
                var importer = new Assimp.AssimpContext();
                importer.SetConfig(new Assimp.Configs.NormalSmoothingAngleConfig(66.0f));
                var scene = importer.ImportFile(objectFile, Assimp.PostProcessPreset.TargetRealTimeMaximumQuality);

                if (!scene.HasMeshes)
                {
                    Console.WriteLine("Scene has no meshes!");
                    return;
                }

                Console.WriteLine("Processing mesh...");
                geo = Geometry.Geometry.FromAssimp(scene.Meshes[0], true, true);
                scene.Clear();
                importer.Dispose();
            }

            bool bUseSymmetricLaplacian = false;
            var indx = Array.FindIndex(args, t => t == "-sym");
            if (indx != -1)
            {
                Console.WriteLine("Using symmetric laplacian...");
                bUseSymmetricLaplacian = true;
            }

            indx = Array.FindIndex(args, t => t == "-lapout");
            if (indx != -1)
            {
                Console.WriteLine("Creating differential structure...");
                var diff = new DifferentialStructure(geo);

                Console.WriteLine("Writing mesh laplacian to file...");
                var outputFile = args[indx + 1];

                if (bUseSymmetricLaplacian)
                    DifferentialStructure.WriteSparseMatrix(diff.InteriorSymmetrizedLaplacian, outputFile);
                else
                    DifferentialStructure.WriteSparseMatrix(diff.InteriorLaplacian, outputFile);
            }

            indx = Array.FindIndex(args, t => t == "-modesin");
            Mat modes = null;
            Vec spec = null;
            GeometryVisualMode visMode = GeometryVisualMode.ViewMesh;
            if (indx != -1)
            {
                Console.WriteLine("Reading mode data...");
                var inputFile = args[indx + 1];
                DifferentialStructure.ReadModeData(inputFile, out modes, out spec);

                var diff = new DifferentialStructure(geo);
                if (bUseSymmetricLaplacian)
                    modes = diff.HalfInverseMassMatrix * modes;

                if (geo.HasBoundary)
                    modes = diff.InteriorToClosureMap * modes;

                visMode = GeometryVisualMode.ViewModes;
            }

            indx = Array.FindIndex(args, t => t == "-viewmass");
            if (indx != -1)
                visMode = GeometryVisualMode.ViewDegreeVector;

            indx = Array.FindIndex(args, t => t == "-viewlapdiag");
            if (indx != -1)
                visMode = GeometryVisualMode.ViewLapDiagonal;

            indx = Array.FindIndex(args, t => t == "-meshout");
            if (indx != -1)
            {
                Console.WriteLine("Saving processed mesh...");
                geo.Save(args[indx + 1]);
            }

            indx = Array.FindIndex(args, t => t == "-noview");
            if (indx != -1)
                return;

            using (var window = new GeometryDisplayWindow(geo))
            {
                window.VisualMode = visMode;
                window.ObjectModes = modes;
                window.ObjectEigenvalues = spec;
                window.Run();
            }
        }
    }
}
