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

namespace GeoView
{
    public interface IRenderObject
    {
        void Attach(Renderer renderer);
        void Unattach(Renderer renderer);
    }

    public class Renderer
    {
        public Action OnUnload;
    }
}
