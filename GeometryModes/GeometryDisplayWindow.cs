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

namespace GeometryModes
{
    enum GeometryVisualMode
    {
        ViewMesh,
        ViewModes
    }

    enum GeometryShader
    {
        BlinnPhong,
        CookTorrance,
        SimpleColor
    }

    class GeometryDisplayWindow : GameWindow
    {
        int triangleVertexBuffer = -1;

        Geometry.Geometry geometry;
        Camera camera = new Camera();
        CameraController controller;
        GeometryVisualMode visualMode;
        SimpleColorShader simpleShader;
        ColoredCookTorranceShader cookShader;
        BlinnPhongShader phongShader;

        int currentMode = 0;

        public GeometryShader DefaultShader { get; set; } = GeometryShader.CookTorrance;

        public GeometryVisualMode VisualMode
        {
            get { return visualMode; }
            set
            {
                visualMode = value;
                if (triangleVertexBuffer != -1)
                    UpdateVisualization();
            }
        }

        public Mat ObjectModes { get; set; }
        public Vec ObjectEigenvalues { get; set; }

        public GeometryDisplayWindow(Geometry.Geometry geometry) : 
            base(800, 600, new GraphicsMode(32, 24, 8, 8), "Result Viewer")
        {
            this.geometry = geometry;

            camera.distanceFromCenter = (geometry.BoundingBox.Upper - geometry.BoundingBox.Lower).Length * 1.0f;
            camera.moveSpeed = (geometry.BoundingBox.Upper - geometry.BoundingBox.Lower).Length * 0.01f;
            camera.rotationSpeed = 0.01f;

            controller = new CameraController(camera);
            controller.Attach(this);
        }

        protected void UpdateVisualization()
        {
            if (visualMode == GeometryVisualMode.ViewModes)
                geometry.VisualizeVertexFunction(ObjectModes.Column(currentMode), Geometry.ColorScheme.Default);
            else if (visualMode == GeometryVisualMode.ViewMesh)
            {
                for (int i = 0; i < geometry.vertices.Count; ++i)
                {
                    var data = geometry.vertices[i];
                    data.Color = Color4.White;
                    geometry.vertices[i] = data;
                }
            }
            GenerateModelBuffers();
        }

        protected override void OnKeyDown(KeyboardKeyEventArgs e)
        {
            if (e.Key == Key.Escape)
                Exit();

            if (ObjectModes != null)
            {
                if (e.Key == Key.Plus)
                {
                    ++currentMode;
                    currentMode = Math.Min(currentMode, ObjectModes.ColumnCount - 1);
                    UpdateVisualization();
                }
                if (e.Key == Key.Minus)
                {
                    --currentMode;
                    currentMode = Math.Max(currentMode, 0);
                    UpdateVisualization();
                }
            }

            base.OnKeyDown(e);
        }

        protected void GenerateModelBuffers()
        {
            float[] vertData = geometry.CreateUnindexedVertexData(11, true, 0, true, 8,
                true, 3, true, 5, false, 0);

            if (triangleVertexBuffer == -1)
                triangleVertexBuffer = GL.GenBuffer();

            GL.BindBuffer(BufferTarget.ArrayBuffer, triangleVertexBuffer);
            GL.BufferData(BufferTarget.ArrayBuffer, vertData.Length * sizeof(float),
                vertData, BufferUsageHint.StaticDraw);
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            
            UpdateVisualization();

            simpleShader = new SimpleColorShader();
            simpleShader.Load();

            cookShader = new ColoredCookTorranceShader();
            cookShader.Load();

            phongShader = new BlinnPhongShader();
            phongShader.Load();
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            GL.Enable(EnableCap.DepthTest);
            GL.ClearColor(0.5f, 0.5f, 1.0f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            GL.BindBuffer(BufferTarget.ArrayBuffer, triangleVertexBuffer);

            GL.EnableVertexAttribArray(0);
            GL.EnableVertexAttribArray(1);
            GL.EnableVertexAttribArray(2);
            GL.EnableVertexAttribArray(3);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 11 * sizeof(float), 0);
            GL.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, 11 * sizeof(float), 3 * sizeof(float));
            GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 11 * sizeof(float), 5 * sizeof(float));
            GL.VertexAttribPointer(3, 3, VertexAttribPointerType.Float, false, 11 * sizeof(float), 8 * sizeof(float));

            var worldMat = Matrix4.Identity;
            var viewMat = camera.ViewMatrix;
            var projMat = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 4.0f, ((float)Width) / ((float)Height), 0.1f, 100.0f);
            var invWorldMat = worldMat;
            invWorldMat.Invert();
            invWorldMat.Transpose();
            var lightDirection = new Vector3(-1.0f, -1.0f, -1.0f);
            lightDirection.Normalize();

            GeometryShader shaderToUse = DefaultShader;
            if (VisualMode == GeometryVisualMode.ViewModes)
                shaderToUse = GeometryShader.SimpleColor;

            switch (shaderToUse)
            {
                case GeometryShader.CookTorrance:
                    cookShader.Enable();
                    cookShader.World.Set(worldMat);
                    cookShader.View.Set(viewMat);
                    cookShader.Projection.Set(projMat);
                    cookShader.WorldInverseTranspose.Set(invWorldMat);
                    cookShader.LightPosition.Set(new Vector3(10.0f, 10.0f, 10.0f));
                    cookShader.EyePosition.Set(camera.Position);
                    cookShader.AmbientStrength.Set(0.15f);
                    cookShader.LightIntensity.Set(0.8f);
                    break;

                case GeometryShader.BlinnPhong:
                    phongShader.Enable();
                    phongShader.World.Set(worldMat);
                    phongShader.View.Set(viewMat);
                    phongShader.Projection.Set(projMat);
                    phongShader.WorldInverseTranspose.Set(invWorldMat);
                    phongShader.LightDirection.Set(lightDirection);
                    phongShader.EyePosition.Set(camera.Position);
                    phongShader.AmbientIntensity.Set(0.1f);
                    phongShader.LightIntensity.Set(0.7f);
                    break;

                case GeometryShader.SimpleColor:
                    simpleShader.Enable();
                    simpleShader.World.Set(worldMat);
                    simpleShader.View.Set(viewMat);
                    simpleShader.Projection.Set(projMat);
                    break;
            }

            GL.DrawArrays(PrimType.Triangles, 0, geometry.FaceCount * 3);

            SwapBuffers();

            base.OnRenderFrame(e);
        }

        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(0, 0, Width, Height);
            base.OnResize(e);
        }

        protected override void OnUnload(EventArgs e)
        {
            GL.DeleteBuffer(triangleVertexBuffer);
            simpleShader.Unload();
            cookShader.Unload();
            base.OnUnload(e);
        }
    }
}
