using System;
using System.Collections.Generic;
using System.Linq;
using OpenTK.Graphics.OpenGL;
using OpenTK;

using System.IO;

namespace GeometryModes
{
    public struct ShaderUniform
    {
        public int Location;
        public ShaderUniform(int location)
        {
            Location = location;
        }

        public void Set(float x)
        {
            GL.Uniform1(Location, x);
        }
        public void Set(double x)
        {
            GL.Uniform1(Location, x);
        }
        public void Set(float x, float y)
        {
            GL.Uniform2(Location, x, y);
        }
        public void Set(double x, double y)
        {
            GL.Uniform2(Location, x, y);
        }
        public void Set(Vector2 vec)
        {
            GL.Uniform2(Location, ref vec);
        }
        public void Set(float x, float y, float z)
        {
            GL.Uniform3(Location, x, y, z);
        }
        public void Set(double x, double y, double z)
        {
            GL.Uniform3(Location, x, y, z);
        }
        public void Set(Vector3 vec)
        {
            GL.Uniform3(Location, ref vec);
        }
        public void Set(float x, float y, float z, float w)
        {
            GL.Uniform4(Location, x, y, z, w);
        }
        public void Set(double x, double y, double z, double w)
        {
            GL.Uniform4(Location, x, y, z, w);
        }
        public void Set(Vector4 vec)
        {
            GL.Uniform4(Location, ref vec);
        }
        public void Set(Matrix2 mat, bool bTranspose = false)
        {
            GL.UniformMatrix2(Location, bTranspose, ref mat);
        }
        public void Set(Matrix3 mat, bool bTranspose = false)
        {
            GL.UniformMatrix3(Location, bTranspose, ref mat);
        }
        public void Set(Matrix4 mat, bool bTranspose = false)
        {
            GL.UniformMatrix4(Location, bTranspose, ref mat);
        }
    }

    public class ShaderUniformAttribute : Attribute
    {
        public readonly string UniformName;

        public ShaderUniformAttribute(string name)
        {
            UniformName = name;
        }
    }

    public class EmbeddedShaderAttribute : Attribute
    {
        public readonly string[] ShaderLocations;
        public readonly ShaderType[] ShaderTypes;

        public EmbeddedShaderAttribute(params object[] locationsAndTypes)
        {
            var locs = new List<string>();
            var types = new List<ShaderType>();

            for (int i = 0; i < locationsAndTypes.Length; i += 2)
            {
                var location = locationsAndTypes[i] as string;
                var type = (ShaderType)locationsAndTypes[i + 1];

                locs.Add(location);
                types.Add(type);
            }

            ShaderLocations = locs.ToArray();
            ShaderTypes = types.ToArray();
        }
    }

    public abstract class ShaderProgram : IRenderObject
    {
        public List<int> Shaders = new List<int>();
        protected int ShaderProgramID = -1;

        public virtual void Attach(Renderer renderer)
        {
            renderer.OnUnload += Unload;
        }

        public virtual void Unattach(Renderer renderer)
        {
            renderer.OnUnload -= Unload;
        }

        public virtual void Unload()
        {
            foreach (var member in Shaders)
            {
                GL.DetachShader(ShaderProgramID, member);
                GL.DeleteShader(member);
            }

            if (ShaderProgramID != -1)
                GL.DeleteProgram(ShaderProgramID);
        }

        public virtual void Load()
        {
            // Load the appropriate shaders
            bool bFail = false;
            if (ShaderProgramID == -1)
            {
                ShaderProgramID = GL.CreateProgram();
                var attr = Attribute.GetCustomAttribute(GetType(), typeof(EmbeddedShaderAttribute));

                if (attr != null)
                {
                    var embeddedAttr = attr as EmbeddedShaderAttribute;
                    for (int i = 0; i < embeddedAttr.ShaderLocations.Length; ++i)
                    {
                        var shader = LoadShaderFromEmbeddedResource(embeddedAttr.ShaderLocations[i], embeddedAttr.ShaderTypes[i], out bFail);

                        if (bFail)
                        {
                            Console.WriteLine($"Failed to compile shader {embeddedAttr.ShaderLocations[i]}! Aborting linkage process!");
                            return;
                        }

                        Shaders.Add(shader);
                        GL.AttachShader(ShaderProgramID, shader);
                    }
                }

                GL.LinkProgram(ShaderProgramID);
            }


            // Set marked uniform locations to their correct values
            var markedMembers = GetType().GetProperties().Where(
                mem => Attribute.IsDefined(mem, typeof(ShaderUniformAttribute)));

            foreach (var member in markedMembers)
            {
                var attrs = member.GetCustomAttributes(false);

                var a = attrs.Select(t => t as ShaderUniformAttribute).FirstOrDefault(t => t != null);
                if (a != null)
                {
                    ShaderUniform unif;
                    unif.Location = GL.GetUniformLocation(ShaderProgramID, a.UniformName);
                    if (unif.Location == -1)
                        Console.WriteLine($"Error: {a.UniformName} not found!");
                    member.SetValue(this, unif);
                }
            }
        }

        public void Enable()
        {
            GL.UseProgram(ShaderProgramID);
        }

        public static int LoadShaderFromStream(Stream s, ShaderType type, out bool hasFailed)
        {
            hasFailed = false;
            string contents;
            using (var reader = new StreamReader(s))
                contents = reader.ReadToEnd();
            var shaderID = GL.CreateShader(type);
            GL.ShaderSource(shaderID, contents);
            GL.CompileShader(shaderID);
            string infoLogVert = GL.GetShaderInfoLog(shaderID);
            if (infoLogVert != String.Empty)
            {
                hasFailed = true;
                Console.WriteLine(infoLogVert);
            }
            return shaderID;
        }

        public int GetUniformLoc(string name)
        {
            return GL.GetUniformLocation(ShaderProgramID, name);
        }

        public static int LoadShaderFromEmbeddedResource(string name, ShaderType type, out bool hasFailed)
        {
            Stream stream = typeof(Program).Assembly.GetManifestResourceStream(name);
            return LoadShaderFromStream(stream, type, out hasFailed);
        }

        protected static int LoadShaderFromFile(string filename, ShaderType type, out bool hasFailed)
        {
            Stream stream = new FileStream(filename, FileMode.Open);
            return LoadShaderFromStream(stream, type, out hasFailed);
        }
    }

    [EmbeddedShader("GeometryModes.Shaders.SimpleColor.vert", ShaderType.VertexShader,
        "GeometryModes.Shaders.SimpleColor.frag", ShaderType.FragmentShader)]
    public class SimpleColorShader : ShaderProgram
    {
        [ShaderUniform("worldMatrix")]
        public ShaderUniform World { get; set; }
        [ShaderUniform("viewMatrix")]
        public ShaderUniform View { get; set; }
        [ShaderUniform("projectionMatrix")]
        public ShaderUniform Projection { get; set; }
    }

    [EmbeddedShader("GeometryModes.Shaders.BlinnPhong.vert", ShaderType.VertexShader,
        "GeometryModes.Shaders.BlinnPhong.frag", ShaderType.FragmentShader)]
    class BlinnPhongShader : ShaderProgram
    {
        [ShaderUniform("worldMatrix")]
        public ShaderUniform World { get; set; }
        [ShaderUniform("viewMatrix")]
        public ShaderUniform View { get; set; }
        [ShaderUniform("projectionMatrix")]
        public ShaderUniform Projection { get; set; }
        [ShaderUniform("worldInverseTranspose")]
        public ShaderUniform WorldInverseTranspose { get; set; }
        [ShaderUniform("specularColor")]
        public ShaderUniform SpecularColor { get; set; }
        [ShaderUniform("lightColor")]
        public ShaderUniform LightColor { get; set; }
        [ShaderUniform("lightDirection")]
        public ShaderUniform LightDirection { get; set; }
        [ShaderUniform("ambientColor")]
        public ShaderUniform AmbientColor { get; set; }
        [ShaderUniform("ambientIntensity")]
        public ShaderUniform AmbientIntensity { get; set; }
        [ShaderUniform("shininess")]
        public ShaderUniform Shininess { get; set; }
        [ShaderUniform("eyePosition")]
        public ShaderUniform EyePosition { get; set; }
        [ShaderUniform("lightIntensity")]
        public ShaderUniform LightIntensity { get; set; }
    }

    [EmbeddedShader("GeometryModes.Shaders.CookTorrance.vert", ShaderType.VertexShader,
       "GeometryModes.Shaders.CookTorrance.frag", ShaderType.FragmentShader)]
    class ColoredCookTorranceShader : ShaderProgram
    {
        [ShaderUniform("model")]
        public ShaderUniform World { get; set; }
        [ShaderUniform("modelInverseTranspose")]
        public ShaderUniform WorldInverseTranspose { get; set; }
        [ShaderUniform("view")]
        public ShaderUniform View { get; set; }
        [ShaderUniform("projection")]
        public ShaderUniform Projection { get; set; }
        [ShaderUniform("eyePosition")]
        public ShaderUniform EyePosition { get; set; }
        [ShaderUniform("lightPosition")]
        public ShaderUniform LightPosition { get; set; }
        [ShaderUniform("specularColor")]
        public ShaderUniform SpecularColor { get; set; }
        [ShaderUniform("F0")]
        public ShaderUniform F0 { get; set; }
        [ShaderUniform("roughness")]
        public ShaderUniform Roughness { get; set; }
        [ShaderUniform("k")]
        public ShaderUniform K { get; set; }
        [ShaderUniform("lightColor")]
        public ShaderUniform LightColor { get; set; }
        [ShaderUniform("ambientStrength")]
        public ShaderUniform AmbientStrength { get; set; }
        [ShaderUniform("lightIntensity")]
        public ShaderUniform LightIntensity { get; set; }
    }
}
