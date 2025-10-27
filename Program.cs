using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using JA.VectorCalculus;
using JA.ScrewCalculus;
using JA.Dynamics;

namespace JA
{
    internal class Program
    {
        static void Main(string[] args)
        {
            World sys = new World(-10*Vector3.UnitY);
            var steel = Material.Library(MaterialSpec.Steel);
            float L = 1.0f, m = 1.0f;
            var box = MassProperties.Box(steel, L, L/100, L/100).WithMass(m);
            var joint = Joint.NewRevolute(Vector3.Zero, Vector3.UnitZ);
            joint.AddMassProperties(box.At(Vector3.UnitX * L/2));
        }
    }
}
