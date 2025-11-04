using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA
{
    using JA.LinearAlgebra;
    using JA.LinearAlgebra.VectorCalculus;
    using JA.LinearAlgebra.ScrewCalculus;

    using JA.Dynamics;
    
    internal class Program
    {
        static void Main(string[] args)
        {
            StackedVector stacekdVector = new StackedVector(3, 3);
            stacekdVector[0] = 1 * Vector3.UnitX;
            stacekdVector[1] = 3 * Vector3.UnitZ;
            Console.WriteLine($"vector={stacekdVector.ToVector()}");

            StackedMatrix stackedMatrix = StackedMatrix.CompatibleWith(stacekdVector);
            stackedMatrix[0, 0]=Matrix3.Identity;
            stackedMatrix[0, 1]=Matrix3.Zero;
            stackedMatrix[1, 0]=-Matrix3.Identity;
            stackedMatrix[1, 1]=Matrix3.Identity;

            Console.WriteLine($"matrix={stackedMatrix.ToMatrix()}");

            World sys = new World(UnitSystem.MKS);
            var steel = Material.Library(MaterialSpec.Steel);
            var linkage = MassProperties.Box(
                steel.ConvertTo(UnitSystem.CMKS),
                30f,    // length in cm
                2f,     // height in cm
                2.111f  // thickness in cm
                ).WithMass(1);
            linkage = linkage.At(Vector3.UnitX * 14.905f);
            Console.WriteLine(linkage);
            //linkage = linkage.ConvertTo(UnitSystem.MKS);
            //Console.WriteLine(linkage);
            var joint = sys.NewRevolute(Vector3.Zero, Vector3.UnitZ);
            joint.AddMassProperties(linkage);
            Console.WriteLine(joint);

            var sim = sys.ToSimulation();
            Console.WriteLine(sim);

            sim.RunTo(1.0, 20);
            Console.WriteLine($"{"Time",8} {"(q)",16} {"(qp)",16}");
            var history = sim.History;
            foreach (var item in history)
            {
                var q = $"({item.Y[0].ToStringList()})";
                var qp = $"({item.Y[1].ToStringList()})";
                Console.WriteLine($"{item.t,8} {q,16} {qp,16}");
            }
        }
    }
}
