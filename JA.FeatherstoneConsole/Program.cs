using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Drawing;

namespace JA
{
    using JA.LinearAlgebra;

    using Vector3 = JA.LinearAlgebra.Vectors.Vector3;
    using Matrix3 = JA.LinearAlgebra.Vectors.Matrix3;

    using Expr = JA.Symbolics.Expr;
    using Mesh3 = JA.LinearAlgebra.Geometry.Mesh3;
    using World = JA.Dynamics.World;
    using Motor = JA.Dynamics.Motor;
    using MassProperties = JA.Dynamics.MassProperties;
    using MotorDefined = JA.Dynamics.MotorDefined;

    internal class Program
    {
        static readonly Random rng = new Random();
        static void Main(string[] args)
        {
            //TestStackedVector();
            //TestMotorDrive();
            //TestMeshObject();
            //TestSimulationTwo();
            TestSimulationChain();
        }

        static void TestSimulationOne()
        {
            World sys = new World(UnitSystem.MKS);
            var steel = Material.Library(MaterialSpec.Steel);
            var linkage = MassProperties.Box(
                steel.ToConverted(UnitSystem.CMKS),
                30f,    // length in cm
                2f,     // height in cm
                2.111f  // thickness in cm
                ).WithMass(1);
            linkage=linkage.At(Vector3.UnitX*14.905f);
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
                var q = $"({item.Y[0].ToStringList(6)})";
                var qp = $"({item.Y[1].ToStringList(6)})";
                Console.WriteLine($"{item.t,8} {q,16} {qp,16}");
            }
        }
        static void TestSimulationTwo()
        {
            World sys = new World(UnitSystem.MKS, Vector3.Zero );
            Material steel = Material.Library(MaterialSpec.Steel);
            MassProperties linkage = MassProperties.Box(
                steel.ToConverted(UnitSystem.CMKS),
                30f,    // length in cm
                2f,     // height in cm
                2.111f  // thickness in cm
                ).WithMass(1);
            linkage=linkage.At(Vector3.UnitX*14.905f);
            Console.WriteLine(linkage);
            //linkage = linkage.ConvertTo(UnitSystem.MKS);
            //Console.WriteLine(linkage);
            var j1 = sys.NewPrismatic(Vector3.Zero, Vector3.UnitX);
            j1.InitialConditions=(0, 1);
            j1.Motor=Motor.ConstForcing(5);
            var j2 =  j1.AddRevolute(Vector3.Zero, Vector3.UnitZ);
            j2.InitialConditions=(Math.PI/6, 0);
            j2.AddMassProperties(linkage);
            Console.WriteLine(j1);
            Console.WriteLine(j2);

            Dynamics.Simulation sim = sys.ToSimulation();
            Console.WriteLine(sim);

            sim.RunTo(1.0, 20);
            Console.WriteLine($"{"Time",8} {"(q)",16} {"(qp)",16}");
            var history = sim.History;
            foreach (var item in history)
            {
                var q = $"({item.Y[0].ToStringList(6)})";
                var qp = $"({item.Y[1].ToStringList(6)})";
                Console.WriteLine($"{item.t,8:f6} {q,16} {qp,16}");
            }
        }

        static void TestSimulationChain()
        {
            Material steel = Material.Library(MaterialSpec.Steel);
            const float length = 30f;
            MassProperties linkage = MassProperties.Box(
                steel.ToConverted(UnitSystem.CMKS),
                length,  // length in cm
                2f,     // height in cm
                2.111f  // thickness in cm
                ).WithMass(1);
            var world = World.BuildSerialChain(6, length/2, linkage);
            Console.WriteLine(world);

            Dynamics.Simulation sim = world.ToSimulation();
            Console.WriteLine(sim);

            sim.RunTo(1.0, 20, out var max_residual_force);
            const int colwt = 46;
            Console.WriteLine($"{"Time",6} {"(q)",colwt} {"(qp)",colwt}");
            var history = sim.History;
            foreach (var item in history)
            {
                var q =  $"({item.Y[0].ToStringList(4)})";
                var qp = $"({item.Y[1].ToStringList(4)})";
                Console.WriteLine($"{item.t,6:f4} {q,colwt} {qp,colwt}");
            }
            Console.WriteLine($"max_residual_force = {max_residual_force}");
        }

        static void TestStackedVector()
        {
            // parts=(3,2) => s={ a, b, c | d, e }
            // s[0] = {a,b,c}   - offset=0, count=3
            // s[1] = {d,e]     - offset=3, count=2
            int  n = 2;
            StackedVector x = StackedVector.FromSizeAndCount(3, n);
            for (int i = 0; i<n; i++)
            {                
                x[i] =  Vector3.RandomVector(5);
            }
            Console.WriteLine(x.Show("vector, x="));

            StackedMatrix A = StackedMatrix.FromSizeAndCountSquare(3, n);
            for (int i = 0; i<n; i++)
            {
                for (int j = 0; j<n; j++)
                {
                    Vector3 a = Vector3.RandomVector(1);
                    Vector3 b = Vector3.RandomVector(1);
                    Matrix3 Aij = Vector3.Outer(a,b);
                    if (i==j)
                    {
                        A[i, j]=1+Aij;
                    }
                    else
                    {
                        A[i, j]=-Aij;
                    }
                }
            }
            Console.WriteLine(A.Show("matrix, A="));

            StackedVector y = A*x;

            Console.WriteLine(y.Show("vector, y=A*x"));

            StackedVector x_est = A.Solve(y, out var maxResidual); 

            Console.WriteLine((y-A*x).Show("vector, y - A*x"));

            Console.WriteLine($"max(y - A*x) = {maxResidual}");
            Console.WriteLine();
            Console.WriteLine($"check(A, x) => {A.IsCompatibleWithCols(x)}");
            Console.WriteLine($"check(A, y) => {A.IsCompatibleWithRows(y)}");

            //Console.WriteLine(Factory.Combine(y.Show("y"), "=", A.Show("A"), "*", x.Show("x")));

        }

        static void TestMotorDrive()
        {

            const double ω = 10;
            const double A = 0.2;
            {
                Expr ex_t = Motor.t;
                Expr ex_q = A*Expr.Sin(ω*ex_t);
                Expr ex_qp = ex_q.PartialDerivative(ex_t);
                Expr ex_qpp = ex_qp.PartialDerivative(ex_t);

                Console.WriteLine($"t={ex_t}");
                Console.WriteLine($"q={ex_q}");
                Console.WriteLine($"qp={ex_qp}");
                Console.WriteLine($"qpp={ex_qpp}");
            }
            Motor motor = Motor.FunctionOfTime(MotorDefined.Position,  (t)=> A*Expr.Sin(ω*t)); 

            Console.WriteLine(motor);

            Console.WriteLine($"{"t",12} {"qpp",16} {"actual",16} {"diff",16}");
            const int n = 16;
            for (int i = 0; i<=n; i++)
            {
                double t = i*Math.PI/(n*ω);
                double qpp = motor.Drive[t,0,0];
                double qpp_actual = -A*ω*ω*Math.Sin(ω*t);
                Console.WriteLine($"{t,12:F4} {qpp,16:f6} {qpp_actual,16:f6} {qpp-qpp_actual,16:g6}");
            }
        }

        static void TestMeshObject()
        {
            const string filePath = @"Models\LINK.STL";

            var origin = new System.Numerics.Vector3(0,0,10.5567f);
            if (Mesh3.ImportSTL(UnitSystem.MMGS, filePath, Color.AliceBlue, out var link, origin))
            {
                Console.WriteLine(link);

                var M = MassProperties.FromMeshAndMass(link, 1.0f);

                Console.WriteLine(M);

                double mass_expect = 1.0;
                Vector3 cg_expect = new Vector3( 149.0463, 0, 0);
                Matrix3 mmoi_expect = Matrix3.Diagonal( 70.2181, 7631.8567, 7631.7081 );

                double mass_actual = M.Mass;
                Vector3 cg_actual = M.CG;
                Matrix3 mmoi_actual = M.Mmoi;

                Console.WriteLine($"Mass: expect={mass_expect:f4}, actual={mass_actual:f4}, diff={(mass_actual-mass_expect):f4}");
                Console.WriteLine($"max delta = { (Math.Abs(mass_actual-mass_expect) / Math.Abs(mass_expect)):p2}");
                Console.WriteLine($"CG:");
                Console.WriteLine(cg_expect.ToArray().Show("expect=","f4"));
                Console.WriteLine(cg_actual.ToArray().Show("actual=","f4"));
                Console.WriteLine((cg_actual-cg_expect).ToArray().Show("diff=","f4"));
                Console.WriteLine($"max delta = { ((cg_actual-cg_expect).ToArray().MaxAbsElement(out _) / cg_expect.ToArray().MaxAbsElement(out _)):p2}");
                Console.WriteLine($"MMOI:");
                Console.WriteLine(mmoi_expect.ToArray2().Show("expect=","f4"));
                Console.WriteLine(mmoi_actual.ToArray2().Show("actual=","f4"));
                Console.WriteLine((mmoi_actual-mmoi_expect).ToArray2().Show("diff=","f4"));
                Console.WriteLine($"max delta = { ((mmoi_actual-mmoi_expect).ToArray().MaxAbsElement(out _) / mmoi_expect.ToArray().MaxAbsElement(out _)):p2}");
            }
            else
            {
                Console.WriteLine($"Error in the import of STL file {filePath}.");
            }
        }
    }
}
