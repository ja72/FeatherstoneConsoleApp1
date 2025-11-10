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
    using System.Diagnostics;

    internal class Program
    {
        static readonly Random rng = new Random();
        static void Main(string[] args)
        {
            TestStackedVector();
            TestMotorDrive();
            //TestSimulationTwo();
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
            var j1 = sys.NewPrismatic(Vector3.Zero, Vector3.UnitX);
            j1.InitialConditions=(0, 1);
            j1.Motor=JointBody.ConstantValue(5);
            var joint =  j1.AddRevolute(Vector3.Zero, Vector3.UnitZ);
            joint.InitialConditions=(Math.PI/6, 0);
            joint.AddMassProperties(linkage);
            Console.WriteLine(j1);
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

        static void TestStackedVector()
        {
            int  n = 2;
            StackedVector x = StackedVector.FromSizeAndCount(3, n);
            for (int i = 0; i<n; i++)
            {
                x[i]= Vector3.RandomVector(5);
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
    }
}
