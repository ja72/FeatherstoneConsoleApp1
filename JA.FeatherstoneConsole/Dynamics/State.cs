using System;
using System.ComponentModel;

using JA.LinearAlgebra;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public enum JointKnown
    {
        [Description("Joint Forcing is known, the inverse dynamics problem.")]
        Forcing,
        [Description("Joint Acceleration is nnown, the forward dynamics problem.")]
        Acceleration,
    }

    public class State
    {
        internal readonly Vector3 gravity;
        internal double time;
        internal readonly double[] q, qp, qpp, tau;
        internal readonly JointKnown[] known;
        internal readonly Simulation simulation;
        internal readonly Kinematics kinematics;
        internal readonly Articulated articulated;
        internal readonly Dynamics dynamics;

        public State(Simulation simulation, double time, StackedVector Y)
        {
            this.simulation=simulation;
            this.kinematics = simulation.kinematics;    // Assign reference to avoid allocating new arrays each time
            this.articulated = simulation.articulated;  // Assign reference to avoid allocating new arrays each time
            this.dynamics = simulation.dynamics;        // Assign reference to avoid allocating new arrays each time
            this.gravity = simulation.Gravity;
            var joints = simulation.Joints;
            int n = simulation.Dof;
            this.time=time;
            this.q=Y[0];
            this.qp=Y[1];
            this.qpp=new double[n];
            this.tau=new double[n];
            this.known=new JointKnown[n];
            for (int i = 0; i<n; i++)
            {
                MotorDefined defined = joints[i].Motor.Defined;
                var drive = joints[i].motor.Compile();
                switch (defined)
                {
                    case MotorDefined.Acceleration:
                    {
                        known[i]=JointKnown.Acceleration;
                        qpp[i]=drive(time, q[i], qp[i]);
                        break;
                    }
                    case MotorDefined.Forcing:
                    {
                        known[i]=JointKnown.Forcing;
                        tau[i]=drive(time, q[i], qp[i]);
                        break;
                    }
                    default:
                    throw new NotSupportedException(defined.ToString());
                }
            }
        }
        public void DoFeatherstone()
        {
            kinematics.Calculate(this);
            articulated.Calculate(this, kinematics);
            dynamics.Calculate(this, kinematics, articulated);
        }
    }
}
