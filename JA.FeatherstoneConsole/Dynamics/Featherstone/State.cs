using System;
using System.ComponentModel;

using JA.LinearAlgebra;
using JA.LinearAlgebra.Vectors;

namespace JA.Dynamics.Featherstone
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
        //internal readonly Vector3 gravity;
        internal double time;
        internal readonly double[] q, qp, qpp, tau;
        internal readonly JointKnown[] known;
        internal readonly double[] qi, qpi;
        internal readonly Simulation simulation;
        internal readonly Kinematics kinematics;
        internal readonly Articulated articulated;
        internal readonly Dynamics dynamics;
        public State(Simulation simulation)
        {
            int dof = simulation.Dof;
            this.simulation = simulation;
            this.time=0;
            this.q = new double[dof];
            this.qp = new double[dof];
            this.qpp = new double[dof];
            this.tau = new double[dof];
            this.known = new JointKnown[dof];
            this.qi = new double[dof];
            this.qpi = new double[dof];
            this.kinematics=new Kinematics(dof);
            this.articulated=new Articulated(dof);
            this.dynamics = new Dynamics(dof);
            SetInitialConditions();
            SetKnownConditions();
        }

        public void Reset()
        {
            time = 0;
            qi.CopyTo(q, 0);
            qpi.CopyTo(qp, 0);
            SetInitialConditions();
        }

        public void SetInitialConditions()
        {
            int dof = simulation.Dof;
            var joints = simulation.Joints;
            for (int i = 0; i<dof; i++)
            {
                (double qi, double qpi) m = joints[i].initialConditions;
                qi[i]=m.qi;
                qpi[i]=m.qpi;
                q[i]=m.qi;
                qp[i]=m.qpi;
            }
        }
        public void SetKnownConditions()
        {
            int dof = simulation.Dof;
            var joints = simulation.Joints;
            for (int i = 0; i<dof; i++)
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
        public void DoFeatherstone(double time, StackedVector Y)
        {
            int dof = simulation.Dof;
            this.time=time;
            var joints = simulation.Joints;
            Y[0].CopyTo(this.q, 0);
            Y[1].CopyTo(this.qp, 0);
            SetKnownConditions();
            kinematics.Calculate(this);
            articulated.Calculate(this, kinematics);
            dynamics.Calculate(this, kinematics, articulated);
        }
    }
}
