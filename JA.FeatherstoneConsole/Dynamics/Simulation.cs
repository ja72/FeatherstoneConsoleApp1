using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;

using JA.Dynamics.Featherstone;
using JA.LinearAlgebra;
using JA.LinearAlgebra.Screws;
using JA.LinearAlgebra.Vectors;

namespace JA.Dynamics
{
    public class Simulation
    {
        readonly Vector3 gravity;
        readonly JointBodyInfo[] joints;
        readonly int[] parents;
        readonly int[][] children;
        readonly List<(double t, StackedVector Y)> history;
        //readonly double[] initialPos, initialVel;
        readonly State state;

        #region Factory
        public Simulation(World world)
        {
            // Set everything to MKS for simulation
            this.Units=UnitSystem.MKS;
            float f_acc = Unit.Acceleration.Convert(world.units, Units);
            gravity=f_acc*world.gravity;
            var allJoints = world.GetAllJoints(Units);
            int n = allJoints.Length;
            this.joints   = allJoints;
            this.parents  = new int[n];
            this.children=new int[n][];
            for (int i_joint = 0; i_joint<n; i_joint++)
            {
                var joint = allJoints[i_joint];
                if (joint.Parent==null)
                {
                    parents[i_joint]=-1;
                }
                else
                {
                    parents[i_joint]=Array.IndexOf(allJoints, joint.Parent);
                }
                children[i_joint]=new int[joint.Children.Count];
                for (int i_child = 0; i_child<joint.Children.Count; i_child++)
                {
                    children[i_joint][i_child]=Array.IndexOf(allJoints, joint.Children[i_child]);
                }
            }

            for (int i = 0; i<n; i++)
            {
                joints[i].DoConvert(Units);
            }
            this.state = new State(this);
            this.history=new List<(double t, StackedVector Y)>();
            history.Add((0, new StackedVector(state.q, state.qp)));
        }


        #endregion

        #region Properties
        public int Dof => joints.Length;
        public Vector3 Gravity => gravity;
        public JointBodyInfo[] Joints => joints;
        public int[] Parents => parents;
        public int[][] Children => children;
        public UnitSystem Units { get; }
        public IReadOnlyList<(double t, StackedVector Y)> History => history;
        public double Time { get => history[history.Count-1].t; }
        public StackedVector Current { get => history[history.Count-1].Y; }
        #endregion

        #region Simulation
        public void Reset()
        {
            this.history.Clear();
            state.Reset();
            history.Add((0, new StackedVector(state.q, state.qp)));
        }
        public void AddSolution(double t, StackedVector Y)
        {
            history.Add((t, Y));
        }
        public void AddSolution((double t, StackedVector Y) solution)
        {
            history.Add(solution);
        }

        public void Integrate(double step)
        {
            var sol = RungeKutta(Time, Current, step);
            AddSolution(sol);
        }
        public void Integrate(double step, out double f_res_max)
        {
            var sol = RungeKutta(Time, Current, step);
            AddSolution(sol);
            var f_res = GetResidualForces(sol.t, sol.Y);
            f_res_max = f_res.Max(f => f.MaxAbs());
        }
        public void RunTo(double endTime, int steps)
            => RunTo(endTime, ( endTime-Time )/steps);
        public void RunTo(double endTime, double step)
        {
            double t = Time;            
            while (t<endTime)
            {
                if (t+step>endTime)
                {
                    step=endTime-t;
                }
                Integrate(step);
                t+=step;
            }
        }
        public void RunTo(double endTime, int steps, out double max_residual_force)
            => RunTo(endTime, ( endTime-Time )/steps, out max_residual_force);
        public void RunTo(double endTime, double step, out double max_residual_force)
        {
            double t = Time;
            max_residual_force = 0;
            while (t<endTime)
            {
                if (t+step>endTime)
                {
                    step=endTime-t;
                }
                Integrate(step, out var f_res_max);
                max_residual_force = Math.Max(max_residual_force, f_res_max);
                t+=step;
            }
        }

        public (double t, StackedVector Y) Euler(double t, StackedVector Y, double step)
        {
            var Yp = GetRate(t,Y);
            return (t+step, Y+step*Yp);
        }
        public (double t, StackedVector Y) RungeKutta(double t, StackedVector Y, double step)
        {
            // Estimate step size based on maximum rotation corresponding to 1° of rotation
            double qp_max = Y[1].Max(x=>Math.Abs(x));
            double est_step = qp_max > 0 ? Math.PI/(180*qp_max) : step;
            step = Math.Min(est_step, step);

            // Y = { q, qp}
            // Yp = d/dt Y = f(t,Y)
            var K0 = GetRate(t, Y);
            var K1 = GetRate(t + step/2, Y + step/2*K0);
            var K2 = GetRate(t + step/2, Y + step/2*K1);
            var K3 = GetRate(t + step, Y + step*K2);

            var ΔY = (step/6) * (K0 + 2*K1 + 2*K2 + K3);

            return (t+step, Y+ΔY);
        }

        public Vector33[] GetResidualForces(double time, StackedVector Y)
        {
            state.DoFeatherstone(time, Y, out var f_res);
            return f_res;
        }
        #endregion

        #region Formatting
        public override string ToString()
        {
            return $"Simulation(Units={Units}, Joints={Joints.Length}, Gravity={Gravity})";
        }
        #endregion

        #region Dynamics

        public StackedVector GetRate(double time, StackedVector Y)
            //tex: $\dot{\rm Y} = f(t,\,{\rm Y})$
        {
            state.DoFeatherstone(time, Y);
            return new StackedVector(state.qp, state.qpp);
        }
        #endregion
    }
}
