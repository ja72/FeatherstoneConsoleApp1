using System;
using System.Collections.Generic;
using System.Linq;

using JA.LinearAlgebra;
using JA.LinearAlgebra.ScrewCalculus;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public class Simulation
    {
        readonly Vector3 gravity;
        readonly Joint[] joints;
        readonly int[] parents;
        readonly int[][] childrens;
        readonly List<(double t, StackedVector Y)> history;

        public Vector3 Gravity => gravity;
        public Joint[] Joints => joints;
        public int[] Parents => parents;
        public int[][] Childrens => childrens;
        public UnitSystem Units { get; }

        public IReadOnlyList<(double t, StackedVector Y)> History => history;
        public double Time { get => history[history.Count-1].t; }
        public StackedVector Current { get => history[history.Count-1].Y; }
        public Simulation(World world)
        {
            // Set everything to MKS for simulation
            this.Units=UnitSystem.MKS;
            float f_acc = Unit.Acceleration.Convert(world.units, Units);
            gravity=f_acc*world.gravity;
            joints=world.GetAllJoints().ToArray();
            parents=world.GetParents(joints);
            childrens=world.GetChildren(joints);

            foreach (var item in joints)
            {
                item.ConvertTo(Units);
            }

            int n = joints.Length;
            this.history=new List<(double t, StackedVector Y)>();
            history.Add((0, new StackedVector(n, n)));

            GetJointTorqueFunction=(t, q, qp) => Factory.CreateArray<double>(q.Length);
        }

        #region Simulation
        public void Reset()
        {
            this.history.Clear();
            int n = joints.Length;
            history.Add((0, new StackedVector(n, n)));
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

        public (double t, StackedVector Y) Euler(double t, StackedVector Y, double step)
        {
            var Yp = GetRate(t,Y);
            return (t+step, Y+step*Yp);
        }
        public (double t, StackedVector Y) RungeKutta(double t, StackedVector Y, double step)
        {
            var K0 = GetRate(t, Y);
            var K1 = GetRate(t + step/2, Y + step/2*K0);
            var K2 = GetRate(t + step/2, Y + step/2*K1);
            var K3 = GetRate(t + step, Y + step*K2);

            var ΔY = (step/6) * (K0 + 2*K1 + 2*K2 + K3);

            return (t +step, Y + ΔY);
        }
        #endregion

        #region Formatting
        public override string ToString()
        {
            return $"Simulation(Units={Units}, Joints={Joints.Length}, Gravity={Gravity})";
        }
        #endregion

        #region Dynamics
        public Func<double, double[], double[], double[]> GetJointTorqueFunction
        {
            get;
            set;
        }
        public StackedVector GetRate(double t, StackedVector Y)
        {
            var f_tau = GetJointTorqueFunction;
            int n = joints.Length;
            double[] q = Y[0].ToArray();
            double[] qp = Y[1].ToArray();
            double[] tau = f_tau(t, q, qp);
            double[] qpp = CalculateAccelerations(t,q,qp,tau);

            return new StackedVector(qp, qpp);
        }
        public double[] CalculateAccelerations(double time, double[] q, double[] qp)
            => CalculateAccelerations(time, q, qp, GetJointTorqueFunction(time, q, qp));
        public double[] CalculateAccelerations(double time, double[] q, double[] qp, double[] tau)
        {
            // Placeholder for dynamics calculation
            // Implement Featherstone's algorithm or other dynamics algorithms here
            int n = joints.Length;
            double[] qpp = new double[n];
            if (q.Length!=n||qp.Length!=n||tau.Length!=n)
            {
                throw new ArgumentException("Input arrays must have the same length as the number of items.");
            }

            // Propagate Kinematics Up The Chain
            Pose3[] top = new Pose3[n];
            Vector3[] cg = new Vector3[n];
            Vector33[] s = new Vector33[n];
            Vector33[] v = new Vector33[n];
            Matrix33[] I = new Matrix33[n];
            Vector33[] k = new Vector33[n];
            Vector33[] p = new Vector33[n];
            Vector33[] l = new Vector33[n];
            Vector33[] w = new Vector33[n];
            for (int i_joint = 0; i_joint<n; i_joint++)
            {
                // Default ground is i mmovable
                Pose3 pos = Pose3.Origin;
                Vector33 vel = Vector33.Zero;

                Joint joint = joints[i_joint];
                int i_parent = parents[i_joint];
                if (i_parent>=0)
                {
                    // Get parent kinematics
                    pos=top[i_parent];
                    vel=v[i_parent];
                }
                // Find the top of the joint pose
                pos+=joint.GetLocalJointStep(q[i_joint]);
                // Set top of joint position
                top[i_joint]=pos;
                // Find Center of mass
                cg[i_joint]=pos+joint.MassProperties.CG;

                //tex: Joint Screw Axis $$s_i = \begin{bmatrix} \vec{r}_i \times \hat{z}_i \\ \hat{z}_i \end{bmatrix}$$
                Matrix3 Ic = Dynamics.GetMmoiMatrix(joint.MassProperties.MMoi, pos.orientation);

                // Get weight wrench
                w[i_joint]=Dynamics.GetWeight(joint.MassProperties.Mass, cg[i_joint], gravity);

                s[i_joint]=joint.GetJointAxis(pos);

                Vector33 sqp = s[i_joint] * qp[i_joint];

                //tex: Velocity Recursion $$v_i = v_{i-1} + s_i \dot{q}_i$$
                v[i_joint]=vel+sqp;

                //tex: Bias Acceleration $$k_i = v_i \times s_i \dot{q}_i$$
                k[i_joint]=Screws.CrossTwist(v[i_joint], sqp);

                //tex: Spatial Inertia $$\mathbf{I}_i = \begin{bmatrix} 
                // m_i & -m_i \vec{c}_i\times \\
                // m_i \vec{c}_i\times & \mathrm{I}_C - m_i \vec{c}_i\times \vec{c}_i\times \end{bmatrix}$$
                I[i_joint]=Dynamics.Spi(joint.MassProperties.Mass, Ic, cg[i_joint]);

                //tex: Momentum $$ \ell_i = \mathbf{I}_i v_i $$
                l[i_joint]=I[i_joint]*v[i_joint];

                //tex: Bias Forces $$ p_i = v_i \times \mathbf{I}_i \mathbf{v}_i$$
                p[i_joint]=Screws.CrossWrench(v[i_joint], l[i_joint]);
            }

            // Propagate Articulated Inertia Down The Chain
            Matrix33[] I_A = new Matrix33[n];
            Vector33[] p_A = new Vector33[n];
            for (int i_joint = n-1; i_joint>=0; i_joint--)
            {
                var joint = joints[i_joint];
                Matrix33 I_Ai = I[i_joint];
                Vector33 I_pi = p[i_joint] - w[i_joint];
                var children = childrens[i_joint];
                for (int i_child = 0; i_child<children.Length; i_child++)
                {
                    var child = joints[i_child];
                    Matrix33 An = I_A[i_child];
                    Vector33 dn = p_A[i_child];
                    double Qn = tau[i_child];
                    Vector33 sn = s[i_child];
                    Vector33 kn = k[i_child];
                    Vector33 Ln = An*sn;
                    Vector33 Tn = Ln/Vector33.Dot(sn,Ln);
                    Matrix33 RUn = 1d - Vector33.Outer(Tn,sn);

                    //tex: $$\begin{aligned}{\bf I}_{i}^{A} & ={\bf I}_{i}+\sum_{n}^{{\rm children}}\left(1-\boldsymbol{T}_{n}\boldsymbol{s}_{n}^{\top}\right){\bf I}_{n}^{A}\\
                    //\boldsymbol{p}_{i}^{A} & =\boldsymbol{p}_{i}+\sum_{n}^{{\rm children}}\left(\boldsymbol{T}_{n}Q_{n}+\left(1-\boldsymbol{T}_{n}\boldsymbol{s}_{n}^{\top}\right)\left({\bf I}_{n}^{A}\boldsymbol{\kappa}_{n}+\boldsymbol{p}_{n}^{A}\right)\right)
                    //\end{aligned}$$

                    I_Ai+=RUn*An;
                    I_pi+=Tn*Qn+RUn*( An*kn+dn );
                }
                I_A[i_joint]=I_Ai;
                p_A[i_joint]=I_pi;
            }

            // Propagate Joint Accelerations up the chain
            Vector33[] a = new Vector33[n];
            Vector33[] f = new Vector33[n];
            for (int i_joint = 0; i_joint<n; i_joint++)
            {
                Vector33 ap = Screws.PureTwist(-gravity);

                Joint joint = joints[i_joint];
                int i_parent = parents[i_joint];
                if (i_parent>=0)
                {
                    // Get parent kinematics
                    ap=a[i_parent];
                }
                //tex: $$\begin{aligned}\ddot{q}_{i} & =\left(\boldsymbol{s}_{i}^{\top}{\bf I}_{n}^{A}\boldsymbol{s}_{i}\right)^{-1}\left(Q_{i}-\boldsymbol{s}_{i}^{\top}\left({\bf I}_{n}^{A}\left(\boldsymbol{a}_{i-1}+\boldsymbol{\kappa}_{i}\right)+\boldsymbol{p}_{i}^{A}\right)\right)\\
                //\boldsymbol{a}_{i} & =\boldsymbol{s}_{i}\ddot{q}_{i}+\boldsymbol{a}_{i-1}+\boldsymbol{\kappa}_{i}\\
                //\boldsymbol{f}_{i} & ={\bf I}_{n}^{A}\boldsymbol{a}_{i}+\boldsymbol{p}_{i}^{A}
                //\end{aligned}$$
                ap+=k[i_joint];
                Vector33 Li = I_A[i_joint]*s[i_joint];
                Vector33 pn = p_A[i_joint];
                double ji = Vector33.Dot(s[i_joint], Li);
                qpp[i_joint]=( tau[i_joint]-Vector33.Dot(
                    s[i_joint], I_A[i_joint]*ap+pn) )/ji;
                a[i_joint]=s[i_joint]*qpp[i_joint]+ap;
                f[i_joint]=I_A[i_joint]*a[i_joint]+pn;
            }
            return qpp;
        }
        #endregion
    }

}
