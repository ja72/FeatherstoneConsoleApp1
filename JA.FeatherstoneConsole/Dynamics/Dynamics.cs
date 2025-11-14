using System;

using JA.LinearAlgebra.ScrewCalculus;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public class Dynamics
    {
        internal readonly int dof;
        internal readonly Vector33[] a;
        internal readonly Vector33[] f;
        public Dynamics(int dof)
        {
            this.dof = dof;
            this.a = new Vector33[dof];
            this.f = new Vector33[dof];
        }
        public void Calculate(State state, Kinematics kinematics, Articulated articulated)
        {
            var simulation = state.simulation;
            Vector3 gravity = state.gravity;
            var joints = simulation.Joints;
            int[] parents = simulation.Parents;
            int[][] children = simulation.Children;
            int n = simulation.Dof;

            // Propagate Joint Accelerations up the chain
            for (int i_joint = 0; i_joint<n; i_joint++)
            {
                var joint = joints[i_joint];
                int i_parent = parents[i_joint];

                Vector33 ap = Twist3.Pure(-gravity);
                if (i_parent>=0)
                {
                    // Get parent kinematics
                    ap=a[i_parent];
                }
                //tex: $$\begin{aligned}\ddot{q}_{i} & =\left(\boldsymbol{s}_{i}^{\top}{\bf I}_{n}^{A}\boldsymbol{s}_{i}\right)^{-1}\left(Q_{i}-\boldsymbol{s}_{i}^{\top}\left({\bf I}_{n}^{A}\left(\boldsymbol{a}_{i-1}+\boldsymbol{\kappa}_{i}\right)+\boldsymbol{p}_{i}^{A}\right)\right)\\
                //\boldsymbol{a}_{i} & =\boldsymbol{s}_{i}\ddot{q}_{i}+\boldsymbol{a}_{i-1}+\boldsymbol{\kappa}_{i}\\
                //\boldsymbol{f}_{i} & ={\bf I}_{n}^{A}\boldsymbol{a}_{i}+\boldsymbol{p}_{i}^{A}
                //\end{aligned}$$
                ap+=kinematics.k[i_joint];
                Vector33 Li = articulated.I_A[i_joint]*kinematics.s[i_joint];
                Vector33 pn = articulated.p_A[i_joint];
                double ji = Vector33.Dot(kinematics.s[i_joint], Li);
                state.qpp[i_joint]=( state.tau[i_joint]-Vector33.Dot(
                    kinematics.s[i_joint], articulated.I_A[i_joint]*ap+pn) )/ji;
                a[i_joint]=kinematics.s[i_joint]*state.qpp[i_joint]+ap;
                f[i_joint]=articulated.I_A[i_joint]*a[i_joint]+pn;
            }
        }

        public Vector33[] ResidualForces(State state, Kinematics kinematics)
        {
            Vector3 gravity = state.simulation.Gravity;
            var joints = state.simulation.Joints;
            int[] parents = state.simulation.Parents;
            int[][] children = state.simulation.Children;

            Vector33[] check = new Vector33[dof];
            // Check Equation Compliance for each body
            for (int i_joint = 0; i_joint<dof; i_joint++)
            {
                var joint = joints[i_joint];
                int i_parent = parents[i_joint];

                // Default ground is immovable
                Pose3 pos = Pose3.Origin;
                Vector33 vel = Vector33.Zero;
                Vector33 ap = Twist3.Pure(-gravity);
                if (i_parent>=0)
                {
                    // Get parent kinematics
                    pos=kinematics.top[i_parent];
                    vel=kinematics.v[i_parent];
                    ap=a[i_parent];
                }

                //tex: Force Balance
                //$$ f_i - \sum_n f_n + w_i = I_i a_i + v_i \times I_i v_i$$

                var jointChildren = children[i_joint];
                Vector33 f_net = f[i_joint];
                for (int index = 0; index<jointChildren.Length; index++)
                {
                    f_net-=f[jointChildren[index]];
                }
                f_net += kinematics.w[i_joint];

                Vector33 f_inertial = kinematics.I[i_joint]*a[i_joint] + kinematics.p[i_joint];

                Vector33 f_check = f_net - f_inertial;

                check[i_joint]=f_check;
            }
            return check;
        }
    }

}