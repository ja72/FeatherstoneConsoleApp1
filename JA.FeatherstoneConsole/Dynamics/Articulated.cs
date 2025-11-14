using JA.LinearAlgebra.ScrewCalculus;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public class Articulated
    {
        internal readonly int count;
        internal readonly Matrix33[] I_A;
        internal readonly Vector33[] p_A;
        public Articulated(int count)
        {
            this.count=count;
            this.I_A = new Matrix33[count];
            this.p_A = new Vector33[count];
        }

        public void Calculate(State state, Kinematics kinematics)
        {
            var simulation = state.simulation;
            var joints = simulation.Joints;
            int[] parents = simulation.Parents;
            int[][] children = simulation.Children;
            int n = simulation.Dof;
            Vector3 gravity = state.gravity;

            // Propagate Articulated Inertia Down The Chain
            for (int i_joint = n-1; i_joint>=0; i_joint--)
            {
                var joint = joints[i_joint];
                Matrix33 I_Ai = kinematics.I[i_joint];
                Vector33 I_pi = kinematics.p[i_joint] - kinematics.w[i_joint];
                var jointChildrenList = children[i_joint];
                for (int index = 0; index<jointChildrenList.Length; index++)
                {
                    int i_child = jointChildrenList[index];
                    var child = joints[i_child];
                    Matrix33 An = I_A[i_child];
                    Vector33 dn = p_A[i_child];
                    double Qn = state.tau[i_child];
                    Vector33 sn = kinematics.s[i_child];
                    Vector33 kn = kinematics.k[i_child];
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
        }
    }
}
