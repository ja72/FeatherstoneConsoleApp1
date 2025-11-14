using JA.LinearAlgebra.ScrewCalculus;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics.Featherstone
{
    public class Kinematics
    {
        internal readonly int dof;
        internal readonly Pose3[] top;
        internal readonly Vector3[] cg;
        internal readonly Vector33[] s;
        internal readonly Vector33[] v;
        internal readonly Matrix33[] I;
        internal readonly Vector33[] k;
        internal readonly Vector33[] p;
        internal readonly Vector33[] l;
        internal readonly Vector33[] w;
        public Kinematics(int dof)
        {
            this.dof = dof;
            this.top=new Pose3[dof];
            this.cg=new Vector3[dof];
            this.s=new Vector33[dof];
            this.v=new Vector33[dof];
            this.I=new Matrix33[dof];
            this.k=new Vector33[dof];
            this.p=new Vector33[dof];
            this.l=new Vector33[dof];
            this.w=new Vector33[dof];
        }
        public int Dof => dof;
        public void Calculate(State state)
        {
            Simulation simulation = state.simulation;
            Vector3 gravity = simulation.Gravity;
            var joints = simulation.Joints;
            int[] parents = simulation.Parents;
            int[][] children = simulation.Children;
            int n = joints.Length;

            // Propagate Kinematics Up The Chain
            for (int i_joint = 0; i_joint<n; i_joint++)
            {
                var joint = joints[i_joint];
                int i_parent = parents[i_joint];

                // Default ground is immovable
                Pose3 pos = Pose3.Origin;
                Vector33 vel = Vector33.Zero;
                if (i_parent>=0)
                {
                    // Get parent kinematics
                    pos=top[i_parent];
                    vel=v[i_parent];
                }
                // Find the top of the joint pose
                pos+=joint.GetLocalJointStep(state.q[i_joint]);
                // Set top of joint position
                top[i_joint]=pos;

                // Get weight wrench
                cg[i_joint] = (pos + joint.MassProperties.CG).Position;
                w[i_joint]= Vector33.Zero; 

                //tex: Spatial Inertia $$\mathbf{I}_i = \begin{bmatrix} 
                // m_i & -m_i \vec{c}_i\times \\
                // m_i \vec{c}_i\times & \mathrm{I}_C - m_i \vec{c}_i\times \vec{c}_i\times \end{bmatrix}$$

                I[i_joint]=joint.MassProperties.Spi(pos.Orientation, cg[i_joint], out var Ic);

                //tex: Joint Screw Axis $$s_i = \begin{bmatrix} \vec{r}_i \times \hat{z}_i \\ \hat{z}_i \end{bmatrix}$$

                s[i_joint]=joint.GetJointAxis(pos);

                Vector33 sqp = s[i_joint] * state.qp[i_joint];

                //tex: Velocity Recursion $$v_i = v_{i-1} + s_i \dot{q}_i$$
                v[i_joint]=vel+sqp;

                //tex: Bias Acceleration $$k_i = v_i \times s_i \dot{q}_i$$
                k[i_joint]=Twist3.Cross(v[i_joint], sqp);

                //tex: Momentum $$ \ell_i = \mathbf{I}_i v_i $$
                l[i_joint]=I[i_joint]*v[i_joint];

                //tex: Bias Forces $$ p_i = v_i \times \mathbf{I}_i \mathbf{v}_i$$
                p[i_joint]=Wrench3.Cross(v[i_joint], l[i_joint]);
            }
        }
    }

}
