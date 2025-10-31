using System;
using System.Linq;

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
        
        public Vector3 Gravity => gravity;
        public Joint[] Joints => joints;
        public int[] Parents => parents;
        public int[][] Childrens => childrens;
        public UnitSystem Units { get; }

        public Simulation(World world)
        {
            // Set everything to MKS for simulation
            this.Units = UnitSystem.MKS;

            float f_acc = Unit.Acceleration.Convert(world.units, Units);

            gravity= f_acc * world.gravity;

            joints=world.GetAllJoints().ToArray();
            parents=world.GetParents(joints);
            childrens=world.GetChildren(joints);

            foreach (var item in joints)
            {
                item.ConvertTo(Units); 
            }
        }

        #region Formatting
        public override string ToString()
        {
            return $"Simulation(Units={Units}, Joints={Joints.Length}, Gravity={Gravity})";
        }
        #endregion

        public double[] CalculateAccelerations(double[] q, double[] qp, double[] tau)
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
            Twist33[] s = new Twist33[n];
            Twist33[] v = new Twist33[n];
            Matrix33[] I = new Matrix33[n];
            Twist33[] k = new Twist33[n];
            Wrench33[] p = new Wrench33[n];
            Wrench33[] l = new Wrench33[n];
            Wrench33[] w = new Wrench33[n];
            for (int i_joint = 0; i_joint<n; i_joint++)
            {
                // Default ground is immovable
                Pose3 pos = Pose3.Origin;
                Twist33 vel = Twist33.Zero;

                Joint joint = joints[i_joint];
                int i_parent = parents[i_joint];
                if (i_parent>=0)
                {
                    // Get parent kinematics
                    pos=top[i_parent];
                    vel=v[i_parent];
                }
                pos+=joint.localPosition;
                Vector3 stepPos;
                Quaternion3 stepOri;
                switch (joint.type)
                {
                    case JointType.Screw:
                    {
                        stepPos=Vector3.Scale(joint.localAxis*joint.pitch, q[i_joint]);
                        stepOri=Quaternion3.FromAxisAngle(joint.localAxis, q[i_joint]);
                        pos+=Pose3.At(stepPos, stepOri);
                        break;
                    }
                    case JointType.Revolute:
                    {
                        stepPos=Vector3.Zero;
                        stepOri=Quaternion3.FromAxisAngle(joint.localAxis, q[i_joint]);
                        pos+=Pose3.At(stepPos, stepOri);
                        break;
                    }
                    case JointType.Prismatic:
                    {
                        stepPos=Vector3.Scale(joint.localAxis, q[i_joint]);
                        stepOri=Quaternion3.Identity;
                        pos+=Pose3.At(stepPos, stepOri);
                        break;
                    }
                    default:
                    throw new NotSupportedException("Unknown joint type.");
                }
                // Set top of joint position
                top[i_joint]=pos;
                // Find Center of mass
                cg[i_joint]=pos+joint.MassProperties.CG;

                //tex: Joint Screw Axis $$s_i = \begin{bmatrix} \vec{r}_i \times \hat{z}_i \\ \hat{z}_i \end{bmatrix}$$
                Matrix3 Ic = Dynamics.GetMmoiMatrix(joint.MassProperties.MMoi, pos.orientation);

                // Get weight wrench
                w[i_joint]=Dynamics.GetWeight(joint.MassProperties.Mass, cg[i_joint], gravity);

                //\vec{r}_i \times \hat{z}_i \\ \hat{z}_i
                //\end{bmatrix}$$
                //\end{bmatrix}$$
                Vector3 axis = pos.orientation.Rotate(joint.localAxis);
                switch (joint.type)
                {
                    case JointType.Screw:
                    {
                        s[i_joint]=Twist33.At(axis, pos.position, joint.pitch);
                        break;
                    }
                    case JointType.Revolute:
                    {
                        s[i_joint]=Twist33.At(axis, pos.position, 0.0);
                        break;
                    }
                    case JointType.Prismatic:
                    {
                        s[i_joint]=Twist33.Pure(axis);
                        break;
                    }
                    default:
                    throw new NotSupportedException("Unknown joint type.");
                }


                Twist33 sqp = s[i_joint] * qp[i_joint];

                //tex: Velocity Recursion $$v_i = v_{i-1} + s_i \dot{q}_i$$
                v[i_joint]=vel+sqp;

                //tex: Bias Acceleration $$k_i = v_i \times s_i \dot{q}_i$$
                k[i_joint]=Twist33.Cross(v[i_joint], sqp);

                //tex: Spatial Inertia $$\mathbf{I}_i = \begin{bmatrix} 
                // m_i & -m_i \vec{c}_i\times \\
                // m_i \vec{c}_i\times & \mathrm{I}_C - m_i \vec{c}_i\times \vec{c}_i\times \end{bmatrix}$$
                I[i_joint]=Dynamics.Spi(joint.MassProperties.Mass, Ic, cg[i_joint]);

                //tex: Momentum $$ \ell_i = \mathbf{I}_i v_i $$
                l[i_joint]=(Wrench33)( I[i_joint]*v[i_joint] );

                //tex: Bias Forces $$ p_i = v_i \times \mathbf{I}_i \mathbf{v}_i$$
                p[i_joint]=Wrench33.Cross(v[i_joint], l[i_joint]);
            }

            // Propagate Articulated Inertia Down The Chain
            Matrix33[] A = new Matrix33[n];
            Wrench33[] d = new Wrench33[n];
            for (int i_joint = n-1; i_joint>=0; i_joint--)
            {
                var joint = joints[i_joint];
                Matrix33 Ai = I[i_joint];
                Wrench33 di = p[i_joint] - w[i_joint];
                var children = childrens[i_joint];
                for (int i_child = 0; i_child<children.Length; i_child++)
                {
                    var child = joints[i_child];
                    Matrix33 An = A[i_child];
                    Wrench33 dn = d[i_child];
                    double Qn = tau[i_child];
                    Twist33 sn = s[i_child];
                }
            }
            throw new NotImplementedException();
            return qpp;
        }
    }

}
