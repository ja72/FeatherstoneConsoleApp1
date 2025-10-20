using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Featherstone.ScrewCalculus;
using Featherstone.VectorCalculus;

namespace Featherstone.Dynamics
{
    public class Item
    {
        internal readonly int bodyIndex;
        internal readonly int jointIndex;
        internal readonly int parentJointIndex;
        internal readonly List<int> childrenIndices;

        public Item(int bodyIndex, int jointIndex, int parentJointIndex, params int[] childrenIndices)
        {
            this.bodyIndex=bodyIndex;
            this.jointIndex=jointIndex;
            this.parentJointIndex=parentJointIndex;
            this.childrenIndices=new List<int>(childrenIndices);
        }
    }
    public class TreeSystem
    {
        internal readonly Vector3 gee;
        internal readonly List<RigidBody> bodies;
        internal readonly List<Joint> joints;
        internal readonly List<Item> items;

        #region Factor
        public TreeSystem()
        {
            gee=new Vector3(0.0, -9.81, 0.0);
            bodies=new List<RigidBody>();
            joints=new List<Joint>();
            items=new List<Item>();
        }

        public int AddRigidBody(RigidBody body)
        {
            int index = bodies.Count;
            bodies.Add(body);
            return index;
        }
        public int AddJoint(int parentIndex, int onBody, int existingJointIndex)
        {
            var joint = joints[existingJointIndex];
            int index = items.Count;
            items.Add(new Item(onBody, existingJointIndex, parentIndex));
            if (parentIndex>=0)
            {
                items[parentIndex].childrenIndices.Add(index);
            }
            return index;
        }
        public int AddJoint(int parentIndex, int onBody, Joint joint)
        {
            joints.Add(joint);
            int index = items.Count;
            items.Add(new Item(onBody, index, parentIndex));
            if (parentIndex>=0)
            {
                items[parentIndex].childrenIndices.Add(index);
            }
            return index;
        }
        #endregion

        #region Property
        public Vector3 Gravity => gee;
        public IReadOnlyList<RigidBody> Bodies => bodies;
        public IReadOnlyList<Joint> Joints => joints;
        public IReadOnlyList<Item> Items => items;
        #endregion

        #region Mechanics

        public double[] CalculateAccelerations(double[] q, double[] qp, double[] tau)
        {
            // Placeholder for dynamics calculation
            // Implement Featherstone's algorithm or other dynamics algorithms here
            int n = items.Count;
            double[] qpp = new double[n];
            if (q.Length!=n||qp.Length!=n||tau.Length!=n)
            {
                throw new ArgumentException("Input arrays must have the same length as the number of items.");
            }

            // Propagate Kinematics Up The Chain
            Pose3[] top = new Pose3[n];
            Pose3[] cg = new Pose3[n];
            Twist33[] s = new Twist33[n];
            Twist33[] v = new Twist33[n];
            Matrix33[] I = new Matrix33[n];
            Twist33[] k = new Twist33[n];
            Wrench33[] p = new Wrench33[n];
            Wrench33[] l = new Wrench33[n];
            Wrench33[] w = new Wrench33[n];
            for (int i_item = 0; i_item<n; i_item++)
            {
                // Default ground is immovable
                Pose3 pos = Pose3.Identity;
                Twist33 vel = Vector33.Zero;

                Item item = items[i_item];
                int i_parent = item.parentJointIndex;
                if(i_parent>=0)
                {
                    // Get parent kinematics
                    var parentItem = items[i_parent];
                    pos = top[parentItem.jointIndex];
                    vel = v[parentItem.jointIndex];
                }
                Joint joint   = joints[item.jointIndex];
                RigidBody body    = bodies[item.bodyIndex];
                pos = joint.GetJointStep(pos, q[i_item]);
                top[i_item] = pos;
                cg[i_item]  = body.GetCenterOfMass(pos);
                w[i_item]   = Wrench33.At(body.mass * gee, cg[i_item].position);

                //tex: Joint Screw Axis $$s_i = \begin{bmatrix}
                //\vec{r}_i \times \hat{z}_i \\ \hat{z}_i
                //\end{bmatrix}$$
                s[i_item]   = joint.GetJointAxis(pos);;
                Twist33 sqp = s[i_item] * qp[i_item];

                //tex: Velocity Recursion $$v_i = v_{i-1} + s_i \dot{q}_i$$
                v[i_item]   = vel + sqp;

                //tex: Bias Acceleration $$k_i = v_i \times s_i \dot{q}_i$$
                k[i_item]   = Twist33.Cross(v[i_item], sqp );

                //tex: Spatial Inertia $$\mathbf{I}_i = \begin{bmatrix} 
                // m_i & -m_i \vec{c}_i\times \\
                // m_i \vec{c}_i\times & \mathrm{I}_C - m_i \vec{c}_i\times \vec{c}_i\times \end{bmatrix}$$
                I[i_item]   = body.Spi(cg[i_item]);

                //tex: Momentum $$ \ell_i = \mathbf{I}_i v_i $$
                l[i_item]   = I[i_item] * v[i_item];

                //tex: Bias Forces $$ p_i = v_i \times \mathbf{I}_i \mathbf{v}_i$$
                p[i_item]   = Wrench33.Cross(v[i_item], l[i_item]);
            }

            // Propagate Articulated Inertia Down The Chain
            Matrix33[] A = new Matrix33[n];
            Wrench33[] d = new Wrench33[n];
            for (int i_item = n-1; i_item>=0; i_item--)
            {
                Matrix33 Ai = I[i_item];
                Wrench33 di = p[i_item] - w[i_item];
                Item item = items[i_item];
                for (int currChild = 0; currChild<item.childrenIndices.Count; currChild++)
                {
                    int i_child = item.childrenIndices[currChild];
                    var child = items[i_child];
                    Matrix33 An = A[i_child];
                    Wrench33 dn = d[i_child];
                    double Qn = tau[i_child];
                    Twist33 sn = s[i_child];
                }
            }
            throw new NotImplementedException();
            return qpp;
        }

        #endregion
    }
}
