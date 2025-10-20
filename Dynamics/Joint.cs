using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Featherstone.ScrewCalculus;
using Featherstone.VectorCalculus;

namespace Featherstone.Dynamics
{
    public enum JointType
    {
        [Description("Combined rotation and parallel translation")]
        Screw,
        [Description("Pure rotation about an axis")]
        Revolute,
        [Description("Pure translation along an axis")]
        Prismatic,
    }

    public class Joint : ITree<Joint>
    {
        internal readonly Joint parent;
        internal readonly List<Joint> children;
        internal readonly JointType type;
        internal readonly Pose3 localPosition;
        internal readonly Vector3 localAxis;
        internal readonly double pitch;

        #region Factory
        Joint(Joint parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch = 0.0)
        {
            this.parent=parent;
            this.children=new List<Joint>();
            this.type=type;
            this.localPosition=localPosition;
            this.localAxis=localAxis;
            this.pitch=type!=JointType.Prismatic ? pitch : double.PositiveInfinity;
            if (parent!=null)
            {
                parent.children.Add(this);
            }
        }

        public Joint AddScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => new Joint(this, JointType.Screw, localPosition, localAxis, pitch);
        public Joint AddRevolute(Pose3 localPosition, Vector3 localAxis)
            => new Joint(this, JointType.Revolute, localPosition, localAxis);
        public Joint AddPrismatic(Pose3 localPosition, Vector3 localAxis)
            => new Joint(this, JointType.Prismatic, localPosition, localAxis);
        public static Joint NewScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => new Joint(null, JointType.Screw, localPosition, localAxis, pitch);
        public static Joint NewRevolute(Pose3 localPosition, Vector3 localAxis)
            => new Joint(null, JointType.Revolute, localPosition, localAxis);
        public static Joint NewPrismatic(Pose3 localPosition, Vector3 localAxis)
            => new Joint(null, JointType.Prismatic, localPosition, localAxis);
        #endregion

        #region Properties
        public Joint Parent => parent;
        public IReadOnlyList<Joint> Children => children;
        public bool IsRoot => parent==null;
        public bool IsLeaf => children.Count==0;
        public JointType Type => type;
        public Pose3 LocalPosition => LocalPosition;
        public Vector3 LocalAxis => localAxis;
        public double Pitch => pitch;


        protected void FillAncestors(ref List<Joint> list)
        {
            list.Add(this);
            foreach (var item in children)
            {
                item.FillAncestors(ref list);
            }
        }

        public Joint[] GetMeAndAllAncestors()
        {
            List<Joint> list = new List<Joint>();
            FillAncestors(ref list);
            return list.ToArray();
        }
        #endregion

        #region Mechanics
        public Pose3 GetJointStep(Pose3 @base, double q)
        {
            Vector3 stepPos;
            Quaternion3 stepOri;
            switch (type)
            {
                case JointType.Screw:
                {
                    stepPos=Vector3.Scale(localAxis*pitch, q);
                    stepOri=Quaternion3.FromAxisAngle(localAxis, q);
                    return @base+Pose3.At(stepPos, stepOri);
                }
                case JointType.Revolute:
                {
                    stepPos=Vector3.Zero;
                    stepOri=Quaternion3.FromAxisAngle(localAxis, q);
                    return @base+Pose3.At(stepPos, stepOri);
                }
                case JointType.Prismatic:
                {
                    stepPos=Vector3.Scale(localAxis, q);
                    stepOri=Quaternion3.Identity;
                    return @base+Pose3.At(stepPos, stepOri);
                }
                default:
                throw new NotSupportedException("Unknown joint type.");
            }
        }
        public Twist33 GetJointAxis(Pose3 top)
        {
            Vector3 axis = top.orientation.Rotate(localAxis);
            switch (type)
            {
                case JointType.Screw:
                {
                    return Twist33.At(axis, top.position, pitch);
                }
                case JointType.Revolute:
                {
                    return Twist33.At(axis, top.position, 0.0);
                }
                case JointType.Prismatic:
                {
                    return Twist33.Pure(axis);
                }
                default:
                    throw new NotSupportedException("Unknown joint type.");
            }
        }
        #endregion
    }
}
