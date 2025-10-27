using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using JA.ScrewCalculus;
using JA.VectorCalculus;

namespace JA.Dynamics
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
        internal Joint parent;
        internal readonly List<Joint> children;
        internal readonly JointType type;
        internal Pose3 localPosition;
        internal Vector3 localAxis;
        internal double pitch;

        #region Factory
        Joint(Joint parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch)
            : this(parent, type, localPosition, localAxis, pitch, MassProperties.Zero)
        { }
        Joint(Joint parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
        {
            this.parent=parent;
            this.children=new List<Joint>();
            this.type=type;
            this.localPosition=localPosition;
            this.localAxis=localAxis;
            this.pitch=type!=JointType.Prismatic ? pitch : double.PositiveInfinity;
            this.MassProperties = massProperties;
            if (parent!=null)
            {
                parent.children.Add(this);
            }
        }

        public Joint AddScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => new Joint(this, JointType.Screw, localPosition, localAxis, pitch);
        public Joint AddRevolute(Pose3 localPosition, Vector3 localAxis)
            => new Joint(this, JointType.Revolute, localPosition, localAxis, 0);
        public Joint AddPrismatic(Pose3 localPosition, Vector3 localAxis)
            => new Joint(this, JointType.Prismatic, localPosition, localAxis, double.PositiveInfinity);
        public static Joint NewScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => new Joint(null, JointType.Screw, localPosition, localAxis, pitch);
        public static Joint NewRevolute(Pose3 localPosition, Vector3 localAxis)
            => new Joint(null, JointType.Revolute, localPosition, localAxis, 0);
        public static Joint NewPrismatic(Pose3 localPosition, Vector3 localAxis)
            => new Joint(null, JointType.Prismatic, localPosition, localAxis, double.PositiveInfinity);
        #endregion

        #region Properties
        public Joint Parent
        {
            get => parent;
            set
            {
                parent?.children.Remove(this);
                parent = value;
                parent?.children.Add(this);
            }
        }
        public IReadOnlyList<Joint> Children => children;
        public bool IsRoot => parent==null;
        public bool IsLeaf => children.Count==0;
        public JointType Type => type;
        public Pose3 LocalPosition
        {
            get => LocalPosition;
            set => LocalPosition=value;
        }
        public Vector3 LocalAxis
        {
            get => localAxis;
            set => localAxis=value;
        }
        public double Pitch
        {
            get => pitch;
            set => pitch=value;
        }
        public MassProperties MassProperties { get; private set; }
        public void ZeroMassProperties() => MassProperties = MassProperties.Zero;
        public void AddMassProperties(MassProperties additionalMassProperties)
        {
            MassProperties += additionalMassProperties;
        }
        public void SubMassProperties(MassProperties additionalMassProperties)
        {
            MassProperties -= additionalMassProperties;
        }

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
