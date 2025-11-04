using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using JA.LinearAlgebra.ScrewCalculus;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{

    public delegate double JointMotor(double time, double q, double qp);

    public enum JointType
    {
        [Description("Combined rotation and parallel translation")]
        Screw,
        [Description("Pure rotation about an axis")]
        Revolute,
        [Description("Pure translation along an axis")]
        Prismatic,
    }

    public class Joint : ITree<Joint>, ICanChangeUnits
    {
        internal Joint parent;
        internal readonly List<Joint> children;
        internal readonly JointType type;
        internal Pose3 localPosition;
        internal Vector3 localAxis;
        internal double pitch;
        internal UnitSystem units;

        #region Factory
        Joint(UnitSystem units, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch)
            : this(units, type, localPosition, localAxis, pitch, MassProperties.Zero)
        { }
        Joint(Joint parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch)
            : this(parent, type, localPosition, localAxis, pitch, MassProperties.Zero)
        { }
        Joint(UnitSystem units, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
        {
            this.units=units;
            this.parent=null;
            this.children=new List<Joint>();
            this.type=type;
            this.localPosition=localPosition;
            this.localAxis=localAxis;
            this.pitch=type!=JointType.Prismatic ? pitch : double.PositiveInfinity;
            this.MassProperties=massProperties.ConvertTo(units);
            this.InitialConditions=(0, 0);
            this.Motor=ConstantValue(0);
        }
        Joint(Joint parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
        {
            this.units=parent?.units??UnitSystem.MKS;
            this.parent=parent;
            this.children=new List<Joint>();
            this.type=type;
            this.localPosition=localPosition;
            this.localAxis=localAxis;
            this.pitch=type!=JointType.Prismatic ? pitch : double.PositiveInfinity;
            this.MassProperties=massProperties.ConvertTo(units);
            if (parent!=null)
            {
                parent.children.Add(this);
            }
            this.InitialConditions=(0, 0);
            this.Motor=ConstantValue(0);
        }

        public Joint AddScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => new Joint(this, JointType.Screw, localPosition, localAxis, pitch);
        public Joint AddRevolute(Pose3 localPosition, Vector3 localAxis)
            => new Joint(this, JointType.Revolute, localPosition, localAxis, 0);
        public Joint AddPrismatic(Pose3 localPosition, Vector3 localAxis)
            => new Joint(this, JointType.Prismatic, localPosition, localAxis, double.PositiveInfinity);
        public static Joint NewScrew(UnitSystem units, Pose3 localPosition, Vector3 localAxis, double pitch)
            => new Joint(units, JointType.Screw, localPosition, localAxis, pitch);
        public static Joint NewRevolute(UnitSystem units, Pose3 localPosition, Vector3 localAxis)
            => new Joint(units, JointType.Revolute, localPosition, localAxis, 0);
        public static Joint NewPrismatic(UnitSystem units, Pose3 localPosition, Vector3 localAxis)
            => new Joint(units, JointType.Prismatic, localPosition, localAxis, double.PositiveInfinity);
        #endregion

        #region Properties
        public UnitSystem Units
        {
            get => units;
            private set => units=value;
        }
        public Joint Parent
        {
            get => parent;
        }
        public static JointMotor ConstantValue(double tau)
            => (t, q, qp) => tau;
        public static JointMotor SpringValue(double springRate, double dampingRate, double preload = 0)
            => (t, q, qp) => preload -springRate*q - dampingRate*qp;
        public static JointMotor FunctionOfTime(Func<double,double> f_time)
            => (t,q,qp) => f_time(t);

        public JointMotor Motor { get; set; }
        public (double q, double qp) InitialConditions
        {
            get;
            set;
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

        public void AttachTo(Joint parent)
        {
            this.parent?.children.Remove(this);
            this.parent=parent;
            this.parent?.children.Add(this);
        }
        public void ZeroMassProperties() => MassProperties=MassProperties.Zero;
        public void AddMassProperties(MassProperties additionalMassProperties)
        {
            MassProperties+=additionalMassProperties.ConvertTo(units);
        }
        public void SubMassProperties(MassProperties additionalMassProperties)
        {
            MassProperties-=additionalMassProperties.ConvertTo(units);
        }

        public R Traverse<R>(R initialValue, Func<Joint, R> operation)
        {
            R result = operation(this);
            foreach (var child in children)
            {
                result=child.Traverse(result, operation);
            }
            return result;
        }
        public void Traverse(Action<Joint> operation)
        {
            operation(this);
            foreach (var child in children)
            {
                child.Traverse(operation);
            }
        }

        #endregion

        #region Mechanics
        public Pose3 GetLocalJointStep(double q)
        {
            Vector3 stepPos;
            Quaternion3 stepOri;
            switch (type)
            {
                case JointType.Screw:
                {
                    stepPos=Vector3.Scale(localAxis*pitch, q);
                    stepOri=Quaternion3.FromAxisAngle(localAxis, q);
                    return localPosition+Pose3.At(stepPos, stepOri);
                }
                case JointType.Revolute:
                {
                    stepPos=Vector3.Zero;
                    stepOri=Quaternion3.FromAxisAngle(localAxis, q);
                    return localPosition+Pose3.At(stepPos, stepOri);
                }
                case JointType.Prismatic:
                {
                    stepPos=Vector3.Scale(localAxis, q);
                    stepOri=Quaternion3.Identity;
                    return localPosition+Pose3.At(stepPos, stepOri);
                }
                default:
                throw new NotSupportedException("Unknown joint type.");
            }
        }
        public Vector33 GetJointAxis(Pose3 top)
        {
            Vector3 axis = top.orientation.Rotate(localAxis);
            switch (type)
            {
                case JointType.Screw:
                {
                    return Vector33.Twist(axis, top.position, pitch);
                }
                case JointType.Revolute:
                {
                    return Vector33.Twist(axis, top.position, 0.0);
                }
                case JointType.Prismatic:
                {
                    return Vector33.Twist(axis);
                }
                default:
                throw new NotSupportedException("Unknown joint type.");
            }
        }
        #endregion

        #region Formatting
        public override string ToString()
        {
            switch (type)
            {
                case JointType.Screw:
                return $"Screw(Units={Units}, LocalPosition={localPosition}, LocalAxis={localAxis}, Pitch={pitch})";
                case JointType.Revolute:
                return $"Revolute(Units={Units}, LocalPosition={localPosition}, LocalAxis={localAxis})";
                case JointType.Prismatic:
                return $"Prismatic(Units={Units}, LocalPosition={localPosition}, LocalAxis={localAxis})";
                default:
                return $"{type}(Units={Units}, LocalPosition={localPosition}, LocalAxis={localAxis}, Pitch={pitch})";
            }
        }
        #endregion

        #region Units
        public void ConvertTo(UnitSystem target)
        {
            if (units==target) return;

            var fl = Unit.Length.Convert(Units, target);

            var newLocalPosition = Pose3.At(
                localPosition.Position * fl,
                localPosition.Orientation
                );
            this.localPosition=newLocalPosition;
            this.pitch*=fl;
            this.MassProperties=this.MassProperties.ConvertTo(target);
            this.Units=target;
        }

        #endregion
    }
}
