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

    public class JointBodyInfo : ICanChangeUnits<JointBodyInfo>, ICanChangeUnits
    {
        internal readonly JointType type;
        internal Pose3 localPosition;
        internal Vector3 localAxis;
        internal double pitch;
        internal UnitSystem units;
        internal JointMotor motor;
        internal MassProperties massProperties;
        internal (double q, double qp) initialConditions;

        public JointBodyInfo(UnitSystem units, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch)
            : this(units, type, localPosition, localAxis, pitch, MassProperties.Zero) { }
        public JointBodyInfo(UnitSystem units, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
        {
            switch (type)
            {
                case JointType.Revolute:
                {   
                    pitch=0;
                    break;
                }
                case JointType.Prismatic:
                {
                    pitch=double.PositiveInfinity;
                    break;
                }
            }
            this.units=units;
            this.type=type;
            this.localPosition=localPosition;
            this.localAxis=localAxis;
            this.pitch=pitch;
            this.massProperties = massProperties.ToConverted(units);
            this.InitialConditions=(0, 0);
            this.Motor=ConstantValue(0);
        }

        #region Properties
        public JointType Type => type;

        public UnitSystem Units
        {
            get => units;
            private set => units=value;
        }
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
        public MassProperties MassProperties
        {
            get => massProperties;
            private set => massProperties=value;
        }

        public (double q, double qp) InitialConditions
        {
            get => initialConditions;
            set => initialConditions=value;
        }

        public JointMotor Motor
        {
            get => motor;
            set => motor=value;
        } 
        #endregion

        #region Standard Motors
        public static JointMotor ConstantValue(double tau)
            => (t, q, qp) => tau;
        public static JointMotor SpringValue(double springRate, double dampingRate, double preload = 0)
            => (t, q, qp) => preload-springRate*q-dampingRate*qp;
        public static JointMotor FunctionOfTime(Func<double, double> f_time)
            => (t, q, qp) => f_time(t);
        public static JointMotor ScaleMotor(JointMotor motor, double factor)
            => (t, q, qp) => factor*motor(t, q, qp);
        #endregion

        #region Mass Properties
        public void AddMassProperties(MassProperties additionalMassProperties)
        {
            MassProperties+=additionalMassProperties.ToConverted(units);
        }
        public void SubMassProperties(MassProperties additionalMassProperties)
        {
            MassProperties-=additionalMassProperties.ToConverted(units);
        }
        public void ZeroMassProperties() => MassProperties=MassProperties.Zero; 
        #endregion

        #region Mechanics
        public Vector33 GetJointAxis(Pose3 top)
        {
            Vector3 axis = top.Orientation.Rotate(localAxis);
            switch (type)
            {
                case JointType.Screw:
                {
                    return Vector33.Twist(axis, top.Position, pitch);
                }
                case JointType.Revolute:
                {
                    return Vector33.Twist(axis, top.Position, 0.0);
                }
                case JointType.Prismatic:
                {
                    return Vector33.Twist(axis);
                }
                default:
                throw new NotSupportedException("Unknown joint type.");
            }
        }

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


        #endregion

        #region Units
        public JointBodyInfo ToConverted(UnitSystem target)
        {
            var fl = Unit.Length.Convert(Units, target);
            var newpitch= pitch * fl;
            var newlocalPosition=localPosition.ToConverted(target);
            var newMassProperties=MassProperties.ToConverted(target);
            var newUnits=target;

            var info = new JointBodyInfo(target, type, newlocalPosition, localAxis, newpitch, newMassProperties);
            // TODO: Convert the motor, for both q,qp (prismatic) and result (frc/trq)
            return info;
        }

        public virtual void DoConvert(UnitSystem target)
        {
            if (units==target) return;

            var fl = Unit.Length.Convert(Units, target);
            this.pitch*=fl;
            this.localPosition=localPosition.ToConverted(target);
            this.MassProperties=this.MassProperties.ToConverted(target);
            this.Units=target;            
        }

        #endregion
    }

    public class JointBody : JointBodyInfo, ITree<JointBody>, ICanChangeUnits
    {
        internal JointBody parent;
        internal readonly List<JointBody> children;

        #region Factory
        JointBody(UnitSystem units, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch)
            : this(units, type, localPosition, localAxis, pitch, MassProperties.Zero)
        { }
        JointBody(JointBody parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch)
            : this(parent, type, localPosition, localAxis, pitch, MassProperties.Zero)
        { }
        JointBody(UnitSystem units, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
            : base(units, type, localAxis, localAxis, pitch, massProperties)
        {
            this.parent=null;
            this.children=new List<JointBody>();
        }
        JointBody(JointBody parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
            : base(parent?.units??UnitSystem.MKS, type, localAxis, localAxis, pitch, massProperties)
        {
            this.parent=parent;
            this.children=new List<JointBody>();
            if (parent!=null)
            {
                parent.children.Add(this);
            }
        }

        public JointBody AddScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => new JointBody(this, JointType.Screw, localPosition, localAxis, pitch);
        public JointBody AddRevolute(Pose3 localPosition, Vector3 localAxis)
            => new JointBody(this, JointType.Revolute, localPosition, localAxis, 0);
        public JointBody AddPrismatic(Pose3 localPosition, Vector3 localAxis)
            => new JointBody(this, JointType.Prismatic, localPosition, localAxis, double.PositiveInfinity);
        public static JointBody NewScrew(UnitSystem units, Pose3 localPosition, Vector3 localAxis, double pitch)
            => new JointBody(units, JointType.Screw, localPosition, localAxis, pitch);
        public static JointBody NewRevolute(UnitSystem units, Pose3 localPosition, Vector3 localAxis)
            => new JointBody(units, JointType.Revolute, localPosition, localAxis, 0);
        public static JointBody NewPrismatic(UnitSystem units, Pose3 localPosition, Vector3 localAxis)
            => new JointBody(units, JointType.Prismatic, localPosition, localAxis, double.PositiveInfinity);

        #endregion
        #region Properties

        public JointBody Parent
        {
            get => parent;
        }
        public IReadOnlyList<JointBody> Children => children;
        public bool IsRoot => parent==null;
        public bool IsLeaf => children.Count==0;

        public void AttachTo(JointBody parent)
        {
            this.parent?.children.Remove(this);
            this.parent=parent;
            this.parent?.children.Add(this);
        }

        public R Traverse<R>(R initialValue, Func<JointBody, R> operation)
        {
            R result = operation(this);
            foreach (var child in children)
            {
                result=child.Traverse(result, operation);
            }
            return result;
        }
        public void Traverse(Action<JointBody> operation)
        {
            operation(this);
            foreach (var child in children)
            {
                child.Traverse(operation);
            }
        }

        #endregion
        #region Mechanics
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
        public override void DoConvert(UnitSystem target)
        {
            base.DoConvert(target);
            foreach (var item in children)
            {
                item.DoConvert(target);
            }
        }

        #endregion
    }
}
