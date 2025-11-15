using System;
using System.ComponentModel;

using JA.LinearAlgebra.Screws;
using JA.LinearAlgebra.Vectors;

namespace JA.Dynamics
{
    public enum JointType
    {
        /// <summary>
        /// Specifies a joint that allows translational and rotational movement about a single axis.
        /// </summary>
        [Description("Combined rotation and parallel translation")]
        Screw,
        /// <summary>
        /// Specifies a joint that allows pure rotational movement about a single axis.
        /// </summary>
        [Description("Pure rotation about an axis")]
        Revolute,
        /// <summary>
        /// Specifies a joint that allows pure translational movement along a single axis.
        /// </summary>
        [Description("Pure translation along an axis")]
        Prismatic,
    }

    /// <summary>
    /// Class to contain information about a joint and body in a mechanical system.
    /// </summary>
    public class JointBodyInfo : ICanChangeUnits<JointBodyInfo>, ICanChangeUnits
    {
        internal readonly JointType type;
        internal Pose3 localPosition;
        internal Vector3 localAxis;
        internal double pitch;
        internal UnitSystem units;
        internal Motor motor;
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
            this.Motor=Motor.ConstForcing(0);
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
            get => localPosition;
            set => localPosition=value;
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

        public Motor Motor
        {
            get => motor;
            set => motor=value;
        } 
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
            Vector3 axis = top.Orientation.RotateVector(localAxis);
            switch (type)
            {
                case JointType.Screw:
                {
                    return Twist3.At(axis, top.Position, pitch);
                }
                case JointType.Revolute:
                {
                    return Twist3.At(axis, top.Position, 0.0);
                }
                case JointType.Prismatic:
                {
                    return Twist3.Pure(axis);
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

    }
}
