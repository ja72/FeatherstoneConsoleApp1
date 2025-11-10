using System;
using System.Collections.Generic;
using System.Linq;

using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public abstract class Element : ITree<Element>
    {
        readonly List<Element> children;
        Element parent;

        #region Factory
        protected Element()
        {
            this.Units=UnitSystem.MKS;
            this.parent=null;
            this.children=new List<Element>();
        }
        protected Element(Element parent)
        {
            this.Units=parent.Units;
            this.parent=parent;
            this.children=new List<Element>();

            if (parent!=null)
            {
                parent.children.Add(this);
            }
        }

        public Link AddLink(Pose3 position, MassProperties massProperties)
        => new Link(this, position, massProperties);
        public Link AddLink(Pose3 position)
            => new Link(this, position, MassProperties.Empty);
        public Joint AddPrismatic(Vector3 axis)
            => new Joint(this, JointType.Prismatic, axis);
        public Joint AddRevolute(Vector3 axis)
            => new Joint(this, JointType.Revolute, axis);
        public Joint AddScrew(Vector3 axis, double pitch)
            => new Joint(this, JointType.Screw, axis, pitch);
        public Joint AddSliderPin(Vector3 sliderAxis, Vector3 pinAxis)
            => AddPrismatic(sliderAxis)
                .AddRevolute(pinAxis);
        public Joint AddSliderFollower(Vector3 sliderAxis, Vector3 pinAxis, double offset)
        {
            Vector3 side = Vector3.Cross(pinAxis, sliderAxis).Normalize();
            return AddPrismatic(sliderAxis)
            .AddLink(side*offset)
            .AddRevolute(pinAxis);
        }

        public Joint AddPlanarJoint(Vector3 firstAxis, Vector3 secondAxis)
        {
            secondAxis=Vector3.Cross(Vector3.Cross(
                firstAxis, secondAxis).Normalize(),
                    firstAxis).Normalize();
            return AddPrismatic(firstAxis)
                .AddPrismatic(secondAxis);
        }

        public Joint AddPlanarRevJoint(Vector3 firstAxis, Vector3 secondAxis)
        {
            var revAxis = Vector3.Cross(firstAxis, secondAxis).Normalize();
            secondAxis=Vector3.Cross(revAxis, firstAxis).Normalize();
            return AddPrismatic(firstAxis)
                .AddPrismatic(secondAxis)
                .AddRevolute(revAxis);
        }
        #endregion

        #region Properties
        public Element Parent => parent;
        public IReadOnlyList<Element> Children => children;
        public bool IsRoot { get => parent==null; }
        public bool IsLeaf { get => children.Count==0; }
        public abstract bool IsJoint { get; }
        public int TotalDOF { get => children.Sum((e) => e.TotalDOF)+ThisDOF; }
        public int ThisDOF { get => IsJoint ? 1 : 0; }
        public UnitSystem Units { get; }
        public virtual void DoConvert(UnitSystem target)
        {
            if (Units==target) return;
            foreach (var item in children)
            {
                item.DoConvert(target);
            }
        }
        #endregion

        public R Traverse<R>(R initialValue, Func<Element, R> operation)
        {
            R result = operation(this);
            foreach (var child in children)
            {
                result=child.Traverse(result, operation);
            }
            return result;
        }

        public void Traverse(Action<Element> operation)
        {
            operation(this);
            foreach (var child in children)
            {
                child.Traverse(operation);
            }
        }

        #region Implementation

        public class Link : Element
        {
            public Link(Pose3 localStep, MassProperties massProperties) : base()
            {
                this.LocalStep=localStep.ToConverted(Units);
                this.MassProperties=massProperties.ToConverted(Units);
            }
            public Link(Element parent, Pose3 localStep, MassProperties massProperties) : base(parent)
            {
                this.LocalStep=localStep.ToConverted(Units);
                this.MassProperties=massProperties.ToConverted(Units);
            }
            public override bool IsJoint => false;
            public Pose3 LocalStep { get; protected set; }
            public MassProperties MassProperties { get; protected set; }

            public override void DoConvert(UnitSystem target)
            {
                LocalStep=LocalStep.ToConverted(target);
                MassProperties=MassProperties.ToConverted(target);
                base.DoConvert(target);
            }
        }

        public class Joint : Element
        {
            readonly JointType type;
            readonly Vector3 axis;
            double pitch;
            Motor motor;
            (double q, double qp) initialConditions;

            #region Factory
            public Joint(JointType type, Vector3 axis, double pitch = 0) : base()
            {
                this.type=type;
                this.axis=axis;
                this.pitch=pitch;
                this.motor=Motor.ConstForcing(0);
                this.initialConditions=(0, 0);
            }
            public Joint(Element parent, JointType type, Vector3 axis, double pitch = 0) : base(parent)
            {
                this.type=type;
                this.axis=axis;
                this.pitch=pitch;
                this.motor=Motor.ConstForcing(0);
                this.initialConditions=(0, 0);
            }
            public static Joint Revolute(Element parent, Vector3 axis)
            {
                return new Joint(parent, JointType.Revolute, axis);
            }
            public static Joint Prismatic(Element parent, Vector3 axis)
            {
                return new Joint(parent, JointType.Prismatic, axis);
            }
            public static Joint Screw(Element parent, Vector3 axis, double pitch)
            {
                return new Joint(parent, JointType.Screw, axis, pitch);
            }
            public static Joint Revolute(Vector3 axis)
            {
                return new Joint(JointType.Revolute, axis);
            }
            public static Joint Prismatic(Vector3 axis)
            {
                return new Joint(JointType.Prismatic, axis);
            }
            public static Joint Screw(Vector3 axis, double pitch)
            {
                return new Joint(JointType.Screw, axis, pitch);
            }
            #endregion

            #region Properties
            public JointType Type { get => type; }
            public Vector3 Axis { get => axis; }
            public double Pitch { get => pitch; }
            public override bool IsJoint => true;
            public Motor Motor
            {
                get => motor;
                set => motor=value;
            }
            public (double q, double qp) InitialConditions
            {
                get => initialConditions;
                set => initialConditions=value;
            }
            #endregion

            public override void DoConvert(UnitSystem target)
            {
                float f_len = Unit.Length.Convert(Units, target);
                float f_tq = Unit.Torque.Convert(Units, target);
                float f_frc = Unit.Force.Convert(Units, target);

                pitch*=f_len;

                base.DoConvert(target);
            }
        }


        #endregion
    }

}
