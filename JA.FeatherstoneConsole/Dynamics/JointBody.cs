using System;
using System.Collections.Generic;

using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    /// <summary>
    /// Class to describe the structure of a mechanical system
    /// </summary>
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
            : base(units, type, localPosition, localAxis, pitch, massProperties)
        {
            this.parent=null;
            this.children=new List<JointBody>();
        }
        JointBody(JointBody parent, JointType type, Pose3 localPosition, Vector3 localAxis, double pitch, MassProperties massProperties)
            : base(parent?.units??UnitSystem.MKS, type, localPosition, localAxis, pitch, massProperties)
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

        #region Formatting
        public override string ToString()
        {
            var info = base.ToString();

            return $"{info} #parents={(IsRoot?0:1)}, #children={children.Count}";
        }
        #endregion

    }
}
