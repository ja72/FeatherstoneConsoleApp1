using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public class World : ICanChangeUnits
    {
        internal UnitSystem units;
        internal Vector3 gravity;
        internal readonly List<Joint> rootJoints;

        #region Factor
        public World(UnitSystem units)
        {
            this.units=units;
            this.gravity = -Vector3.UnitY * units.EarthGravity();
            this.rootJoints=new List<Joint>();
        }
        #endregion

        #region Property
        public UnitSystem Units { get => units; }
        public Vector3 Gravity { get => gravity; set => gravity=value; }
        public IReadOnlyList<Joint> RootJoints => rootJoints;

        public void ConvertTo(UnitSystem target)
        {

        }

        #endregion

        #region Structure
        public Joint NewScrew(Pose3 localPosition, Vector3 localAxis, double pitch)
            => NewScrew(Units, localPosition, localAxis, pitch);
        public Joint NewScrew(UnitSystem units, Pose3 localPosition, Vector3 localAxis, double pitch)
        {
            var joint = Joint.NewScrew(units, localPosition, localAxis, pitch);
            rootJoints.Add(joint);
            return joint;
        }
        public Joint NewRevolute(Pose3 localPosition, Vector3 localAxis)
            => NewRevolute(Units, localPosition, localAxis);
        public Joint NewRevolute(UnitSystem units, Pose3 localPosition, Vector3 localAxis)
        {
            var joint = Joint.NewRevolute(units, localPosition, localAxis);
            rootJoints.Add(joint);
            return joint;
        }
        public Joint NewPrismatic(Pose3 localPosition, Vector3 localAxis)
            => NewPrismatic(Units, localPosition, localAxis);
        public Joint NewPrismatic(UnitSystem units, Pose3 localPosition, Vector3 localAxis)
        {
            var joint = Joint.NewPrismatic(units, localPosition, localAxis);
            rootJoints.Add(joint);
            return joint;
        }

        public List<Joint> GetAllJoints()
        {
            var allJoints=new List<Joint>();
            foreach (var joint in rootJoints)
            {
                joint.Traverse( (j)=> allJoints.Add(j) );
            }
            return allJoints;
        }

        internal int[] GetParents(Joint[] allJoints)
        {
            int n = allJoints.Length;
            int[] parents = new int[n];
            for (int i = 0; i<n; i++)
            {
                var joint = allJoints[i];
                if (joint.Parent==null)
                {
                    parents[i]=-1;
                }
                else
                {
                    parents[i]=Array.IndexOf(allJoints, joint.Parent);
                }
            }
            return parents;
        }

        internal int[][] GetChildren(Joint[] allJoints)
        {
            int n = allJoints.Length;
            int[][] children = new int[n][];
            for (int i = 0; i<n; i++)
            {
                var joint = allJoints[i];
                children[i]=new int[joint.Children.Count];
                for (int j = 0; j<joint.Children.Count; j++)
                {
                    children[i][j]=Array.IndexOf(allJoints, joint.Children[j]);
                }
            }
            return children;
        }

        #endregion

        #region Mechanics
        public Simulation ToSimulation()
        {
            return new Simulation(this);
        }


        #endregion
    }

}
