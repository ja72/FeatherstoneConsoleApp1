using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

using JA.Geometry;

namespace JA.Dynamics
{
    using Vector3 = JA.VectorCalculus.Vector3;
    using Matrix3 = JA.VectorCalculus.Matrix3;
    using Quaternion3 = JA.VectorCalculus.Quaternion3;
    using Pose3 = JA.VectorCalculus.Pose3;
    using Vector33 = JA.ScrewCalculus.Vector33;
    using Matrix33 = JA.ScrewCalculus.Matrix33;

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public readonly struct MassProperties :
        ICanChangeUnits<MassProperties>,
        IEquatable<MassProperties>
    {
        public static readonly MassProperties Zero = new MassProperties(UnitSystem.SI, 0, Matrix3.Zero, Vector3.Zero);
        readonly (float mass, Matrix3 mmoi, Vector3 cg) data;
        internal (float mass, Matrix3 mmoi, Vector3 cg) Data => data;

        #region Factory
        public MassProperties(UnitSystem units, float mass, Matrix3 mmoi, Vector3 cg) : this(units, (mass, mmoi, cg)) { }
        public MassProperties(UnitSystem units, (float mass, Matrix3 mmoi, Vector3 cg) data)
        {
            this.Units = units;
            this.data = data;
        }
        public static UnitSystem DefaultUnits { get; } = UnitSystem.SI;
        public static MassProperties Empty { get; } = new MassProperties(DefaultUnits, 0.0f, Matrix3.Scalar(0.0f), Vector3.Zero);
        public static MassProperties Default { get; } = new MassProperties(DefaultUnits, 1.0f, Matrix3.Scalar(1.0f), Vector3.Zero);
        public MassProperties WithMass(float mass) 
            => data.mass>0 
            ? new MassProperties(Units, ( mass, (mass/data.mass)*data.mmoi, data.cg ))
            : new MassProperties(Units, ( mass, data.mmoi, data.cg ));
        #endregion

        #region Properties
        public UnitSystem Units { get; }
        public float Mass
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.mass;
        }
        public Matrix3 MMoi
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.mmoi;
        }
        public Vector3 CG
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.cg;
        }

        public MassProperties At(Pose3 @base)
        {
            var newCg = Pose3.Add(@base, data.cg);
            return new MassProperties(Units, data.mass, data.mmoi, newCg);
        }

        #endregion

        #region Geometry
        public static MassProperties Cylinder(UnitSystem units, float mass, float radius, float height)
            => new MassProperties(units,
                mass,
                Matrix3.Diagonal(
                    mass / 4 * radius * radius + mass / 12 * height * height,
                    mass / 4 * radius * radius + mass / 12 * height * height,
                    mass / 2 * radius * radius),
                    Vector3.Zero);
        public static MassProperties Cylinder(Material material, float radius, float height)
            => Cylinder(material.Units, material.Density * (float)Math.PI * radius * radius * height, radius, height);
        public static MassProperties Sphere(UnitSystem units, float mass, float radius)
            => new MassProperties(units,
                mass,
                Matrix3.Diagonal(
                    2 * mass / 5 * radius * radius,
                    2 * mass / 5 * radius * radius,
                    2 * mass / 5 * radius * radius),
                    Vector3.Zero);
        public static MassProperties Sphere(Material material, float radius)
            => Sphere(material.Units, material.Density * (4f / 3f) * (float)Math.PI * radius * radius * radius, radius);
        public static MassProperties Box(UnitSystem units, float mass, float width, float height, float thickness)
        {
            float I_xx = mass/12*( thickness*thickness+height*height );
            float I_yy = mass/12*( width*width+thickness*thickness );
            float I_zz = mass/12*( width*width+height*height );
            return new MassProperties(units,
                        mass,
                        Matrix3.Diagonal(
                            I_xx,
                            I_yy,
                            I_zz),
                        Vector3.Zero);
        }
        public static MassProperties Box(Material material, float width, float height, float thickness) 
            => Box(material.Units, material.Density*width*height*thickness, width, height, thickness);

        public static MassProperties FromMeshAndMass(Mesh mesh, float mass)
        {
            GetVolumeProperties(mesh, out _, out var c, out var I);
            I *= mass;
            return new MassProperties(mesh.Units, mass, I, c);
        }
        public static MassProperties FromMeshAndDensity(Mesh mesh, float density)
        {
            GetVolumeProperties(mesh, out var volume, out var c, out var I);
            float mass = density * volume;
            I *= mass;
            return new MassProperties(mesh.Units, mass, I, c);
        }
        static void GetVolumeProperties(Mesh mesh, out float volume, out Vector3 center, out Matrix3 specificMmoi)
        {
            volume = 0;
            center = Vector3.Zero;
            specificMmoi = Matrix3.Zero;
            float dV;
            System.Numerics.Vector3 dc;
            Matrix3 dI;
            for (int index = 0; index < mesh.ElementList.Count; index++)
            {
                var triangles = mesh.GetTriangles(index);
                
                foreach (var triangle in triangles)
                {
                    dV = System.Numerics.Vector3.Dot(
                        triangle.A, 
                            System.Numerics.Vector3.Cross(triangle.B, triangle.C)) / 6;

                    dc = (triangle.A + triangle.B + triangle.C) / 4;
                    var AB = Vector3.FromVector(triangle.A+triangle.B);
                    var BC = Vector3.FromVector(triangle.B+triangle.C);
                    var CA = Vector3.FromVector(triangle.C+triangle.A);
                    var ABAB = AB.MomentTensor();
                    var BCBC = BC.MomentTensor();
                    var CACA = CA.MomentTensor();
                    dI = ( ABAB + BCBC + CACA ) / 20;

                    volume += dV;
                    center += Vector3.FromVector(dV * dc);
                    specificMmoi += dV * dI;
                }
            }
            center /= volume;
            specificMmoi /= volume;

            specificMmoi -= center.MomentTensor();
        }

        #endregion

        #region Mechanics
        public Vector33 GetWeight(Pose3 pose, Vector3 gravity, out Vector3 cg)
        {
            cg = Pose3.Add(pose, data.cg);
            return Dynamics.GetWeight(data.mass, cg, gravity);
        }
        public Matrix33 Spi(Pose3 pose,out Matrix3 wolrdInertiaAtCg)
        {
            return Spi(pose.Orientation, Pose3.Add(pose, data.cg), out wolrdInertiaAtCg);
        }
        public Matrix33 Spi(Quaternion3 orientation, Vector3 cg, out Matrix3 wolrdInertiaAtCg)
        {
            wolrdInertiaAtCg = Dynamics.GetMmoiMatrix(data.mmoi, orientation);
            return Dynamics.Spi(data.mass, wolrdInertiaAtCg, cg);
        }
        public Matrix33 Spm(Pose3 pose, out Matrix3 wolrdInertiaAtCgInverse)
        {
            return Spm(pose.Orientation, Pose3.Add(pose, data.cg), out wolrdInertiaAtCgInverse);
        }
        public Matrix33 Spm(Quaternion3 orientation, Vector3 cg, out Matrix3 wolrdInertiaAtCgInverse)
        {
            wolrdInertiaAtCgInverse = Dynamics.GetMmoiMatrix(data.mmoi, orientation, true);
            return Dynamics.Spm(data.mass, wolrdInertiaAtCgInverse, cg);
        }
        #endregion

        #region Algebra
        public static MassProperties Scale(float factor, MassProperties a)
        {
            return new MassProperties(a.Units, factor * a.Mass, factor * a.MMoi, a.CG);
        }

        public static MassProperties Add(MassProperties a, MassProperties b)
        {
            if (a.Units != b.Units) throw new NotSupportedException();
            float m = a.Mass + b.Mass;
            var cg = (a.CG * a.Mass + b.CG * b.Mass)/m;
            var mmoi = a.MMoi + a.Mass * a.CG.MomentTensor() + b.MMoi + b.Mass * b.CG.MomentTensor();
            return new MassProperties(a.Units, m, mmoi - m * cg.MomentTensor(), cg);
        }
        public static MassProperties Subtract(MassProperties a, MassProperties b)
        {
            if (a.Units != b.Units) throw new NotSupportedException();
            float m = a.Mass - b.Mass;
            var cg = (a.CG * a.Mass - b.CG * b.Mass) / m;
            var mmoi = a.MMoi + a.Mass * a.CG.MomentTensor() - b.MMoi - b.Mass * b.CG.MomentTensor();
            return new MassProperties(a.Units, m, mmoi - m * cg.MagnitudeSquared, cg);
        }

        public static MassProperties operator +(MassProperties a, MassProperties b) => Add(a, b);
        public static MassProperties operator -(MassProperties a, MassProperties b) => Subtract(a, b);
        public static MassProperties operator *(float factor, MassProperties a) => Scale(factor, a);
        public static MassProperties operator *(MassProperties a, float factor) => Scale(factor, a);
        public static MassProperties operator /(MassProperties a, float divisor) => Scale(1/divisor, a);
        #endregion

        #region Formatting
        public override string ToString()
            => $"Body(Units={Units}, Mass={Mass:g3}, MMOI={MMoi:g3}, CG={CG:g3})";

        #endregion

        #region Units
        public MassProperties ConvertTo(UnitSystem target)
        {
            if (Units == target) return this;
            var copy = (
                Unit.Mass.Convert(Units, target)*data.mass,
                Unit.MassMomentOfInertia.Convert(Units, target) * data.mmoi,
                Unit.Length.Convert(Units, target) * data.cg);
            return new MassProperties(target, copy);
        }
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(MassProperties)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is MassProperties other)
            {
                return Equals(other);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="MassProperties"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="MassProperties"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(MassProperties other)
        {
            return data.Equals(other.data);
        }

        /// <summary>
        /// Calculates the hash code for the <see cref="MassProperties"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            return data.GetHashCode();
        }

        #endregion

    }
}
