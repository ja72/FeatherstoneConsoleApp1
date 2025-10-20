using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

using Featherstone.ScrewCalculus;
using Featherstone.VectorCalculus;

namespace Featherstone.Dynamics
{
    public class RigidBody
    {
        internal double mass;
        internal Matrix3 bodyInertiaAtCg;
        internal Vector3 centerOfMass;

        #region Factory
        public RigidBody(double mass, Matrix3 bodyInertiaAtCg, Vector3 centerOfMass)
        {
            this.mass=mass;
            this.bodyInertiaAtCg=bodyInertiaAtCg;
            this.centerOfMass=centerOfMass;
        } 
        #endregion

        #region Properties
        public double Mass { get => mass; set => mass=value; }
        public Matrix3 BodyInertiaAtCg { get => bodyInertiaAtCg; set => bodyInertiaAtCg=value; }
        public Vector3 CenterOfMass { get => centerOfMass; set => centerOfMass=value; }
        #endregion

        #region Mechanics
        public void RotateInertiaTensor(Quaternion3 orientation)
            => RotateInertiaTensor(orientation.ToRotation());
        public void RotateInertiaTensor(Matrix3 rotation)
        {
            bodyInertiaAtCg = rotation * bodyInertiaAtCg * Matrix3.Transpose(rotation);
        }
        public void OffsetCenterOfMass(Vector3 delta)
        {
            centerOfMass += delta;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Pose3 GetCenterOfMass(Pose3 top)
        {
            return top+Pose3.At(centerOfMass);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 GetBodyIntertiaTensor(bool inverse = false)
        {
            if (inverse)
            {
                if (bodyInertiaAtCg.TryInvert(out var IcgInv))
                {
                    return IcgInv;
                }
                throw new InvalidOperationException("Body inertia tensor is not invertible.");
            }
            return bodyInertiaAtCg;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 GetInertiaTensor(Quaternion3 orientation, bool inverse = false)
        {
            var R = orientation.ToRotation();
            return GetInertiaTensor(R, inverse);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 GetInertiaTensor(Matrix3 rotation, bool inverse = false)
        {
            var I = GetBodyIntertiaTensor(inverse);
            return rotation*I*Matrix3.Transpose(rotation);
        }
        public Matrix33 Spi(Pose3 cg)
        {
            // Spatial inertia matrix:
            // [ m*I3           -m*cgx  ]
            // [ m*cgx   Icm-m*cgx*cgx  ]
            var mI3 = Matrix3.Diagonal(mass,mass,mass);
            var R = cg.orientation.ToRotation();
            var I_body = GetBodyIntertiaTensor();
            var inertia = R * I_body * Matrix3.Transpose(R);
            var mcgx = Matrix3.CrossOp(Vector3.Scale(cg.position, mass));
            return new Matrix33(
                mI3,
                Matrix3.Negate(mcgx),
                mcgx,
                inertia+Matrix3.Scale(Matrix3.MomentTensor(cg.position), mass)
            );
        }
        public Matrix33 Spm(Pose3 cg)
        {
            // Spatial mobility matrix:
            // [ 1/m-cgx*I_inv*cgx  cgx*I_inv ]
            // [ -I_inv*cgx           I_inv   ]
            var I_body_inv = GetBodyIntertiaTensor(true);
            var R = cg.orientation.ToRotation();
            var I_inv = R * I_body_inv * Matrix3.Transpose(R);
            double m_inv = 1/mass;
            var mI3 = Matrix3.Diagonal(m_inv,m_inv,m_inv);
            var cgx = Matrix3.CrossOp(cg.position);
            var a11 = Matrix3.Subtract(mI3, Matrix3.Product(cgx, Matrix3.Product(I_inv, cgx)));
            var a12 = Matrix3.Product(cgx, I_inv);
            var a21 = Matrix3.Negate(Matrix3.Product(I_inv, cgx));
            var a22 = I_inv;
            return new Matrix33(
                a11,
                a12,
                a21,
                a22
            );
        }

        #endregion
    }
}
