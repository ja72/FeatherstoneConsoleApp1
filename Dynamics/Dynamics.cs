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
    public static class Dynamics
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Pose3 GetJointStep(double q, Pose3 @base, JointType type, Vector3 localAxis, double pitch = 0)
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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Twist33 GetJointAxis(Pose3 top, JointType type, Vector3 localAxis, double pitch = 0)
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
        public static Matrix3 GetMmoiMatrix(this RigidBody body, Pose3 position, bool inverse = false)
        {
            Matrix3 I_body = body.bodyInertiaAtCg;
            if(inverse && I_body.TryInvert(out var I_body_inv))
            {
                I_body=I_body_inv;
            }
            Matrix3 R = position.Orientation.ToRotation();
            return R*I_body*R.Transpose();
        }
        /// <summary>
        /// Spatial Inertia Matrix
        /// </summary>
        /// <param name="mass">Body mass</param>
        /// <param name="Ic">Body mass moment of inertia matrix about center of gravity, and aligned with global coordinates
        /// </param>
        /// <param name="cg">Body center of mass position</param>
        /// <returns></returns>
        public static Matrix33 Spi(double mass, Matrix3 Ic, Vector3 cg)
        {
            // Spatial inertia matrix:
            // [ m*I3           -m*cgx  ]
            // [ m*cgx   Icm-m*cgx*cgx  ]
            var a11 = Matrix3.Scalar(mass);
            var a21 = Matrix3.CrossOp(Vector3.Scale(cg, mass));
            var a12 = Matrix3.Negate(a21);
            var a22 = Ic + mass * Matrix3.MomentTensor(cg);
            return new Matrix33(
                a11,
                a12,
                a21,
                a22
            );
        }
        /// <summary>
        /// Spatial Inverse Inertia Matrix
        /// </summary>
        /// <param name="mass">Body mass</param>
        /// <param name="Ic_inv">Inverse body mass moment of inertia matrix about center of gravity, and aligned with global coordinates
        /// </param>
        /// <param name="cg">Body center of mass position</param>
        /// <returns></returns>
        public static Matrix33 Spm(double mass, Matrix3 Ic_inv, Vector3 cg)
        {
            // Spatial mobility matrix:
            // [ 1/m-cgx*I_inv*cgx  cgx*I_inv ]
            // [ -I_inv*cgx           I_inv   ]
            var mI3 = Matrix3.Scalar(1/mass);
            var cgx = Matrix3.CrossOp(cg);
            var a12 = (cgx * Ic_inv);
            var a11 = mI3 - a12 * cgx;
            var a21 = -(Matrix3.Product(Ic_inv, cgx));
            var a22 = Ic_inv;
            return new Matrix33(
                a11,
                a12,
                a21,
                a22
            );
        }

    }
}
