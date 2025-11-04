using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

using JA.LinearAlgebra.ScrewCalculus;
using JA.LinearAlgebra.VectorCalculus;

namespace JA.Dynamics
{
    public static class Dynamics
    {
        public static Matrix3 GetMmoiMatrix(Matrix3 bodyInertiaAtCg, Quaternion3 orientation, bool inverse = false)
        {
            Matrix3 I_body = bodyInertiaAtCg;
            if(inverse && I_body.TryInvert(out var I_body_inv))
            {
                I_body=I_body_inv;
            }
            Matrix3 R = orientation.ToRotation();
            return R*I_body*R.Transpose();
        }

        public static Vector33 GetWeight(double mass, Vector3 cg, Vector3 gravity)
        {
            // W = [  m*g  ]
            //     [ cgx*m*g]
            var fg = Vector3.Scale(gravity, mass);
            return Screws.WrenchAt(fg, cg);
        }

        /// <summary>
        /// Spatial Inertia Matrix
        /// </summary>
        /// <param name="mass">Body mass</param>
        /// <param name="worldInertiaAtCg">Body mass moment of inertia matrix about center of gravity, and aligned with global coordinates
        /// </param>
        /// <param name="cg">Body center of mass position</param>
        /// <returns></returns>
        public static Matrix33 Spi(double mass, Matrix3 worldInertiaAtCg, Vector3 cg)
        {
            // Spatial inertia matrix:
            // [ m*I3           -m*cgx  ]
            // [ m*cgx   Icm-m*cgx*cgx  ]
            var a11 = Matrix3.Scalar(mass);
            var a21 = Matrix3.CrossOp(cg, mass);
            var a12 = Matrix3.Negate(a21);
            var a22 = worldInertiaAtCg + Matrix3.MomentTensor(cg, mass);
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
        /// <param name="worldInertiaAtCgInverse">Inverse body mass moment of inertia matrix about center of gravity, and aligned with global coordinates
        /// </param>
        /// <param name="cg">Body center of mass position</param>
        /// <returns></returns>
        public static Matrix33 Spm(double mass, Matrix3 worldInertiaAtCgInverse, Vector3 cg)
        {
            // Spatial mobility matrix:
            // [ 1/m-cgx*I_inv*cgx  cgx*I_inv ]
            // [ -I_inv*cgx           I_inv   ]
            var mI3 = Matrix3.Scalar(1/mass);
            var cgx = Matrix3.CrossOp(cg);
            var a12 = (cgx * worldInertiaAtCgInverse);
            var a11 = mI3 - a12 * cgx;
            var a21 = Matrix3.Product(worldInertiaAtCgInverse, -cgx);
            var a22 = worldInertiaAtCgInverse;
            return new Matrix33(
                a11,
                a12,
                a21,
                a22
            );
        }
    }
}
