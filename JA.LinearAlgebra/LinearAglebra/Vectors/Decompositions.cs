
using System;
using System.Diagnostics;

using static System.Math;

namespace JA.LinearAlgebra.Vectors
{
    /// <summary>
    /// Polar and SVD decompositions of 3×3 matrices.
    /// </summary>
    /// <remarks>Code taken from https://theorangeduck.com/page/closed-form-matrix-decompositions</remarks>
    public static class Decompositions
    {
        #region LU

        /// <summary>
        /// Decompose Matrix into <code>M = L*U</code>
        /// </summary>
        /// <param name="M">The Matrix</param>
        /// <param name="L">A lower triangular matrix with diagonal elements</param>
        /// <param name="U">A upper triangular matrix</param>
        public static void LU(this Matrix3 M, out Matrix3 L, out Matrix3 U)
        {
            double A11 = M.m11, A12 = M.m12, A13 = M.m13;
            double A21 = M.m21, A22 = M.m22, A23 = M.m23;
            double A31 = M.m31, A32 = M.m32, A33 = M.m33;

            double U12=A12/A11,U13=A13/A11,U23=(A21*U13-A23)/(A21*U12-A22);
            double L11=A11,L22=A22-A21*U12,L33=A31*(U12*U23-U13)-A32*U23+A33;
            double L21=A21,L31=A31,L32=A32-A31*U12;

            L=new Matrix3(L11, 0, 0, L21, L22, 0, L31, L32, L33);
            U=new Matrix3(1, U12, U13, 0, 1, U23, 0, 0, 1);
        }
        /// <summary>
        /// Decompose Matrix into <code>M = L*D*U</code>
        /// </summary>
        /// <param name="M">The Matrix</param>
        /// <param name="L">A lower triangular matrix</param>
        /// <param name="U">A upper triangular matrix</param>
        /// <param name="D">A diagonal matrix</param>
        public static void LUD(this Matrix3 M, out Matrix3 L, out Matrix3 U, out Matrix3 D)
        {
            double A11 = M.m11, A12 = M.m12, A13 = M.m13;
            double A21 = M.m21, A22 = M.m22, A23 = M.m23;
            double A31 = M.m31, A32 = M.m32, A33 = M.m33;

            double D11=A11,L21=A21/A11,L31=A31/A11;
            double U12=A12/A11,D22=A22-A21*U12,L32=( A32-A31*U12 )/D22;
            double U13=A13/D11, U23=A23/D22-A13*L21/D22, D33=A13*(L21*L32-L31)-A23*L32+A33;

            L=new Matrix3(1, 0, 0, L21, 1, 0, L31, L32, 1);
            U=new Matrix3(1, U12, U13, 0, 1, U23, 0, 0, 1);
            D=new Matrix3(D11, 0, 0, 0, D22, 0, 0, 0, D33);
        }
        #endregion

        #region SVD
        public static void mat3_svd_stable(Matrix3 M, out Matrix3 U, out Vector3 s, out Matrix3 V)
        // More numerically stable version of mat3_svd
        {
            double scale = Matrix3.MaxAbs(M);
            if (scale==0.0)
            {
                U=Matrix3.Identity;
                s=new Vector3();
                V=Matrix3.Identity;
                return;
            }

            Matrix3 R, S;
            mat3_polar(M/scale, out R, out S, out s);

            V=mat3_sym_evecs_from_evals(S, s);
            U=Matrix3.Product(R, V);
            s=scale*s;
        }
        public static void mat3_svd(Matrix3 M, out Matrix3 U, out Vector3 s, out Matrix3 V)
        // Compute SVD using polar decomposition and symmetric eigenvector computation
        {
            Matrix3 R, S;
            mat3_polar(M, out R, out S, out s);

            V=mat3_sym_evecs_from_evals(S, s);
            U=Matrix3.Product(R, V);
        }
        static Vector3 mat3_sym_evec0(Matrix3 M, double eval0)
        // For a symmetric matrix returns the first eigen vector given the first eigen value
        {
            Trace.Assert(M.IsSymmetric());

            var row0 = new Vector3(M.m11 - eval0, M.m12, M.m13);
            var row1 = new Vector3(M.m21, M.m22 - eval0, M.m23);
            var row2 = new Vector3(M.m31, M.m32, M.m33 - eval0);

            var r0xr1 = Vector3.Cross(row0, row1);
            var r0xr2 = Vector3.Cross(row0, row2);
            var r1xr2 = Vector3.Cross(row1, row2);

            double d0 = Vector3.Dot(r0xr1, r0xr1);
            double d1 = Vector3.Dot(r0xr2, r0xr2);
            double d2 = Vector3.Dot(r1xr2, r1xr2);

            if (d0==0.0&&d1==0.0&&d2==0.0)
            {
                // No valid eigen vectors
                return new Vector3(1.0, 0.0, 0.0);
            }
            else
            {
                // Return largest candidate
                return d0>=d1&&d0>=d2 ? r0xr1/Sqrt(d0) :
                       d1>=d0&&d1>=d2 ? r0xr2/Sqrt(d1) :
                                              r1xr2/Sqrt(d2);
            }
        }

        static Vector3 mat3_sym_evec1(Matrix3 M, Vector3 evec0, double eval1)
        // For a symmetric matrix returns the second eigen vector given the first eigen vector 
        // and the second eigen value
        {
            Trace.Assert(M.IsSymmetric());

            Vector3 u = Abs(evec0.x) > Abs(evec0.y) ?
                    new Vector3(-evec0.z, 0.0, +evec0.x) / Sqrt(evec0.x * evec0.x + evec0.z * evec0.z) :
                    new Vector3(0.0, +evec0.z, -evec0.y) / Sqrt(evec0.y * evec0.y + evec0.z * evec0.z);

            var v = Vector3.Cross(evec0, u);

            double m00 = Vector3.Dot(u, Matrix3.Product(M, u)) - eval1;
            double m01 = Vector3.Dot(u, Matrix3.Product(M, v));
            double m11 = Vector3.Dot(v, Matrix3.Product(M, v)) - eval1;

            if (Abs(m00)>=Abs(m11))
            {
                if (Max(Abs(m00), Abs(m01))<=0.0)
                {
                    return u;
                }

                if (Abs(m00)>=Abs(m01))
                {
                    m01/=m00;
                    m00=1.0/Sqrt(1.0+m01*m01);
                    return m01*m00*u-m00*v;
                }
                else
                {
                    m00/=m01;
                    m01=1.0/Sqrt(1.0+m00*m00);
                    return m01*u-m00*m01*v;
                }
            }
            else
            {
                if (Max(Abs(m00), Abs(m01))<=0.0)
                {
                    return u;
                }

                if (Abs(m11)>=Abs(m01))
                {
                    m01/=m11;
                    m11=1.0/Sqrt(1.0+m01*m01);
                    return m11*u-m01*m11*v;
                }
                else
                {
                    m11/=m01;
                    m01=1.0/Sqrt(1.0+m11*m11);
                    return m11*m01*u-m01*v;
                }
            }
        }

        public static Matrix3 mat3_sym_evecs_from_evals(Matrix3 M, Vector3 evals)
        // Find the eigen vectors from the eigen values of a symmetric 3x3 matrix
        {
            // Assert matrix is symmetric
            Trace.Assert(M.IsSymmetric());

            // Check matrix has some magnitude
            if (M.m12*M.m12+M.m13*M.m13+M.m23*M.m23<=0.0)
            {
                return Matrix3.Identity;
            }

            // Compute Eigen Vectors from Eigen Values
            if (Matrix3.Determinant(M)>=0.0)
            {
                Vector3 evec0 = mat3_sym_evec0(M, evals.x);
                Vector3 evec1 = mat3_sym_evec1(M, evec0, evals.y);
                var evec2 = Vector3.Cross(evec1, evec0);
                return Matrix3.FromColumns(evec0, evec1, evec2);
            }
            else
            {
                Vector3 evec2 = mat3_sym_evec0(M, evals.z);
                Vector3 evec1 = mat3_sym_evec1(M, evec2, evals.y);
                var evec0 = Vector3.Cross(evec2, evec1);
                return Matrix3.FromColumns(evec0, evec1, evec2);
            }
        }
        #endregion

        #region Polar
        public static void mat3_polar_stable(Matrix3 M, out Matrix3 R, out Matrix3 S, out Vector3 s)
        // More numerically stable version of mat3_polar
        {
            double scale = Matrix3.MaxAbs(M);
            if (scale==0.0)
            {
                R=Matrix3.Identity;
                S=M;
                s=new Vector3();
                return;
            }

            mat3_polar(M/scale, out R, out S, out s);
            S=scale*S;
            s=scale*s;
        }
        public static void mat3_polar(Matrix3 M, out Matrix3 R, out Matrix3 S, out Vector3 s)
        // Computes the polar decomposition and singular values of a matrix M using the 
        // closed-form solution
        {
            double A = Matrix3.NormSquared(M);
            double B = Matrix3.NormSquared(Matrix3.TransposeProduct(M, M));
            double C = Matrix3.Determinant(M);

            double f = mat3_f_trace_cg(A, B, C, out s);
            double denom = 4.0*f*f*f - 4.0*A*f - 8.0*C;

            if (Abs(denom)<1e-10)
            {
                R=Matrix3.Identity;
                S=M;
                return;
            }

            double dfdA = (2.0*f*f + 2.0*A) / denom;
            double dfdB = -2.0 / denom;
            double dfdC = (8.0*f) / denom;

            Matrix3 dAdM = 2.0 * M;
            Matrix3 dBdM = 4.0 * Matrix3.Product(M, Matrix3.TransposeProduct(M, M));
            Matrix3 dCdM = mat3_ddet(M);

            R=dfdA*dAdM+dfdB*dBdM+dfdC*dCdM;
            S=Matrix3.TransposeProduct(R, M);
        }

        static Matrix3 mat3_ddet(Matrix3 M)
        // Partial derivative of matrix determinant (adj(M)^T)
        {
            var da = Vector3.Cross(M.Column2, M.Column3);
            var db = Vector3.Cross(M.Column3, M.Column1);
            var dc = Vector3.Cross(M.Column1, M.Column2);
            return Matrix3.FromColumns(da, db, dc);
        }

        static double cubic_max_abs_root(double a, double b, double c)
        // returns the real root with the largest magnitude which solves the cubic 
        // equation of form x^3 + a*x^2 + b*x + c
        {
            double q = (a*a - 3.0*b) / 9.0;
            double r = (2.0*a*a*a - 9.0*a*b + 27.0*c) / 54.0;

            if (r*r<q*q*q)
            {
                // Three Real Roots
                double t = Acos( Numerics.Clamp(r / Sqrt(q*q*q), -1.0, 1.0));
                double x0 = -2.0 * Sqrt(q) * Cos((t             ) / 3.0) - a / 3.0;
                double x1 = -2.0 * Sqrt(q) * Cos((t + 2.0 * PI) / 3.0) - a / 3.0;
                double x2 = -2.0 * Sqrt(q) * Cos((t - 2.0 * PI) / 3.0) - a / 3.0;
                return Abs(x0)>Abs(x1)&&Abs(x0)>Abs(x2) ? x0 :
                       Abs(x1)>Abs(x2)&&Abs(x1)>Abs(x0) ? x1 : x2;
            }
            else
            {
                // One Real Root
                double e = Pow(Sqrt(r*r - q*q*q) + Abs(r), 1.0 / 3.0);
                e=r>0.0 ? -e : e;
                double f = e == 0.0 ? 0.0 : q / e;
                return ( e+f )-a/3.0;
            }
        }

        static double mat3_f_trace_cg(double A, double B, double C, out Vector3 s)
        // Computes tr(M^T R) and the singular values from A, B and C
        {
            // Compute polynomial coefficients
            double b = -2.0 * A;
            double c = -8.0 * C;
            double d = -A*A + 2.0 * B;

            // Find root with largest magnitude using cubic resolvent coefficients 
            double y = cubic_max_abs_root(-b, -4.0*d, -c*c + 4.0*b*d);

            // Find quadratics for each pair of quartic roots
            double q1, p1, q2, p2;

            double D = y*y - 4.0*d;
            if (D<1e-10)
            {
                double D2 = Max(-4.0 * (b - y), 0.0);
                q1=q2=y*0.5;
                p1=+Sqrt(D2)*0.5;
                p2=-Sqrt(D2)*0.5;
            }
            else
            {
                q1=( y+Sqrt(D) )*0.5;
                q2=( y-Sqrt(D) )*0.5;
                p1=( -c )/( q1-q2 );
                p2=( +c )/( q1-q2 );
            }

            // Find first two roots
            double D01 = Max(p1*p1 - 4.0*q1, 0.0);
            double x0 = (-p1 + Sqrt(D01)) * 0.5;
            double x1 = (-p1 - Sqrt(D01)) * 0.5;

            // Find second two roots
            double D23 = Max(p2*p2 - 4.0*q2, 0.0);
            double x2 = (-p2 - Sqrt(D23)) * 0.5;
            double x3 = (-p2 + Sqrt(D23)) * 0.5;

            // Singular Values
            s=new Vector3(
                ( x0+x3 )*0.5,
                ( x1+x3 )*0.5,
                ( x2+x3 )*0.5);

            // return trace root
            return x3;
        }
        #endregion

    }
}