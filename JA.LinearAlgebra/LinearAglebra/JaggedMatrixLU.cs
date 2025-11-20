using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;

using static System.Math;

namespace JA.LinearAlgebra
{

    /// <summary>
    /// Code taken from http://msdn.microsoft.com/en-us/magazine/jj863137.aspx
    /// </summary>
    public static class JaggedMatrixLU
    {
        public const double ZERO_TOL = 1e-12;

        static double[][] CopyJaggedMatrix(this double[][] matrix)
        {
            int rows = matrix.Length;
            int cols = rows > 0 ?  matrix[0].Length : 0;
            double[][] result = new double[rows][];
            for (int i = 0; i<rows; i++)
            {
                result[i]=new double[cols];
                Array.Copy(matrix[i], result[i], cols);
            }
            return result;
        }

        public static bool MatrixDecompose(this double[][] matrix, out double[][] lu, out int[] perm, out int toggle)
        {
            double tol = ZERO_TOL;
            // Doolittle LUP decomposition.
            // assumes matrix is square.
            int n = matrix.Length; // convenience
            lu = CopyJaggedMatrix(matrix);
            perm=Enumerable.Range(0, n).ToArray();
            toggle=1;
            for (int j = 0; j<n-1; ++j) // each column
            {
                double colMax = Abs(lu[j][j]); // largest val col j
                int pRow = j;
                for (int i = j+1; i<n; ++i)
                {
                    var row_i = lu[i];
                    if (row_i[j]>colMax)
                    {
                        colMax=row_i[j];
                        pRow=i;
                    }
                }
                if (pRow!=j) // swap rows
                {
                    //Swap(ref lu[pRow], ref lu[j]);
                    (lu[j], lu[pRow])=(lu[pRow], lu[j]);

                    //Swap(ref perm[pRow], ref perm[j]);
                    (perm[j], perm[pRow])=(perm[pRow], perm[j]);

                    toggle=-toggle; // row-swap toggle
                }
                var row_j = lu[j];
                var lu_jj = row_j[j];

                if (Abs(lu[j][j])<=tol)
                {
                    return false; // consider a throw
                }
                for (int i = j+1; i<n; ++i)
                {
                    var row_i = lu[i];
                    var lu_ij = row_i[j];
                    lu_ij/=lu_jj;
                    row_i[j]=lu_ij;
                    for (int k = j+1; k<n; ++k)
                    {
                        row_i[k]-=lu_ij*row_j[k];
                    }
                    lu[i]=row_i;
                }
            } // maj column loop
            return true;
        }
        static bool HelperSolve(double[][] luMatrix, double[] b, out double[] x)
        {
            // solve luMatrix * x = b
            int n = luMatrix.Length;
            x=b.Clone() as double[];

            for (int i = 1; i<n; ++i)
            {
                double sum = x[i];
                var row = luMatrix[i];
                for (int j = 0; j<i; ++j)
                {
                    sum-=row[j]*x[j];
                }
                x[i]=sum;
            }
            var d = luMatrix[n - 1][n - 1];
            if (Abs(d)==0)
            {
                return false;
            }

            x[n-1]/=d;
            for (int i = n-2; i>=0; --i)
            {
                var row = luMatrix[i];
                d=row[i];
                if (Abs(d)==0)
                {
                    return false;
                }

                double sum = x[i];
                for (int j = i+1; j<n; ++j)
                {
                    sum-=row[j]*x[j];
                }
                x[i]=sum/d;
            }
            return true;
        }
        static bool HelperSolve(double[][] luMatrix, double[][] B, out double[][] X)
        {
            // solve luMatrix * X = B
            int n = luMatrix.Length;
            int m = B[0].Length;
            X=new double[B.Length][];
            for (int i = 0; i<B.Length; i++)
            {
                X[i]=B[i].Clone() as double[];
            }
            for (int k = 0; k<m; k++)
            {
                for (int i = 1; i<n; ++i)
                {
                    double sum = X[i][k];
                    for (int j = 0; j<i; ++j)
                    {
                        sum-=luMatrix[i][j]*X[j][k];
                    }
                    X[i][k]=sum;
                }
                var d = luMatrix[n - 1][n - 1];
                if (Abs(d)==0)
                {
                    return false;
                }

                X[n-1][k]/=d;
                for (int i = n-2; i>=0; --i)
                {
                    d=luMatrix[i][i];
                    if (Abs(d)==0)
                    {
                        return false;
                    }

                    double sum = X[i][k];
                    for (int j = i+1; j<n; ++j)
                    {
                        sum-=luMatrix[i][j]*X[j][k];
                    }
                    X[i][k]=sum/d;
                }
            }
            return true;
        }
        public static double[][] MatrixInverse(this double[][] matrix)
        {
            if (MatrixInverse(matrix, out double[][] inverse))
            {
                return inverse;
            }
            throw new ArgumentException("Singular Matrix", nameof(matrix));
        }
        public static bool MatrixInverse(this double[][] matrix, out double[][] inverse)
        {
            if (!MatrixDecompose(matrix, out double[][] lu, out int[] perm, out _))
            {
                inverse=CopyJaggedMatrix(matrix);
                return false;
            }
            return LuInverse(lu, perm, out inverse);
        }
        internal static bool LuInverse(this double[][] lu, int[] perm, out double[][] inverse)
        {
            int n = lu.Length;
            inverse=new double[n][];
            for (int j = 0; j<n; j++)
            {
                inverse[j]=new double[n];
            }
            double[] b = new double[n];
            for (int i = 0; i<n; ++i)
            {
                for (int j = 0; j<n; ++j)
                {
                    if (i==perm[j])
                    {
                        b[j]=1;
                    }
                    else
                    {
                        b[j]=0;
                    }
                }
                if (HelperSolve(lu, b, out double[] x))
                {
                    for (int j = 0; j<n; ++j)
                    {
                        inverse[j][i]=x[j];
                    }
                }
                else
                {
                    return false;
                }
            }
            return true;
        }
        public static double MatrixDeterminant(this double[][] matrix)
        {
            if (!MatrixDecompose(matrix, out double[][] lum, out _, out int toggle))
            {
                throw new ArgumentException("Unable to compute MatrixDeterminant", nameof(matrix));
            }
            return LuDeterminant(lum, toggle);
        }
        internal static double LuDeterminant(double[][] lu, int toggle)
        {
            double result = toggle;
            for (int i = 0; i<lu.Length; ++i)
            {
                result*=lu[i][i];
            }
            return result;
        }
        public static double[] SystemSolve(this double[][] matrix, double[] b, out double maxResidual)
        {
            if (SystemSolve(matrix, b, out double[] x, out maxResidual))
            {
                return x;
            }
            throw new ArgumentException("Singular Matrix", nameof(matrix));
        }
        public static bool SystemSolve(this double[][] matrix, double[] b, out double[] x, out double maxResidual)
        {
            if (MatrixDecompose(matrix, out double[][] lum, out int[] perm, out _))
            {
                var result = SystemSolve(lum, perm, b, out x);
                var residual = NativeArrays.ArrayAdd(matrix.JaggedProduct(x), b, -1.0);
                maxResidual=residual.FindMaxAbs(out _);
                return result;
            }
            x=Array.Empty<double>();
            maxResidual=double.NaN;
            return false;
        }
        internal static bool SystemSolve(this double[][] lu, int[] perm, double[] b, out double[] x)
        {
            double[] bp = new double[b.Length];
            for (int i = 0; i<lu.Length; ++i)
            {
                bp[i]=b[perm[i]];
            }
            if (HelperSolve(lu, bp, out x))
            {
                return true;
            }
            x=Array.Empty<double>();
            return false;
        }
        public static double[][] SystemSolve(this double[][] matrix, double[][] B, out double maxResidual)
        {
            if (SystemSolve(matrix, B, out double[][] x, out maxResidual))
            {
                return x;
            }
            throw new ArgumentException("Singular Matrix", nameof(matrix));
        }
        public static bool SystemSolve(this double[][] matrix, double[][] B, out double[][] X, out double maxResidual)
        {
            if (MatrixDecompose(matrix, out double[][] lum, out int[] perm, out _))
            {
                var result = SystemSolve(lum, perm, B, out X);
                var residual = NativeArrays.JaggedAdd(matrix.JaggedProduct(X), B, -1.0);
                maxResidual=residual.FindMaxAbs(out _, out _);
                return result;
            }
            X=Array.Empty<double[]>();
            maxResidual =double.NaN;
            return false;
        }
        internal static bool SystemSolve(this double[][] lum, int[] perm, double[][] B, out double[][] X)
        {
            double[][] Bp = new double[B.Length][];
            for (int i = 0; i<lum.Length; ++i)
            {
                Bp[i]=B[perm[i]];
            }
            if (!HelperSolve(lum, Bp, out X))
            {
                X=Array.Empty<double[]>();
                return false;
            }
            return true;
        }
        /// <summary>
        /// Solve system of y=A*x+b, but only some 'x' unknown.
        /// When 'x' is known, the is a corresponding 'y' which is not known
        /// </summary>
        /// <param name="A">The coefficient matrix</param>
        /// <param name="b">The constant matrix</param>
        /// <param name="x_known">The variable vector, with only known values. Returns the full vector</param>
        /// <param name="y_known">The diving vector with only known values. Return the full vector</param>
        /// <param name="known_x_index">A list of integer indeces for which 'x' is known and 'y' is unknown</param>
        public static bool SystemSolve(this double[][] A, double[] b, ref double[] x_known, ref double[] y_known, params int[] known_x_index)
        {
            int N=b.Length;
            int K=known_x_index.Length;

            //Create projection matrices
            // x_known = tr(P)*x        y_known = tr(R)*x
            // x_unknw = tr(R)*x        y_unknw = tr(P)*y
            double[][] P=Factory.CreateJaggedArray(N, K);
            double[][] R=Factory.CreateJaggedArray(N, N-K);
            bool[] known_x=new bool[N];
            // and Create full 'x' and 'y' arrays
            double[] x=new double[N];
            double[] y=new double[N];

            for (int k = 0; k<K; k++)
            {
                known_x[known_x_index[k]]=true;
            }
            for (int i = 0, k = 0, u = 0; i<N; i++)
            {
                if (known_x[i])
                {
                    if (x_known.Length<N) x[i]=x_known[k];
                    P[i][k++]=1;
                }
                else
                {
                    if (y_known.Length<N) y[i]=y_known[u];
                    R[i][u++]=1;
                }
            }
            if (K<N)
            {
                if (x_known.Length==N) x=x_known;
                if (y_known.Length==N) y=y_known;
                var PT=NativeArrays.JaggedTranspose(P);
                var RT=NativeArrays.JaggedTranspose(R);
                x_known=PT.JaggedProduct(x);
                y_known=RT.JaggedProduct(y);
                var Axb=A.JaggedProduct(x).ArrayAdd(b);
                var rhs=y_known.ArrayAdd(RT.JaggedProduct(Axb), -1);
                var J=RT.JaggedProduct(A.JaggedProduct(R));
                var xu=SystemSolve(J, rhs, out _);
                if (xu!=null)
                {
                    x_known=x.ArrayAdd(R.JaggedProduct(xu));
                    y_known=A.JaggedProduct(x_known).ArrayAdd(b);
                    return true;
                }
                return false;
            }
            else
            {
                y_known=A.JaggedProduct(x_known).ArrayAdd(b);
                return true;
            }
        }

        public static double[][] PermArrayToMatrix(int[] perm)
        {
            // Doolittle perm array to corresponding matrix
            int n = perm.Length;
            double[][] result = new double[n][];
            for (int i = 0; i<n; ++i)
            {
                result[i]=new double[n];
                for (int j = 0; j<n; j++)
                {
                    if (j==perm[i])
                    {
                        result[i][j]=1;
                    }
                    else
                    {
                        result[i][j]=0;
                    }
                }
            }
            return result;
        }
        public static double[][] UnPermute(double[][] luProduct, int[] perm)
        {
            double[][] result = CopyJaggedMatrix(luProduct);
            int[] unperm = new int[perm.Length];
            for (int i = 0; i<perm.Length; ++i)
            {
                unperm[perm[i]]=i; // create un-perm array
            }
            for (int r = 0; r<luProduct.Length; ++r) // each row
            {
                result[r]=luProduct[unperm[r]];
            }
            return result;
        }
        public static double[] VectorPermute(this double[] vector, int[] row_perm)
        {
            double[] result=new double[vector.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i]=vector[row_perm[i]];
            }
            return result;
        }

        public static bool MatrixTransposeAndPermute(this double[][] matrix, int[] row_perm, ref double[][] t_matrix)
        {
            int n=matrix.Length;
            int m=matrix[0].Length;
            if (t_matrix.Length!=m) return false;
            //double[][] trans=new double[m][];
            for (int i = 0; i<m; i++)
            {
                var row=new double[n];
                for (int j = 0; j<n; j++)
                {
                    row[j]=matrix[row_perm[j]][i];
                }
                t_matrix[i]=row;
            }
            return true;
        }

    }
}
