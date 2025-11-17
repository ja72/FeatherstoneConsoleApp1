using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

using static System.Math;

namespace JA.LinearAlgebra
{
    public static class NativeArrays
    {
        #region Double Vectors
        public static double MaxElement(this double[] vector, out int index)
        {
            if(vector==null||vector.Length==0)
            {
                index=-1;
                return 0;
            }
            double max=vector[0];
            index=0;
            for(int i=1; i<vector.Length; i++)
            {
                double x_val = vector[i];
                if (x_val>max)
                {
                    max=x_val;
                    index=i;
                }
            }
            return max;
        }
        public static double MinElement(this double[] vector, out int index)
        {
            if(vector==null||vector.Length==0)
            {
                index=-1;
                return 0;
            }
            double min=vector[0];
            index=0;
            for(int i=1; i<vector.Length; i++)
            {
                double x_val = vector[i];
                if (x_val<min)
                {
                    min=x_val;
                    index=i;
                }
            }
            return min;
        }
        public static double MaxAbsElement(this double[] vector, out int index)
        {
            if(vector==null||vector.Length==0)
            {
                index=-1;
                return 0;
            }
            double max=Abs(vector[0]);
            index=0;
            for(int i=1; i<vector.Length; i++)
            {
                double x_abs = Abs(vector[i]);
                if (x_abs>max)
                {
                    max=x_abs;
                    index=i;
                }
            }
            return max;
        }
        public static double VectorElementSum(this double[] vector)
        {
            if(vector==null||vector.Length==0) return 0;
            double result=vector[0];
            for(int i=1; i<vector.Length; i++)
            {
                result+=vector[i];
            }
            return result;
        }
        public static double VectorElementProduct(this double[] vector)
        {
            if(vector==null||vector.Length==0) return 1;
            double result=vector[0];
            for(int i=1; i<vector.Length; i++)
            {
                result*=vector[i];
            }
            return result;
        }
        public static double VectorNorm1(this double[] vector)
        {
            double sum=0;
            for(int i=0; i<vector.Length; i++)
            {
                sum+=Abs(vector[i]);
            }
            return sum;
        }
        public static double VectorNorm2(this double[] vector)
        {
            return Sqrt(VectorDot(vector, vector));
        }
        public static double VectorNormP(this double[] vector, double p)
        {
            if(p==0) return 1;
            if(p==1) return VectorNorm1(vector);
            if(p==2) return VectorNorm2(vector);

            double sum=0;
            for(int i=0; i<vector.Length; i++)
            {
                sum+=Pow(Abs(vector[i]),p);
            }
            return Pow( sum,1/p);
        }
        public static double VectorNormInf(this double[] vector)
        {
            double max=0;
            for(int i=0; i<vector.Length; i++)
            {
                max=Max(max, Abs(vector[i]));
            }
            return max;
        }

        public static void VectrorScale(this double[] vector, double scale, ref double[] result)
        {
            if(vector.Length!=result.Length)
            {
                throw new Exception("Non-conformable vectors in VectorScale");
            }
            for(int i=0; i<vector.Length; i++)
            {
                result[i]=scale*vector[i];
            }
        }
        public static double[] VectrorScale(this double[] vector, double scale)
        {
            var result=new double[vector.Length];
            VectrorScale(vector, scale, ref result);
            return result;
        }

        public static void VectorAdd(this double[] vector, double[] other, ref double[] result, double other_factor=1)
        {
            if(vector.Length!=other.Length
                || vector.Length!=result.Length)
            {
                throw new ArgumentException("Non-conformable matrices in VectorAddition","other");
            }
            for(int i=0; i<result.Length; i++)
            {
                result[i]=vector[i]+other_factor*other[i];
            }
        }
        public static double[] VectorAdd(this double[] vector, double[] other, double other_factor=1)
        {
            var result=new double[vector.Length];
            VectorAdd(vector, other, ref result, other_factor);
            return result;
        }
        public static double[] VectorMultiply(this double[] vector, double[] other)
        {
            var result=new double[vector.Length];
            VectorMultiply(vector, other, ref result);
            return result;
        }

        public static void VectorMultiply(this double[] vector, double[] other, ref double[] result)
        {
            if(vector.Length!=other.Length||vector.Length!=result.Length)
            {
                throw new Exception("Non-conformable vectors in VectorAddition");
            }
            for(int i=0; i<result.Length; i++)
            {
                result[i]=vector[i]*other[i];
            }
        }
        public static double[] VectorDivide(this double[] vector, double[] other)
        {
            var result=new double[vector.Length];
            VectorDivide(vector, other, ref result);
            return result;
        }
        public static void VectorDivide(this double[] vector, double[] other, ref double[] result)
        {
            if(vector.Length!=other.Length||vector.Length!=result.Length)
            {
                throw new Exception("Non-conformable vectors in VectorAddition");
            }
            for(int i=0; i<result.Length; i++)
            {
                result[i]=vector[i]/other[i];
            }
        }

        public static double VectorDot(this double[] vector, double[] other)
        {
            var result=0.0;
            if(vector.Length!=other.Length)
            {
                throw new IndexOutOfRangeException("Vectors must be the same size for inner product.");
            }
            for(int i=0; i<vector.Length; i++)
            {
                result+=vector[i]*other[i];
            }
            return result;            
        }
        public static void VectorOuter(this double[] vector, double[] other, ref double[][] result)
        {
            if(vector.Length!=result.Length || other.Length!=result[0].Length)
            {
                throw new IndexOutOfRangeException("Non conformal size in outer product result.");
            }
            for(int i=0; i<result.Length; i++)
            {
                var row=new double[other.Length];
                double vi=vector[i];
                for(int j=0; j<row.Length; j++)
                {
                    row[j]=vi*other[j];
                }
                result[i]=row;
            }
        }
        public static void VectorOuter(this double[] vector, double[] other, ref double[,] result)
        {
            int n = result.GetLength(0);
            int m = result.GetLength(1);

            if (vector.Length!=n || other.Length!=m)
            {
                throw new IndexOutOfRangeException("Non conformal size in outer product result.");
            }
            for(int i=0; i<n; i++)
            {
                var row=new double[other.Length];
                double vi=vector[i];
                for(int j=0; j<row.Length; j++)
                {
                    row[j]=vi*other[j];
                }
                result.SetRow(i, row);
            }
        }
        public static double[][] VectorOuterJagged(this double[] vector, double[] other)
        {            
            var result=new double[vector.Length][];
            VectorOuter(vector, other, ref result);
            return result;
        }
        public static double[,] VectorOuterArray2(this double[] vector, double[] other)
        {
            var result=new double[vector.Length, other.Length ];
            VectorOuter(vector, other, ref result);
            return result;
        }
        public static bool VectorEquals(this double[] vector, double[] other)
        {
            return VectorEquals(vector, other, 1e-11);
        }
        public static bool VectorEquals(this double[] vector, double[] other, double epsilon)
        {
            // true if all values in A == corresponding values in B
            if(vector.Length!=other.Length) return false;
            for(int i=0; i<vector.Length; ++i) // each row of A and B
                if(Abs(vector[i]-other[i])>epsilon)
                    return false;
            return true;
        }

        #endregion

        #region Double Array2 Matrices
        /// <summary>
        /// Matrix-Vector product y=A*x
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[] MatrixProduct(this double[,] matrix, double[] other)
        {
            if(matrix.Length==0) return new double[0];
            int aRows=matrix.GetLength(0); int aCols=matrix.GetLength(1);
            int bRows=other.Length;
            if(aCols!=bRows)
                throw new Exception("Non-conformable matrices in MatrixProduct");
            double[] result=new double[aRows];
            for(int i=0; i<aRows; ++i) // each row of A
                for(int k=0; k<aCols; ++k)
                    result[i]+=matrix[i,k]*other[k];
            return result;
        }
        /// <summary>
        /// Matrix-Matrix product Y=A*X
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[,] MatrixProduct(this double[,] matrix, double[,] other)
        {
            if(matrix.Length==0) { return new double[0,0]; }
            int aRows=matrix.GetLength(1); int aCols=matrix.GetLength(1);
            int bRows=other.GetLength(0);  int bCols=other.GetLength(1);
            if(aCols!=bRows)
                throw new Exception("Non-conformable matrices in MatrixProduct");
            var result=Factory.CreateArray2(aRows, bCols);
            for(int i=0; i<aRows; ++i) // each row of A
                for(int j=0; j<bCols; ++j) // each col of B
                    for(int k=0; k<aCols; ++k)
                        result[i,j]+=matrix[i,k]*other[k,j];
            return result;
        }

        #endregion

        #region Double Jagged Matrices
        public static double MaxAbsElement(this double[][] matrix, out int row, out int col)
        {
            double max=0;
            row=-1;
            col=-1;
            if(matrix==null||matrix.Length==0)
            {
                return 0;
            }
            for(int i=0; i<matrix.Length; i++)
            {
                var m_row = matrix[i];
                double x_abs = m_row.MaxAbsElement(out int temp_index);
                if (x_abs>max)
                {
                    max=x_abs;
                    row=i;
                    col=temp_index;
                }
            }
            return max;
        }
        static bool JaggedTranspose(this double[][] matrix, ref double[][] t_matrix)
        {
            int n=matrix.Length;
            int m=matrix[0].Length;
            if(t_matrix.Length!=m) return false;
            for(int i=0; i<m; i++)
            {
                var row=new double[n];
                for(int j=0; j<n; j++)
                {
                    row[j]=matrix[j][i];
                }
                t_matrix[i]=row;
            }
            return true;
        }
        public static double[][] JaggedTranspose(this double[][] matrix)
        {
            int n=matrix.Length;
            int m=matrix[0].Length;
            double[][] t_matrix = Factory.CreateJaggedArray(m,n);
            JaggedTranspose(matrix,ref t_matrix);
            return t_matrix;
        }

        public static void MatrixScale(this double[][] matrix, double factor, ref double[][] result)
        {
            for(int i=0; i<matrix.Length; i++)
            {
                matrix[i].VectrorScale(factor, ref result[i]);
            }
        }
        public static double[][] MatrixScale(this double[][] matrix, double factor)
        {
            var result=new double[matrix.Length][];
            MatrixScale(matrix, factor, ref result);
            return result;
        }

        public static void MatrixAdd(this double[][] matrix, double[][] other, ref double[][] result, double other_factor=1)
        {
            for(int i=0; i<result.Length; i++)
            {
                result[i].VectorAdd(other[i], other_factor);
            }
        }
        public static double[][] MatrixAdd(this double[][] matrix, double[][] other, double other_factor=1)
        {
            var result=Factory.CreateJaggedArray(matrix.Length, matrix[0].Length);
            MatrixAdd(matrix, other, ref result, other_factor);
            return result;
        }
        /// <summary>
        /// Element by element multiplication of two Matrices
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <param name="result"></param>
        public static void MatrixMultiply(this double[][] matrix, double[][] other, ref double[][] result)
        {
            for(int i=0; i<result.Length; i++)
            {
                result[i].VectorMultiply(other[i]);
            }
        }
        /// <summary>
        /// Element by element multiplication of two Matrices
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[][] MatrixMultiply(this double[][] matrix, double[][] other)
        {
            var result=new double[matrix.Length][];
            MatrixMultiply(matrix, other, ref result);
            return result;
        }
        public static double MatrixNorm2(this double[][] matrix)
        {
            double sum=0;
            for(int i=0; i<matrix.Length; i++)
            {
                double[] row=matrix[i];
                for(int j=0; j<row.Length; j++)
                {
                    sum+=row[j]*row[j]/(row.Length*row.Length);
                }
            }
            return Sqrt(sum)/matrix.Length;
        }
        public static double[][] CopyArray2(this double[][] matrix)
        {
            int rows=matrix.Length;
            int cols=matrix[0].Length;
            var result=Factory.CreateJaggedArray(rows, cols);
            for(int i=0; i<rows; i++)
            {
                Array.Copy(matrix[i], result[i], cols);
            }
            return result;
        }
        public static bool MatrixEquals(this double[][] matrixA, double[][] matrixB)
        {
            return MatrixEquals(matrixA, matrixB, 1e-11);
        }
        public static bool MatrixEquals(this double[][] matrixA, double[][] matrixB, double epsilon)
        {
            // true if all values in A == corresponding values in B
            int aRows=matrixA.Length; int aCols=matrixA[0].Length;
            int bRows=matrixB.Length; int bCols=matrixB[0].Length;
            if(aRows!=bRows||aCols!=bCols) return false;
            for(int i=0; i<aRows; ++i) // each row of A and B
                for(int j=0; j<bCols; ++j) // each col of A and B
                    if(Abs(matrixA[i][j]-matrixB[i][j])>epsilon)
                        return false;
            return true;
        }
        /// <summary>
        /// Matrix-Vector product y=A*x
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[] MatrixProduct(this double[][] matrix, double[] other)
        {
            if(matrix.Length==0) return new double[0];
            int aRows=matrix.Length; int aCols=matrix[0].Length;
            int bRows=other.Length;
            if(aCols!=bRows)
                throw new Exception("Non-conformable matrices in MatrixProduct");
            double[] result=new double[aRows];
            for(int i=0; i<aRows; ++i) // each row of A
                for(int k=0; k<aCols; ++k)
                    result[i]+=matrix[i][k]*other[k];
            return result;
        }
        /// <summary>
        /// Matrix-Matrix product Y=A*X
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[][] MatrixProduct(this double[][] matrix, double[][] other)
        {
            if(matrix.Length==0) { return new double[0][]; }
            int aRows=matrix.Length; int aCols=matrix[0].Length;
            int bRows=other.Length; int bCols=other[0].Length;
            if(aCols!=bRows)
                throw new Exception("Non-conformable matrices in MatrixProduct");
            var result=Factory.CreateJaggedArray(aRows, bCols);
            for(int i=0; i<aRows; ++i) // each row of A
                for(int j=0; j<bCols; ++j) // each col of B
                    for(int k=0; k<aCols; ++k)
                        result[i][j]+=matrix[i][k]*other[k][j];
            return result;
        }

        #endregion

    }
}
