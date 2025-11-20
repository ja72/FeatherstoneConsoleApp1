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
        public const double ZERO_TOL = 1e-11;
        
        #region Array Copy and Slices
        public static double[] CopyArray(this double[] array)
        {
            return array.Clone() as double[];
        }
        public static double[,] CopyArray2(this double[,] array)
        {
            return array.Clone() as double[,];
        }
        public static double[][] CopyJagged(this double[][] matrix)
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

        public static T[] Slice<T>(this T[] array, int offset, int size)
        {
            T[] result = new T[size];
            Array.Copy(array, offset, result, 0, size);
            return result;
        }
        public static void Inject<T>(this T[] array, int offset, T[] values)
        {
            int size = Math.Min(values.Length, array.Length-offset);
            Array.Copy(values,0, array, offset, size);
        }

        public static double[] GetRow(this double[,] matrix, int row)
        {
            const int sz = sizeof(double);
            int cols = matrix.GetLength(1);
            double[] result = new double[cols];
            int offset = row*cols;
            Buffer.BlockCopy(matrix, offset*sz, result, 0, cols*sz);
            return result;
        }
        public static double[] GetRowSlice(this double[,] matrix, int row, int colOffset, int colSize)
        {
            const int sz = sizeof(double);
            int cols = matrix.GetLength(1);
            double[] result = new double[cols];
            int offset = row*cols + colOffset;
            Buffer.BlockCopy(matrix, offset*sz, result, 0, colSize*sz);
            return result;
        }
        public static void SetRow(this double[,] matrix, int row, double[] values)
        {
            const int sz = sizeof(double);
            int cols = matrix.GetLength(1);
            int offset = row*cols;
            cols=Math.Min(cols, values.Length);
            Buffer.BlockCopy(values,0, matrix, offset*sz, cols*sz);
        }
        public static void SetRowSlice(this double[,] matrix, int rowOffset, int colOffset, double[] values)
        {
            const int sz = sizeof(double);
            int cols = matrix.GetLength(1);
            int offset = rowOffset*cols + colOffset;
            cols=Math.Min(cols, values.Length);
            Buffer.BlockCopy(values,0, matrix, offset*sz, cols*sz);
        }
        public static double[] GetColumn(this double[,] matrix, int col)
        {
            int rows = matrix.GetLength(0);
            double[] result = new double[rows];
            for (int i = 0; i<rows; i++)
            {
                result[i]=matrix[i, col];
            }
            return result;
        }
        public static void SetColumn(this double[,] matrix, int col, double[] values)
        {
            int rows = Math.Min(matrix.GetLength(0), values.Length);
            for (int i = 0; i<rows; i++)
            {
                matrix[i, col] = values[i];
            }
        }
        #endregion

        #region Conversions
        public static double[] ToArray(this double[,] matrix)
        {
            double[] flat = new double[matrix.Length];
            Buffer.BlockCopy(matrix, 0, flat, 0, Buffer.ByteLength(matrix));
            return flat;
        }
        public static double[] ToArray(this double[][] jagged)
        {
            const int sz = sizeof(double);
            int rows = jagged.Length;
            int cols = rows>0 ? jagged[0].Length : 0;
            double[] flat = new double[rows*cols];
            int offset = 0;
            for (int i = 0; i<rows; i++)
            {
                Buffer.BlockCopy(jagged[i], 0, flat, offset*sz , Buffer.ByteLength(jagged[i]));
                offset+=cols;
            }
            return flat;
        }
        public static double[,] ToArray2(this double[] elements, int rows, int cols)
        {
            const int sz = sizeof(double);
            double[,] result = new double[rows, cols];
            int offset = 0;
            for (int i = 0; i<rows; i++)
            {
                Buffer.BlockCopy(
                    elements, offset*sz,
                    result, offset*sz,
                    cols*sz );
                offset+=cols;
            }
            return result;
        }
        public static double[,] ToArray2(this double[][] jagged)
        {
            const int sz = sizeof(double);
            int rows = jagged.Length;
            int cols = rows>0 ? jagged[0].Length : 0;
            double[,] result = new double[rows, cols];
            int offset = 0;
            for (int i = 0; i<rows; i++)
            {
                Buffer.BlockCopy(
                    jagged[i], 0,
                    result, offset*sz,
                    cols*sz );
                offset+=cols;
            }
            return result;
        }
        public static double[][] ToJaggedArray(this double[] elements, int rows, int cols)
        {
            const int sz = sizeof(double);
            double[][] result = new double[rows][];
            int offset = 0;
            for (int i = 0; i<rows; i++)
            {
                double[] temp = new double[cols];
                Buffer.BlockCopy(
                    elements, offset*sz,
                    temp, 0,
                    cols*sz );
                result[i]=temp;
                offset+=cols;
            }
            return result;
        }
        public static double[][] ToJaggedArray(this double[,] matrix)
        {
            const int sz = sizeof(double);
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            double[][] result = new double[rows][];
            int offset = 0;
            for (int i = 0; i<rows; i++)
            {
                double[] temp = new double[cols];
                Buffer.BlockCopy(
                    matrix, offset*sz,
                    temp, 0,
                    cols*sz );
                result[i]=temp;
                offset+=cols;
            }
            return result;
        }

        #endregion

        #region Transposes
        public static bool Array2Transpose<T>(this T[,] matrix, ref T[,] t_matrix)
        {
            int n=matrix.GetLength(0);
            int m=matrix.GetLength(1);
            int u=t_matrix.GetLength(0);
            int v=t_matrix.GetLength(1);
            if(u!=m || v!=n) return false;
            for(int i=0; i<m; i++)
            {
                for(int j=i; j<n; j++)
                {
                    t_matrix[i,j]=matrix[j, i];
                    if (j>i)
                    {
                        t_matrix[j, i]=matrix[i, j];
                    }
                }
            }
            return true;
        }
        public static T[,] Array2Transpose<T>(this T[,] matrix)
        {
            int n=matrix.GetLength(0);
            int m=matrix.GetLength(1);
            T[,] t_matrix = Factory.CreateArray2<T>(m, n);
            Array2Transpose(matrix,ref t_matrix);
            return t_matrix;
        }

        public static bool JaggedTranspose<T>(this T[][] matrix, ref T[][] t_matrix)
        {
            int n=matrix.Length;
            int m=matrix[0].Length;
            if(t_matrix.Length!=m) return false;
            for(int i=0; i<m; i++)
            {
                var row=new T[n];
                for(int j=0; j<n; j++)
                {
                    row[j]=matrix[j][i];
                }
                t_matrix[i]=row;
            }
            return true;
        }
        public static bool JaggedTranspose<T>(this T[][] matrix, ref T[][] t_matrix, T empty)
        {
            int n=matrix.Length;
            int m = matrix.Max(u=> u.Length);
            if (t_matrix.Length!=m)
            {
                return false;
            }
            for(int i=0; i<m; i++)
            {
                var row=new T[n];
                for(int j=0; j<n; j++)
                {
                    if (i<matrix[j].Length)
                    {
                        row[j]=matrix[j][i];
                    }
                    else
                    {
                        row[j]=empty;
                    }
                }
                t_matrix[i]=row;
            }
            return true;
        }

        public static T[][] JaggedTranspose<T>(this T[][] matrix)
        {
            int n=matrix.Length;
            int m=matrix[0].Length;
            T[][] t_matrix = Factory.CreateJaggedArray<T>(m,n);
            JaggedTranspose(matrix,ref t_matrix);
            return t_matrix;
        }
        #endregion

        #region Algebra
        public static void ArrayScale(this double[] vector, double scale, ref double[] result)
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
        public static double[] ArrayScale(this double[] vector, double scale)
        {
            var result=new double[vector.Length];
            ArrayScale(vector, scale, ref result);
            return result;
        }

        public static void ArrayAdd(this double[] vector, double[] other, ref double[] result, double other_factor=1)
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
        public static double[] ArrayAdd(this double[] vector, double[] other, double other_factor=1)
        {
            var result=new double[vector.Length];
            ArrayAdd(vector, other, ref result, other_factor);
            return result;
        }
        public static double[] ArrayMultiply(this double[] vector, double[] other)
        {
            var result=new double[vector.Length];
            ArrayMultiply(vector, other, ref result);
            return result;
        }

        public static void ArrayMultiply(this double[] vector, double[] other, ref double[] result)
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
        public static double[] ArrayDivide(this double[] vector, double[] other)
        {
            var result=new double[vector.Length];
            ArrayDivide(vector, other, ref result);
            return result;
        }
        public static void ArrayDivide(this double[] vector, double[] other, ref double[] result)
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

        public static double ArrayDot(this double[] vector, double[] other)
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
        public static void JaggedOuter(this double[] vector, double[] other, ref double[][] result)
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
        public static double[][] JaggedOuter(this double[] vector, double[] other)
        {            
            var result=new double[vector.Length][];
            JaggedOuter(vector, other, ref result);
            return result;
        }

        public static void Array2Outer(this double[] vector, double[] other, ref double[,] result)
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
        public static double[,] Array2Outer(this double[] vector, double[] other)
        {
            var result=new double[vector.Length, other.Length ];
            Array2Outer(vector, other, ref result);
            return result;
        }

        public static void JaggedScale(this double[][] matrix, double factor, ref double[][] result)
        {
            for(int i=0; i<matrix.Length; i++)
            {
                matrix[i].ArrayScale(factor, ref result[i]);
            }
        }
        public static double[][] JaggedScale(this double[][] matrix, double factor)
        {
            var result=new double[matrix.Length][];
            JaggedScale(matrix, factor, ref result);
            return result;
        }

        public static void JaggedAdd(this double[][] matrix, double[][] other, ref double[][] result, double other_factor=1)
        {
            for(int i=0; i<result.Length; i++)
            {
                result[i].ArrayAdd(other[i], other_factor);
            }
        }
        public static double[][] JaggedAdd(this double[][] matrix, double[][] other, double other_factor=1)
        {
            var result=Factory.CreateJaggedArray(matrix.Length, matrix[0].Length);
            JaggedAdd(matrix, other, ref result, other_factor);
            return result;
        }
        /// <summary>
        /// Element by element multiplication of two Matrices
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <param name="result"></param>
        public static void JaggedMultiply(this double[][] matrix, double[][] other, ref double[][] result)
        {
            for(int i=0; i<result.Length; i++)
            {
                result[i].ArrayMultiply(other[i]);
            }
        }
        /// <summary>
        /// Element by element multiplication of two Matrices
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[][] JaggedMultiply(this double[][] matrix, double[][] other)
        {
            var result=new double[matrix.Length][];
            JaggedMultiply(matrix, other, ref result);
            return result;
        }
        /// <summary>
        /// Matrix-Vector product y=A*x
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[] Array2Product(this double[,] matrix, double[] other)
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
        public static double[,] Array2Product(this double[,] matrix, double[,] other)
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
        /// <summary>
        /// Matrix-Vector product y=A*x
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static double[] JaggedProduct(this double[][] matrix, double[] other)
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
        public static double[][] JaggedProduct(this double[][] matrix, double[][] other)
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

        #region Statistics
        public static double FindMax(this double[] vector, out int index)
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
        public static double FindMin(this double[] vector, out int index)
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
        public static double FindMaxAbs(this double[] vector, out int index)
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
        public static double FindMaxAbs(this double[][] matrix, out int row, out int col)
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
                double x_abs = m_row.FindMaxAbs(out int temp_index);
                if (x_abs>max)
                {
                    max=x_abs;
                    row=i;
                    col=temp_index;
                }
            }
            return max;
        }
        public static double ArraySum(this double[] vector)
        {
            if(vector==null||vector.Length==0) return 0;
            double result=vector[0];
            for(int i=1; i<vector.Length; i++)
            {
                result+=vector[i];
            }
            return result;
        }
        public static double ArrayProduct(this double[] vector)
        {
            if(vector==null||vector.Length==0) return 1;
            double result=vector[0];
            for(int i=1; i<vector.Length; i++)
            {
                result*=vector[i];
            }
            return result;
        }
        public static double ArrayNorm1(this double[] vector)
        {
            double sum=0;
            for(int i=0; i<vector.Length; i++)
            {
                sum+=Abs(vector[i]);
            }
            return sum;
        }
        public static double ArrayNorm2(this double[] vector)
        {
            return Sqrt(ArrayDot(vector, vector));
        }
        public static double ArrayNormP(this double[] vector, double p)
        {
            if(p==0) return 1;
            if(p==1) return ArrayNorm1(vector);
            if(p==2) return ArrayNorm2(vector);

            double sum=0;
            for(int i=0; i<vector.Length; i++)
            {
                sum+=Pow(Abs(vector[i]),p);
            }
            return Pow( sum,1/p);
        }
        public static double ArrayNormInf(this double[] vector)
        {
            double max=0;
            for(int i=0; i<vector.Length; i++)
            {
                max=Max(max, Abs(vector[i]));
            }
            return max;
        }
        public static double ArrayNorm2(this double[][] matrix)
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

        #endregion

        #region Equality
        public static bool ArrayEquals<T>(this T[] vector, T[] other)
            where T: IEquatable<T>
        {
            if (vector.Length !=  other.Length) return false;
            for (int i = 0; i < vector.Length; i++)
            {
                if( !vector[i].Equals(other[i]))
                {
                    return false;
                }
            }
            return true;
        }
        public static bool ArrayEquals<T>(this T[] vector, T[] other, IEqualityComparer<T> comparer)
        {
            if (vector.Length !=  other.Length) return false;
            for (int i = 0; i < vector.Length; i++)
            {
                if( !comparer.Equals(vector[i], other[i]) )
                {
                    return false;
                }
            }
            return true;
        }
        public static bool ArrayEquals(this double[] vector, double[] other)
        {
            return ArrayEquals(vector, other, ZERO_TOL);
        }
        public static bool ArrayEquals(this double[] vector, double[] other, double epsilon)
        {
            // true if all values in A == corresponding values in B
            if(vector.Length!=other.Length) return false;
            for(int i=0; i<vector.Length; ++i) // each row of A and B
                if(Abs(vector[i]-other[i])>epsilon)
                    return false;
            return true;
        }

        public static bool JaggedEquals(this double[][] matrixA, double[][] matrixB)
        {
            return JaggedEquals(matrixA, matrixB, ZERO_TOL);
        }
        public static bool JaggedEquals(this double[][] matrixA, double[][] matrixB, double epsilon)
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
        #endregion
    }
}
