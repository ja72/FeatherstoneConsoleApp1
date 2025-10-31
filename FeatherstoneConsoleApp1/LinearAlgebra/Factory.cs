using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;

namespace JA
{

    public static class Factory
    {
        static readonly Random rnd=new Random();

        #region Generic Array
        public static T[] CreateArray<T>(int size)
        {
            return new T[size];
        }
        public static T[] CreateArray<T>(int size, params T[] elements)
        {
            var vector=new T[size];
            Array.Copy(elements, 0, vector, 0, Math.Min(vector.Length, elements.Length));
            return vector;
        }
        public static T[] CreateArray<T>(int size, Func<T> init)
        {
            var vector=new T[size];
            for (int i = 0; i<vector.Length; i++)
            {
                vector[i]=init();
            }
            return vector;
        }
        public static T[] CreateArray<T>(int size, Func<int, T> init)
        {
            var vector=new T[size];
            for (int i = 0; i<vector.Length; i++)
            {
                vector[i]=init(i);
            }
            return vector;
        }

        #endregion

        #region Generic Array2
        public static T[][] CreateArray2<T>(int rows, int cols)
        {
            var matrix=new T[rows][];
            for (int i = 0; i<rows; i++)
            {
                matrix[i]=new T[cols];
            }
            return matrix;
        }
        public static T[][] CreateArray2<T>(int rows, int cols, params T[] elements)
        {
            var matrix=new T[rows][];
            for (int i = 0, k = 0; i<rows; i++, k+=cols)
            {
                var row =new T[cols];
                Array.Copy(elements, k, row, 0, cols);
                matrix[i]=row;
            }
            return matrix;
        }
        public static T[][] CreateArray2<T>(int rows, int cols, Func<T> init)
        {
            var matrix=new T[rows][];
            for (int i = 0, k = 0; i<rows; i++, k+=cols)
            {
                var row=new T[cols];
                for (int j = 0; j<cols; j++)
                {
                    row[j]=init();
                }
                matrix[i]=row;
            }
            return matrix;
        }
        public static T[][] CreateArray2<T>(int rows, int cols, Func<int, int, T> init)
        {
            var matrix=new T[rows][];
            for (int i = 0, k = 0; i<rows; i++, k+=cols)
            {
                var row=new T[cols];
                for (int j = 0; j<cols; j++)
                {
                    row[j]=init(i, j);
                }
                matrix[i]=row;
            }
            return matrix;
        }

        #endregion

        #region Vector
        public static double[] CreateVector(int size)
        {
            return new double[size];
        }
        public static double[] CreateVector(int size, params double[] elements)
        {
            var vector=new double[size];
            Array.Copy(elements, 0, vector, 0, Math.Min(vector.Length, elements.Length));
            return vector;
        }
        public static double[] CreateVector(int size, Func<int, double> init)
        {
            var vector=new double[size];
            for (int i = 0; i<vector.Length; i++)
            {
                vector[i]=init(i);
            }
            return vector;
        }

        public static double[] CreateElementVector(int size, int unit_element_index)
        {
            var vector=new double[size];
            vector[unit_element_index]=1;
            return vector;
        }
        public static double[] CreateRandomVector(int size, double min_value = 0, double max_value = 1)
        {
            var vector=new double[size];
            for (int i = 0; i<vector.Length; i++)
            {
                vector[i]=min_value+( max_value-min_value )*rnd.NextDouble();
            }
            return vector;
        }

        /// <summary>
        /// Fill in array with a linear number series.
        /// </summary>
        /// <param name="list">The array to fill</param>
        /// <param name="first_value">The first element value</param>
        /// <param name="step_value">The step in value for each element (opt)</param>
        public static void FillSeries(this double[] list, double first_value, double step_value)
        {
            int count = list.Length;
            if (count==1)
            {
                list[0]=first_value;
            }
            else
            {
                for (int i = 0; i<count; i++)
                {
                    list[i]=first_value+step_value*i;
                }
            }
        }


        /// <summary>
        /// Fill in array with a linear number series.
        /// </summary>
        /// <param name="count">The number of elements to fill</param>
        /// <param name="first_value">The first element value</param>
        /// <param name="step_value">The step in value for each element (opt)</param>
        public static double[] FillSeries(int count, double first_value, double step_value)
        {
            if(count == 0)
            {
                return Array.Empty<double>();
            }
            if (count<0)
            {
                throw new ArgumentOutOfRangeException(nameof(count), "Count must be non-negative.");
            }
            double[] list = new double[count];
            FillSeries(list, first_value, step_value);
            return list;
        }
        #endregion

        #region Matrix
        public static double[][] CreateMatrix(int rows, int cols)
        {
            var matrix=new double[rows][];
            for(int i=0; i<rows; i++)
            {
                matrix[i]=new double[cols];
            }
            return matrix;
        }
        public static double[][] CreateMatrix(int rows, int cols, params double[] elements)
        {
            var matrix=new double[rows][];
            int k=0;
            for(int i=0; i<rows; i++)
            {
                var row=new double[cols];
                Array.Copy(elements, k, row, 0, Math.Min(cols, elements.Length-k));
                k+=cols;
                matrix[i]=row;
            }
            return matrix;
        }
        public static double[][] CreateMatrix(int rows, int cols, Func<int, int, double> init)
        {
            var matrix=new double[rows][];
            for(int i=0; i<rows; i++)
            {
                var row=new double[cols];
                for(int j=0; j<row.Length; j++)
                {
                    row[j]=init(i, j);
                }
                matrix[i]=row;
            }
            return matrix;
        }

        public static double[][] CreateIdentityMatrix(int rows, int cols=0, double scale=1)
        {            
            if(cols==0) { cols=rows; }
            var matrix=new double[rows][];
            for(int i=0; i<rows; i++)
            {
                var row=new double[cols];
                if(i<cols)
                {
                    row[i]=scale;
                }
                matrix[i]=row;
            }
            return matrix;
        }
        public static double[][] CreateDiagonalMatrix(double[] diag_values)
        {
            var matrix=new double[diag_values.Length][];
            for(int i=0; i<matrix.Length; i++)
            {
                var row=new double[diag_values.Length];
                row[i]=diag_values[i];
                matrix[i]=row;
            }
            return matrix;
        }

        public static double[][] CreateRandomMatrix(int rows, int cols, double min_value=0, double max_value=1)
        {
            var matrix=new double[rows][];
            for(int i=0; i<matrix.Length; i++)
            {
                matrix[i]=CreateRandomVector(cols, min_value, max_value);
            }
            return matrix;
        }
        #endregion

        #region Conversions
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
    }
}
