using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Reflection;
using System.Text;

namespace JA.LinearAlgebra
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
        public static T[,] CreateArray2<T>(int rows, int cols)
        {
            return new T[rows, cols];
        }
        public static T[,] CreateArray2<T>(int rows, int cols, T fill)
        {
            var result = new T[rows, cols];
            var elements = Enumerable.Repeat(fill, rows * cols).ToArray();
            Array.Copy(elements, result, elements.Length);
            return result;
        }

        public static T[,] CreateArray2<T>(int rows, int cols, params T[] elements)
        {
            var result=new T[rows, cols];
            Array.Copy(elements, result, elements.Length);
            return result;
        }
        public static T[,] CreateArray2<T>(int rows, int cols, Func<int, int, T> init)
        {
            var result=new T[rows, cols];
            for (int i = 0, k = 0; i<rows; i++, k+=cols)
            {
                for (int j = 0; j<cols; j++)
                {
                    result[i, j]=init(i,j);
                }
            }
            return result;
        }

        #endregion

        #region Generic Jagged Matrix
        public static T[][] CreateJaggedArray<T>(int rows, int cols)
        {
            var matrix=new T[rows][];
            for(int i=0; i<rows; i++)
            {
                matrix[i]=new T[cols];
            }
            return matrix;
        }
        public static T[][] CreateJaggedArray<T>(int rows, int cols, T fill)
        {
            var matrix=new T[rows][];
            for(int i=0; i<rows; i++)
            {
                var row = new T[cols];
                for (int j = 0; j<cols; j++)
                {
                    row[j]=fill;
                }
                matrix[i]=row;
            }
            return matrix;
        }
        public static T[][] CreateJaggedArray<T>(int rows, int cols, Func<int,int,T> init)
        {
            var matrix=new T[rows][];
            for(int i=0; i<rows; i++)
            {
                var row = new T[cols];
                for (int j = 0; j<cols; j++)
                {
                    row[j]=init(i,j);
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

        #region Jagged Double Matrix

        public static double[][] CreateJaggedArray(int rows, int cols)
        {
            var matrix=new double[rows][];
            for(int i=0; i<rows; i++)
            {
                matrix[i]=new double[cols];
            }
            return matrix;
        }
        public static double[][] CreateJaggedArray(int rows, int cols, params double[] elements)
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
        public static double[][] CreateJaggedArray(int rows, int cols, Func<int, int, double> init)
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

        public static double[][] CreateJaggedIdentityMatrix(int rows, int cols=0, double scale=1)
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
        public static double[][] CreateJaggedDiagonalMatrix(double[] diag_values)
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

        public static double[][] CreateJaggedRandomMatrix(int rows, int cols, double min_value=0, double max_value=1)
        {
            var matrix=new double[rows][];
            for(int i=0; i<matrix.Length; i++)
            {
                matrix[i]=CreateRandomVector(cols, min_value, max_value);
            }
            return matrix;
        }
        #endregion

        #region Array2 Matrix
        public static double[,] CopyArray2(this double[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            double[,] result = new double[rows, cols];
            Array.Copy(matrix, result, matrix.Length);
            return result;
        }

        public static double[,] CreateArray2(int rows, int cols)
        {
            var matrix=new double[rows, cols];
            return matrix;
        }
        public static double[,] CreateArray2(int rows, int cols, params double[] elements)
        {
            return elements.ToArray2(rows,cols);
        }
        public static double[,] CreateArray2(int rows, int cols, Func<int, int, double> init)
        {
            var matrix=new double[rows,cols];
            for(int i=0; i<rows; i++)
            {
                for(int j=0; j<cols; j++)
                {
                    matrix[i, j]=init(i, j);
                }
            }
            return matrix;
        }

        public static double[,] CreateIdentityArray2(int rows, int cols=0, double scale=1)
        {            
            if(cols==0) { cols=rows; }
            var matrix=new double[rows,cols];
            for(int i=0; i<rows; i++)
            {
                matrix[i, i]=1;
            }
            return matrix;
        }
        public static double[,] CreateDiagonalArray2(double[] diag_values)
        {
            var matrix=new double[diag_values.Length, diag_values.Length];
            for(int i=0; i<matrix.Length; i++)
            {
                matrix[i,i]=diag_values[i];
            }
            return matrix;
        }

        public static double[,] CreateRandomArray2(int rows, int cols, double min_value=0, double max_value=1)
        {
            var data = CreateRandomVector(rows*cols, min_value, max_value);
            return data.ToArray2(rows, cols);
        }
        #endregion

    }
}
