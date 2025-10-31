using System;
using System.Collections.Generic;
using System.Text;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace JA.LinearAlgebra
{
    public class Matrix : IEquatable<Matrix>
    {
        readonly int rows, cols;
        readonly double[][] elements;

        #region Factory
        public Matrix(params double[][] elements)
        {
            this.elements=elements;
            this.rows=elements!=null?elements.Length:0;
            this.cols=rows>0?elements[0].Length:0;
        }
        public Matrix(Vector[] values, bool by_column = true)
        {
            if(!by_column)
            {
                this.rows=values.Length;
                this.cols=rows>0?values[0].Size:0;
                this.elements=new double[rows][];
                for(int i=0; i<elements.Length; i++)
                {
                    elements[i]=values[i].ToArray();
                }
            }
            else
            {
                this.cols=values.Length;
                this.rows=cols>0?values[0].Size:0;
                this.elements=new double[rows][];
                for(int i=0; i<elements.Length; i++)
                {
                    var row=new double[cols];
                    for(int j=0; j<row.Length; j++)
                    {
                        row[j]=values[j][i];
                    }
                    elements[i]=row;
                }
            }
        }
        public Matrix(Matrix other)
        {
            this.elements=other.elements;
            this.rows=elements!=null?elements.Length:0;
            this.cols=rows>0?elements[0].Length:0;
        }
        public Matrix(int rows, int columns, params double[] values)
        {
            this.rows = rows;
            this.cols = columns;
            if (rows==0)
            {
                this.elements=Array.Empty<double[]>();
                return;
            }
            this.elements = new double[rows][];

            if (values.Length<rows*cols)
            {
                throw new ArgumentException($"Expecting {rows*cols} values.", nameof(values));
            }

            this.elements=values.ToJaggedArray(rows, columns);
        }
        public static Matrix FromArray2(double[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int columns = matrix.GetLength(1);
            return new Matrix(rows, columns, matrix.ToArray());
        }
        public static explicit operator Matrix(double[][] elements)
        {
            return new Matrix(elements);
        }
        public static explicit operator double[][](Matrix matrix)
        {
            return matrix.elements;
        }
        public static explicit operator Matrix(double value)
        {
            return new Matrix(new double[][] { new double[] { value } });
        }
        public static explicit operator Matrix(Vector[] values)
        {
            return new Matrix(values);
        }
        public static explicit operator double(Matrix matrix)
        {
            if(matrix.cols==1&&matrix.rows==1)
            {
                return matrix[0, 0];
            }
            throw new IndexOutOfRangeException("Matrix must contain one element only for conversion to scalar. ");
        }
        #endregion

        #region Properties
        public double[][] Elements { get { return elements; } }
        public int Rows { get { return rows; } }
        public int Cols { get { return cols; } }
        public double[] this[int row] { get { return elements[row]; } set { elements[row]=value; } }
        public double this[int row, int col]
        {
            get { return elements[row][col]; }
            set { elements[row][col]=value; }
        }
        public double[][] ToJaggedArray() { return elements; }
        public double[,] ToArray2() { return elements.ToArray2(); }
        public override string ToString()
        {
            return $"[{string.Join("|", elements.Select( row=> string.Join(",", row) ) )}]";
        }
        #endregion

        #region Methods
        public double[] GetRow(int row_index)
        {
            return elements[row_index];
        }
        public void SetRow(int row_index, double[] row)
        {
            elements[row_index]=row;
        }
        public Matrix Transpose()
        {
            double[][] result = LinearAlgebra.MatrixTranspose(elements);
            return new Matrix(result);
        }
        #endregion

        #region System of Equations

        public Vector Solve(Vector b, out double maxResidual)
        {
            if(rows!=cols)
            {
                throw new InvalidOperationException("Matrix must be square to solve system of equations. ");
            }
            if(b.Size!=rows)
            {
                throw new InvalidOperationException("Right-hand side vector size must match matrix row count. ");
            }
            double[] x = JaggedMatrixLU.SystemSolve(elements, b.Elements, out maxResidual);
            return new Vector(x);
        }
        public Matrix Solve(Matrix B, out double maxResidual)
        {
            if(rows!=cols)
            {
                throw new InvalidOperationException("Matrix must be square to solve system of equations. ");
            }
            if(B.Rows!=rows)
            {
                throw new InvalidOperationException("Right-hand side vector size must match matrix row count. ");
            }
            double[][] X = JaggedMatrixLU.SystemSolve(elements, B.elements, out maxResidual);
            return new Matrix(X);
        }
        #endregion


        #region Operators
        public static Matrix operator ~(Matrix A)
        {
            return A.Transpose();
        }
        public static Matrix operator +(Matrix A, Matrix B)
        {
            return new Matrix(A.elements.MatrixAdd(B.elements, 1));
        }
        public static Matrix operator -(Matrix A)
        {
            return new Matrix(A.elements.MatrixScale(-1));
        }
        public static Matrix operator -(Matrix A, Matrix B)
        {
            return new Matrix(A.elements.MatrixAdd(B.elements, -1));
        }
        public static Matrix operator *(double factor, Matrix B)
        {
            return new Matrix(B.elements.MatrixScale(factor));
        }
        public static Matrix operator *(Matrix A, double factor)
        {
            return new Matrix(A.elements.MatrixScale(factor));
        }
        public static Matrix operator /(Matrix A, double divisor)
        {
            return new Matrix(A.elements.MatrixScale(1/divisor));
        }
        public static Vector operator *(Matrix A, Vector v)
        {
            return new Vector(A.elements.MatrixProduct(v.Elements));
        }
        public static Matrix operator *(Matrix A, Matrix B)
        {
            return new Matrix(A.elements.MatrixProduct(B.elements));
        }

        #endregion


        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Vector)</code></returns>
        public override bool Equals(object obj)
        {
            if(obj is Matrix)
            {
                return Equals((Matrix)obj);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among vectors
        /// </summary>
        /// <param name="other">The other vector to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Matrix other)
        {
            return elements.MatrixEquals(other.elements);
        }

        /// <summary>
        /// Calculates the hash code for the vector
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            return elements.Aggregate(17, (hash,val) => hash * 23 + val.GetHashCode());
        }

        #endregion
    }
}
