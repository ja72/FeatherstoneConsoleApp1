using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace JA.LinearAlgebra
{
    public class StackedMatrix
    {
        readonly double[,] elements;
        readonly IReadOnlyList<int> rowPartitions;
        readonly IReadOnlyList<int> colPartitions;
        readonly int rows;
        readonly int cols;

        #region Factory
        public StackedMatrix(IReadOnlyList<int> rowPartitions, IReadOnlyList<int> colPartitions)
        {
            this.rowPartitions=rowPartitions??throw new ArgumentNullException(nameof(rowPartitions));
            this.colPartitions=colPartitions??throw new ArgumentNullException(nameof(colPartitions));

            this.rows=rowPartitions.Sum();
            this.cols=colPartitions.Sum();

            this.elements=new double[rows, cols];
        }
        public StackedMatrix(IReadOnlyList<int> rowPartitions, IReadOnlyList<int> colPartitions, double[][] values)
            : this(rowPartitions, colPartitions, values.ToArray2() ) { }
        public StackedMatrix(IReadOnlyList<int> rowPartitions, IReadOnlyList<int> colPartitions, double[,] values)
        {
            this.rowPartitions=rowPartitions??throw new ArgumentNullException(nameof(rowPartitions));
            this.colPartitions=colPartitions??throw new ArgumentNullException(nameof(colPartitions));

            this.rows=rowPartitions.Sum();
            this.cols=colPartitions.Sum();

            int n = values.GetLength(0);
            int m = values.GetLength(1);

            if (n==rows&&m==cols)
            {
                this.elements=values;
            }
            else
            {
                this.elements=new double[rows, cols];
                Array.Copy(values, 0, elements, 0, values.Length);
            }
        }
        public static StackedMatrix FromSizeAndCountSquare(int size, int count)
        {
            var partitions = Enumerable.Repeat(size, count).ToArray();
            return new StackedMatrix(partitions, partitions);
        }
        public static StackedMatrix FromSizeAndCount(int rowSize, int rowCount, int colSize , int colCount)
        {
            var row_part = Enumerable.Repeat(rowSize, rowCount).ToArray();
            var col_part = Enumerable.Repeat(colSize, colCount).ToArray();
            return new StackedMatrix(row_part, col_part);
        }
        public static StackedMatrix CompatibleWith(StackedVector rowVector, StackedVector colVector)
            => new StackedMatrix(rowVector.Partitions, colVector.Partitions);
        public static StackedMatrix CompatibleWith(StackedVector stackedVector)
            => new StackedMatrix(stackedVector.Partitions, stackedVector.Partitions);
        #endregion

        #region Properties

        public double[,] Elements => elements;
        public IReadOnlyList<int> RowPartitions => rowPartitions;
        public IReadOnlyList<int> ColPartitions => colPartitions;
        public int Rows { get => elements.GetLength(0); }
        public int Columns { get => elements.GetLength(1); }
        public bool IsSquare { get => Rows==Columns; }
        public Matrix ToMatrix() => Matrix.FromArray2(elements);
        public int GetRowOffset(int rowPartitionIndex)
        {
            return rowPartitions.Take(rowPartitionIndex).Sum();
        }
        public int GetColsOffset(int colPartitionIndex)
        {
            return colPartitions.Take(colPartitionIndex).Sum();
        }

        public double[][] this[int rowPartitionIndex, int colPartitionIndex]
        {
            get
            {
                int rowOffset = GetRowOffset(rowPartitionIndex);
                int colOffset = GetColsOffset(colPartitionIndex);
                int rowSize = rowPartitions[rowPartitionIndex];
                int colSize = colPartitions[colPartitionIndex];

                double[][] result = new double[rowSize][];
                for (int i = 0; i<rowSize; i++)
                {
                    result[i]=elements.GetRowSlice(rowOffset+i, colOffset, colSize);
                }

                return result;
            }
            set
            {
                int rowOffset = GetRowOffset(rowPartitionIndex);
                int colOffset = GetColsOffset(colPartitionIndex);
                int rowSize = rowPartitions[rowPartitionIndex];
                int colSize = colPartitions[colPartitionIndex];

                for (int i = 0; i<rowSize; i++)
                {
                    elements.SetRowSlice(rowOffset+i, colOffset, value[i]);
                }
            }
        }

        public bool IsCompatibleWithRows(StackedVector other) => Enumerable.SequenceEqual(rowPartitions, other.Partitions);
        public bool IsCompatibleWithCols(StackedVector other) => Enumerable.SequenceEqual(colPartitions, other.Partitions);
        #endregion

        #region Algebra
        public static StackedVector Product(StackedMatrix A, StackedVector B)
        {
            if (!A.IsCompatibleWithCols(B))
            {
                throw new ArgumentException("Incompatible partitions.:", nameof(B));
            }
            var values = NativeArrays.Array2Product(A.elements, B.Elements);
            return new StackedVector(A.rowPartitions, values);
        }
        public StackedVector Solve(StackedVector B, out double maxResidual)
        {
            if (!IsCompatibleWithRows(B))
            {
                throw new ArgumentException("Incompatible partitions.:", nameof(B));
            }

            var values = JaggedMatrixLU.SystemSolve(ToJaggedArray(), B.Elements, out maxResidual);
            return new StackedVector(colPartitions, values);
        }
        public StackedMatrix Inverse()
        {
            var values = JaggedMatrixLU.MatrixInverse(ToJaggedArray());
            return new StackedMatrix(colPartitions, rowPartitions, values);
        }
        #endregion

        #region Operators
        public static StackedVector operator *(StackedMatrix a, StackedVector b) => Product(a, b);
        public static StackedVector operator /(StackedVector a, StackedMatrix b) => b.Solve(a, out _);
        #endregion

        #region Conversions
        public double[][] ToJaggedArray() => elements.ToJaggedArray();
        public double[,] ToArray2() => elements;

        public Matrix[,] ToMatrixArray()
        {
            Matrix[,] result = new Matrix[rowPartitions.Count, colPartitions.Count];
            for (int i = 0; i<rows; i++)
            {
                for (int j = 0; j<cols; j++)
                {
                    result[i, j]=new Matrix(this[i, j]);
                }
            }
            return result;
        } 
        #endregion
    }

}
