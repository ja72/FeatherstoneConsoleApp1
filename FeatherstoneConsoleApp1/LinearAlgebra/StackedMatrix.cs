using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
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

        public StackedMatrix(IReadOnlyList<int> rowPartitions, IReadOnlyList<int> colPartitions)
        {
            this.rowPartitions=rowPartitions??throw new ArgumentNullException(nameof(rowPartitions));
            this.colPartitions=colPartitions??throw new ArgumentNullException(nameof(colPartitions));

            this.rows=rowPartitions.Sum();
            this.cols=colPartitions.Sum();

            this.elements =new double[rows, cols];
        }
        public static StackedMatrix CompatibleWith(StackedVector rowVector, StackedVector colVector)
            => new StackedMatrix(rowVector.Partitions, colVector.Partitions);
        public static StackedMatrix CompatibleWith(StackedVector stackedVector)
            => new StackedMatrix(stackedVector.Partitions, stackedVector.Partitions);

        public IReadOnlyList<int> RowPartitions => rowPartitions;
        public IReadOnlyList<int> ColPartitions => colPartitions;
        public int Rows {get => elements.GetLength(0);}
        public int Columns {get => elements.GetLength(1);}
        public bool IsSquare {  get=> Rows == Columns; }
        public Matrix ToMatrix() => Matrix.FromArray2(elements);
        public int GetRowOffset(int rowPartitionIndex)
        {
            return  rowPartitions.Take(rowPartitionIndex).Sum();
        }
        public int GetColsOffset(int colPartitionIndex)
        {
            return  colPartitions.Take(colPartitionIndex).Sum();
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
                    result[i] = elements.GetRowSlice(rowOffset + i, colOffset, colSize);
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
                    elements.SetRowSlice(rowOffset + i, colOffset, value[i]);
                }
            }
        }
        public double[][] ToJaggedArray() => elements.ToJaggedArray();
        public double[,] ToArray2() => elements;

        public Matrix[,] ToMatrixArray()
        {
            Matrix[,] result = new Matrix[rowPartitions.Count, colPartitions.Count];
            for (int i = 0; i<rows; i++)
            {
                for (int j = 0; j<cols; j++)
                {
                    result[i,j] = new Matrix( this[i,j] );
                }
            }
            return result;
        }
    }

}
