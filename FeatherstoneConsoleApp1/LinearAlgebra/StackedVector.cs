using System;
using System.Collections.Generic;
using System.Linq;

namespace JA.LinearAlgebra
{
    public class StackedVector
    {
        readonly double[] elements;
        readonly IReadOnlyList<int> partitions;
        readonly int size;

        public StackedVector(params int[] partition_sizes)
            : this( partition_sizes.ToList() ) { }
        public StackedVector(IReadOnlyList<int> partition_sizes)            
        {
            this.partitions=partition_sizes;      
            this.size=partition_sizes.Sum();
            this.elements=new double[size];
        }
        public StackedVector(double[] values, params int[] partition_sizes)
        {
            this.partitions=partition_sizes;
            int n = partition_sizes.Sum();
            if (n!=values.Length)
            {
                throw new ArgumentException("Size mismach", nameof(partition_sizes));
            }
            this.size=n;
            this.elements=values;
        }
        public StackedVector(params double[][] partitions)
        {
            int n = partitions.Length;
            double[][] values = new double[n][];
            int[] part = new int[n];
            for (int i = 0; i<n; i++)
            {
                values[i]=partitions[i];
                part[i]=values[i].Length;
            }
            this.size=n;
            this.partitions=part;
            this.elements=Factory.ToArray(values);
        }
        public StackedVector(params Vector[] partitions)
            : this( partitions.Select(v => v.Elements ).ToArray() )
        { }
        public static implicit operator Vector(StackedVector stacked) => stacked.ToVector();
        public IReadOnlyList<int> Partitions => partitions;
        public int Size { get => size; }
        public Vector ToVector() => new Vector(elements);
        public int GetOffset(int partitionIndex)
        {
            return  partitions.Take(partitionIndex).Sum();
        }
        public double[] this[int partitionIndex]
        {
            get
            {
                int offset=partitions.Take(partitionIndex).Sum();
                int size = partitions[partitionIndex];
                double[] result = new double[size];
                Array.Copy(elements, offset, result, 0, result.Length);
                return result;
            }
            set
            {
                int offset=partitions.Take(partitionIndex).Sum();
                int size = Math.Min(value.Length, partitions[partitionIndex]);
                Array.Copy(value, 0, elements, offset, size);
            }
        }
        public double[] ToArray() => elements;
        public Vector[] ToVectorArray()
        {
            Vector[] result = new Vector[partitions.Count];
            for (int i = 0; i<partitions.Count; i++)
            {
                result[i] = new Vector(this[i]);
            }
            return result;
        }

        #region Algebra
        public static StackedVector Add(StackedVector A, StackedVector B)
        {
            var result = new double[A.elements.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i]=A.elements[i]+B.elements[i];
            }
            return new StackedVector(result, A.partitions.ToArray());
        }
        public static StackedVector Subtract(StackedVector A, StackedVector B)
        {
            var result = new double[A.elements.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i]=A.elements[i]-B.elements[i];
            }
            return new StackedVector(result, A.partitions.ToArray());
        }
        public static StackedVector Negate(StackedVector A)
        {
            var result = new double[A.elements.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i]=-A.elements[i];
            }
            return new StackedVector(result, A.partitions.ToArray());
        }
        public static StackedVector Scale(double factor, StackedVector A)
        {
            var result = new double[A.elements.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i]=factor*A.elements[i];
            }
            return new StackedVector(result, A.partitions.ToArray());
        }
        #endregion

        #region Operators
        public static StackedVector operator +(StackedVector a) => a;
        public static StackedVector operator +(StackedVector a, StackedVector b) => Add(a, b);
        public static StackedVector operator -(StackedVector a) => Negate(a);
        public static StackedVector operator -(StackedVector a, StackedVector b) => Subtract(a, b);
        public static StackedVector operator *(double a, StackedVector b) => Scale(a, b);
        public static StackedVector operator *(StackedVector a, double b) => Scale(b, a);
        public static StackedVector operator /(StackedVector a, double b) => Scale(1/b, a);
        #endregion



    }

}
