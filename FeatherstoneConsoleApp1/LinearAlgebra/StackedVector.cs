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
                Array.Copy(value,0, elements, offset, size);
            }
        }
        public double[] ToArray() => elements;
        public Vector[] ToVectorArray()
        {
            Vector[] result = new Vector[partitions.Count];
            for (int i = 0; i<partitions.Count; i++)
            {
                result[i]=new Vector(this[i]);
            }
            return result;
        }
    }

}
