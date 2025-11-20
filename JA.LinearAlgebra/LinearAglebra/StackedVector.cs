using System;
using System.Collections.Generic;
using System.Linq;

namespace JA.LinearAlgebra
{
    public class StackedVector
    {
        readonly double[] elements;
        readonly IReadOnlyList<int> partitions;

        #region Factory
        public StackedVector(params int[] partition_sizes)
            : this(partition_sizes.ToList()) { }
        public StackedVector(IReadOnlyList<int> partition_sizes)
        {
            this.partitions=partition_sizes;
            var size=partition_sizes.Sum();
            this.elements=new double[size];
        }
        public StackedVector(IReadOnlyList<int> partition_sizes, double[] values)
        {
            this.partitions=partition_sizes;
            var size=partition_sizes.Sum();
            if (size==values.Length)
            {
                this.elements=values;
            }
            else
            {
                this.elements=new double[size];
                values.CopyTo(elements, 0);
            }
        }
        public StackedVector(double[] values, params int[] partition_sizes)
            : this(partition_sizes, values) { }
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
            this.partitions=part;
            this.elements=NativeArrays.ToArray(values);
        }
        public StackedVector(params Vector[] partitions)
            : this(partitions.Select(v => v.Elements).ToArray()) 
        { }
        public static StackedVector CompatibleWith(StackedVector vector)
            => new StackedVector(vector.partitions);
        public static StackedVector FromSizeAndCount(int size, int count)
            => new StackedVector(Enumerable.Repeat(size, count).ToArray());

        #endregion

        #region Properties
        public IReadOnlyList<int> Partitions => partitions;
        public double[] Elements => elements;
        public int Size { get => elements.Length; }
        public Vector ToVector() => new Vector(elements);
        public int GetOffset(int partitionIndex)
        {
            return partitions.Take(partitionIndex).Sum();
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

        public bool IsCompatibleWith(StackedVector other)
        {
            return Enumerable.SequenceEqual(partitions, other.partitions);
        }

        #endregion

        #region Conversions
        public static implicit operator Vector(StackedVector stacked) => stacked.ToVector();
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
        #endregion

        #region Algebra
        public static StackedVector Add(StackedVector A, StackedVector B)
        {
            if (!A.IsCompatibleWith(B))
            {
                throw new ArgumentException("Incompatible Partitions.", nameof(B));
            }
            var result = new double[A.elements.Length];
            for (int i = 0; i<result.Length; i++)
            {
                result[i]=A.elements[i]+B.elements[i];
            }
            return new StackedVector(result, A.partitions.ToArray());
        }
        public static StackedVector Subtract(StackedVector A, StackedVector B)
        {
            if (!A.IsCompatibleWith(B))
            {
                throw new ArgumentException("Incompatible Partitions.", nameof(B));
            }
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
        public static double Dot(StackedVector A, StackedVector B)
        {
            if (!A.IsCompatibleWith(B))
            {
                throw new ArgumentException("Incompatible Partitions.", nameof(B));
            }
            return NativeArrays.ArrayDot(A.elements, B.elements);
        }
        public static StackedMatrix Outer(StackedVector A, StackedVector B)
        {
            var values = NativeArrays.Array2Outer(A.elements, B.elements);

            return new StackedMatrix(A.partitions, B.partitions, values);
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

        #region Formatting
        public override string ToString()
            => ToString("g5");
        public string ToString(string format)
        {
            string[] parts = new string[partitions.Count];
            for (int i = 0; i<parts.Length; i++)
            {
                parts[i] = this[i].ToStringList(format);
            }
            return $"({string.Join("|", parts)})";
        }
        public string ToString(int decimals)
        {
            string[] parts = new string[partitions.Count];
            for (int i = 0; i<parts.Length; i++)
            {
                parts[i] = this[i].ToStringList(decimals);
            }
            return $"({string.Join("|", parts)})";
        }

        #endregion

    }

}
