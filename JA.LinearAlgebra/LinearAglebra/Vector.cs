using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Text;

namespace JA.LinearAlgebra
{
    using static JA.LinearAlgebra.NativeArrays;

    public class Vector : IEquatable<Vector>, ICloneable
    {
        readonly int size;
        readonly double[] elements;

        #region Factory
        public static readonly Vector Empty=new Vector(0);
        public Vector(int size)
        {
            this.size=size;
            if (size==0)
            {
                this.elements = Array.Empty<double>();
            }
            else
            {
                this.elements=new double[size];
            }
        }
        public Vector(int size, IEnumerable<double> collection) : this(size)
        {
            int index=0;
            foreach (var item in collection)
            {
                elements[index++]=item;
            }
        }
        public Vector(int size, Func<int, double> init)
            : this(size)
        {
            for(int i=0; i<elements.Length; i++)
            {
                elements[i]=init(i);
            }
        }
        Vector(double[] elements, bool as_readonly)
        {
            this.elements=elements;
            this.size=elements.Length;
        }
        public Vector(params double[] elements)
        {
            this.elements=elements;
            this.size=elements.Length;
        }
        public Vector(Vector other)
        {
            this.elements=new double[other.elements.Length];
            Array.Copy(other.elements, this.elements, elements.Length);
            this.size=other.size;
        }
        public static explicit operator Vector(double[] elements)
        {
            return new Vector(elements);
        }
        public static implicit operator double[](Vector vector)
        {
            return vector.elements;
        }
        public static Vector FillStartEnd(int count, double first_value, double last_value)
        {
            double[] result=new double[count];
            double step = (last_value-first_value)/(count-1);
            result.FillSeries(first_value, step);
            return new Vector(result);
        }
        public static Vector FillStartStep(int count, double first_value, double step_value)
        {
            double[] result=new double[count];
            result.FillSeries(first_value, step_value);
            return new Vector(result);
        }        
        #endregion

        #region Properties
        public double[] Elements { get { return elements; } }
        public int Size { get { return size; } }
        public double[] ToArray()
        {
            return elements;
        }
        public double this[int index]
        {
            get { return elements[index]; }
            set
            {
                elements[index]=value;
            }
        }

        public Vector Slice(int offset, int size) => new Vector(elements.Slice(offset, size));
        public void Inject(int offset, Vector values) => NativeArrays.Inject(elements, offset, values.elements);
        public override string ToString()
        {
            return $"[{string.Join(",", elements)}]";
        }
        #endregion

        #region Methods
        public Vector AsReadOnly() { return new Vector(elements, true); }

        public static Vector operator+(Vector v, Vector u)
        {
            return new Vector( NativeArrays.ArrayAdd(v.elements, u.elements));
        }
        public static Vector operator-(Vector v)
        {
            return new Vector( NativeArrays.ArrayScale(v.elements, -1));
        }
        public static Vector operator-(Vector v, Vector u)
        {
            return new Vector( NativeArrays.ArrayAdd(v.elements, u.elements, -1));
        }
        public static Vector operator*(double factor, Vector v)
        {
            return new Vector(NativeArrays.ArrayScale(v.elements, factor));
        }
        public static Vector operator*(Vector v, double factor)
        {
            return new Vector( NativeArrays.ArrayScale(v.elements, factor));
        }
        public static Vector operator/(Vector v, double divisor)
        {
            return  new Vector( NativeArrays.ArrayScale(v.elements, 1/divisor));
        }
        public static double Dot(Vector vector, Vector other) => NativeArrays.ArrayDot(vector.elements, other.elements);
        public static Matrix Outer(Vector vector, Vector other) => new Matrix(NativeArrays.JaggedOuter(vector.elements, other.elements));
        public double Sum() => NativeArrays.ArraySum(elements);
        public double Norm2() => NativeArrays.ArrayNorm2(elements);
        public double Norm1() => NativeArrays.ArrayNorm1(elements);
        public double NormP(double p) => NativeArrays.ArrayNormP(elements, p);
        public double NormInf() => NativeArrays.ArrayNormInf(elements);
        public double SumSquares() => NativeArrays.ArrayDot(elements, elements);
        #endregion

        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Vector)</code></returns>
        public override bool Equals(object obj)
        {
            if(obj is Vector vec)
            {
                return Equals(vec);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among vectors
        /// </summary>
        /// <param name="other">The other vector to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Vector other)
        {
            return NativeArrays.ArrayEquals(elements, other.elements);
        }

        /// <summary>
        /// Calculates the hash code for the vector
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode() 
            => elements.Aggregate(17, (hash, val) => HashCode.Combine(hash, val.GetHashCode()));

        #endregion

        #region ICloneable Members

        public Vector Clone() { return new Vector(this); }

        object ICloneable.Clone()
        {
            return Clone();
        }

        #endregion

        #region IEnumerable<double> Members

        public IEnumerator<double> GetEnumerator()
        {
            for (int i=0; i<elements.Length; i++)
            {
                yield return elements[i];
            }
        }


        #endregion
    }

}
