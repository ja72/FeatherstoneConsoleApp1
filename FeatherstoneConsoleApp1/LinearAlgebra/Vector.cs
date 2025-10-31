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
    public class Vector : IEquatable<Vector>, ICloneable
    {
        readonly int size;
        readonly double[] elements;

        #region Factory
        public static readonly Vector Empty=new Vector(0);
        public Vector(int size)
        {
            this.elements=new double[size];
            this.size=size;
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
        public static explicit operator double[](Vector vector)
        {
            return vector.elements;
        }
        public static Vector FillStartEnd(int count, double first_value, double last_value)
        {
            double[] result=new double[count];
            result.FillSeries(first_value, first_value + last_value * (count-1));
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


        public override string ToString()
        {
            return $"[{string.Join(",", elements)}]";
        }
        #endregion

        #region Methods
        public Vector AsReadOnly() { return new Vector(elements, true); }

        public static Vector operator+(Vector v, Vector u)
        {
            return new Vector( v.elements.VectorAdd(u.elements));
        }
        public static Vector operator-(Vector v)
        {
            return new Vector( v.elements.VectrorScale(-1));
        }
        public static Vector operator-(Vector v, Vector u)
        {
            return new Vector( v.elements.VectorAdd(u.elements, -1));
        }
        public static Vector operator*(double factor, Vector v)
        {
            return new Vector(v.elements.VectrorScale(factor));
        }
        public static Vector operator*(Vector v, double factor)
        {
            return new Vector( v.elements.VectrorScale(factor));
        }
        public static Vector operator/(Vector v, double divisor)
        {
            return  new Vector( v.elements.VectrorScale(1/divisor));
        }
        public static Vector operator/(Vector v, Vector u)
        {
            return new Vector( v.elements.VectorDivide(u.elements));
        }
        public double Dot(Vector other) { return elements.VectorDot(other.elements); }
        public Matrix Outer(Vector other) { return new Matrix( elements.VectorOuter(other.elements)); }
        public double Sum() { return elements.VectorElementSum(); }
        public double Norm2() { return elements.VectorNorm2(); }
        public double Norm1() { return elements.VectorNorm1(); }
        public double NormP(double p) { return elements.VectorNormP(p); }
        public double NormInf() { return elements.VectorNormInf(); }
        public double SumSquares() { return elements.VectorDot(elements); } 
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
            return elements.VectorEquals(other.elements);
        }

        /// <summary>
        /// Calculates the hash code for the vector
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            return elements.Aggregate(17, (hash, val) => hash * 23 + val.GetHashCode());
        }

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
