namespace JA.LinearAlgebra.Tests;

using Microsoft.VisualStudio.TestTools.UnitTesting;

using Vector = JA.LinearAlgebra.Vector;

[TestClass]
public class VectorTests
{
    [TestMethod]
    public void Test_Vector_StaticMethods()
    {
        var empty_vector = new Vector(Array.Empty<double>());
        Assert.IsTrue( Vector.Empty.Equals( empty_vector ) );

        var expected_vector = new Vector(1.0, 3.0, 5.0, 7.0, 9.0);
        Assert.IsTrue( Vector.FillStartStep( 5, 1.0, 2.0 ).Equals( expected_vector ) );
        Assert.IsTrue( Vector.FillStartEnd( 5, 1.0, 9.0 ).Equals( expected_vector ) );        
    }

    [TestMethod]
    public void Test_Vector_Construction()
    {
        const int n_size = 5;
        var vector1 = new Vector(n_size);
        Assert.AreEqual(n_size, vector1.Size);
        for(int i=0; i<n_size; i++)
        {
            Assert.AreEqual(0.0, vector1[i]);
        }
        var vector2 = new Vector(n_size, new double[] { 1.0, 2.0, 3.0, 4.0, 5.0 });
        Assert.AreEqual(n_size, vector2.Size);
        for(int i=0; i<n_size; i++)
        {
            Assert.AreEqual(i+1.0, vector2[i]);
        }
        var vector3 = new Vector(n_size, index => index * 1.5);
        Assert.AreEqual(n_size, vector3.Size);
        for(int i=0; i<n_size; i++)
        {
            Assert.AreEqual(i * 1.5, vector3[i]);
        }
        var vector4 = new Vector(vector2);
        Assert.AreEqual(n_size, vector4.Size);
        for(int i=0; i<n_size; i++)
        {
            Assert.AreEqual(i+1.0, vector4[i]);
        }
    }
    [TestMethod]
    public void Test_Vector_Conversions()
    {
        var original_array = new double[] { 10.0, 20.0, 30.0 };
        var vector = (Vector)original_array;
        Assert.AreEqual(original_array.Length, vector.Size);
        for(int i=0; i<original_array.Length; i++)
        {
            Assert.AreEqual(original_array[i], vector[i]);
        }
        var converted_array = (double[])vector;
        Assert.HasCount(original_array.Length, converted_array);
        for(int i=0; i<original_array.Length; i++)
        {
            Assert.AreEqual(original_array[i], converted_array[i]);
        }
    }
    [TestMethod]
    public void Test_Vector_Equality()
    {
        var vector1 = new Vector(1.0, 2.0, 3.0);
        var vector2 = new Vector(1.0, 2.0, 3.0);
        var vector3 = new Vector(4.0, 5.0, 6.0);
        Assert.IsTrue(vector1.Equals(vector2));
        Assert.IsFalse(vector1.Equals(vector3));
    }
    [TestMethod]
    public void Test_Vector_Algebra()
    {
        var vector1 = new Vector(1.0, 2.0, 3.0);
        var vector2 = new Vector(4.0, 5.0, 6.0);

        var add_vector_expected = new Vector(5.0, 7.0, 9.0);
        var sub_vector_expected = new Vector(3.0, 3.0, 3.0);
        var mul_vector_expected = new Vector(2.0, 4.0, 6.0);
        var div_vector_expected = new Vector(2.0, 2.5, 3.0);

        var add_vector = vector1 + vector2;
        var sub_vector = vector2 - vector1;
        var mul_vector = vector1 * 2.0;
        var div_vector = vector2/2.0;

        Assert.IsTrue(add_vector.Equals(add_vector_expected));
        Assert.IsTrue(sub_vector.Equals(sub_vector_expected));
        Assert.IsTrue(mul_vector.Equals(mul_vector_expected));
        Assert.IsTrue(div_vector.Equals(div_vector_expected));

        var dot_product_expected = 32.0;
        var dot_product = Vector.Dot(vector1, vector2);
        Assert.AreEqual(dot_product_expected, dot_product);
    }
}
