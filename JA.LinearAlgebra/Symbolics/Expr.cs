using System;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;

namespace JA.Symbolics
{
    public enum UnaryFunction
    {
        Sign,
        Abs,
        Neg,
        Exp,
        Log,
        Sin,
        Cos,
        Tan,
        Asin,
        Acos,
        Atan,
    }
    public enum BinaryOperator
    {
        Add,
        Subtract,
        Multiply,
        Divide,
        Power,
        Hypot,
        Min,
        Max,
    }
    public delegate double NonaryFunc();
    public delegate double UnaryFunc( double x1 );
    public delegate double BinaryFunc( double x1, double x2 );
    public delegate double TertiaryFunc( double x1, double x2, double x3 );

    public abstract class Expr : IEquatable<Expr>, IFormattable
    {
        readonly Dictionary<int, string> variables;

        #region Factory
        protected Expr(Dictionary<int, string> variables)
        {
            this.variables=variables;
        }
        public static Expr Zero { get; } = 0;
        public static Expr One { get; } = 1;
        public abstract Expr PartialDerivative(string variable);
        public Expr PartialDerivative(Expr expr)
        {
            if (expr.IsVariable(out var varExpr))
            {
                return PartialDerivative(varExpr.Name);
            }
            return Zero;
        }

        public static implicit operator Expr(double value) => Const(value);
        public static ConstExpr Const(double value) =>
            new ConstExpr(value);
        public static VariableExpr Variable(string name, int index)
            => new VariableExpr(name, index);
        #endregion

        #region Properties
        public IReadOnlyDictionary<int, string> Variables => variables;
        public abstract double this[params double[] args] { get; }

        public bool IsConstant(out ConstExpr expr)
        {
            if (this is ConstExpr constExpr)
            {
                expr=constExpr;
                return true;
            }
            expr=null;
            return false;
        }
        public bool IsConstant(double value)
        {
            if (this is ConstExpr constExpr)
            {
                return constExpr.Value==value;
            }
            return false;
        }
        public bool IsVariable(out VariableExpr expr)
        {
            if (this is VariableExpr variableExpr)
            {
                expr=variableExpr;
                return true;
            }
            expr=null;
            return false;
        }
        public bool IsVariable(string name)
        {
            if (this is VariableExpr variableExpr)
            {
                return variableExpr.Name.Equals(name);
            }
            return false;
        }
        public bool IsVariable(int index)
        {
            if (this is VariableExpr variableExpr)
            {
                return variableExpr.Index==index;
            }
            return false;
        }
        public bool IsScale(out ScaleExpr expr)
        {
            if (this is ScaleExpr unary)
            {
                expr=unary;
                return true;
            }
            expr=null;
            return false;
        }
        public bool IsRaise(out RaiseExpr expr)
        {
            if (this is RaiseExpr unary)
            {
                expr=unary;
                return true;
            }
            expr=null;
            return false;
        }
        public bool IsUnaryBase(out Expr argument)
        {
            if (this is UnaryExprBase unaryExpr)
            {
                argument=unaryExpr.Argument;
                return true;
            }
            argument=null;
            return false;
        }
        public bool IsUnaryFunction(out UnaryFunctionExpr expr)
        {
            if (this is UnaryFunctionExpr unary)
            {
                expr=unary;
                return true;
            }
            expr=null;
            return false;
        }
        public bool IsUnaryFunction(UnaryFunction function, out Expr argument)
        {
            if (this is UnaryFunctionExpr unary)
            {
                if (unary.Function==function)
                {
                    argument=unary.Argument;
                    return true;
                }
            }
            argument=null;
            return false;
        }
        public bool IsBinaryOperator(out BinaryOperatorExpr expr)
        {
            if (this is BinaryOperatorExpr binaryExpr)
            {
                expr=binaryExpr;
                return true;
            }
            expr=null;
            return false;
        }
        public bool IsBinaryOperator(BinaryOperator @operator, out Expr left, out Expr right)
        {
            if (this is BinaryOperatorExpr binaryExpr)
            {
                if (binaryExpr.Operator==@operator)
                {
                    left=binaryExpr.Left;
                    right=binaryExpr.Right;
                    return true;
                }
            }
            left=right=null;
            return false;
        }
        #endregion

        #region Algebra
        public static Expr Scale(double factor, Expr expr)
        {
            if (factor==0) return Zero;
            if (factor==1) return expr;
            if (expr.IsConstant(0)) return Zero;
            if (expr.IsConstant(1)) return factor;
            if (expr.IsScale(out var exprScaleExpr))
            {
                return ( factor*exprScaleExpr.Factor )*exprScaleExpr.Argument;
            }
            return new ScaleExpr(expr, factor);
        }

        public static Expr Sign(Expr expr)
        {
            if (expr.IsConstant(0)) return Zero;
            if (expr.IsConstant(out var constExpr))
            {
                return Math.Sign(constExpr.Value);
            }
            if (expr.IsUnaryFunction(UnaryFunction.Neg, out Expr argument))
            {
                return -Sign(argument);
            }
            if (expr.IsScale(out var scaleExpr))
            {
                return Math.Sign(scaleExpr.Factor)*Sign(scaleExpr.Argument);
            }
            return new UnaryFunctionExpr(expr, UnaryFunction.Sign);
        }

        public static Expr Abs(Expr expr)
        {
            if (expr.IsConstant(0)) return Zero;
            if (expr.IsConstant(out var constExpr))
            {
                return Math.Abs(constExpr.Value);
            }
            if (expr.IsUnaryFunction(UnaryFunction.Neg, out Expr argument))
            {
                return Abs(argument);
            }
            if (expr.IsScale(out var scaleExpr))
            {
                return Math.Abs(scaleExpr.Factor)*Abs(scaleExpr.Argument);
            }
            return new UnaryFunctionExpr(expr, UnaryFunction.Abs);
        }

        public static Expr Negate(Expr expr)
        {
            if (expr.IsConstant(0)) return Zero;
            if (expr.IsConstant(out var constExpr))
            {
                return -constExpr.Value;
            }
            if (expr.IsUnaryFunction(UnaryFunction.Neg, out var argument))
            {
                return argument;
            }
            if (expr.IsScale(out var scaleExpr))
            {
                return ( -scaleExpr.Factor )*scaleExpr.Argument;
            }
            return new UnaryFunctionExpr(expr, UnaryFunction.Neg);
        }

        public static Expr Sqr(Expr expr) => Raise(expr, 2);
        public static Expr Sqrt(Expr expr) => Raise(expr, 0.5);
        public static Expr Cub(Expr expr) => Raise(expr, 3);
        public static Expr Cubrt(Expr expr) => Raise(expr, 1/3d);
        public static Expr Exp(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Exp);
        public static Expr Log(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Log);
        public static Expr Sin(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Sin);
        public static Expr Cos(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Cos);
        public static Expr Tan(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Tan);
        public static Expr Asin(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Asin);
        public static Expr Acos(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Acos);
        public static Expr Atan(Expr expr) => new UnaryFunctionExpr(expr, UnaryFunction.Atan);
        public static Expr Add(Expr lhs, Expr rhs)
        {
            if (lhs.IsConstant(0)) return rhs;
            if (rhs.IsConstant(0)) return lhs;
            if (lhs==rhs)
            {
                return 2*lhs;
            }
            if (lhs.IsConstant(out var lhsConst2)&&rhs.IsConstant(out var rhsConst2))
            {
                return lhsConst2.Value+rhsConst2.Value;
            }
            if (lhs.IsUnaryFunction(out var lhsUnaryExpr))
            {
                if (lhsUnaryExpr.Function==UnaryFunction.Neg)
                {
                    return rhs-lhsUnaryExpr.Argument;
                }
            }
            if (rhs.IsUnaryFunction(out var rhsUnaryExpr))
            {
                if (rhsUnaryExpr.Function==UnaryFunction.Neg)
                {
                    return lhs-rhsUnaryExpr.Argument;
                }
            }
            return new BinaryOperatorExpr(lhs, rhs, BinaryOperator.Add);
        }

        public static Expr Subtract(Expr lhs, Expr rhs)
        {
            if (lhs.IsConstant(0)) return Negate(rhs);
            if (rhs.IsConstant(0)) return lhs;
            if (lhs==rhs)
            {
                return Zero;
            }
            if (lhs.IsConstant(out var lhsConst2)&&rhs.IsConstant(out var rhsConst2))
            {
                return lhsConst2.Value-rhsConst2.Value;
            }
            if (lhs.IsUnaryFunction(out var lhsUnaryExpr))
            {
                if (lhsUnaryExpr.Function==UnaryFunction.Neg)
                {
                    return -( lhsUnaryExpr.Argument+rhs );
                }
            }
            if (rhs.IsUnaryFunction(out var rhsUnaryExpr))
            {
                if (rhsUnaryExpr.Function==UnaryFunction.Neg)
                {
                    return lhs+rhsUnaryExpr.Argument;
                }
            }
            return new BinaryOperatorExpr(lhs, rhs, BinaryOperator.Subtract);
        }

        public static Expr Multiply(Expr lhs, Expr rhs)
        {
            if (lhs.IsConstant(0)) return Zero;
            if (lhs.IsConstant(1)) return rhs;
            if (rhs.IsConstant(1)) return lhs;
            if (lhs.IsConstant(out var lhsConst2)&&rhs.IsConstant(out var rhsConst2))
            {
                return lhsConst2.Value*rhsConst2.Value;
            }
            if (lhs.IsConstant(out var lhsConst))
            {
                return Scale(lhsConst.Value, rhs);
            }
            if (rhs.IsConstant(out var rhsConst))
            {
                return Scale(rhsConst.Value, lhs);
            }
            if (lhs==rhs)
            {
                return Raise(lhs, 2);
            }
            return new BinaryOperatorExpr(lhs, rhs, BinaryOperator.Multiply);
        }

        public static Expr Divide(Expr lhs, Expr rhs)
        {
            if (lhs.IsConstant(0)) return Zero;
            if (rhs.IsConstant(1)) return lhs;
            if (lhs.IsConstant(out var lhsConst2)&&rhs.IsConstant(out var rhsConst2))
            {
                return lhsConst2.Value/rhsConst2.Value;
            }
            if (rhs.IsConstant(out var rhsConst))
            {
                return Scale(1/rhsConst.Value, lhs);
            }
            if (lhs==rhs)
            {
                return One;
            }
            return new BinaryOperatorExpr(lhs, rhs, BinaryOperator.Divide);
        }
        public static Expr Raise(Expr lhs, double exponent)
        {
            if (lhs.IsConstant(0)) return Zero;
            if (lhs.IsConstant(1)) return One;
            if (exponent==0) return One;
            if (exponent==1) return lhs;
            if (lhs.IsConstant(out var lhsConst))
            {
                return Math.Pow(lhsConst.Value, exponent);
            }
            if (exponent<0) return 1/Raise(lhs, -exponent);
            return new RaiseExpr(lhs, exponent);
        }

        public static Expr Power(Expr lhs, Expr rhs)
        {
            if (rhs.IsConstant(out var rhsConst))
            {
                if (rhsConst.Value==0) return One;
                if (rhsConst.Value==1) return lhs;
                if (rhsConst.Value==2) return Sqr(lhs);
                if (rhsConst.Value==3) return Cubrt(lhs);
                return Raise(lhs, rhsConst.Value);
            }
            return new BinaryOperatorExpr(lhs, rhs, BinaryOperator.Power);
        }
        #endregion

        #region Conversions
        public abstract Expression GetExpression();

        public NonaryFunc ToFunction()
        {
            var body = GetExpression();
            var lambda = Expression.Lambda<NonaryFunc>(body);
            return lambda.Compile();
        }
        public UnaryFunc ToFunction(Expr x1)
        {
            var p1 = x1.GetExpression() as ParameterExpression;                
            var body = GetExpression();
            var lambda = Expression.Lambda<UnaryFunc>(body, p1);
            return lambda.Compile();
        }
        public BinaryFunc ToFunction(Expr x1, Expr x2)
        {
            var p1 = x1.GetExpression() as ParameterExpression;                
            var p2 = x2.GetExpression() as ParameterExpression;                
            var body = GetExpression();
            var lambda = Expression.Lambda<BinaryFunc>(body, p1, p2);
            return lambda.Compile();
        }
        public TertiaryFunc ToFunction(Expr x1, Expr x2, Expr  x3)
        {
            var p1 = x1.GetExpression() as ParameterExpression;                
            var p2 = x2.GetExpression() as ParameterExpression;                
            var p3 = x3.GetExpression() as ParameterExpression;  
            var body = GetExpression();
            var lambda = Expression.Lambda<TertiaryFunc>(body, p1, p2, p3);
            return lambda.Compile();
        }

        #endregion

        #region Operators        
        public static Expr operator -(Expr rhs) { return Negate(rhs); }
        public static Expr operator *(double factor, Expr expr) => Scale(factor, expr);
        public static Expr operator *(Expr expr, double factor) => Scale(factor, expr);
        public static Expr operator /(Expr lhs, double rhs) { return Scale(1/rhs, lhs); }
        public static Expr operator +(Expr lhs, Expr rhs) { return Add(lhs, rhs); }
        public static Expr operator -(Expr lhs, Expr rhs) { return Subtract(lhs, rhs); }
        public static Expr operator *(Expr lhs, Expr rhs) { return Multiply(lhs, rhs); }
        public static Expr operator /(Expr lhs, Expr rhs) { return Divide(lhs, rhs); }
        public static Expr operator ^(Expr lhs, Expr rhs) { return Power(lhs, rhs); }
        #endregion

        #region Formatting
        public override string ToString() => ToString("g4");
        public string ToString(string formatting) => ToString(formatting, null);
        public abstract string ToString(string formatting, IFormatProvider formatProvider);
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Expr)</code></returns>
        public override bool Equals(object obj)
        {
            return obj is Expr expr&&Equals(expr);
        }

        /// <summary>
        /// Checks for equality among <see cref="Expr"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Expr"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public abstract bool Equals(Expr other);

        /// <summary>
        /// Calculates the hash code for the <see cref="Expr"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public abstract override int GetHashCode();

        #endregion

        #region Implementations
        public class ConstExpr : Expr
        {
            static readonly Dictionary<int, string> empty = new Dictionary<int, string>();
            readonly double value;

            public ConstExpr(double value)
                : base(empty)
            {
                this.value=value;
            }
            public static implicit operator double(ConstExpr expr) => expr.value;
            public double Value => value;
            public override double this[params double[] args] => value;
            public override Expr PartialDerivative(string variable) => Zero;

            public override string ToString(string formatting, IFormatProvider formatProvider)
            {
                return this.value.ToString(formatting, formatProvider);
            }

            public override Expression GetExpression()
            {
                return Expression.Constant(value, typeof(double));
            }

            public override bool Equals(Expr other)
            {
                if (other is ConstExpr otherExpr)
                {
                    return value==otherExpr.value;
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc=( -1521134295 )*hc+value.GetHashCode();
                    return hc;
                }
            }
        }
        public class VariableExpr : Expr
        {
            readonly string name;
            readonly int index;

            public VariableExpr(string name, int index)
                : base(new Dictionary<int, string>() { [index]=name })
            {
                this.name=name??throw new ArgumentNullException(nameof(name));
                this.index=index;
            }

            public string Name => name;
            public int Index => index;
            public override double this[params double[] args]
            {
                get
                {
                    if (index<args.Length)
                    {
                        return args[index];
                    }
                    return 0;
                }
            }
            public override Expr PartialDerivative(string variable)
            {
                if (variables[Index].Equals(variable))
                {
                    return One;
                }
                return Zero;
            }
            public override string ToString(string formatting, IFormatProvider formatProvider)
            {
                return name.ToString();
            }
            public override Expression GetExpression()
            {
                return Expression.Parameter(typeof(double), name);
            }
            public override bool Equals(Expr other)
            {
                if (other is VariableExpr otherExpr)
                {
                    return name.Equals(otherExpr.name);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc=( -1521134295 )*hc+name.GetHashCode();
                    return hc;
                }
            }
        }

        public abstract class UnaryExprBase : Expr
        {
            readonly Expr argument;

            protected UnaryExprBase(Expr argument) : base(argument.variables)
            {
                this.argument=argument??throw new ArgumentNullException(nameof(argument));
            }

            public Expr Argument => argument;
        }
        public class ScaleExpr : UnaryExprBase
        {
            private readonly double factor;

            public ScaleExpr(Expr argument, double factor) : base(argument)
            {
                this.factor=factor;
            }
            public double Factor => factor;
            public override double this[params double[] args] => factor*Argument[args];
            public override Expr PartialDerivative(string variable)
                => factor*Argument.PartialDerivative(variable);
            public override string ToString(string formatting, IFormatProvider formatProvider)
            {
                string x = ((float)factor).ToString(formatting, formatProvider);
                string inv_x = (1/(float)factor).ToString(formatting, formatProvider);
                string y = Argument.ToString(formatting, formatProvider);
                if (inv_x.Length<x.Length)
                {
                    return $"{y}/{inv_x}";
                }
                else
                {
                    if (Argument.IsVariable(out var argVarExpr))
                    {
                        return $"{x}{y}";
                    }
                    return $"{x}*{y}";
                }
            }
            public override Expression GetExpression()
            {
                var argExpr = Argument.GetExpression();
                Expression factorExpr = Expression.Constant(factor, typeof(double));
                return Expression.Multiply(factorExpr, argExpr);
            }
            public override bool Equals(Expr other)
            {
                if (other is ScaleExpr otherExpr)
                {
                    return factor==otherExpr.factor
                        &&Argument.Equals(otherExpr.Argument);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc=( -1521134295 )*hc+factor.GetHashCode();
                    hc=( -1521134295 )*hc+Argument.GetHashCode();
                    return hc;
                }
            }

        }
        public class RaiseExpr : UnaryExprBase
        {
            readonly double exponent;
            public RaiseExpr(Expr argument, double expoent) : base(argument)
            {
                this.exponent=expoent;
            }
            public double Exponent => exponent;
            public override double this[params double[] args] => Math.Pow(Argument[args], Exponent);
            public override Expr PartialDerivative(string variable)
            {
                double n = exponent;
                Expr x = Argument;
                Expr xp = x.PartialDerivative(variable);
                return n*xp*Raise(x, n-1);
            }
            public override string ToString(string formatting, IFormatProvider formatProvider)
            {
                string x = Argument.ToString(formatting, formatProvider);
                string n = ((float)exponent).ToString(formatting, formatProvider);
                return $"{x}^{n}";
            }
            public override Expression GetExpression()
            {
                var argExpr = Argument.GetExpression();
                var exponentExpr = Expression.Constant(exponent, typeof(double));
                return Expression.Call(typeof(Math).GetMethod("Pow", new Type[] { typeof(double), typeof(double) }), argExpr, exponentExpr);
            }

            public override bool Equals(Expr other)
            {
                if (other is RaiseExpr otherExpr)
                {
                    return Exponent==otherExpr.Exponent
                        &&Argument.Equals(otherExpr.Argument);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc=( -1521134295 )*hc+Exponent.GetHashCode();
                    hc=( -1521134295 )*hc+Argument.GetHashCode();
                    return hc;
                }
            }
        }
        public class UnaryFunctionExpr : UnaryExprBase
        {
            readonly UnaryFunction function;
            public UnaryFunctionExpr(Expr argument, UnaryFunction unary)
                : base(argument)
            {
                this.function=unary;
            }
            public UnaryFunction Function { get => function; }
            public override double this[params double[] args]
            {
                get
                {
                    double x = Argument[args];
                    switch (function)
                    {
                        case UnaryFunction.Sign: return Math.Sign(x);
                        case UnaryFunction.Abs: return Math.Abs(x);
                        case UnaryFunction.Neg: return -x;
                        case UnaryFunction.Exp: return Math.Exp(x);
                        case UnaryFunction.Log: return Math.Log(x);
                        case UnaryFunction.Sin: return Math.Sin(x);
                        case UnaryFunction.Cos: return Math.Cos(x);
                        case UnaryFunction.Tan: return Math.Tan(x);
                        case UnaryFunction.Asin: return Math.Asin(x);
                        case UnaryFunction.Acos: return Math.Acos(x);
                        case UnaryFunction.Atan: return Math.Atan(x);
                        default:
                        throw new NotSupportedException(function.ToString());
                    }
                }
            }
            public override Expr PartialDerivative(string variable)
            {
                Expr x = Argument;
                Expr xp = Argument.PartialDerivative(variable);
                switch (function)
                {
                    case UnaryFunction.Sign: return Zero;
                    case UnaryFunction.Abs: return xp*Sign(x);
                    case UnaryFunction.Neg: return -xp;
                    case UnaryFunction.Exp: return xp*Exp(x);
                    case UnaryFunction.Log: return xp/x;
                    case UnaryFunction.Sin: return xp*Cos(x);
                    case UnaryFunction.Cos: return -xp*Sin(x);
                    case UnaryFunction.Tan: return xp/Sqr(Cos(x));
                    case UnaryFunction.Asin: return xp/Sqrt(1-Sqr(x));
                    case UnaryFunction.Acos: return -xp/Sqrt(1-Sqr(x));
                    case UnaryFunction.Atan: return xp/( 1+Sqr(x) );
                    default:
                    throw new NotSupportedException(function.ToString());
                }
            }
            public override string ToString(string formatting, IFormatProvider formatProvider)
            {
                string x = Argument.ToString(formatting, formatProvider);
                bool isArgBin = Argument.IsBinaryOperator(BinaryOperator.Add, out _, out _)
                    || Argument.IsBinaryOperator(BinaryOperator.Subtract, out _, out _);
                switch (function)
                {
                    case UnaryFunction.Sign: return $"sign({x})";
                    case UnaryFunction.Abs: return $"abs({x})";
                    case UnaryFunction.Neg: return isArgBin ? $"-({x})" : $"-{x}";
                    case UnaryFunction.Exp: return $"exp({x})";
                    case UnaryFunction.Log: return $"log({x})";
                    case UnaryFunction.Sin: return $"sin({x})";
                    case UnaryFunction.Cos: return $"cos({x})";
                    case UnaryFunction.Tan: return $"tan({x})";
                    case UnaryFunction.Asin: return $"asin({x})";
                    case UnaryFunction.Acos: return $"acos({x})";
                    case UnaryFunction.Atan: return $"atan({x})";
                    default:
                    throw new NotSupportedException(function.ToString());
                }
            }
            public override Expression GetExpression()
            {
                var argExpr = Argument.GetExpression();

                if(function==UnaryFunction.Neg )
                {
                    return Expression.Negate(argExpr);
                }
                string methodName = function.ToString();

                return Expression.Call(typeof(Math).GetMethod(methodName, new Type[] { typeof(double) }), argExpr);
            }

            public override bool Equals(Expr other)
            {
                if (other is UnaryFunctionExpr otherExpr)
                {
                    return function==otherExpr.function
                        &&Argument.Equals(otherExpr.Argument);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc=( -1521134295 )*hc+function.GetHashCode();
                    hc=( -1521134295 )*hc+Argument.GetHashCode();
                    return hc;
                }
            }
        }


        public class BinaryOperatorExpr : Expr
        {
            public BinaryOperatorExpr(Expr left, Expr right, BinaryOperator @operator)
                : base(left.variables.Concat(right.variables).GroupBy(kvp => kvp.Key).ToDictionary(g => g.Key, g => g.First().Value))
            {
                Left=left??throw new ArgumentNullException(nameof(left));
                Right=right??throw new ArgumentNullException(nameof(right));
                this.Operator=@operator;
            }

            public Expr Left { get; }
            public Expr Right { get; }
            public BinaryOperator Operator { get; }

            public override double this[params double[] args]
            {
                get
                {
                    double x = Left[args];
                    double y = Right[args];
                    switch (Operator)
                    {
                        case BinaryOperator.Add: return x+y;
                        case BinaryOperator.Subtract: return x-y;
                        case BinaryOperator.Multiply: return x*y;
                        case BinaryOperator.Divide: return x/y;
                        case BinaryOperator.Power: return Math.Pow(x, y);
                        case BinaryOperator.Hypot: return Math.Sqrt(x*x+y*y);
                        case BinaryOperator.Min: return Math.Min(x, y);
                        case BinaryOperator.Max: return Math.Max(x, y);
                        default:
                        throw new NotSupportedException(Operator.ToString());
                    }
                }
            }
            public override Expr PartialDerivative(string variable)
            {
                Expr x = Left, y = Right;
                Expr xp = x.PartialDerivative(variable), yp = y.PartialDerivative(variable);

                switch (Operator)
                {
                    case BinaryOperator.Add: return xp+yp;
                    case BinaryOperator.Subtract: return xp-yp;
                    case BinaryOperator.Multiply: return xp*y+yp*x;
                    case BinaryOperator.Divide: return ( xp*y-yp*x )/Sqr(y);
                    case BinaryOperator.Power: return ( yp*x*Log(x)+xp*y )*Power(x, y-1);
                    case BinaryOperator.Hypot: return ( xp*x+yp*y )/this;
                    case BinaryOperator.Min: return ( xp+yp )/2-Sign(x-y)*( ( yp-xp )/2 );
                    case BinaryOperator.Max: return ( xp+yp )/2+Sign(x-y)*( ( yp-xp )/2 );
                    default:
                    throw new NotSupportedException(Operator.ToString());
                }
            }

            #region Formatting
            public override string ToString(string formatting, IFormatProvider formatProvider)
            {
                string x =Left.ToString(formatting, formatProvider);
                string y =Right.ToString(formatting, formatProvider);

                bool isLeftBin = Left.IsBinaryOperator(BinaryOperator.Add, out _, out _)
                            || Left.IsBinaryOperator(BinaryOperator.Subtract, out _, out _);
                bool isRightBin = Right.IsBinaryOperator(BinaryOperator.Add, out _, out _)
                            || Right.IsBinaryOperator(BinaryOperator.Subtract, out _, out _);
                bool isLeftUnary = Left.IsUnaryBase(out _);
                bool isRightUnary = Right.IsUnaryBase(out _);

                switch (Operator)
                {
                    case BinaryOperator.Add:
                    case BinaryOperator.Subtract:
                    case BinaryOperator.Multiply:
                    case BinaryOperator.Divide:
                    case BinaryOperator.Power:
                    {
                        if (!isLeftUnary||!isLeftBin)
                        {
                            x=$"({x})";
                        }
                        if (!isRightUnary||isRightBin)
                        {
                            y=$"({y})";
                        }
                        break;
                    }
                }

                switch (Operator)
                {
                    case BinaryOperator.Add: return $"{x}+{y}";
                    case BinaryOperator.Subtract: return $"{x}-{y}";
                    case BinaryOperator.Multiply: return $"{x}*{y}";
                    case BinaryOperator.Divide: return $"{x}/{y}";
                    case BinaryOperator.Power: return $"{x}^{y}";
                    case BinaryOperator.Hypot: return $"hypot({x},{y})";
                    case BinaryOperator.Min: return $"min({x},{y})";
                    case BinaryOperator.Max: return $"max({x},{y})";
                    default:
                    throw new NotSupportedException(Operator.ToString());
                }
            }
            #endregion

            public override Expression GetExpression()
            {
                var leftExpr = Left.GetExpression();
                var righExpr = Right.GetExpression();

                switch (Operator)
                {
                    case BinaryOperator.Add:
                        return Expression.Add(leftExpr, righExpr);
                    case BinaryOperator.Subtract:
                        return Expression.Subtract(leftExpr, righExpr);
                    case BinaryOperator.Multiply:
                        return Expression.Multiply(leftExpr, righExpr);
                    case BinaryOperator.Divide:
                        return Expression.Divide(leftExpr, righExpr);
                    case BinaryOperator.Power:
                        return Expression.Power(leftExpr, righExpr);
                    case BinaryOperator.Hypot:
                        return Expression.Call(typeof(Math).GetMethod("Sqrt", new Type[] { typeof(double) }),
                            Expression.Add(
                                Expression.Multiply(leftExpr, leftExpr),
                                Expression.Multiply(righExpr, righExpr)
                            ));
                    case BinaryOperator.Min:
                        return Expression.Call(typeof(Math).GetMethod("Min", new Type[] { typeof(double), typeof(double) }), leftExpr, righExpr);
                    case BinaryOperator.Max:
                        return Expression.Call(typeof(Math).GetMethod("Max", new Type[] { typeof(double), typeof(double) }), leftExpr, righExpr);
                    default:
                        throw new NotSupportedException(Operator.ToString());
                }

            }

            public override bool Equals(Expr other)
            {
                if (other is BinaryOperatorExpr otherExpr)
                {
                    return Operator==otherExpr.Operator
                        &&Left.Equals(otherExpr.Left)
                        &&Right.Equals(otherExpr.Right);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc=( -1521134295 )*hc+Operator.GetHashCode();
                    hc=( -1521134295 )*hc+Left.GetHashCode();
                    hc=( -1521134295 )*hc+Right.GetHashCode();
                    return hc;
                }
            }
        }
        #endregion

    }

}