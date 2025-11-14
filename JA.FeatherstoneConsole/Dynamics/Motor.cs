using System;
using System.Linq.Expressions;
using System.Security.Cryptography;

namespace JA.Dynamics
{
    public enum MotorDefined
    {
        Position,
        Velocity,
        Acceleration,
        Forcing,
    }

    public class Motor 
    {
        public static readonly Expr t = Expr.Variable("t", 0);
        public static readonly Expr q = Expr.Variable("q", 1);
        public static readonly Expr qp = Expr.Variable("qp", 2);

        public Motor(MotorDefined defined, Expr known)
        {
            switch (defined)
            {
                case MotorDefined.Position:
                {
                    Drive=known.PartialDerivative("t").PartialDerivative("t");
                    Defined=MotorDefined.Acceleration;
                    break;
                }
                case MotorDefined.Velocity:
                {
                    Drive=known.PartialDerivative("t");
                    Defined=MotorDefined.Acceleration;
                    break;
                }
                case MotorDefined.Acceleration:
                {
                    Drive=known;
                    Defined=MotorDefined.Acceleration;
                    break;
                }
                case MotorDefined.Forcing:
                {
                    Drive=known;
                    Defined=MotorDefined.Forcing;
                    break;
                }
                default:
                throw new NotSupportedException(defined.ToString());
            }

        }

        public static Motor ConstForcing(double value)
            => new Motor(MotorDefined.Forcing, value);
        public static Motor ConstAcceleration(double value)
            => new Motor(MotorDefined.Acceleration, value);
        public static Motor FunctionOfTime(MotorDefined defined, Func<Expr, Expr> f)
            => new Motor(defined, f(t));
        public static Motor SpringDamper(double spring, double preload, double damping)
            => new Motor(MotorDefined.Forcing, preload-spring*q-damping*qp);
        public static Motor FunctionOfPosition(MotorDefined defined, Func<Expr, Expr> f)
            => new Motor(defined, f(q));
        public static Motor FunctionOfPositionAndSpeed(MotorDefined defined, Func<Expr, Expr, Expr> f)
            => new Motor(defined, f(q,qp));

        public MotorDefined Defined { get; }
        public Expr Drive { get; }
        public Func<double, double, double, double> Compile()
        {
            return (t,q,qp) => Drive[t,q,q];
        }
        public override string ToString()
        {
            return $"Motor({Defined}, f(t,q,qp)={Drive})";
        }
    }


}