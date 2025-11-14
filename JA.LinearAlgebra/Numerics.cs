using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using static System.Math;

namespace JA
{
    public static class Numerics
    {
        static readonly Random rng = new Random();
        public static Random RandomNumberGenerator { get => rng; }
        public static int Clamp(this int x, int low=0, int high=1)
            => Max(low, Min(high, x));
        public static float Clamp(this float x, float low=0, float high=1)
            => Max(low, Min(high, x));
        public static double Clamp(this double x, double low=0, double high=1)
            => Max(low, Min(high, x));

        public static double Uniform(this Random random, double low = 0, double high = 1)
            => low+( high-low )*random.NextDouble();

    }
}
