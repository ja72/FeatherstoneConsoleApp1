using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA
{
    public static class Extensions
    {
        public static float Cap(this float v, float min, float max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }
        public static float CapAbs(this float v, float min, float max)
        {
            if (Math.Abs(v) < min) return min;
            if (Math.Abs(v) > max) return max;
            return v;
        }
    }
}
