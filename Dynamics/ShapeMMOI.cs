

using Featherstone.VectorCalculus;

namespace Featherstone.Dynamics
{
    public static class ShapeMMOI
    {
        public static Matrix3 Box(double mass, double Δx, double Δy, double Δz)
        {
            var Ix = (1.0 / 12.0) * mass * (Δy * Δy + Δz * Δz);
            var Iy = (1.0 / 12.0) * mass * (Δx * Δx + Δz * Δz);
            var Iz = (1.0 / 12.0) * mass * (Δx * Δx + Δy * Δy);
            return Matrix3.Diagonal(Ix, Iy, Iz);
        }
        public static Matrix3 Box(double mass, Vector3 xAxis, Vector3 yAxis, double Δx, double Δy, double Δz)
        {
            Vector3 zAxis = Vector3.Cross(xAxis, yAxis).Normalize();
            yAxis = Vector3.Cross(zAxis, xAxis).Normalize();
            Matrix3 R = Matrix3.FromColumns(xAxis, yAxis, zAxis);
            Matrix3 I = Box(mass, Δx, Δy, Δz);
            return R * I * Matrix3.Transpose(R);
        }
    }
}
