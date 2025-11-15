using System.Runtime.CompilerServices;

namespace JA.LinearAlgebra.Screws
{
    //using JA.LinearAlgebra.VectorCalculus;
    
    using Vector3 = Vectors.Vector3;
    using Matrix3 = Vectors.Matrix3;

    public static class Twist3
    {
        public static ScrewLayout Layout {get; } = ScrewLayout.Axis;
        #region Twists
        public static Vector33 At(Vector3 value, Vector3 position, double pitch = 0)
            => new Vector33(
                value*pitch+Vector3.Cross(position, value),
                value
            );
        public static Vector33 Pure(Vector3 value)
            => new Vector33(
                value,
                Vector3.Zero
                );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix33 CrossOp(Vector33 twist)
        {
            var wx = twist.angular.CrossOp();
            var vx = twist.linear.CrossOp();
            return new Matrix33(wx, vx, Matrix3.Zero, wx);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 Cross(Vector33 @this, Vector33 twist)
        {
            //tex: Twist Differential Operator
            //$$\begin{bmatrix}v\\ \omega
            //\end{bmatrix}\times=\begin{bmatrix}\omega\times & v\times\\
            // 0 & \omega\times \end{bmatrix}$$
            return new Vector33(
                Vector3.Cross(@this.angular, twist.linear)+Vector3.Cross(@this.linear, twist.angular),
                Vector3.Cross(@this.angular, twist.angular)
            );
        }
        /// <summary>
        /// By convention the angular vector is the line vector for twists.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 LineVector(Vector33 twist)
            => twist.angular;
        /// <summary>
        /// By convention the linear vector is the moment vector for twists.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 MomentVector(Vector33 twist)
            => twist.linear;
        /// <summary>
        /// The magnitude of a twist.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Magnitude(Vector33 twist)
            => twist.angular.Magnitude;
        /// <summary>
        /// The direction of a twist line.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Direction(Vector33 twist)
            => twist.angular.Normalize();
        /// <summary>
        /// The pitch of a twist.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Pitch(Vector33 twist)
            //tex: $$h = \frac{ \omega \cdot v}{\|\omega\|^2}$$
            => Vector3.Dot(twist.angular, twist.linear)/twist.angular.MagnitudeSquared;
        /// <summary>
        /// The point of a twist line closest to the origin.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Position(Vector33 twist)
            //tex: $$r = \frac{ \omega \times v}{\|\omega\|^2}$$
            => Vector3.Cross(twist.angular, twist.linear)/twist.angular.MagnitudeSquared;
        #endregion

    }
}
