using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

using JA.LinearAlgebra.VectorCalculus;

namespace JA.LinearAlgebra.ScrewCalculus
{
    public static class Screws
    {
        #region Twists
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 TwistAt(Vector3 angular, Vector3 position, double pitch = 0.0)
            => new Vector33(
                angular*pitch+Vector3.Cross(position, angular),
                angular
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 PureTwist(Vector3 linear)
            => new Vector33(
                linear,
                Vector3.Zero
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 CrossTwist(Vector33 @this, Vector33 twist)
        {
            //tex: Twist Differential Operator $$\begin{bmatrix}v\\
            //\omega
            //\end{bmatrix}\times=\begin{bmatrix}\omega\times & v\times\\
            // 0 & \omega\times
            //\end{bmatrix}$$
            return new Vector33(
                Vector3.Cross(@this.angular, twist.linear)+Vector3.Cross(@this.linear, twist.angular),
                Vector3.Cross(@this.angular, twist.angular)
            );
        }
        /// <summary>
        /// By convention the angular vector is the line vector for twists.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 TwistLineVector(this Vector33 twist)
            => twist.angular;
        /// <summary>
        /// By convention the linear vector is the moment vector for twists.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 TwistMomentVector(this Vector33 twist)
            => twist.linear;
        /// <summary>
        /// The magnitude of a twist.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double TwistMagnitude(this Vector33 twist)
            => twist.angular.Magnitude;
        /// <summary>
        /// The direction of a twist line.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 TwistDirection(this Vector33 twist)
            => twist.angular.Normalize();
        /// <summary>
        /// The pitch of a twist.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double TwistPitch(this Vector33 twist)
            //tex: $$h = \frac{ \omega \cdot v}{\|\omega\|^2}$$
            => Vector3.Dot(twist.angular, twist.linear)/twist.angular.MagnitudeSquared;
        /// <summary>
        /// The point of a twist line closest to the origin.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 TwistPosition(this Vector33 twist)
            //tex: $$r = \frac{ \omega \times v}{\|\omega\|^2}$$
            => Vector3.Cross(twist.angular, twist.linear)/twist.angular.MagnitudeSquared;
        #endregion

        #region Wrenches

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 WrenchAt(Vector3 angular, Vector3 position, double pitch = 0.0)
            => new Vector33(
                angular,
                angular*pitch+Vector3.Cross(position, angular)
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 PureWrench(Vector3 linear)
            => new Vector33(
                Vector3.Zero,
                linear
            );
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector33 CrossWrench(Vector33 @this, Vector33 wrench)
        {
            //tex: Wrench Differential Operator $$\begin{bmatrix}v\\
            //\omega
            //\end{bmatrix}\times=\begin{bmatrix}\omega\times & 0\\
            //v\times & \omega\times
            //\end{bmatrix}$$
            return new Vector33(
                Vector3.Cross(@this.angular, wrench.linear),
                Vector3.Cross(@this.linear, wrench.linear)+Vector3.Cross(@this.angular, wrench.angular)
            );
        }
        /// <summary>
        /// By convention the angular vector is the line vector for wrenches.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 WrenchLineVector(this Vector33 wrench)
            => wrench.linear;
        /// <summary>
        /// By convention the linear vector is the moment vector for wrenches.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 WrenchMomentVector(this Vector33 wrench)
            => wrench.angular;
        /// <summary>
        /// The magnitude of a wrench.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double WrenchMagnitude(this Vector33 wrench)
            => wrench.linear.Magnitude;
        /// <summary>
        /// The direction of a wrench line.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 WrenchDirection(this Vector33 wrench)
            => wrench.linear.Normalize();
        /// <summary>
        /// The pitch of a wrench.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double WrenchPitch(this Vector33 wrench)
            //tex: $$h = \frac{ F \cdot \tau}{\|F\|^2}$$
            => Vector3.Dot(wrench.linear, wrench.angular)/wrench.linear.MagnitudeSquared;
        /// <summary>
        /// The point of a wrench line closest to the origin.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 WrenchPosition(this Vector33 wrench)
            //tex: $$r = \frac{ F \times \tau}{\|F\|^2}$$
            => Vector3.Cross(wrench.linear, wrench.angular)/wrench.linear.MagnitudeSquared;
        #endregion

    }
}
