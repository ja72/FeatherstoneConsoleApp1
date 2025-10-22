using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Featherstone.VectorCalculus
{
    public readonly struct Pose3
    {
        internal readonly Vector3 position;
        internal readonly Quaternion3 orientation;

        #region Factory
        Pose3(Vector3 position, Quaternion3 orientation)
        {
            this.position = position;
            this.orientation = orientation;
        }
        public static implicit operator Pose3(Vector3 position) => Pose3.At(position);
        public static implicit operator Pose3(Quaternion3 orientation) => Pose3.About(orientation);
        public static implicit operator Vector3(Pose3 pose) => pose.position;
        public static implicit operator Quaternion3(Pose3 pose) => pose.orientation;
        public static Pose3 Identity { get; } = new Pose3(Vector3.Zero, Quaternion3.Identity);
        public static Pose3 About(Quaternion3 orientation) => new Pose3(Vector3.Zero, orientation);
        public static Pose3 At(Vector3 position) => new Pose3(position, Quaternion3.Identity);
        public static Pose3 At(Vector3 position, Quaternion3 orientation) => new Pose3(position, orientation);
        #endregion

        #region Properties
        public Vector3 Position => this.position;
        public Quaternion3 Orientation => this.orientation;
        #endregion

        #region Algebra
        public static Pose3 Add(Pose3 @base, Pose3 local)
        {
            // [p3, R3] = [p1 + R1 p2, R1 R2]
            var topOrientation = @base.orientation * local.orientation;
            var topPosition = @base.position + @base.orientation.Rotate(local.position);
            return new Pose3(topPosition, topOrientation);
        }
        public static Pose3 Subtract(Pose3 top, Pose3 local)
        {
            // [p1, R1] = [p3 - R3 R2^T p2, R3 R2^T]
            var baseOrientation = top.orientation * Quaternion3.Conjugate(local.orientation);
            var basePosition = top.position - baseOrientation.Rotate(local.position);
            return new Pose3(basePosition, baseOrientation);
        }
        public Pose3 Inverse()
        {
            // [p2, R2] = [-R2^T p2, R2^T]
            var invOrientation = Quaternion3.Conjugate(this.orientation);
            var invPosition = invOrientation.Rotate(-this.position);
            return new Pose3(invPosition, invOrientation);
        }
        public static Pose3 operator +(Pose3 a, Pose3 b) => Add(a, b);
        public static Pose3 operator -(Pose3 a, Pose3 b) => Subtract(a, b);
        public static Pose3 operator + (Pose3 a, Vector3 b) => Add(a, At(b));
        #endregion
    }
}
