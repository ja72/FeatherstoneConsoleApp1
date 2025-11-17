using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA.LinearAlgebra.Vectors
{
    public readonly struct Pose3 : ICanConvertUnits<Pose3>
    {
        internal readonly Vector3 position;
        internal readonly Quaternion3 orientation;

        #region Factory
        Pose3(Vector3 position, Quaternion3 orientation)
        {
            this.position    = position;
            this.orientation = orientation;
        }
        public static implicit operator Pose3(Vector3 position) => Translation(position);
        public static implicit operator Pose3(Quaternion3 orientation) => Rotation(orientation);
        public static explicit operator Vector3(Pose3 pose) => pose.position;
        public static explicit operator Quaternion3(Pose3 pose) => pose.orientation;
        public static Pose3 Origin { get; } = new Pose3(Vector3.Zero, Quaternion3.Identity);
        public static Pose3 Translation(double x, double y, double z) 
            => Translation(new Vector3(x, y, z));
        public static Pose3 Translation(Vector3 delta) 
            => new Pose3(delta, Quaternion3.Identity);
        public static Pose3 Rotation(Vector3 axis, double angle) 
            => Rotation(Quaternion3.FromAxisAngle(axis, angle));
        public static Pose3 Rotation(Quaternion3 rotation) 
            => new Pose3(Vector3.Zero, rotation);
        public static Pose3 TranslationRotation(Vector3 delta, Quaternion3 rotation) 
            => new Pose3(delta, rotation);
        #endregion

        #region Properties
        public Vector3 Position => this.position;
        public Quaternion3 Orientation => this.orientation;

        public bool IsRotationOnly => this.position.Equals(Vector3.Zero);
        public bool IsTranslationOnly => this.orientation.Equals(Quaternion3.Identity);
        public bool IsIdentity => IsRotationOnly && IsTranslationOnly;

        public Pose3 Translate(Vector3 delta) => this + Translation(delta);
        public Pose3 Rotate(Quaternion3 rotation) => this + Rotation(rotation);
        public Pose3 TranslateRotate(Vector3 delta, Quaternion3 rotation) => Add( this, TranslationRotation(delta, rotation));
        #endregion

        #region Algebra
        public static Pose3 Add(Pose3 @base, Pose3 local)
        {
            // [p3, R3] = [p1 + R1 p2, R1 R2]
            var topOrientation = @base.orientation * local.orientation;
            var topPosition = @base.position + @base.orientation.RotateVector(local.position);
            return new Pose3(topPosition, topOrientation);
        }
        public static Pose3 Subtract(Pose3 top, Pose3 local)
        {
            // [p1, R1] = [p3 - R3 R2^T p2, R3 R2^T]
            var baseOrientation = top.orientation * Quaternion3.Conjugate(local.orientation);
            var basePosition = top.position - baseOrientation.RotateVector(local.position);
            return new Pose3(basePosition, baseOrientation);
        }
        public Pose3 Inverse()
        {
            // [p2, R2] = [-R2^T p2, R2^T]
            var invOrientation = Quaternion3.Conjugate(this.orientation);
            var invPosition = invOrientation.RotateVector(-this.position);
            return new Pose3(invPosition, invOrientation);
        }
        public static Pose3 operator +(Pose3 a, Pose3 b) => Add(a, b);
        public static Pose3 operator -(Pose3 a, Pose3 b) => Subtract(a, b);
        public static Pose3 operator + (Pose3 a, Vector3 b) => Add(a, Translation(b));
        #endregion

        #region Formatting
        public override string ToString()
        {
            if(IsTranslationOnly)
            {
                return $"T({position})";
            }
            if(IsRotationOnly)
            {
                return $"R({orientation})";
            }
            return $"TR({position}, {orientation})";
        }


        public Pose3 ToConvertedFrom(UnitSystem units, UnitSystem target)
        {
            if(units == target) return this;
            float f_len = Unit.Length.Convert(units, target);
            return new Pose3(f_len*Position, Orientation);
        }

        #endregion

    }
}
