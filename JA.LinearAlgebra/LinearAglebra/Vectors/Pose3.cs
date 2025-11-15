using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA.LinearAlgebra.Vectors
{
    public readonly struct Pose3 : ICanChangeUnits<Pose3>
    {
        internal readonly Vector3 position;
        internal readonly Quaternion3 orientation;

        #region Factory
        Pose3(UnitSystem units, Vector3 position, Quaternion3 orientation)
        {
            this.Units       = units;
            this.position    = position;
            this.orientation = orientation;
        }
        public static implicit operator Pose3(Vector3 position) => At(position);
        public static implicit operator Pose3(Quaternion3 orientation) => About(orientation);
        public static explicit operator Vector3(Pose3 pose) => pose.position;
        public static explicit operator Quaternion3(Pose3 pose) => pose.orientation;
        public static Pose3 Origin { get; } = new Pose3(UnitSystem.MKS, Vector3.Zero, Quaternion3.Identity);
        public static Pose3 About(Quaternion3 orientation) => new Pose3(UnitSystem.MKS, Vector3.Zero, orientation);
        public static Pose3 At(Vector3 position) => At(UnitSystem.MKS, position);
        public static Pose3 At(Vector3 position, Quaternion3 orientation) => At(UnitSystem.MKS, position, orientation);
        public static Pose3 At(UnitSystem unit, Vector3 position) => new Pose3(unit, position, Quaternion3.Identity);
        public static Pose3 At(UnitSystem unit, Vector3 position, Quaternion3 orientation) => new Pose3(unit, position, orientation);
        #endregion

        #region Properties
        public UnitSystem Units { get; }
        public Vector3 Position => this.position;
        public Quaternion3 Orientation => this.orientation;

        public bool IsRotationOnly => this.position.Equals(Vector3.Zero);
        public bool IsTranslationOnly => this.orientation.Equals(Quaternion3.Identity);
        public bool IsIdentity => IsRotationOnly && IsTranslationOnly;

        #endregion

        #region Algebra
        public static Pose3 Add(Pose3 @base, Pose3 local)
        {
            UnitSystem units = @base.Units;
            local=local.ToConverted(units); 
            // [p3, R3] = [p1 + R1 p2, R1 R2]
            var topOrientation = @base.orientation * local.orientation;
            var topPosition = @base.position + @base.orientation.RotateVector(local.position);
            return new Pose3(units, topPosition, topOrientation);
        }
        public static Pose3 Subtract(Pose3 top, Pose3 local)
        {
            UnitSystem units = top.Units;
            local=local.ToConverted(units); 
            // [p1, R1] = [p3 - R3 R2^T p2, R3 R2^T]
            var baseOrientation = top.orientation * Quaternion3.Conjugate(local.orientation);
            var basePosition = top.position - baseOrientation.RotateVector(local.position);
            return new Pose3(units, basePosition, baseOrientation);
        }
        public Pose3 Inverse()
        {
            // [p2, R2] = [-R2^T p2, R2^T]
            var invOrientation = Quaternion3.Conjugate(this.orientation);
            var invPosition = invOrientation.RotateVector(-this.position);
            return new Pose3(Units, invPosition, invOrientation);
        }
        public static Pose3 operator +(Pose3 a, Pose3 b) => Add(a, b);
        public static Pose3 operator -(Pose3 a, Pose3 b) => Subtract(a, b);
        public static Pose3 operator + (Pose3 a, Vector3 b) => Add(a, At(b));
        #endregion

        #region Formatting
        public override string ToString()
        {
            if(IsTranslationOnly)
            {
                return $"Pose3(Position: {position})";
            }
            if(IsRotationOnly)
            {
                return $"Pose3(Orientation: {orientation})";
            }
            return $"Pose3(Position: {position}, Orientation: {orientation})";
        }

        public Pose3 ToConverted(UnitSystem target)
        {
            if (Units==target) return this;
            float f_len = Unit.Length.Convert(Units, target);
            return new Pose3(target, f_len*Position, Orientation);
        }

        #endregion

    }
}
