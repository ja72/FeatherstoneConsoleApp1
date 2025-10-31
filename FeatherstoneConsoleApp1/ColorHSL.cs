using System;
using System.Drawing;

namespace JA
{
    public readonly struct ColorHSL : IEquatable<ColorHSL>
    {
        readonly (float h, float s, float l, float a) data;
        public ColorHSL(float h, float s, float l, float a = 1)
        {
            this.data=(h,s,l,a);
        }

        public static implicit operator ColorHSL(Color color) => FromColor(color);
        public static explicit operator Color(ColorHSL hsl) => hsl.ToColor();

        /// <summary> 
        /// Converts RGB to HSL 
        /// </summary> 
        /// <remarks>Takes advantage of whats already built in to .NET by using the Color.GetHue, Color.GetSaturation and Color.GetBrightness methods</remarks> 
        /// <param name="color">A Color to convert</param> 
        /// <returns>An HSL tuple</returns> 
        public static ColorHSL FromColor(Color color)
        {
            var h = color.GetHue() / 360f;
            var s = color.GetSaturation();
            var l = color.GetBrightness();
            var a = color.A/255f;

            return new ColorHSL(h, l, s, a);
        }

        /// <summary>
        /// Converts a color from HSL to RGB
        /// </summary>
        /// <remarks>Adapted from the algorithm in Foley and Van-Dam</remarks>
        /// <returns>A Color structure containing the equivalent RGB values</returns>
        public Color ToColor()
        {
            float r, g, b;
            float temp1, temp2;
            // Clamp HSL between 0..1
            var (H, S, L)= (
                Math.Max(0, Math.Min(1f, data.h)),
                Math.Max(0, Math.Min(1f, data.s)),
                Math.Max(0, Math.Min(1f, data.l)));

            var alpha = Math.Max(0, Math.Min(1f, data.a));
            if (L == 0) r = g = b = 0;
            else
            {
                if (S == 0) r = g = b = L;
                else
                {
                    temp2 = L <= 0.5f ? L * (1f + S) : L + S - L * S;
                    temp1 = 2f * L - temp2;

                    var t3 = new[] { H + 1 / 3f, H, H - 1 / 3f };
                    var clr = new float[] { 0, 0, 0 };
                    for (int i = 0; i < 3; i++)
                    {
                        if (t3[i] < 0) t3[i] += 1f;
                        if (t3[i] > 1) t3[i] -= 1f;

                        if (6.0 * t3[i] < 1.0) clr[i] = temp1 + (temp2 - temp1) * t3[i] * 6f;
                        else if (2.0 * t3[i] < 1.0) clr[i] = temp2;
                        else if (3.0 * t3[i] < 2.0) clr[i] = temp1 + (temp2 - temp1) * (2 / 3f - t3[i]) * 6;
                        else clr[i] = temp1;
                    }
                    r = clr[0];
                    g = clr[1];
                    b = clr[2];
                }
            }

            return Color.FromArgb(
                (int)( 255*alpha ),
                (int)( 255*r ),
                (int)( 255*g ),
                (int)( 255*b ));

        }

        public ColorHSL Next() => AddHue(120);
        public ColorHSL Prev() => AddHue(-120);

        public ColorHSL AddHue(float hueAngle) => SetHue(data.h+hueAngle);
        public ColorHSL SetHue(float hueAngle)
        {
            int n = (int)Math.Floor(hueAngle/360);
            hueAngle -= 360* n;

            return new ColorHSL(
                        hueAngle,
                        data.s,
                        data.l,
                        data.a);
        }
        public ColorHSL AddSaturation(float satValue) => SetSaturation(data.s+satValue);
        public ColorHSL SetSaturation(float satValue)
        {
            int n = (int)Math.Floor(satValue);
            satValue -= n;

            return new ColorHSL(
                        data.h,
                        satValue,
                        data.l,
                        data.a);
        }
        public ColorHSL AddLightness(float lightValue) => SetLightness(data.l+lightValue);
        public ColorHSL SetLightness(float lightValue)
        {
            int n = (int)Math.Floor(lightValue);
            lightValue -= n;

            return new ColorHSL(
                        data.h,
                        data.s,
                        lightValue,
                        data.a);
        }
        public ColorHSL AddOpaqueness(float alphaValue) => SetOpaqueness(data.a+alphaValue);
        public ColorHSL SetOpaqueness(float alphaValue)
        {
            int n = (int)Math.Floor(alphaValue);
            alphaValue -= n;

            return new ColorHSL(
                        data.h,
                        data.s,
                        data.l,
                        alphaValue);
        }

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(ColorHSL)</code></returns>
        public override bool Equals(object obj)
        {
            return obj is ColorHSL vector&&Equals(vector);
        }

        public static bool operator ==(ColorHSL target, ColorHSL other) { return target.Equals(other); }
        public static bool operator !=(ColorHSL target, ColorHSL other) { return !( target==other ); }


        /// <summary>
        /// Checks for equality among <see cref="ColorHSL"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="ColorHSL"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(ColorHSL other)
        {
            return data.Equals(other.data);
        }

        /// <summary>
        /// Calculates the hash code for the <see cref="ColorHSL"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc=( -1521134295 )*hc+data.GetHashCode();
                return hc;
            }
        }

        #endregion

    }
}
