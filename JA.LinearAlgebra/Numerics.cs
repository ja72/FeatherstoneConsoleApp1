using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using JA.LinearAlgebra;

using static System.Math;

namespace JA
{
    using Vector = JA.LinearAlgebra.Vector;
    using StackedVector = JA.LinearAlgebra.StackedVector;
    using Matrix = JA.LinearAlgebra.Matrix;
    using StackedMatrix = JA.LinearAlgebra.StackedMatrix;

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

        #region Formatting
        public static string ToStringList(this double[] elements, string format, string delimiter = ",")
        {
            return string.Join(delimiter, elements.Select(x=> x.ToString(format)));
        }
        public static string ToStringList(this double[] elements, int decimals, string delimiter = ",")
        {
            return string.Join(delimiter, elements.Select(x=> ((float)Math.Round(x, decimals)).ToString()));
        }
        public static string Show(this Vector values, string title = "", string format = "g4")
            => Show(values.ToArray(), title, format);
        public static string Show(this double[] values, string title = "", string format = "g4")
        {
            StringBuilder sb = new StringBuilder(values.Length * 20 );
            if (!string.IsNullOrEmpty(title))
            {
                sb.AppendLine(title);
            }

            string[] parts = values.Select((x)=> x.ToString(format)).ToArray();
            const int minWidth = 3;
            int width = Math.Max(minWidth, parts.Max((s)=>s.Length));

            for (int i = 0; i<parts.Length; i++)
            {
                var item = $"| {parts[i].PadLeft(width)} |";
                sb.AppendLine(item);
            }
            sb.AppendLine();
            return sb.ToString();
        }
        public static string Show(this StackedVector values, string title = "", string format = "g4")
        {
            int partCount = values.Partitions.Count;
            StringBuilder sb = new StringBuilder( (values.Size+partCount-1) * 20 );
            if (!string.IsNullOrEmpty(title))
            {
                sb.AppendLine(title);
            }
            var parts = new List<string>();
            int index = 0;
            const int minWidth = 3;
            int width = minWidth;
            for (int partIndex = 0; partIndex<partCount; partIndex++)
            {
                for (int i = 0; i<values.Partitions[partIndex]; i++)
                {
                    string item = values.Elements[index].ToString(format);
                    width=Math.Max(width, item.Length);
                    parts.Add(item);
                    index++;
                }
                if (partIndex<partCount-1)
                {
                    parts.Add(string.Empty);
                }
            }
            var split = new string('-', width);

            for (int i = 0; i<parts.Count; i++)
            {
                if (parts[i].Length>0)
                {
                    sb.AppendLine($"| {parts[i].PadLeft(width)} |");
                }
                else
                {
                    sb.AppendLine($"| {split} |");
                }
            }
            sb.AppendLine();
            return sb.ToString();
        }
        public static string Show(this Matrix values, string title = "", string format = "g4")
            => Show(values.ToJaggedArray(), title, format);
        public static string Show(this double[,] values, string title = "", string format = "g4")
            => Show(Factory.ToJaggedArray(values), title, format);
        public static string Show(this double[][] values, string title = "", string format = "g4")
        {
            int n = values.Length;
            int m = n>0 ? values[0].Length : 0;
            StringBuilder sb = new StringBuilder(n * m * 20 );
            if (!string.IsNullOrEmpty(title))
            {
                sb.AppendLine(title);
            }
            string[][] parts = values.Select((row)=> row.Select(x=>x.ToString(format)).ToArray()).ToArray();
            const int minWidth = 3;
            int[] width = Enumerable.Repeat(minWidth, m).ToArray();
            for (int i = 0; i<n; i++)
            {
                string[] row = parts[i];
                for (int j = 0; j<row.Length; j++)
                {
                    width[j]=Math.Max(width[j], row[j].Length);
                }
            }

            for (int i = 0; i<n; i++)
            {
                string[] row = parts[i];
                sb.Append("| ");
                for (int j = 0; j<row.Length; j++)
                {
                    sb.Append($"{row[j].PadLeft(width[j])} ");
                }
                sb.AppendLine("|");
            }

            return sb.ToString();
        }
        public static string Show(this StackedMatrix values, string title = "", string format = "g4")
        {
            int rowPartCount = values.RowPartitions.Count;
            int colPartCount = values.ColPartitions.Count;
            int n = values.Rows;
            int m = values.Columns;
            StringBuilder sb = new StringBuilder((n * m + rowPartCount-1) * 20 );
            if (!string.IsNullOrEmpty(title))
            {
                sb.AppendLine(title);
            }
            List<string[]> parts = new List<string[]>();
            const int minWidth = 3;
            int[] width = Enumerable.Repeat(minWidth, m).ToArray();
            int rowIndex = 0;
            for (int rowPartIndex = 0; rowPartIndex<rowPartCount; rowPartIndex++)
            {
                for (int i = 0; i<values.RowPartitions[rowPartIndex]; i++)
                {
                    int colIndex = 0;
                    List<string> row = new List<string>(n+values.ColPartitions.Count);
                    for (int colPartIndex = 0; colPartIndex<colPartCount; colPartIndex++)
                    {
                        for (int j = 0; j<values.ColPartitions[colPartIndex]; j++)
                        {
                            string item = values.Elements[rowIndex, colIndex].ToString(format);
                            row.Add(item);
                            width[colIndex]=Math.Max(width[colIndex], item.Length);
                            colIndex++;
                        }
                        if (colPartIndex<colPartCount-1)
                        {
                            row.Add(string.Empty);
                        }
                    }
                    parts.Add(row.ToArray());
                    rowIndex++;
                }
                if (rowPartIndex<rowPartCount-1)
                {
                    int colIndex = 0;
                    List<string> row = new List<string>(n+values.ColPartitions.Count);
                    for (int colPartIndex = 0; colPartIndex<colPartCount; colPartIndex++)
                    {
                        for (int j = 0; j<values.ColPartitions[colPartIndex]; j++)
                        {
                            string item = new string('-', width[colIndex]);
                            row.Add(item);
                            colIndex++;
                        }
                        if (colPartIndex<colPartCount-1)
                        {
                            row.Add(string.Empty);
                        }
                    }
                    parts.Add(row.ToArray());
                }

            }

            for (int i = 0; i<parts.Count; i++)
            {
                var row = parts[i];
                sb.Append("| ");
                int colIndex = 0;
                for (int j = 0; j<row.Length; j++)
                {
                    if (row[j].Length>0)
                    {
                        sb.Append($"{row[j].PadLeft(width[colIndex++])} ");
                    }
                    else
                    {
                        sb.Append(" | ");
                    }
                }
                sb.AppendLine("|");
            }

            return sb.ToString();
        }

        public static string Combine(params string[] elements)
        {
            string[][] parts = new string[elements.Length][];
            int rows = 0;
            for (int i = 0; i<parts.Length; i++)
            {
                // [ "| a |" , "| b |" ]
                // [ "| c |" , "| d |" ]
                var items = new List<string>();
                var tr = new StringReader(elements[i]);
                while (tr.Peek() >= 0)
                {
                    items.Add( tr.ReadLine() );
                }
                parts[i]=items.ToArray();
                rows=Math.Max(rows, items.Count);
            }
            string[][] show = new string[rows][];

            LinearAlgebra.Factory.JaggedTranspose(parts, ref show, string.Empty);

            StringBuilder sb = new StringBuilder();
            for (int i = 0; i<show.Length; i++)
            {
                // [ "| a |" , "| c |" ]
                // [ "| b |" , "| d |" ]
                string[] row = show[i];
                for (int j = 0; j<row.Length; j++)
                {
                    sb.Append($"{row[j]} ");
                }
                sb.AppendLine();
            }

            return sb.ToString();
        }

        #endregion

    }
}
