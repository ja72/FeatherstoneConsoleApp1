using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using JA.Dynamics;

namespace JA
{
    public interface ITree<T> where T : ITree<T>
    {
        T Parent { get; }
        IReadOnlyList<T> Children { get; }

        bool IsRoot { get; }
        bool IsLeaf { get; }

        R Traverse<R>(R initialValue, Func<T, R> operation);
        void Traverse(Action<T> operation);

    }
}
