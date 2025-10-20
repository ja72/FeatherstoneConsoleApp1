using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Featherstone
{
    public interface ITree<T> where T : ITree<T>
    {
        T Parent { get; }
        IReadOnlyList<T> Children { get; }

        bool IsRoot { get; }
        bool IsLeaf { get; }

        T[] GetMeAndAllAncestors();
    }
}
