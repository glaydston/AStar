using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ATPS
{
    interface IVizinhos<N>
    {
        IEnumerable<N> Neighbours { get; }
    }
}
