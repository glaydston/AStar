using System;
using System.Collections;

namespace ATPS
{
	/// <summary>
	/// AdjacencyList maintains a list of neighbors for a particular <see cref="Node"/>.  It is derived from CollectionBase
	/// and provides a strongly-typed collection of <see cref="Vertices"/> instances.
	/// </summary>
	public class ListaAdjacente : CollectionBase
	{
		/// <summary>
		/// Adds a new <see cref="Vertices"/> instance to the AdjacencyList.
		/// </summary>
		/// <param name="e">The <see cref="Vertices"/> instance to add.</param>
		protected internal virtual void Add(Vertices e)
		{
			base.InnerList.Add(e);
		}

		/// <summary>
		/// Returns a particular <see cref="Vertices"/> instance by index.
		/// </summary>
		public virtual Vertices this[int index]
		{
			get { return (Vertices) base.InnerList[index]; }
			set { base.InnerList[index] = value; }
		}
	}
}
