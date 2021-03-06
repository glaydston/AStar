namespace ATPS
{
    /// <summary>
    /// EdgeToNeighbor represents an edge eminating from one <see cref="Node"/> to its neighbor.  The EdgeToNeighbor
    /// class, then, contains a reference to the neighbor and the weight of the edge.
    /// </summary>
    public class Vertices
    {
        #region Public Properties

        /// <summary>
        /// The weight of the edge.
        /// </summary>
        /// <remarks>A value of 0 would indicate that there is no weight, and is the value used when an unweighted
        /// edge is added via the <see cref="Grafo"/> class.</remarks>
        public virtual double Cost { get; private set; }

        /// <summary>
        /// The neighbor the edge is leading to.
        /// </summary>
        public virtual Node Neighbor { get; private set; }

        #endregion

        #region Constructors

        public Vertices(Node neighbor) : this(neighbor, 0)
        {

        }

        public Vertices(Node neighbor, double cost)
        {
            Cost = cost;
            Neighbor = neighbor;
        }

        #endregion

        #region Public Methods

        public override string ToString()
        {
            return string.Format("Vizinho = {0} | Custo = {1}", Neighbor.Key, Cost);
        }

        #endregion
    }
}
