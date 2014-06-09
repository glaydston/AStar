namespace Romania
{
    /// <summary>
    /// A TNode is uniquely identified by its string Key.  A TNode also has a Data property of type object
    /// that can be used to store any extra information associated with the TNode.
    /// 
    /// 
    /// The TNode has a property of type AdjacencyList, which represents the node's neighbors.  To add a neighbor,
    /// the TNode class exposes an AddDirected() method, which adds a directed edge with an (optional) weight to
    /// some other TNode.  These methods are marked internal, and are called by the Graph class.
    /// </summary>
    public partial class Node
    {

        #region Public Properties

        /// <summary>
        /// Returns the TNode's Key.
        /// </summary>
        public string Key { get; private set; }


        /// <summary>
        /// Returns an AdjacencyList of the TNode's neighbors.
        /// </summary>
        public AdjacencyList Neighbors { get; private set; }

        /// <summary>
        /// Returns the TNode's Path Parent.
        /// </summary>
        public Node PathParent { get; set; }

        #endregion

        #region Constructors

        public Node(string key)
            : this(key, null)
        {
        }


        public Node(string key, AdjacencyList neighbors)
        {
            Key = key;

            if (neighbors == null)
            {
                Neighbors = new AdjacencyList();
            }
            else
            {
                Neighbors = neighbors;
            }
        }



        #endregion

        #region Public Methods

        #region Add Methods

        /// <summary>
        /// Adds an unweighted, directed edge from this node to the passed-in TNode n.
        /// </summary>
        internal void AddDirected(Node n)
        {
            AddDirected(new EdgeToNeighbor(n));
        }

        /// <summary>
        /// Adds a weighted, directed edge from this node to the passed-in TNode n.
        /// </summary>
        /// <param name="cost">The weight of the edge.</param>
        internal void AddDirected(Node n, int cost)
        {
            AddDirected(new EdgeToNeighbor(n, cost));
        }

        /// <summary>
        /// Adds an edge based on the data in the passed-in EdgeToNeighbor instance.
        /// </summary>
        internal void AddDirected(EdgeToNeighbor e)
        {
            Neighbors.Add(e);
        }

        #endregion

        public override string ToString()
        {
            return string.Format("Chave = {0} ",
                Key);
        }

        #endregion
    }
}
