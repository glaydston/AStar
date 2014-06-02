using System;
using System.Collections.Generic;
using System.Linq;


namespace AStar
{
    class AStar
    {
        static void Main(string[] args)
        {
            do
            {
                // Creating the Graph...
                Graph graph = new Graph();

                FillGraphWithGridMap(graph);

                DistanceType distanceType = DistanceType.km;

                // Using distance from latitude and longitude
                //FillGraphWithEarthMap(graph, distanceType);

                // Prints on the screen the distance from a city to its neighbors.
                // Used mainly for debug information.
                // DistanceBetweenNodes(graph, DistanceType.Kilometers);

                Console.WriteLine("Essas são as cidades que você pode escolher como Origem e Destino na Roménia: \n");

                // Prints on screen the cities that you can choose as Start and Destination.
                foreach (Node n in graph.Nodes.Cast<Node>().OrderBy(n => n.Key))
                {
                    Console.WriteLine(n.Key);
                }

                string startCity = GetStartCity(graph);

                string destinationCity = GetDestinationCity(graph);

                Node start = graph.Nodes[startCity];

                Node destination = graph.Nodes[destinationCity];

                // Function which tells us the exact distance between two neighbours.
                Func<Node, Node, double> distance = (node1, node2) =>
                                                    node1.Neighbors.Cast<EdgeToNeighbor>().Single(
                                                        etn => etn.Neighbor.Key == node2.Key).Cost;

                // Estimation/Heuristic function (Manhattan distance)
                // It tells us the estimated distance between the last node on a proposed path and the destination node.
                //Func<Node, double> manhattanEstimation = n => Math.Abs(n.X - destination.X) + Math.Abs(n.Y - destination.Y);

                // Estimation/Heuristic function (Haversine distance)
                // It tells us the estimated distance between the last node on a proposed path and the destination node.
                Func<Node, double> haversineEstimation =
                    n => Haversine.Distance(n, destination, DistanceType.km);

                //Path<Node> shortestPath = FindPath(start, destination, distance, manhattanEstimation);
                Path<Node> shortestPath = FindPath(start, destination, distance, haversineEstimation);

                Console.WriteLine("\nEste é o menor caminho baseado no algoritmo de busca A*:\n");

                // Prints the shortest path.
                foreach (Path<Node> path in shortestPath.Reverse())
                {
                    if (path.PreviousSteps != null)
                    {
                        Console.WriteLine(string.Format("De {0, -15}  para  {1, -15} -> Custo total = {2:#.###} {3}",
                                          path.PreviousSteps.LastStep.Key, path.LastStep.Key, path.TotalCost, distanceType));
                    }
                }

                Console.Write("\nDeseja buscar novamente? Sim/Não? ");
            }
            while (Console.ReadLine().ToLower() == "sim");
        }

        /// <summary>
        /// Prints on screen the distance from a city to its neighbors.
        /// </summary>
        /// <param name="graph">The Graph</param>
        /// <param name="distanceType">The distance type: KM or Miles</param>
        private static void DistanceBetweenNodes(Graph graph, DistanceType distanceType)
        {
            // First we cast the Graph.Nodes which is a NodeList to type Node and then we order the list of Nodes by the Node Key
            // so that we get the list of cities in ascending order.
            foreach (Node n in graph.Nodes.Cast<Node>().OrderBy(n => n.Key))
            {
                // For each city neighbor we gets its information and print it on screen.
                foreach (EdgeToNeighbor etn in n.Neighbors)
                {
                    Console.WriteLine("Distância de {0} para {1} é -> {2:#.##} {3}", n.Key, etn.Neighbor.Key, etn.Cost, distanceType);
                }
            }
        }

        /// <summary>
        /// Gets the Destination city.
        /// </summary>
        /// <param name="graph">The Graph</param>
        /// <returns>Name of Destination city</returns>
        private static string GetDestinationCity(Graph graph)
        {
            string destinationCity;
            do
            {
                Console.Write("\nInforme a cidade de Destino: ");

                destinationCity = Console.ReadLine();
            }
            while (!graph.Nodes.ContainsKey(destinationCity));
            return destinationCity;
        }

        /// <summary>
        /// Gets the Start city.
        /// </summary>
        /// <param name="graph">The Graph</param>
        /// <returns>Name of Start city</returns>
        private static string GetStartCity(Graph graph)
        {
            string startCity;
            do
            {
                Console.Write("\nInforme a cidade de Origem: ");

                startCity = Console.ReadLine();
            }
            while (!graph.Nodes.ContainsKey(startCity));
            return startCity;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="graph"></param>
        private static void FillGraphWithGridMap(Graph graph)
        {
            // Vertexes (nome, data, x, y)
            graph.AddNode("Arad", null, 46, -114);
            graph.AddNode("Bucharest", null, 452, -329);
            graph.AddNode("Fagaras", null, 329, -169);
            graph.AddNode("Giurgiu", null, 418, -404);
            graph.AddNode("Lugoj", null, 142, -263);
            graph.AddNode("Mehadia", null, 146, -314);
            graph.AddNode("Oradea", null, 98, -9);
            graph.AddNode("Pitesti", null, 347, -278);
            graph.AddNode("Rimnicu Vilcea", null, 232, -221);
            graph.AddNode("Sibiu", null, 198, -158);
            graph.AddNode("Timisoara", null, 49, -221);
            graph.AddNode("Zerind", null, 68, -61);

            // Edges

            // Arad <-> Zerind
            graph.AddUndirectedEdge("Arad", "Zerind", 75);
            // Arad <-> Timisoara
            graph.AddUndirectedEdge("Arad", "Timisoara", 118);
            // Arad <-> Sibiu
            graph.AddUndirectedEdge("Arad", "Sibiu", 140);

            // Bucharest <-> Giurgiu
            graph.AddUndirectedEdge("Bucharest", "Giurgiu", 90);
            // Bucharest <-> Pitesti
            graph.AddUndirectedEdge("Bucharest", "Pitesti", 101);
            // Bucharest <-> Fagaras
            graph.AddUndirectedEdge("Bucharest", "Fagaras", 211);

            // Fagaras <-> Sibiu
            graph.AddUndirectedEdge("Fagaras", "Sibiu", 90);

            // Lugoj <-> Mehadia
            graph.AddUndirectedEdge("Lugoj", "Mehadia", 70);
            // Lugoj <-> Timisoara
            graph.AddUndirectedEdge("Lugoj", "Timisoara", 111);

            // Mehadia <-> Giurgiu
            graph.AddUndirectedEdge("Mehadia", "Giurgiu", 200);

            // Oradea <-> Zerind
            graph.AddUndirectedEdge("Oradea", "Zerind", 71);
            // Oradea <-> Sibiu
            graph.AddUndirectedEdge("Oradea", "Sibiu", 151);

            // Pitesti <-> Rimnicu Vilcea
            graph.AddUndirectedEdge("Pitesti", "Rimnicu Vilcea", 97);

            // Rimnicu Vilcea <-> Sibiu
            graph.AddUndirectedEdge("Rimnicu Vilcea", "Sibiu", 80);

        }

        /// <summary>
        /// Fills a Graph with Romania map information.
        /// The Graph contains Nodes that represents Cities of Romania.
        /// Each Node has as its key the City name and its Latitude and Longitude associated information.
        /// Nodes are vertexes in the Graph. Connections between Nodes are edges.
        /// </summary>
        /// <param name="graph">The Graph to be filled</param>
        /// <param name="distanceType">The DistanceType (KM or Miles) between neighbor cities</param>
        private static void FillGraphWithEarthMap(Graph graph, DistanceType distanceType)
        {
            // 12 Vertexes
            Node arad = new Node("Arad", null, 46.1792414, 21.3150154); // Creating a Node(key, data, latitude, longitude)
            graph.AddNode(arad); // Adding the Node to the Graph...

            Node bucharest = new Node("Bucharest", null, 44.4479237, 26.097879);
            graph.AddNode(bucharest);

            Node fagaras = new Node("Fagaras", null, 45.843342, 24.977871);
            graph.AddNode(fagaras);

            Node giurgiu = new Node("Giurgiu", null, 43.8959986, 25.9550199);
            graph.AddNode(giurgiu);

            Node lugoj = new Node("Lugoj", null, 45.688011, 21.9161);
            graph.AddNode(lugoj);

            Node mehadia = new Node("Mehadia", null, 44.906575, 22.360437);
            graph.AddNode(mehadia);

            Node oradea = new Node("Oradea", null, 47.06094, 21.9276655);
            graph.AddNode(oradea);

            Node pitesti = new Node("Pitesti", null, 44.858801, 24.8666793);
            graph.AddNode(pitesti);

            Node rimnicuVilcea = new Node("Rimnicu Vilcea", null, 45.110039, 24.382641);
            graph.AddNode(rimnicuVilcea);

            Node sibiu = new Node("Sibiu", null, 45.7931069, 24.1505932);
            graph.AddNode(sibiu);

            Node timisoara = new Node("Timisoara", null, 45.7479372, 21.2251759);
            graph.AddNode(timisoara);

            Node zerind = new Node("Zerind", null, 46.6247847, 21.5170587);
            graph.AddNode(zerind);

            // 20 Edges
            // Arad <-> Sibiu
            graph.AddUndirectedEdge(arad, sibiu, Haversine.Distance(arad, sibiu, distanceType));
            // Arad <-> Timisoara
            graph.AddUndirectedEdge(arad, timisoara, Haversine.Distance(arad, timisoara, distanceType));
            // Arad <-> Zerind
            graph.AddUndirectedEdge(arad, zerind, Haversine.Distance(arad, zerind, distanceType));

            // Bucharest <-> Fagaras
            graph.AddUndirectedEdge(bucharest, fagaras, Haversine.Distance(bucharest, fagaras, distanceType));
            // Bucharest <-> Giurgiu
            graph.AddUndirectedEdge(bucharest, giurgiu, Haversine.Distance(bucharest, giurgiu, distanceType));
            // Bucharest <-> Pitesti
            graph.AddUndirectedEdge(bucharest, pitesti, Haversine.Distance(bucharest, pitesti, distanceType));

            // Fagaras <-> Pitesti
            graph.AddUndirectedEdge(fagaras, pitesti, Haversine.Distance(fagaras, pitesti, distanceType));
            // Fagaras <-> Sibiu
            graph.AddUndirectedEdge(fagaras, sibiu, Haversine.Distance(fagaras, sibiu, distanceType));

            // Lugoj <-> Mehadia
            graph.AddUndirectedEdge(lugoj, mehadia, Haversine.Distance(lugoj, mehadia, distanceType));
            // Lugoj <-> Rimnicu Vilcea
            graph.AddUndirectedEdge(lugoj, rimnicuVilcea, Haversine.Distance(lugoj, rimnicuVilcea, distanceType));
            // Lugoj <-> Sibiu
            graph.AddUndirectedEdge(lugoj, sibiu, Haversine.Distance(lugoj, sibiu, distanceType));
            // Lugoj <-> Timisoara
            graph.AddUndirectedEdge(lugoj, timisoara, Haversine.Distance(lugoj, timisoara, distanceType));
            // Lugoj <-> Zerind
            graph.AddUndirectedEdge(lugoj, zerind, Haversine.Distance(lugoj, zerind, distanceType));

            // Mehadia <-> Rimnicu Vilcea
            graph.AddUndirectedEdge(mehadia, rimnicuVilcea, Haversine.Distance(mehadia, rimnicuVilcea, distanceType));
            // Mehadia <-> Timisoara
            graph.AddUndirectedEdge(mehadia, timisoara, Haversine.Distance(mehadia, timisoara, distanceType));
            // Mehadia <-> Giurgiu
            graph.AddUndirectedEdge(mehadia, giurgiu, Haversine.Distance(mehadia, giurgiu, distanceType));

            // Oradea <-> Sibiu
            graph.AddUndirectedEdge(oradea, sibiu, Haversine.Distance(oradea, sibiu, distanceType));
            // Oradea <-> Zerind
            graph.AddUndirectedEdge(oradea, zerind, Haversine.Distance(oradea, zerind, distanceType));

            // Pitesti <-> Rimnicu Vilcea
            graph.AddUndirectedEdge(pitesti, rimnicuVilcea, Haversine.Distance(pitesti, rimnicuVilcea, distanceType));

            // Rimnicu Vilcea <-> Sibiu
            graph.AddUndirectedEdge(rimnicuVilcea, sibiu, Haversine.Distance(rimnicuVilcea, sibiu, distanceType));
        }

        /// <summary>
        /// This is the method responsible for finding the shortest path between a Start and Destination cities using the A*
        /// search algorithm.
        /// </summary>
        /// <typeparam name="TNode">The Node type</typeparam>
        /// <param name="start">Start city</param>
        /// <param name="destination">Destination city</param>
        /// <param name="distance">Function which tells us the exact distance between two neighbours.</param>
        /// <param name="estimate">Function which tells us the estimated distance between the last node on a proposed path and the
        /// destination node.</param>
        /// <returns></returns>
        static public Path<TNode> FindPath<TNode>(
            TNode start,
            TNode destination,
            Func<TNode, TNode, double> distance,
            Func<TNode, double> estimate) where TNode : IHasNeighbours<TNode>
        {
            var closed = new HashSet<TNode>();

            var queue = new PriorityQueue<double, Path<TNode>>();

            queue.Enqueue(0, new Path<TNode>(start));

            while (!queue.IsEmpty)
            {
                var path = queue.Dequeue();

                if (closed.Contains(path.LastStep))
                    continue;

                if (path.LastStep.Equals(destination))
                    return path;

                closed.Add(path.LastStep);

                foreach (TNode n in path.LastStep.Neighbours)
                {
                    double d = distance(path.LastStep, n);

                    var newPath = path.AddStep(n, d);

                    queue.Enqueue(newPath.TotalCost + estimate(n), newPath);
                }

                ViewOtherPaths(queue, estimate);
            }

            return null;
        }

        /// <summary>
        /// This method can be used to view the other paths inside the PriorityQueue.
        /// </summary>
        /// <typeparam name="TNode">The Node type</typeparam>
        /// <param name="queue">The priority queue</param>
        /// <param name="estimate">Function which tells us the estimated distance between the last node on a proposed path and the
        /// destination node.</param>
        private static void ViewOtherPaths<TNode>(PriorityQueue<double, Path<TNode>> queue, Func<TNode, double> estimate)
        {
            Console.WriteLine("\nCaminhos possíveis:\n");

            // The priority queue is composed of KeyValuePairs which has as key a double value (the TotalCost) and
            // has as Value a Queue which contains Paths.
            foreach (KeyValuePair<double, Queue<Path<TNode>>> kvp in queue)
            {
                // For each path in the Queue...
                foreach (Path<TNode> otherPath in kvp.Value)
                {
                    // Reverse the Path so that we get the order of the cities in a more meaningful way...
                    var otherPathReversed = otherPath.Cast<Path<Node>>().Reverse();

                    // Prints on screen the Cities that are part of this path.
                    foreach (Path<Node> path in otherPathReversed)
                    {
                        if (path.PreviousSteps != null)
                        {
                            Console.WriteLine(string.Format("De {0, -14} para {1, -14} -> Custo total = {2:#.###} {3}",
                                              path.PreviousSteps.LastStep.Key, path.LastStep.Key, path.TotalCost, DistanceType.km));
                        }
                    }

                    // Get the estimation cost of the other possible path.
                    double otherPathEstimation = estimate(otherPath.LastStep);

                    // Prints on the screen the relevant information so that it gets easier to debug the code and see how
                    // the A* search algorithm really does the job...
                    Console.WriteLine("Estimativa          = {0:0.###} {1}", otherPathEstimation, DistanceType.km);
                    Console.WriteLine("Custo do caminho = {0:0.###} {1} = (Custo total + Estimativa)", kvp.Key, DistanceType.km);
                }

                Console.WriteLine();
            }
        }
    }

    sealed partial class Node : IHasNeighbours<Node>
    {
        public IEnumerable<Node> Neighbours
        {
            get
            {
                List<Node> nodes = new List<Node>();

                foreach (EdgeToNeighbor etn in Neighbors)
                {
                    nodes.Add(etn.Neighbor);
                }

                return nodes;
            }
        }
    }
}
