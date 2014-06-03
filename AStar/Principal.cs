using System;
using System.Collections.Generic;
using System.Linq;


namespace ATPS
{
    class Principal
    {
        static void Main(string[] args)
        {
            do
            {
                Grafo graph = new Grafo();

                FillGraphWithGridMap(graph);

                DistanceType distanceType = DistanceType.km;

                // Prints on the screen the distance from a city to its neighbors.
                // Used mainly for debug information.
                DistanceBetweenNodes(graph, DistanceType.km);

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
                                                    node1.Neighbors.Cast<Vertices>().Single(
                                                        etn => etn.Neighbor.Key == node2.Key).Cost;

                
                // Estimation/Heuristic function (Haversine distance)
                // It tells us the estimated distance between the last node on a proposed path and the destination node.
                Func<Node, double> haversineEstimation =
                    n => Haversine.Distance(n, destination, DistanceType.km);

                Caminho<Node> shortestPath = FindPath(start, destination, distance, haversineEstimation);

                Console.WriteLine("\nEste é o menor caminho baseado no algoritmo de busca A*:\n");

                // Prints the shortest path.
                foreach (Caminho<Node> path in shortestPath.Reverse())
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
        private static void DistanceBetweenNodes(Grafo graph, DistanceType distanceType)
        {
            // First we cast the Graph.Nodes which is a NodeList to type Node and then we order the list of Nodes by the Node Key
            // so that we get the list of cities in ascending order.
            foreach (Node n in graph.Nodes.Cast<Node>().OrderBy(n => n.Key))
            {
                // For each city neighbor we gets its information and print it on screen.
                foreach (Vertices etn in n.Neighbors)
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
        private static string GetDestinationCity(Grafo graph)
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
        private static string GetStartCity(Grafo graph)
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
        private static void FillGraphWithGridMap(Grafo graph)
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
        static public Caminho<TNode> FindPath<TNode>(
            TNode start,
            TNode destination,
            Func<TNode, TNode, double> distance,
            Func<TNode, double> estimate) where TNode : IVizinhos<TNode>
        {
            var closed = new HashSet<TNode>();

            var queue = new FilaPrioridade<double, Caminho<TNode>>();

            queue.Enqueue(0, new Caminho<TNode>(start));

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
        /// This method can be used to view the other paths inside the FilaPrioridade.
        /// </summary>
        /// <typeparam name="TNode">The Node type</typeparam>
        /// <param name="queue">The priority queue</param>
        /// <param name="estimate">Function which tells us the estimated distance between the last node on a proposed path and the
        /// destination node.</param>
        private static void ViewOtherPaths<TNode>(FilaPrioridade<double, Caminho<TNode>> queue, Func<TNode, double> estimate)
        {
            Console.WriteLine("\nCaminhos possíveis:\n");

            // The priority queue is composed of KeyValuePairs which has as key a double value (the TotalCost) and
            // has as Value a Queue which contains Paths.
            foreach (KeyValuePair<double, Queue<Caminho<TNode>>> kvp in queue)
            {
                // For each path in the Queue...
                foreach (Caminho<TNode> otherPath in kvp.Value)
                {
                    // Reverse the Caminho so that we get the order of the cities in a more meaningful way...
                    var otherPathReversed = otherPath.Cast<Caminho<Node>>().Reverse();

                    // Prints on screen the Cities that are part of this path.
                    foreach (Caminho<Node> path in otherPathReversed)
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

    sealed partial class Node : IVizinhos<Node>
    {
        public IEnumerable<Node> Neighbours
        {
            get
            {
                List<Node> nodes = new List<Node>();

                foreach (Vertices etn in Neighbors)
                {
                    nodes.Add(etn.Neighbor);
                }

                return nodes;
            }
        }
    }
}
