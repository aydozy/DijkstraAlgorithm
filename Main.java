import java.util.*;

//-----------------------------------------------------
//Title: Main Class
//Author: Ayda Nil Özyürek
//Description: to calculate the total costs of all vehicles entering the facility using Dijkstra.
//------------------------------------------------

public class Main {

	static class Node implements Comparable<Node> {
		int vertex;
		int distance;

		public Node(int vertex, int distance) {
			this.vertex = vertex;
			this.distance = distance;
		}

		@Override
		public int compareTo(Node other) {
			return distance < other.distance ? -1 : distance > other.distance ? 1 : 0;
		}

	}

	static class Edge {
		int vertex;
		int weight;

		public Edge(int vertex, int weight) {
			this.vertex = vertex;
			this.weight = weight;
		}
	}

	static ArrayList<List<Edge>> adj;
	static final int INF = Integer.MAX_VALUE;
	static int N, M, F;
	static int[] capacity, distance;
	static boolean[] visited;

	public static void main(String[] args) {

		Scanner sc = new Scanner(System.in);

		N = sc.nextInt(); // number of nodes
		M = sc.nextInt(); // number of edges
		F = sc.nextInt(); // parking fee

		// -------------------------------------------------------
		// Capacity: each node's capacity can be stored in this array. The array size is
		// N, which means that it's composed of the same amount of elements as a single
		// node. In this array, the capacity of each node is set to the corresponding
		// index.
		//
		// Distance: To store distance for each node, this array is used. The size of
		// this array remains unchanged at N, the distance between each node is kept in
		// its corresponding index. To begin with, the distance of every node is
		// established as INF.
		//
		// Visited: To keep track of the visit status of each node, this array is used.
		// The size is N again and each node's visit status will be checked in the
		// corresponding index to this array. The visit status of each node is initially
		// set to false.
		// -------------------------------------------------------

		capacity = new int[N];
		distance = new int[N];
		visited = new boolean[N];

		adj = new ArrayList<>();

		// -------------------------------------------------------
		// The structure of the graph is built by using this code block for storage
		// capacities, start distances, visit states and edges of nodes.
		// -------------------------------------------------------

		for (int i = 0; i < N; i++) {
			capacity[i] = sc.nextInt();
			distance[i] = INF;
			visited[i] = false;
			adj.add(new ArrayList<>());
		}
		for (int i = 0; i < M; i++) {

			int U = sc.nextInt() - 1; // It is reduced by 1 to be suitable for indexing the nodes.
			int V = sc.nextInt() - 1; // It is reduced by 1 to be suitable for indexing the nodes.
			int W = sc.nextInt();
			adj.get(U).add(new Edge(V, W));
			adj.get(V).add(new Edge(U, W));
		}

		// -------------------------------------------------------
		// This code uses Dijkstra's algorithms to determine and display the parking fee
		// for each vehicle that enters a parking facility.
		// -------------------------------------------------------

		int K = sc.nextInt(); // number of vehicles enter the parking facility
		for (int i = 0; i < K; i++) {
			int cost = dijkstra();
			System.out.print(cost + " ");
		}
	}

	// -------------------------------------------------------
	// Dijkstra's algorithm, taking into consideration distance, capacity and
	// parking charges associated with each node, shall be performed by dijkstra()
	// to find the shortest path in a graph. Taking into account the distance
	// travelled and the parking fee, it provides for the minimum cost of parking in
	// the available slots.
	// -------------------------------------------------------

	static int dijkstra() {

		PriorityQueue<Node> pq = new PriorityQueue<>();
		pq.add(new Node(0, 0));

		Arrays.fill(distance, INF);
		distance[0] = 0;

		while (!pq.isEmpty()) {
			Node node = pq.poll();
			int u = node.vertex;
			if (visited[u])
				continue;
			visited[u] = true;
			for (Edge e : adj.get(u)) {
				int v = e.vertex;
				int w = e.weight;
				if (visited[v])
					continue;
				if (distance[v] > distance[u] + w) {
					distance[v] = distance[u] + w;
					pq.add(new Node(v, distance[v]));
				}
			}
		}

		int minCost = INF;
		int slot = -1;
		for (int i = 0; i < N; i++) {
			if (distance[i] != INF && capacity[i] > 0) {
				int cost = F + distance[i];
				// minCost = Math.min(minCost, cost);
				if (cost < minCost) {
					minCost = cost;
					slot = i;
				}
			}
		}

		/*
		 * for (int i = 0; i < N; i++) { if (distance[i] != INF && capacity[i] > 0) {
		 * capacity[i]--; break; } } Arrays.fill(visited, false); return minCost;
		 */

		// -------------------------------------------------------
		// This part of the code updates a selected parking slot's capacity, resets
		// visited status at nodes and returns its minimum cost if any valid route or
		// parking slots are found.
		// -------------------------------------------------------
		if (slot != -1)

		{
			capacity[slot]--;
		}
		Arrays.fill(visited, false);
		return minCost != INF ? minCost : -1;

	}
}
