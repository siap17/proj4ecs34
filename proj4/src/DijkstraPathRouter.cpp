#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <any>

// Placeholder for TVertexID and other types that may need to be defined in your project
using TVertexID = int;
using TEdgeWeight = double;

// Structure to store an edge (destination vertex, weight)
struct Edge {
    TVertexID dest;
    TEdgeWeight weight;
};

// Class to implement Dijkstra's Algorithm for shortest path routing
class CDijkstraPathRouter {
public:
    // Constructor
    CDijkstraPathRouter() {
        // Initialization if needed
    }

    // Destructor
    ~CDijkstraPathRouter() {
        // Cleanup if necessary
    }

    // Function to add a vertex
    TVertexID AddVertex(std::any tag) noexcept {
        TVertexID id = vertices.size(); // Assign a new ID based on current size
        vertices.push_back(tag);
        adjacencyList.emplace(id, std::vector<Edge>());
        return id;
    }

    // Function to get a vertex tag by ID
    std::any GetVertexTag(TVertexID id) const noexcept {
        if (id < vertices.size()) {
            return vertices[id];
        }
        return std::any(); // Return empty if ID is invalid
    }

    // Function to add an edge between two vertices with a weight
    bool AddEdge(TVertexID src, TVertexID dest, TEdgeWeight weight, bool bidir = false) noexcept {
        if (src >= vertices.size() || dest >= vertices.size() || weight < 0) {
            return false; // Invalid edge
        }

        adjacencyList[src].push_back(Edge{dest, weight});
        if (bidir) {
            adjacencyList[dest].push_back(Edge{src, weight});
        }
        return true;
    }

    // Function to find the shortest path from src to dest
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
        // Dijkstra's Algorithm
        std::unordered_map<TVertexID, double> dist;
        std::unordered_map<TVertexID, TVertexID> prev;
        std::priority_queue<std::pair<double, TVertexID>, 
                            std::vector<std::pair<double, TVertexID>>, 
                            std::greater<std::pair<double, TVertexID>>> pq;

        // Initialize distances and priority queue
        for (const auto& vertex : adjacencyList) {
            dist[vertex.first] = std::numeric_limits<double>::infinity();
        }
        dist[src] = 0;
        pq.push({0, src});

        // Main loop of Dijkstra's algorithm
        while (!pq.empty()) {
            TVertexID u = pq.top().second;
            pq.pop();

            if (u == dest) {
                break; // Found the shortest path to destination
            }

            for (const auto& edge : adjacencyList[u]) {
                TVertexID v = edge.dest;
                double weight = edge.weight;
                double alt = dist[u] + weight;
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    pq.push({alt, v});
                }
            }
        }

        // Reconstruct the shortest path
        path.clear();
        TVertexID u = dest;
        while (prev.find(u) != prev.end()) {
            path.push_back(u);
            u = prev[u];
        }
        std::reverse(path.begin(), path.end()); // Reverse to get correct order
        return dist[dest];
    }

    // Function to get the number of vertices
    std::size_t VertexCount() const noexcept {
        return vertices.size();
    }

private:
    std::vector<std::any> vertices; // Stores vertex tags (could be coordinates, names, etc.)
    std::unordered_map<TVertexID, std::vector<Edge>> adjacencyList; // Adjacency list to store edges
};