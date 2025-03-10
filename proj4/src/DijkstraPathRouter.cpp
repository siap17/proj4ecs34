#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <vector>
#include <any>
#include <limits>
#include <queue>
#include <chrono>

// Define the SImplementation struct to hold all private members
struct CDijkstraPathRouter::SImplementation {
    using TVertexID = std::size_t; // Define TVertexID
    static constexpr double NoPathExists = std::numeric_limits<double>::infinity();

    std::size_t nextVertexID = 0; // Counter for assigning unique vertex IDs
    std::unordered_map<TVertexID, std::any> vertices; // Map of vertex IDs to their tags
    std::unordered_map<TVertexID, std::vector<std::pair<TVertexID, double>>> adjacencyList; // Adjacency list for the graph
};

// Constructor
CDijkstraPathRouter::CDijkstraPathRouter() : DImplementation(std::make_unique<SImplementation>()) {}

// Destructor
CDijkstraPathRouter::~CDijkstraPathRouter() = default;

// Returns the number of vertices in the path router
std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->vertices.size();
}

// Adds a vertex with the tag provided
CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    auto id = DImplementation->nextVertexID++; // Assign a unique ID and increment the counter
    DImplementation->vertices[id] = tag; // Store the vertex tag
    DImplementation->adjacencyList[id] = {}; // Initialize an empty adjacency list for the vertex
    return id;
}

// Gets the tag of the vertex specified by id
std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    auto it = DImplementation->vertices.find(id);
    if (it != DImplementation->vertices.end()) {
        return it->second;
    }
    return std::any(); // Return an empty any if the vertex ID is not found
}

// Adds an edge between src and dest vertices with a weight
bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    if (DImplementation->vertices.find(src) == DImplementation->vertices.end() ||
        DImplementation->vertices.find(dest) == DImplementation->vertices.end() ||
        weight < 0) {
        return false; // Invalid vertices or negative weight
    }
    DImplementation->adjacencyList[src].push_back({dest, weight}); // Add edge from src to dest
    if (bidir) {
        DImplementation->adjacencyList[dest].push_back({src, weight}); // Add reverse edge if bidirectional
    }
    return true;
}

// Precompute function (not used in this implementation)
bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    return true; // No precomputation needed
}

// Finds the shortest path from src to dest using Dijkstra's algorithm
double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    if (DImplementation->vertices.find(src) == DImplementation->vertices.end() ||
        DImplementation->vertices.find(dest) == DImplementation->vertices.end()) {
        return SImplementation::NoPathExists; // Invalid source or destination
    }

    // Priority queue to store vertices and their distances
    using QueueElement = std::pair<double, TVertexID>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

    // Map to store distances from src to each vertex
    std::unordered_map<TVertexID, double> distances;
    for (const auto &vertex : DImplementation->vertices) {
        distances[vertex.first] = std::numeric_limits<double>::infinity();
    }
    distances[src] = 0.0;

    // Map to store the previous vertex in the shortest path
    std::unordered_map<TVertexID, TVertexID> previous;

    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [currentDistance, currentVertex] = pq.top();
        pq.pop();

        if (currentVertex == dest) {
            break; // Found the shortest path to destination
        }

        if (currentDistance > distances[currentVertex]) {
            continue; // Skip if a shorter path to currentVertex has already been found
        }

        for (const auto &[neighbor, weight] : DImplementation->adjacencyList[currentVertex]) {
            double distanceThroughCurrent = currentDistance + weight;
            if (distanceThroughCurrent < distances[neighbor]) {
                distances[neighbor] = distanceThroughCurrent;
                previous[neighbor] = currentVertex;
                pq.push({distanceThroughCurrent, neighbor});
            }
        }
    }

    if (distances[dest] == std::numeric_limits<double>::infinity()) {
        return SImplementation::NoPathExists; // No path exists
    }

    // Reconstruct the path
    for (TVertexID at = dest; at != src; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());

    return distances[dest];
}