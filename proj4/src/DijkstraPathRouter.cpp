#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <vector>
#include <any>
#include <limits>
#include <queue>
#include <algorithm> 

// SImplementation struct to hold all private members
struct CDijkstraPathRouter::SImplementation {
    using TVertexID = std::size_t; 
    static constexpr double NoPathExists = std::numeric_limits<double>::infinity();

    std::size_t nextVertexID = 0; // counter for assigning unique vertex IDs
    std::unordered_map<TVertexID, std::any> vertices; // map of vertex IDs to their tags
    std::unordered_map<TVertexID, std::vector<std::pair<TVertexID, double>>> adjacencyList; // adj list for the graph
};

// constructor
CDijkstraPathRouter::CDijkstraPathRouter() : DImplementation(std::make_unique<SImplementation>()) {}

// destructor
CDijkstraPathRouter::~CDijkstraPathRouter() = default;


std::size_t CDijkstraPathRouter::VertexCount() const noexcept { // ret # of vertices in the path router
    return DImplementation->vertices.size();
}

// adds a vertex w/  tag provided
CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    auto id = DImplementation->nextVertexID++; // assing  unique ID and incr the counter
    DImplementation->vertices[id] = tag; // store the vertex tag
    DImplementation->adjacencyList[id] = {}; // init an empty adj list for the vertex
    return id;
}

// gets tag of the vertex specified by id
std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    auto it = DImplementation->vertices.find(id);
    if (it != DImplementation->vertices.end()) {
        return it->second;
    }
    return std::any(); // ret empty any if the vertex ID  not found
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept { // adds an edge between src and dest vertices with a weight
    if (DImplementation->vertices.find(src) == DImplementation->vertices.end() ||
        DImplementation->vertices.find(dest) == DImplementation->vertices.end() ||
        weight < 0) {
        return false; // invalid vertices / negative weight
    }
    DImplementation->adjacencyList[src].push_back({dest, weight}); // add edge from src to dest
    if (bidir) {
        DImplementation->adjacencyList[dest].push_back({src, weight}); // add reverse edge if both ways (bidirectional)
    }
    return true;
}

// precompute function 
bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    return true; // no precomputation needed
}

// finds shortest path from src to dest using Dijkstras alg (we love dijkstras!!!!!!!!!!!)
double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    if (DImplementation->vertices.find(src) == DImplementation->vertices.end() ||
        DImplementation->vertices.find(dest) == DImplementation->vertices.end()) {
        path.clear(); // clear path vector
        return NoPathExists; 
    }

    // priority queue to store vertices and their distances
    using QueueElement = std::pair<double, TVertexID>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

    // map to store distances from src to each vertex
    std::unordered_map<TVertexID, double> distances;
    for (const auto &vertex : DImplementation->vertices) {
        distances[vertex.first] = NoPathExists; // instead of using infinity doing no path exists
    }
    distances[src] = 0.0;

    // map to store the previous vertex in the shortest path
    std::unordered_map<TVertexID, TVertexID> previous;

    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [currentDistance, currentVertex] = pq.top();
        pq.pop();

        if (currentVertex == dest) {
            break; // found the shortest path to dest
        }

        if (currentDistance > distances[currentVertex]) {
            continue; // skip if a shorter path to currentVertex has alr been found
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

    if (distances[dest] == NoPathExists) { 
        path.clear(); // clear the path vector if no path exists
        return NoPathExists; 
    }

    // remake the path
    path.clear(); // clear the path vector before reconstructing
    for (TVertexID at = dest; at != src; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end()); // reverse the path to get the correct order

    return distances[dest];
}