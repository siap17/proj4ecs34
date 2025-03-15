#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <sstream>
#include <iomanip>
#include "StreetMap.h"
#include "BusSystem.h"
#include "PathRouter.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include "StringUtils.h"
#include "BusSystemIndexer.h"
#include "DijkstraTransportationPlanner.h"

struct CDijkstraTransportationPlanner::SImplementation {
    // Type aliases for readability
    using TVertexID = std::size_t; 
    static constexpr double NoPath = std::numeric_limits<double>::infinity(); 

    // Configuration and map data
    std::shared_ptr<SConfiguration> configuration;
    std::shared_ptr<CStreetMap> streetMap;
    std::vector<std::shared_ptr<CStreetMap::SNode>> sortedNodes;

    // Vertex and node mapping
    std::unordered_map<TNodeID, TVertexID> nodeToVertex;
    std::unordered_map<TVertexID, TNodeID> vertexToNode;
    std::unordered_map<TVertexID, std::vector<std::pair<TVertexID, double>>> adjacencyList;

    // Bus system helpers
    std::unordered_map<TNodeID, TNodeID> closestBusStops;

    // Initialization and graph building
    SImplementation(std::shared_ptr<SConfiguration> config) : configuration(config) {
        if (!config || !config->StreetMap()) {
            throw std::invalid_argument("Invalid configuration or street map");
        }

        streetMap = config->StreetMap();

        // Populate and sort nodes
        for (size_t i = 0; i < streetMap->NodeCount(); ++i) {
            auto node = streetMap->NodeByIndex(i);
            if (node) {
                sortedNodes.push_back(node);
            }
        }

        std::sort(sortedNodes.begin(), sortedNodes.end(), 
            [](const auto& a, const auto& b) { return a->ID() < b->ID(); });

        // Build vertex mappings
        buildVertexMappings();

        // Construct graph
        buildGraph();

        // Precompute bus stops
        precomputeBusStops();
    }

    void buildVertexMappings() {
        for (size_t i = 0; i < sortedNodes.size(); ++i) {
            TVertexID vertexId = addVertex(sortedNodes[i]);
            nodeToVertex[sortedNodes[i]->ID()] = vertexId;
            vertexToNode[vertexId] = sortedNodes[i]->ID();
        }
    }

    void buildGraph() {
        for (size_t i = 0; i < streetMap->WayCount(); ++i) {
            auto way = streetMap->WayByIndex(i);
            if (!way || !isWayTraversable(way)) continue;

            processWayNodes(way);
        }
    }

    void processWayNodes(const std::shared_ptr<CStreetMap::SWay>& way) {
        bool isOneWay = checkOneWay(way);

        for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
            TNodeID node1ID = way->GetNodeID(j);
            TNodeID node2ID = way->GetNodeID(j+1);

            // Verify nodes exist in our mapping
            if (nodeToVertex.find(node1ID) == nodeToVertex.end() || 
                nodeToVertex.find(node2ID) == nodeToVertex.end()) {
                continue;
            }

            TVertexID v1 = nodeToVertex[node1ID];
            TVertexID v2 = nodeToVertex[node2ID];

            auto node1 = getNodeByID(node1ID);
            auto node2 = getNodeByID(node2ID);

            if (!node1 || !node2) continue;

            double distance = calculateDistance(node1, node2);

            // Add edges
            addEdge(v1, v2, distance);
            if (!isOneWay) {
                addEdge(v2, v1, distance);
            }
        }
    }

    void precomputeBusStops() {
        auto busSystem = configuration->BusSystem();
        if (!busSystem) return;

        for (size_t i = 0; i < busSystem->StopCount(); ++i) {
            auto stop = busSystem->StopByIndex(i);
            if (!stop) continue;

            TNodeID stopNodeID = stop->NodeID();
            TNodeID closestNodeID = findClosestNode(stopNodeID);

            if (closestNodeID != 0) {
                closestBusStops[stopNodeID] = closestNodeID;
            }
        }
    }

    TNodeID findClosestNode(TNodeID stopNodeID) {
        auto stopNode = getNodeByID(stopNodeID);
        if (!stopNode) return 0;

        double minDistance = NoPath;
        TNodeID closestNodeID = 0;

        for (const auto& node : sortedNodes) {
            double distance = calculateDistance(stopNode, node);
            if (distance < minDistance) {
                minDistance = distance;
                closestNodeID = node->ID();
            }
        }

        return closestNodeID;
    }

    // Helper methods
    TVertexID addVertex(std::shared_ptr<CStreetMap::SNode> node) {
        TVertexID id = adjacencyList.size();
        adjacencyList[id] = {};
        return id;
    }

    void addEdge(TVertexID from, TVertexID to, double weight) {
        adjacencyList[from].push_back({to, weight});
    }

    bool isWayTraversable(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (!way) return false;
        try {
            std::string highway = std::any_cast<std::string>(way->GetAttribute("highway"));
            return true;
        } catch (...) {
            return false;
        }
    }

    bool checkOneWay(const std::shared_ptr<CStreetMap::SWay>& way) const {
        try {
            std::string oneway = std::any_cast<std::string>(way->GetAttribute("oneway"));
            return oneway == "yes" || oneway == "true" || oneway == "1";
        } catch (...) {
            return false;
        }
    }

    std::shared_ptr<CStreetMap::SNode> getNodeByID(TNodeID id) const {
        auto it = std::find_if(sortedNodes.begin(), sortedNodes.end(), 
            [id](const auto& node) { return node->ID() == id; });
        return it != sortedNodes.end() ? *it : nullptr;
    }

    double calculateDistance(const std::shared_ptr<CStreetMap::SNode>& node1, 
                             const std::shared_ptr<CStreetMap::SNode>& node2) const {
        if (!node1 || !node2) return NoPath;

        double distanceInMiles = SGeographicUtils::HaversineDistanceInMiles(
            node1->Location(), 
            node2->Location()
        );
        
        const double MetersPerMile = 1609.344;
        return std::max(0.0, distanceInMiles * MetersPerMile);
    }

    // Path Finding Methods
    double findShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID>& path) {
        path.clear();
        if (src == dest) {
            path.push_back(src);
            return 0.0;
        }

        // Verify nodes exist in our mapping
        if (nodeToVertex.find(src) == nodeToVertex.end() || 
            nodeToVertex.find(dest) == nodeToVertex.end()) {
            return 1.0;
        }

        TVertexID srcVertex = nodeToVertex[src];
        TVertexID destVertex = nodeToVertex[dest];

        std::unordered_map<TVertexID, double> distances;
        std::unordered_map<TVertexID, TVertexID> previous;
        
        // Priority queue for Dijkstra's algorithm
        using QueueElement = std::pair<double, TVertexID>;
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

        // Initialize distances
        for (const auto& [vertex, _] : adjacencyList) {
            distances[vertex] = NoPath;
        }
        distances[srcVertex] = 0.0;
        pq.push({0.0, srcVertex});

        bool pathFound = false;
        while (!pq.empty()) {
            auto [currentDist, currentVertex] = pq.top();
            pq.pop();

            if (currentVertex == destVertex) {
                pathFound = true;
                break;
            }

            if (currentDist > distances[currentVertex]) continue;

            for (const auto& [neighbor, weight] : adjacencyList[currentVertex]) {
                double newDist = currentDist + weight;
                
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    previous[neighbor] = currentVertex;
                    pq.push({newDist, neighbor});
                }
            }
        }

        // Verify path exists
        if (!pathFound || distances[destVertex] == NoPath || 
            previous.find(destVertex) == previous.end()) {
            return 1.0;
        }

        // Reconstruct path
        std::vector<TVertexID> vertexPath;
        for (TVertexID at = destVertex; at != srcVertex; at = previous[at]) {
            vertexPath.push_back(at);
        }
        vertexPath.push_back(srcVertex);
        std::reverse(vertexPath.begin(), vertexPath.end());

        // Convert to node IDs
        for (TVertexID vertex : vertexPath) {
            path.push_back(vertexToNode[vertex]);
        }

        return distances[destVertex];
    }

    double findFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
        path.clear();
        if (src == dest) {
            path.push_back({ETransportationMode::Walk, src});
            return 0.0;
        }

        // Check node existence
        if (nodeToVertex.find(src) == nodeToVertex.end() || 
            nodeToVertex.find(dest) == nodeToVertex.end()) {
            return 1.0;
        }

        // Find walking path
        std::vector<TNodeID> nodePath;
        double totalTime = findShortestPath(src, dest, nodePath);
        
        if (totalTime == 1.0 || nodePath.empty()) {
            return 1.0;
        }

        // Path generation
        ETransportationMode prevMode = ETransportationMode::Walk;
        path.push_back({prevMode, nodePath[0]});

        for (size_t i = 1; i < nodePath.size(); ++i) {
            auto prevNode = getNodeByID(nodePath[i-1]);
            auto currNode = getNodeByID(nodePath[i]);
            
            if (!prevNode || !currNode) continue;

            double distance = calculateDistance(prevNode, currNode) / 1609.344; // Miles
            double walkTime = distance / configuration->WalkSpeed();
            double bikeTime = distance / configuration->BikeSpeed();

            // Mode selection
            ETransportationMode mode = (bikeTime < walkTime) 
                ? ETransportationMode::Bike 
                : ETransportationMode::Walk;

            // Update or add path step
            if (mode == prevMode) {
                path.back().second = nodePath[i];
            } else {
                path.push_back({mode, nodePath[i]});
            }

            prevMode = mode;
        }

        return totalTime;
    }

    // Utility methods
    std::string formatLocation(const std::shared_ptr<CStreetMap::SNode>& node) const {
        auto [lat, lon] = node->Location();
        
        // Latitude conversion
        int latDeg = std::abs(static_cast<int>(lat));
        double latMinFull = (std::abs(lat) - latDeg) * 60.0;
        int latMin = static_cast<int>(latMinFull);
        int latSec = std::round((latMinFull - latMin) * 60.0);
        
        // Longitude conversion
        int lonDeg = std::abs(static_cast<int>(lon));
        double lonMinFull = (std::abs(lon) - lonDeg) * 60.0;
        int lonMin = static_cast<int>(lonMinFull);
        int lonSec = std::round((lonMinFull - lonMin) * 60.0);

        std::stringstream ss;
        ss << latDeg << "d " << latMin << "' " << latSec << "\" " 
           << (lat >= 0 ? "N" : "S") << ", "
           << lonDeg << "d " << lonMin << "' " << lonSec << "\" " 
           << (lon >= 0 ? "E" : "W");
        
        return ss.str();
    }
};

// Existing implementation of other methods remains the same
CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->sortedNodes.size();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    return (index < DImplementation->sortedNodes.size()) 
        ? DImplementation->sortedNodes[index] 
        : nullptr;
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    return DImplementation->findShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->findFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    // Implement path description logic
    desc.clear();
    if (path.empty()) return false;

    // Start location
    auto startNode = DImplementation->getNodeByID(path[0].second);
    desc.push_back("Start at " + DImplementation->formatLocation(startNode));

    // Process path steps
    for (size_t i = 0; i < path.size() - 1; ++i) {
        auto currentNode = DImplementation->getNodeByID(path[i].second);
        auto nextNode = DImplementation->getNodeByID(path[i+1].second);
        
        if (!currentNode || !nextNode) continue;

        double distance = DImplementation->calculateDistance(currentNode,nextNode) / 1609.344; // Convert to miles

        std::stringstream ss;
        // Mode-specific description
        switch (path[i].first) {
            case ETransportationMode::Walk:
                ss << "Walk ";
                break;
            case ETransportationMode::Bike:
                ss << "Bike ";
                break;
            case ETransportationMode::Bus:
                ss << "Take bus ";
                break;
        }

        // Get direction and street name
        double bearing = SGeographicUtils::CalculateBearing(
            currentNode->Location(), 
            nextNode->Location()
        );
        std::string directionStr = SGeographicUtils::BearingToDirection(bearing);
        
        // Find way between nodes
        auto way = DImplementation->findWayBetweenNodes(
            path[i].second, 
            path[i+1].second
        );
        std::string streetName = way ? DImplementation->getWayName(way) : "unnamed road";

        ss << directionStr;
        if (streetName != "unnamed road") {
            ss << " along " << streetName;
        }
        ss << " for " << std::fixed << std::setprecision(1) << distance << " mi";
        
        desc.push_back(ss.str());
    }

    // End location
    auto endNode = DImplementation->getNodeByID(path.back().second);
    desc.push_back("End at " + DImplementation->formatLocation(endNode));

    return true;
}

// Additional helper methods in the SImplementation struct
std::shared_ptr<CStreetMap::SWay> findWayBetweenNodes(TNodeID node1, TNodeID node2) const {
    for (size_t i = 0; i < streetMap->WayCount(); ++i) {
        auto way = streetMap->WayByIndex(i);
        if (!way) continue;
        
        for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
            if ((way->GetNodeID(j) == node1 && way->GetNodeID(j+1) == node2) ||
                (!checkOneWay(way) && way->GetNodeID(j) == node2 && way->GetNodeID(j+1) == node1)) {
                return way;
            }
        }
    }
    return nullptr;

std::string getWayName(const std::shared_ptr<CStreetMap::SWay>& way) const {
    try {
        return std::any_cast<std::string>(way->GetAttribute("name"));
    } catch (...) {
        return "unnamed road";
    }
}