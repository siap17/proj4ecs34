#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>    // Added for std::priority_queue
#include <sstream>  // Added for std::stringstream
#include "StreetMap.h"
#include "BusSystem.h"
#include "PathRouter.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include "StringUtils.h"
#include "BusSystemIndexer.h"
#include "DijkstraTransportationPlanner.h"

struct CDijkstraTransportationPlanner::SImplementation{
    
    using TVertexID = std::size_t; 
    static constexpr double NoPath = std::numeric_limits<double>::infinity(); 

    std::size_t nextVertexID = 0; 
    std::unordered_map<TVertexID, std::any> vertices; 
    std::unordered_map<TVertexID, std::vector<std::pair<TVertexID, double>>> adjList; 

    std::unordered_map<TNodeID, TVertexID> nodeToVertex; 
    std::unordered_map<TVertexID, TNodeID> vertexToNode; 
    
    std::shared_ptr<SConfiguration> configuration; 
    std::shared_ptr<CStreetMap> Map; 
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNode; 

    SImplementation(std::shared_ptr<SConfiguration> config) : configuration(config) {
        if (!config || !config->StreetMap()){
            return; 
        }

        Map = config->StreetMap(); 

        for (size_t i = 0; i < Map->NodeCount(); ++i){
            auto node = Map->NodeByIndex(i); 
            if (node){
                SortedNode.push_back(node); 

                TVertexID vID = AddVertex(node); 

                nodeToVertex[node->ID()] = vID; 
                vertexToNode[vID] = node->ID(); 
            }
        }

        std::sort(SortedNode.begin(), SortedNode.end(), 
                 [](const auto& a, const auto& b) { return a->ID() < b->ID(); });

        BuildGraph(); 
    }

    TVertexID AddVertex(std::any tag){
        TVertexID id = nextVertexID++; 
        vertices[id] = tag; 
        adjList[id] = {}; 
        return id; 
    }

    void BuildGraph(){
        for (size_t i = 0; i < Map->WayCount(); ++i){
            auto way = Map->WayByIndex(i);
            if (!way || !IsWayTraversable(way)){
                continue; 
            } 

            for (size_t j = 0; j < way->NodeCount() - 1; ++j){
                TNodeID node1ID = way->GetNodeID(j); 
                TNodeID node2ID = way->GetNodeID(j+1);

                if (nodeToVertex.find(node1ID) == nodeToVertex.end() || 
                    nodeToVertex.find(node2ID) == nodeToVertex.end()){
                    continue; 
                } 

                TVertexID v1 = nodeToVertex[node1ID]; 
                TVertexID v2 = nodeToVertex[node2ID]; 

                auto node1 = GetNodeByID(node1ID); 
                auto node2 = GetNodeByID(node2ID); 

                if (!node1 || !node2){
                    continue; 
                }

                double distance = CalculateDistance(node1, node2); 

                // Use IsOneWay consistently
                bool oneWay = IsOneWay(way); 
                AddEdge(v1, v2, distance); 
                if (!oneWay){
                    AddEdge(v2, v1, distance); 
                }
            }
        }
    }

    void AddEdge(TVertexID from, TVertexID to, double weight){
        adjList[from].push_back({to, weight}); 
    }

    bool IsWayTraversable(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (!way) {
            return false; 
        } 
        try {
            std::string highway = std::any_cast<std::string>(way->GetAttribute("highway")); 
            return true; 
        } catch (...){
            return false; 
        }
    }

    bool IsOneWay(const std::shared_ptr<CStreetMap::SWay>& way) const {
        try {
            std::string oneway = std::any_cast<std::string>(way->GetAttribute("oneway")); 
            return oneway == "yes" || oneway == "true" || oneway == "1"; 
        } catch (...){
            return false; 
        }
    }

    std::string GetWayName(const std::shared_ptr<CStreetMap::SWay>& way) const {
        try {
            return std::any_cast<std::string>(way->GetAttribute("name"));
        } catch (...) {
            return "unnamed road";
        }
    }

    std::shared_ptr<CStreetMap::SNode> GetNodeByID(TNodeID id) const {
        for (const auto& node : SortedNode) {
            if (node->ID() == id) {
                return node;
            }
        }
        return nullptr;
    }

    double CalculateDistance(const std::shared_ptr<CStreetMap::SNode>& node1, 
                         const std::shared_ptr<CStreetMap::SNode>& node2) const {
    // Use GeographicUtils for Haversine distance
    double distanceInMiles = SGeographicUtils::HaversineDistanceInMiles(node1->Location(), node2->Location());
    
    // Convert miles to meters (1 mile = 1609.344 meters)
    const double MetersPerMile = 1609.344;
    return distanceInMiles * MetersPerMile;
}

    std::size_t NodeCount() const noexcept {
        return SortedNode.size();
    }
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
       if (index < SortedNode.size()){
        return SortedNode[index]; 
       }
       return nullptr; 
    }

    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID>& path) {
        path.clear(); 

        // Check if src and dest nodes exist in our mapping
        if (nodeToVertex.find(src) == nodeToVertex.end() || 
            nodeToVertex.find(dest) == nodeToVertex.end()){
            return NoPath; 
        }

        // Get vertex IDs for source and destination
        TVertexID srcVertex = nodeToVertex[src];
        TVertexID destVertex = nodeToVertex[dest];

        // Priority queue for Dijkstra's algorithm
        using QueueElement = std::pair<double, TVertexID>; // (distance, vertex)
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

        // Initialize distances and previous vertex map
        std::unordered_map<TVertexID, double> distances;
        std::unordered_map<TVertexID, TVertexID> previous;

        // Set initial distances to infinity
        for (const auto& [vertex, _] : vertices) {
            distances[vertex] = NoPath;
        }
        distances[srcVertex] = 0.0;

        // Start from source
        pq.push({0.0, srcVertex});

        // Main Dijkstra loop
        while (!pq.empty()) {
            auto [currentDist, currentVertex] = pq.top();
            pq.pop();

            // If we reached the destination, we're done
            if (currentVertex == destVertex) {
                break;
            }

            // Skip if we already found a better path
            if (currentDist > distances[currentVertex]) {
                continue;
            }

            // Process all neighbors
            for (const auto& [neighbor, weight] : adjList[currentVertex]) {
                double newDist = currentDist + weight;
                
                // If we found a better path, update
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    previous[neighbor] = currentVertex;
                    pq.push({newDist, neighbor});
                }
            }
        }

        // Check if we found a path
        if (distances[destVertex] == NoPath) {
            return NoPath;
        }

        // Reconstruct the path
        std::vector<TVertexID> vertexPath;
        for (TVertexID at = destVertex; at != srcVertex; at = previous[at]) {
            vertexPath.push_back(at);
        }
        vertexPath.push_back(srcVertex);
        
        // Reverse to get source-to-destination order
        std::reverse(vertexPath.begin(), vertexPath.end());

        // Convert vertex IDs to node IDs
        for (TVertexID vertex : vertexPath) {
            path.push_back(vertexToNode[vertex]);
        }

        return distances[destVertex];
    }

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
    path.clear();

    // Check if src and dest nodes exist in our mapping
    if (nodeToVertex.find(src) == nodeToVertex.end() || 
        nodeToVertex.find(dest) == nodeToVertex.end()){
        return NoPath; 
    }

    // Get the bus system if available
    auto busSystem = configuration->BusSystem();
    bool hasBusSystem = busSystem != nullptr;
    
    // Create a specialized graph for time-based routing
    struct EdgeInfo {
        TNodeID toNode;
        double time;
        ETransportationMode mode;
    };
    
    std::unordered_map<TNodeID, std::vector<EdgeInfo>> timeGraph;
    
    // Build walking edges from street map
    for (size_t i = 0; i < Map->WayCount(); ++i) {
        auto way = Map->WayByIndex(i);
        if (!way || !IsWayTraversable(way)) {
            continue;
        }
        
        for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
            TNodeID node1ID = way->GetNodeID(j);
            TNodeID node2ID = way->GetNodeID(j+1);
            
            auto node1 = GetNodeByID(node1ID);
            auto node2 = GetNodeByID(node2ID);
            
            if (!node1 || !node2) {
                continue;
            }
            
            // Calculate distance and times
            double distance = CalculateDistance(node1, node2);
            double walkTime = distance / configuration->WalkSpeed();
            double bikeTime = distance / configuration->BikeSpeed();
            
            // Add edges to time graph
            bool oneWay = IsOneWay(way);
            
            // Walking edges
            timeGraph[node1ID].push_back({node2ID, walkTime, ETransportationMode::Walk});
            if (!oneWay) {
                timeGraph[node2ID].push_back({node1ID, walkTime, ETransportationMode::Walk});
            }
            
            // Biking edges
            timeGraph[node1ID].push_back({node2ID, bikeTime, ETransportationMode::Bike});
            if (!oneWay) {
                timeGraph[node2ID].push_back({node1ID, bikeTime, ETransportationMode::Bike});
            }
        }
    }
    
    // Add bus edges if bus system is available
    if (hasBusSystem) {
        for (size_t i = 0; i < busSystem->RouteCount(); ++i) {
            auto route = busSystem->RouteByIndex(i);
            if (!route || route->StopCount() < 2) {
                continue;
            }
            
            // For each consecutive pair of stops in the route
            for (size_t j = 0; j < route->StopCount() - 1; ++j) {
                auto stopID1 = route->GetStopID(j);
                auto stopID2 = route->GetStopID(j + 1);
                
                auto stop1 = busSystem->StopByID(stopID1);
                auto stop2 = busSystem->StopByID(stopID2);
                
                if (!stop1 || !stop2) {
                    continue;
                }
                
                TNodeID nodeID1 = stop1->NodeID();
                TNodeID nodeID2 = stop2->NodeID();
                
                auto node1 = GetNodeByID(nodeID1);
                auto node2 = GetNodeByID(nodeID2);
                
                if (!node1 || !node2) {
                    continue;
                }
                
                // Calculate bus travel time (with a stop time penalty)
                double distance = CalculateDistance(node1, node2);
                double busSpeed = configuration->DefaultSpeedLimit() * 0.8; // Assume buses are 80% of speed limit
                double busTime = (distance / (busSpeed * 1609.344 / 3600.0)) + configuration->BusStopTime();
                
                // Add bus edge (one way in the route direction)
                timeGraph[nodeID1].push_back({nodeID2, busTime, ETransportationMode::Bus});
            }
        }
    }
    
    // Now run a modified Dijkstra on the time graph
    std::priority_queue<std::pair<double, TNodeID>, std::vector<std::pair<double, TNodeID>>, std::greater<>> pq;
    std::unordered_map<TNodeID, double> times;
    std::unordered_map<TNodeID, std::pair<TNodeID, ETransportationMode>> previous;
    
    // Initialize all times to infinity
    for (const auto& [nodeID, _] : timeGraph) {
        times[nodeID] = NoPath;
    }
    times[src] = 0.0;
    
    // Start from source
    pq.push({0.0, src});
    
    // Main Dijkstra loop
    while (!pq.empty()) {
        auto [currentTime, currentNode] = pq.top();
        pq.pop();
        
        // If we reached the destination, we're done
        if (currentNode == dest) {
            break;
        }
        
        // Skip if we already found a better path
        if (currentTime > times[currentNode]) {
            continue;
        }
        
        // Process neighbors
        for (const auto& edge : timeGraph[currentNode]) {
            double newTime = currentTime + edge.time;
            
            if (newTime < times[edge.toNode]) {
                times[edge.toNode] = newTime;
                previous[edge.toNode] = {currentNode, edge.mode};
                pq.push({newTime, edge.toNode});
            }
        }
    }
    
    // No path found
    if (times[dest] == NoPath) {
        return NoPath;
    }
    
    // Reconstruct the path
    std::vector<std::pair<TNodeID, ETransportationMode>> reversePath;
    TNodeID current = dest;
    
    while (current != src) {
        auto [prevNode, mode] = previous[current];
        reversePath.push_back({current, mode});
        current = prevNode;
    }
    
    // Add the source node (with an arbitrary mode, it will be overridden)
    reversePath.push_back({src, ETransportationMode::Walk});
    
    // Reverse to get the correct order
    std::reverse(reversePath.begin(), reversePath.end());
    
    // Normalize the path (the source should use the mode of the first step)
    if (reversePath.size() >= 2) {
        reversePath[0].second = reversePath[1].second;
    }
    
    // Convert to TTripStep format
    for (const auto& [nodeID, mode] : reversePath) {
        path.push_back({mode, nodeID});
    }
    
    return times[dest];
}
    bool GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
        desc.clear();

        if (path.size() < 2) {
            return false; // Need at least start and end for a valid path
        }

        // Start description
        std::stringstream ss;
        ss << "Start at node " << path[0].second;
        desc.push_back(ss.str());

        // Process each step in the path
        for (size_t i = 1; i < path.size(); ++i) {
            ETransportationMode mode = path[i].first;
            TNodeID nodeID = path[i].second;
            TNodeID prevNodeID = path[i-1].second;

            // Get the nodes
            auto current = GetNodeByID(nodeID);
            auto previous = GetNodeByID(prevNodeID);

            if (!current || !previous) {
                continue;
            }

            double stepDistance = CalculateDistance(previous, current);

            // Fixed function name and added proper null handling
            std::shared_ptr<CStreetMap::SWay> way = FindWayBetweenNodes(prevNodeID, nodeID);
            std::string wayName = way ? GetWayName(way) : "unknown road";

            std::stringstream stepSs;
            switch (mode) {
                case ETransportationMode::Walk:
                    stepSs << "Walk to node " << nodeID << " via " << wayName 
                          << " (" << FormatDistance(stepDistance) << ")";
                    break;
                case ETransportationMode::Bike:
                    stepSs << "Bike to node " << nodeID << " via " << wayName 
                          << " (" << FormatDistance(stepDistance) << ")";
                    break;
                case ETransportationMode::Bus:
                    stepSs << "Take bus to node " << nodeID << " via " << wayName 
                          << " (" << FormatDistance(stepDistance) << ")";
                    break;
            }
            desc.push_back(stepSs.str());
        }

        // Add arrival message - fixed to add space after "node"
        std::stringstream arrivalSs;
        arrivalSs << "Arrive at node " << path.back().second;
        desc.push_back(arrivalSs.str());

        return true;
    }

    std::shared_ptr<CStreetMap::SWay> FindWayBetweenNodes(TNodeID node1, TNodeID node2) const {
        for (size_t i = 0; i < Map->WayCount(); ++i) {
            auto way = Map->WayByIndex(i);
            if (!way) {
                continue;
            }
            
            // Fixed GetNodeID and renamed isOneWay to IsOneWay
            for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
                if ((way->GetNodeID(j) == node1 && way->GetNodeID(j+1) == node2) ||
                    (!IsOneWay(way) && way->GetNodeID(j) == node2 && way->GetNodeID(j+1) == node1)) {
                    return way;
                }
            }
        }
        
        return nullptr;
    }

    // Fixed FormatDistance function with proper spacing
    std::string FormatDistance(double meters) const {
        if (meters < 1000) {
            // Less than 1 km, show in meters
            int roundedMeters = static_cast<int>(meters + 0.5);
            return std::to_string(roundedMeters) + " meters";
        } else {
            // 1 km or more, show in kilometers with 1 decimal place
            double km = meters / 1000.0;
            std::stringstream ss;
            ss.precision(1);
            ss << std::fixed << km << " kilometers";
            return ss.str();
        }
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config){
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept{
    return DImplementation->NodeCount();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept{
    return DImplementation->SortedNodeByIndex(index);
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    return DImplementation->FindShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    return DImplementation->GetPathDescription(path, desc);
}