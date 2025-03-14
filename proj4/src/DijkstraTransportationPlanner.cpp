#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <sstream>
#include "StreetMap.h"
#include "BusSystem.h"
#include "PathRouter.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include "StringUtils.h"
#include "BusSystemIndexer.h"
#include "DijkstraTransportationPlanner.h"

struct CDijkstraTransportationPlanner::SImplementation {
    
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
        // Special case: if src and dest are the same
        if (src == dest) {
            path.push_back(src);
            return 0.0;
        }
        
        // Check if src and dest nodes exist in our mapping
        if (nodeToVertex.find(src) == nodeToVertex.end() || nodeToVertex.find(dest) == nodeToVertex.end()){
            return  1.0; 
        }

        if (distances[destVertex] == NoPath || previous.find(destVertex) == previous.end()){
            return 1.0; 
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
        if (distances[destVertex] == NoPath || previous.find(destVertex) == previous.end()) {
            return 1.0;
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

    // Helper function to find the closest bus stop to a node
    TNodeID FindClosestBusStop(TNodeID nodeID, std::shared_ptr<CBusSystem> busSystem) const {
        auto node = GetNodeByID(nodeID);
        if (!node || !busSystem) {
            return 0;
        }
        
        double minDistance = std::numeric_limits<double>::max();
        TNodeID closestStopNodeID = 0;
        
        for (size_t i = 0; i < busSystem->StopCount(); i++) {
            auto stop = busSystem->StopByIndex(i);
            if (!stop) continue;
            
            auto stopNode = GetNodeByID(stop->NodeID());
            if (!stopNode) continue;
            
            double distance = CalculateDistance(node, stopNode);
            if (distance < minDistance) {
                minDistance = distance;
                closestStopNodeID = stop->NodeID();
            }
        }
        
        return closestStopNodeID;
    }

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
        path.clear();
        // Special case: if src and dest are the same
        if (src == dest) {
            path.push_back({ETransportationMode::Walk, src});
            return 0.0;
        }


        if (nodeToVertex.find(src) == nodeToVertex.end() || nodeToVertex.find(dest) == nodeToVertex.end()){
            return 1.0; 
        }
    

        // First, calculate walking time
        std::vector<TNodeID> walkPath;
        double walkDistance = FindShortestPath(src, dest, walkPath);
        double walkTime = (walkDistance == NoPath) ? NoPath : walkDistance / configuration->WalkSpeed();
        
        // Walking-only path to use as fallback
        std::vector<TTripStep> walkOnlyPath;
        if (walkDistance != NoPath) {
            for (TNodeID node : walkPath) {
                walkOnlyPath.push_back({ETransportationMode::Walk, node});
            }
        }
        
        // Get bus system if available
        auto busSystem = configuration->BusSystem();
        if (!busSystem) {
            // No bus system, just return walking path
            path = walkOnlyPath;
            return walkTime;
        }
        
        // Try to find a bus route
        // First, find closest bus stops to src and dest
        TNodeID srcBusStopNodeID = FindClosestBusStop(src, busSystem);
        TNodeID destBusStopNodeID = FindClosestBusStop(dest, busSystem);
        
        // If we couldn't find a bus stop near either src or dest, just walk
        if (srcBusStopNodeID == 0 || destBusStopNodeID == 0) {
            path = walkOnlyPath;
            return walkTime;
        }
        
        // Find path to and from bus stops
        std::vector<TNodeID> pathToStop;
        double distanceToStop = FindShortestPath(src, srcBusStopNodeID, pathToStop);
        double timeToStop = (distanceToStop == NoPath) ? NoPath : distanceToStop / configuration->WalkSpeed();
        
        std::vector<TNodeID> pathFromStop;
        double distanceFromStop = FindShortestPath(destBusStopNodeID, dest, pathFromStop);
        double timeFromStop = (distanceFromStop == NoPath) ? NoPath : distanceFromStop / configuration->WalkSpeed();
        
        // If we can't walk to/from stops, use walking-only path
        if (timeToStop == NoPath || timeFromStop == NoPath) {
            path = walkOnlyPath;
            return walkTime;
        }
        
        // Find fastest bus route between stops
        double fastestBusTime = NoPath;
        std::vector<TTripStep> fastestBusPath;
        
        // Loop through all bus routes
        for (size_t i = 0; i < busSystem->RouteCount(); i++) {
            auto route = busSystem->RouteByIndex(i);
            if (!route) continue;
            
            // Find src and dest positions in the route
            int srcPos = -1, destPos = -1;
            
            for (size_t j = 0; j < route->StopCount(); j++) {
                auto stopID = route->GetStopID(j);
                auto stop = busSystem->StopByID(stopID);
                if (!stop) continue;
                
                if (stop->NodeID() == srcBusStopNodeID) {
                    srcPos = j;
                }
                if (stop->NodeID() == destBusStopNodeID) {
                    destPos = j;
                }
            }
            
            // If we found both stops and src comes before dest in the route
            if (srcPos != -1 && destPos != -1 && srcPos < destPos) {
                // Calculate bus time
                double totalBusDistance = 0.0;
                std::vector<TNodeID> busPart;
                
                for (int j = srcPos; j <= destPos; j++) {
                    auto stopID = route->GetStopID(j);
                    auto stop = busSystem->StopByID(stopID);
                    if (!stop) continue;
                    
                    busPart.push_back(stop->NodeID());
                    
                    if (j > srcPos) {
                        auto prevStopID = route->GetStopID(j-1);
                        auto prevStop = busSystem->StopByID(prevStopID);
                        if (!prevStop) continue;
                        
                        auto prevNode = GetNodeByID(prevStop->NodeID());
                        auto currNode = GetNodeByID(stop->NodeID());
                        if (!prevNode || !currNode) continue;
                        
                        totalBusDistance += CalculateDistance(prevNode, currNode);
                    }
                }
                
                // Calculate bus time (in seconds)
                // Assuming bus speed is 80% of default speed limit
                double busSpeed = configuration->DefaultSpeedLimit() * 0.8 * 0.44704; // mph * 0.44704 = m/s
                double busTimeNoStops = totalBusDistance / busSpeed;
                
                // Add stop time for each stop except the last one
                double busTime = busTimeNoStops + (destPos - srcPos) * configuration->BusStopTime();
                
                // Calculate total time for this route
                double totalTime = timeToStop + busTime + timeFromStop;
                
                // If this is faster than our best so far, update
                if (totalTime < fastestBusTime) {
                    fastestBusTime = totalTime;
                    
                    // Build the path
                    fastestBusPath.clear();
                    
                    // Walk to first bus stop
                    for (TNodeID node : pathToStop) {
                        fastestBusPath.push_back({ETransportationMode::Walk, node});
                    }
                    
                    // Bus ride (skip first to avoid duplicate)
                    for (size_t j = 1; j < busPart.size(); j++) {
                        fastestBusPath.push_back({ETransportationMode::Bus, busPart[j]});
                    }
                    
                    // Walk from last bus stop to destination (skip first to avoid duplicate)
                    for (size_t j = 1; j < pathFromStop.size(); j++) {
                        fastestBusPath.push_back({ETransportationMode::Walk, pathFromStop[j]});
                    }
                }
            }
        }
        
        // Compare bus route with walking-only route
        if (walkTime != NoPath && (fastestBusTime == NoPath || walkTime <= fastestBusTime)) {
            // Walking is faster
            path = walkOnlyPath;
            return walkTime;
        } else if (fastestBusTime != NoPath) {
            // Bus is faster
            path = fastestBusPath;
            return fastestBusTime;
        }
        
        // No path found
        return NoPath;
    }

    bool GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }
        
        // Special case for single-node path
        if (path.size() == 1) {
            std::stringstream ss;
            ss << "Start and end at node " << path[0].second;
            desc.push_back(ss.str());
            return true;
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

            // Find way between nodes
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

        // Add arrival message
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
            
            for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
                if ((way->GetNodeID(j) == node1 && way->GetNodeID(j+1) == node2) ||
                    (!IsOneWay(way) && way->GetNodeID(j) == node2 && way->GetNodeID(j+1) == node1)) {
                    return way;
                }
            }
        }
        
        return nullptr;
    }

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