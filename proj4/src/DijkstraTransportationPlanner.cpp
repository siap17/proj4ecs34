//included headers here 
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
#include <any>
#include "StreetMap.h"
#include "BusSystem.h"
#include "PathRouter.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include "StringUtils.h"
#include "BusSystemIndexer.h"
#include "DijkstraTransportationPlanner.h"

//initialized a structure that includes Dijikstra Implementation 
struct CDijkstraTransportationPlanner::SImplementation {
    using TVertexID = std::size_t; //This tracks the size for graph vertices 
    static constexpr double NoPath = std::numeric_limits<double>::infinity(); //represents an infinite distance 

    std::size_t nextVertexID = 0; //set this to 0  
    std::unordered_map<TVertexID, std::any> vertices; //stores vertex data 
    std::unordered_map<TVertexID, std::vector<std::pair<TVertexID, double>>> adjList; // stores adjacencyList 

    std::unordered_map<TNodeID, TVertexID> nodeToVertex; //maps the nodeToVertex 
    std::unordered_map<TVertexID, TNodeID> vertexToNode; 
    
    std::shared_ptr<SConfiguration> configuration; 
    std::shared_ptr<CStreetMap> streetMap; 
    std::vector<std::shared_ptr<CStreetMap::SNode>> sortedNodes; 

    SImplementation(std::shared_ptr<SConfiguration> config) : configuration(config) { //constructor with SImplementation 
        if (!config || !config->StreetMap()){ //using smart pointers to manage memory 
            return; 
        }

        streetMap = config->StreetMap(); //ensures a valid streetmap 

        for (size_t i = 0; i < streetMap->NodeCount(); ++i){ //this loads the nodes essentially 
            auto node = streetMap->NodeByIndex(i); //iterate through every node and AddVertex
            if (node){
                sortedNodes.push_back(node); 

                TVertexID vID = AddVertex(node); 

                nodeToVertex[node->ID()] = vID; 
                vertexToNode[vID] = node->ID(); 
            }
        }

        std::sort(sortedNodes.begin(), sortedNodes.end(), //sorts through all the nodes 
                 [](const auto& a, const auto& b) { return a->ID() < b->ID(); });

        BuildGraph(); //Builds the graph 
    }

    // Helper methods for locating ways and getting names
    std::shared_ptr<CStreetMap::SWay> findWayBetweenNodes(TNodeID node1, TNodeID node2) const {
        for (size_t i = 0; i < streetMap->WayCount(); ++i) { //actually buiding the graph representation 
            auto way = streetMap->WayByIndex(i);
            if (!way) continue;
            
            for (size_t j = 0; j < way->NodeCount() - 1; ++j) { //progressing through each street 
                if ((way->GetNodeID(j) == node1 && way->GetNodeID(j+1) == node2) ||
                    (!IsOneWay(way) && way->GetNodeID(j) == node2 && way->GetNodeID(j+1) == node1)) {
                    return way;
                }
            }
        }
        return nullptr; //else return nullptr 
    }

    std::string getWayName(const std::shared_ptr<CStreetMap::SWay>& way) const {
        try {
            return std::any_cast<std::string>(way->GetAttribute("name"));
        } catch (...) {
            return "unnamed road";
        }
    }

    TVertexID AddVertex(std::any tag){
        TVertexID id = nextVertexID++; 
        vertices[id] = tag; 
        adjList[id] = {}; 
        return id; 
    }

    void BuildGraph(){
        for (size_t i = 0; i < streetMap->WayCount(); ++i){
            auto way = streetMap->WayByIndex(i); //builds a way 
            if (!way || !IsWayTraversable(way)){ //continues 
                continue; 
            } 

            for (size_t j = 0; j < way->NodeCount() - 1; ++j){ //iterates through 
                TNodeID node1ID = way->GetNodeID(j); 
                TNodeID node2ID = way->GetNodeID(j+1);

                if (nodeToVertex.find(node1ID) == nodeToVertex.end() ||  //if can go through and find end then continue 
                    nodeToVertex.find(node2ID) == nodeToVertex.end()){
                    continue; 
                } 

                TVertexID v1 = nodeToVertex[node1ID]; //establishes vetex 1, vertex 2 and node1, node2 
                TVertexID v2 = nodeToVertex[node2ID]; 

                auto node1 = GetNodeByID(node1ID);  //grabs node1 and nod2 id 
                auto node2 = GetNodeByID(node2ID); 

                if (!node1 || !node2){ //if dont exist, continue 
                    continue; 
                }

                double distance = CalculateDistance(node1, node2); //this gets the distance 

                bool oneWay = IsOneWay(way); //if one way then we add the distance 
                AddEdge(v1, v2, distance); 
                if (!oneWay){
                    AddEdge(v2, v1, distance); 
                }
            }
        }
    }
//this function works to add and edge to Adjacency list 
    void AddEdge(TVertexID from, TVertexID to, double weight){
        adjList[from].push_back({to, weight}); 
    }
//checks if the way is traversable 
    bool IsWayTraversable(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (!way) {
            return false; 
        } 
        try { //if not then we take "the highway" 
            std::string highway = std::any_cast<std::string>(way->GetAttribute("highway")); 
            return true; 
        } catch (...){
            return false; 
        }
    }

    bool IsOneWay(const std::shared_ptr<CStreetMap::SWay>& way) const {
        try { //checks if it is oneway 
            std::string oneway = std::any_cast<std::string>(way->GetAttribute("oneway")); 
            return oneway == "yes" || oneway == "true" || oneway == "1"; 
        } catch (...){
            return false; 
        }
    }
//this grabs name of the way 
    std::string GetWayName(const std::shared_ptr<CStreetMap::SWay>& way) const {
        try { 
            return std::any_cast<std::string>(way->GetAttribute("name"));
        } catch (...) {
            return "unnamed road";
        }
    }
//This gets the node by ID and initializes a loop through sortedNode and return Node 
    std::shared_ptr<CStreetMap::SNode> GetNodeByID(TNodeID id) const {
        for (const auto& node : sortedNodes) {
            if (node->ID() == id) {
                return node;
            }
        }
        return nullptr;
    }
//This calculates the distance 
    double CalculateDistance(const std::shared_ptr<CStreetMap::SNode>& node1, 
                             const std::shared_ptr<CStreetMap::SNode>& node2) const {
        // Use GeographicUtils for Haversine distance
        if (!node1 || !node2){ //NoPath if no node1 and node2 
            return NoPath; 
        }
//This is the DistanceInMiles, grab location of each node 
        double DistanceInMiles = SGeographicUtils::HaversineDistanceInMiles(
            node1->Location(), 
            node2->Location() 
        ); 
        // Convert miles to meters (1 mile = 1609.344 meters)
        const double MetersPerMile = 1609.344;
        return std::max(0.0, DistanceInMiles * MetersPerMile); 
    }
//Get size_t of NodeCount() 
    std::size_t NodeCount() const noexcept {
        return sortedNodes.size();
    }
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
       if (index < sortedNodes.size()){
        return sortedNodes[index]; 
       }
       return nullptr; 
    }
    
    // Formatting helpers
    std::string FormatLocation(const std::shared_ptr<CStreetMap::SNode>& node) const {
        auto [lat, lon] = node->Location();
        
        // This converts the latitude
        int latDeg = std::abs(static_cast<int>(lat));
        double latMinFull = (std::abs(lat) - latDeg) * 60.0;
        int latMin = static_cast<int>(latMinFull);
        int latSec = std::round((latMinFull - latMin) * 60.0);
        
        // This converts the lognitude 
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

//This finds the shortest path 
    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID>& path) {
        path.clear();
        
        // Special case: if src and dest are same 
        if (src == dest) {
            path.push_back(src); //Then we push it back in the path 
            return 0.0;
        }
        
        // Check if src and dest nodes exist in our mapping
        if (nodeToVertex.find(src) == nodeToVertex.end() || 
            nodeToVertex.find(dest) == nodeToVertex.end()) {
            return CPathRouter::NoPathExists;  // Then we return NoPathExists 
        }
        
        // This gets the VertexID 
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
            distances[vertex] = CPathRouter::NoPathExists;  // Use the correct constant
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
        
        // Check if we found a path - update this condition
        if (distances[destVertex] == CPathRouter::NoPathExists || previous.find(destVertex) == previous.end()) {
            return CPathRouter::NoPathExists;  // Use the correct constant
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
        // Special case: if src and dest are the same
        if (src == dest) {
            path.push_back({ETransportationMode::Walk, src});
            return 0.0;
        }

        // Check if src and dest nodes exist in our mapping
        if (nodeToVertex.find(src) == nodeToVertex.end() || 
            nodeToVertex.find(dest) == nodeToVertex.end()){
            return NoPath; 
        } 

        // Get the nodes path first
        std::vector<TNodeID> nodePath;
        double totalDistance = FindShortestPath(src, dest, nodePath);
        
        if (totalDistance == NoPath || nodePath.empty()) {
            return NoPath;
        }

        // Calculate total time based on transportation modes
        double totalTime = 0.0;
        
        // Path generation
        ETransportationMode currentMode = ETransportationMode::Walk;
        path.push_back({currentMode, nodePath[0]});

        for (size_t i = 1; i < nodePath.size(); ++i) {
            auto prevNode = GetNodeByID(nodePath[i-1]);
            auto currNode = GetNodeByID(nodePath[i]);
            
            if (!prevNode || !currNode) continue;

            double distanceMeters = CalculateDistance(prevNode, currNode);
            double distanceMiles = distanceMeters / 1609.344;
            
            // Calculate time for each mode
            double walkTime = distanceMiles / configuration->WalkSpeed();
            double bikeTime = distanceMiles / configuration->BikeSpeed();
            
            // Select fastest mode
            ETransportationMode bestMode = (bikeTime < walkTime) 
                ? ETransportationMode::Bike 
                : ETransportationMode::Walk;
                
            double segmentTime = (bestMode == ETransportationMode::Bike) ? bikeTime : walkTime;
            totalTime += segmentTime;

            // Add to path with best mode
            if (bestMode == currentMode) {
                // Update the last step with new destination
                path.back().second = nodePath[i];
            } else {
                // Add new step with new mode
                path.push_back({bestMode, nodePath[i]});
                currentMode = bestMode;
            }
        }

        return totalTime;
    }

    bool GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
        desc.clear();
        if (path.empty()) return false;

        // Special case for single-node path
        if (path.size() == 1) {
            auto node = GetNodeByID(path[0].second);
            if (!node) return false;
            
            std::stringstream ss;
            ss << "Start and end at " << FormatLocation(node);
            desc.push_back(ss.str());
            return true;
        }

        // Start location
        auto startNode = GetNodeByID(path[0].second);
        if (!startNode) return false;
        desc.push_back("Start at " + FormatLocation(startNode));

        // Track last node in current mode segment
        TNodeID lastNodeID = path[0].second;
        
        // Process path steps
        for (size_t i = 0; i < path.size(); ++i) {
            auto currentNode = GetNodeByID(path[i].second);
            if (!currentNode) continue;
            
            // Skip the first node (already handled as start)
            if (i == 0) continue;
            
            auto prevNode = GetNodeByID(lastNodeID);
            lastNodeID = path[i].second;
            
            if (!prevNode) continue;
            
            double distance = CalculateDistance(prevNode, currentNode) / 1609.344; // Convert to miles

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
                prevNode->Location(), 
                currentNode->Location()
            );
            std::string directionStr = SGeographicUtils::BearingToDirection(bearing);
            
            // Find way between nodes
            auto way = findWayBetweenNodes(
                prevNode->ID(), 
                currentNode->ID()
            );
            std::string streetName = way ? getWayName(way) : "unnamed road";

            ss << directionStr;
            if (streetName != "unnamed road") {
                ss << " along " << streetName;
            }
            ss << " for " << std::fixed << std::setprecision(1) << distance << " mi";
            
            desc.push_back(ss.str());
        }

        // End location
        auto endNode = GetNodeByID(path.back().second);
        if (!endNode) return false;
        desc.push_back("End at " + FormatLocation(endNode));

        return true;
    }
};

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
    return DImplementation->FindShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    return DImplementation->GetPathDescription(path, desc);
}
