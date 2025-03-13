#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>
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

    SImplementation(std::shared_ptr<SConfiguration> config){
        if (!config || !config->StreetMap()){
            return; 
        }

        Map = config ->StreetMap(); 

        for (size_t i = 0; i < Map->NodeCount(); ++i){
            auto node = Map->NodeByIndex(i); 
            if (node){
                SortedNode.push_back(node); 

                TVertexID vID = AddVertex(node); 

                nodeToVertex[node->ID()] = vID; 
                vertexToNode[vID] = node->ID(); 

            }
        }

        std::sort(SortedNode.begin(), SortedNode.end(), [](const auto& a, const auto& b){return a->ID() < b->ID(); });

        BuildGraph(); 
    };

    TVertexID AddVertex(std::any tag){
        TVertexID id = nextVertexID++; 
        vertices[id] = tag; 
        adjList[id] = {}; 
        return id; 
    }

    void BuildGraph(){
        for (size_t i = 0; i < Map->WayCount(); ++i){
            auto way = Map -> WayByIndex(i);
            if (!way || !IsWayTraversable(way)){
                continue; 
            } 

            for (size_t j = 0; j < way -> NodeCount() - 1; ++j){
                TNodeID node1ID = way -> GetNodeID(j); 
                TNodeID node2ID = way -> GetNodeID(j+1);

                if (nodeToVertex.find(node1ID) == nodeToVertex.end() || nodeToVertex.find(node2ID) == nodeToVertex.end()){
                    continue; 
                } 

                TVertexID v1 = nodeToVertex[node1ID]; 
                TVertexID v2 = nodeToVertex[node2ID]; 

                auto node1 = GetNodeByID(node1ID); 
                auto node2 = GetNodeByID(node2ID); 

                if (!node1 || !node2){
                    continue; 
                }

                double distance = CalculateDistance (node1, node2); 

                bool isOneWay = isOneWay(way); 
                AddEdge(v1,v2, distance); 
                if (!isOneWay){
                    AddEdge(v2, v1, distance); 
                }
            }
        }
    }

    void AddEdge(TVertexID from, TVertexID to, double weight){
        adjList[from].push_back({to, weight}); 
    }

    bool IsWayTraversable(const std::shared_ptr<CStreetMap::SWay>&way) const {
        if (!way) {
            return false; 
        } try{
            std::string highway = std::any_cast<std::string>(way->GetAttribute("highway")); 
            return true; 
        } catch (...){
            return false; 
        }
    }

    bool isOneWay(const std::shared_ptr<CStreetMap::SWay>&way) const{
        try {
            std::string oneway = std::any_cast<std::string>(way->GetAttribute("oneway")); 
            return oneway == "yes" || oneway == "true" || oneway == "1"; 
        } catch (...){
            return false; 
        }
    }

    std::string GetWayName(const std::shared_ptr<CStreetMap::SWay>&way) const {
        try {
            return std::any_cast<std::string>(way->GetAttribute("name")); 
        } catch (...){
            return "unnamed road"; 
        }
    }

    std::shared_ptr<CStreetMap::SNode> GetNodeByID(TNodeID id) const {
        for (const auto& node : SortedNode){
            if (node->ID() == id){
                return node; 
            }
        }
        return nullptr; 
    }

    double CalculateDistance ( const std::shared_ptr<CStreetMap::SNode>& node1, const std::shared_ptr<CStreetMap::SNode>& node2) const {
        constexpr double EarthRadius = 6371000.0; 

        double lat1 = node1 -> Latitude() * M_PI / 180.0; 
        double lat2 = node2 -> Latitude() * M_PI / 180.0; 

        double lon1 = node1 -> Longitude() * M_PI /180.0; 
        double lon2 = node2 -> Longitude() * M_PI / 180.0; 

        double dlat = lat2 - lat1; 
        double dlon = lon2 - lon1; 

        double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2); 
        double c = 2 * atan2(sqrt(a), sqrt(1-a)); 

        return EarthRadius * c; 

    }
    
    std::size_t NodeCount() const noexcept{
        return SortedNode.size();
    };
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
       if (index < SortedNode.size()){
        return SortedNode[index]; 
       }
       return nullptr; 
    };

    double FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path) {
        path.clear(); 

        if (nodeToVertex.find(src) == nodeToVertex.end() || nodeToVertex.find(dest) == nodeToVertex.end()){
            return NoPath; 
        }

        TVertexID srcVertex = nodeToVertex[src]; 
        TVertexID destVertex = nodeToVertex[dest]; 

        using QueueElement = std::pair<double, TVertexID>; 
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq; 

        std::unordered_map<TVertexID, double> distances; 
        std::unordered_map<TVertexID, TVertexID> previous; 

        for (const auto& [vertex, _] : vertices){
            distances[vertex] = NoPath; 

        }
        distances[srcVertex] = 0.0; 

        pq.push({0.0, srcVertex}); 

        while (!pq.empty()){
            auto [currentDist, currentVertex] = pq.top(); 
            pq.pop(); 

            if (currentVertex == destVertex){
                break; 
            }

            if (currentDist > distances[currentVertex]){
                continue; 
            }

            for (const auto& [neighbor, weight] : adjList[currentVertex]){
                double newDist = currentDist + weight; 
                
                if (newDist < distances[neighbor]){
                    distances[neighbor] = newDist; 
                    previous[neighbor] = currentVertex; 
                    pq.push({newDist, neighbor}); 
                }
            }
        }

        if (distances[destVertex] == NoPath){
            return NoPath; 
        }

        std::vector<TVertexID> vertexPath; 
        for (TVertexID at = destVertex; at != srcVertex; at = previous[at]){
            vertexPath.push_back(at); 
        }
        vertexPath.push_back(srcVertex); 

        std::reverse(vertexPath.begin(), vertexPath.end()); 
        for (TVertexID vertex: vertexPath){
            path.push_back(vertexToNode[vertex]); 
        }
        return distances[destVertex]; 
    };

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector< TTripStep > &path) {
        path.clear(); 

        if (nodeToVertex.find(src) == nodeToVertex.end() || nodeToVertex.find(dest) == nodeToVertex.end()){
            return NoPath; 
        }

        std::vector<TNodeID> nodePath; 

        double distance = FindShortestPath(src, dest, nodePath); 

        if (distance == NoPath){
            return NoPath; 
        }

        for (TNodeID node : nodePath){
            path.push_back({ETransportationMode::Walk, node}); 
        }

        double walkSpeed = configuration->WalkSpeed(); 
        double time = distance/walkSpeed; 
        
        return time; 
    };

    bool GetPathDescription(const std::vector< TTripStep > &path, std::vector< std::string > &desc) const {
        desc.clear(); 
        if (path.size() < 2){
            return false; 
        }
        
        std::stringstream ss; 
        ss<<"Start at node "<<path[0].second; 
        desc.push_back(ss.str()); 

        for (size_t i = 1; i < path.size(); ++i){
            ETransportationMode mode = path[i].first; 
            TNodeID nodeID = path[i].second; 
            TNodeID prevNodeID = path[i-1].second; 

            auto current = GetNodeByID(nodeID); 
            auto previous = GetNodeByID(prevNodeID); 

            if (!current || !previous){
                continue; 
            }

            double stepDistance = CalculateDistance(previous, current); 

            std::shared_ptr<CStreetMap::SWay> way = FindWayBetweenNode(prevNodeID, nodeID); 
            std::string wayName = way ? GetWayName(way); 

            std::stringstream stepSs; 
            switch(mode){
                case ETransportationMode::Walk: 
                    stepSs<<"Walk to node " << nodeID << "via " << wayName << "(" << FormatDistance(stepDistance) << ")"; 
                    break; 
                
                case ETransportationMode::Bus: 
                    stepSs<<"Take bus to node " << nodeID << "via " << wayName <<  "(" << FormatDistance(stepDistance) << ")"; 
                    break; 
            }

            desc.push_back(stepSs.str()); 
        }

        std::stringstream arrivalSs; 
        arrivalSs<< "Arrive at node" << path.back().second; 
        desc.push_back(arrivalSs.str()); 

        return true; 
    };

    std::shared_ptr<CStreetMap::SWay> FindWayBetweenNodes(TNodeID node1, TNodeID node2) const{
        for (size_t i = 0; i < Map->WayCount(); ++i){
            auto way = Map->WayByIndex(i); 
            if (!way){
                continue; 
            }

            for (size_t j = 0; j < way ->NodeCount()-1; ++j){
                if ((way ->GetNodeID(j) == node1 && way->GetNodeID(j+1) == node2) || (!isOneWay(way) && way ->GetNode(j) == node2 && way->GetNodeID(j+1) == node1)){
                    return way; 
                }
            }
        }
        return nullptr; 
    }

    std::string FormatDistance(double meters) const {
        if (meters < 1000){
            int roundedMeters = static_cast <int>(meters  + 0.5); 
            return std:: to_string(roundedMeters) + "meters"; 

        } else {
            double km = meters/1000.0; 
            std::stringstream ss; 
            ss.precision(1); 
            ss <<std::fixed <<km<< "kilometers"; 
            return ss.str(); 
        }
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config){
    DImplementation = std::make_unique<SImplementation>(config);
};

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner(){

};


std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept{
    return DImplementation->NodeCount();
};

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept{
    return DImplementation->SortedNodeByIndex(index);
};


double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path) {
    return DImplementation->FindShortestPath(src,dest,path);
};

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TTripStep > &path) {
    return DImplementation->FindFastestPath(src,dest,path);
};

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector< TTripStep > &path, std::vector< std::string > &desc) const {
    return DImplementation->GetPathDescription(path,desc);
};