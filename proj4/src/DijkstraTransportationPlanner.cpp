#include "DijkstraTransportationPlanner.h" // Main header for the transportation planner
#include "DijkstraPathRouter.h"           // For CDijkstraPathRouter
#include "StreetMap.h"                    // For CStreetMap
#include "BusSystem.h"                    // For CBusSystem
#include "GeographicUtils.h"              // For CGeographicUtils
#include <queue>                           // For priority_queue
#include <unordered_map>                   // For unordered_map
#include <set>                             // For set
#include <cmath>                           // For sqrt
#include <algorithm>                       // For sort
#include <sstream>                         // For stringstream
#include <iomanip>                         // For setprecision
#include <iostream>                        // For standard I/O

// Implementation of the transportation planner
struct CDijkstraTransportationPlanner::SImplementation {
    // Shared pointer to configuration
    std::shared_ptr<SConfiguration> Config;
    // Vector of sorted nodes
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNodes;
    // Shared pointer to distance router
    std::shared_ptr<CDijkstraPathRouter> DistanceRouter;
    // Shared pointer to time router
    std::shared_ptr<CDijkstraPathRouter> TimeRouter;
    // Mapping from node ID to distance vertex ID
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeIDToDistanceVertexID;
    // Mapping from node ID to time vertex ID
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeIDToTimeVertexID;
    // Mapping from distance vertex ID to node ID
    std::unordered_map<CPathRouter::TVertexID, CStreetMap::TNodeID> DistanceVertexIDToNodeID;
    // Mapping from time vertex ID to node ID
    std::unordered_map<CPathRouter::TVertexID, CStreetMap::TNodeID> TimeVertexIDToNodeID;
    // Mapping from node ID to index
    std::unordered_map<CStreetMap::TNodeID, size_t> NodeIDToIndex;
    // Mapping from bus stop ID to node ID
    std::unordered_map<CBusSystem::TStopID, CStreetMap::TNodeID> StopIDToNodeID;
    // Mapping from bus stop ID to stop name
    std::unordered_map<CBusSystem::TStopID, std::string> StopIDToStopName;
    // Mapping from node ID to bus stop ID
    std::unordered_map<CStreetMap::TNodeID, CBusSystem::TStopID> NodeIDToStopID;
    // Mapping from node ID to bus route information
    std::unordered_map<CStreetMap::TNodeID, std::set<std::pair<std::string, CStreetMap::TNodeID>>> BusRouteInfo;

    // Constructor
    SImplementation(std::shared_ptr<SConfiguration> config)
        : Config(config) {
        auto StreetMap = Config->StreetMap();
        auto BusSystem = Config->BusSystem();

        // Initialize routers
        DistanceRouter = std::make_shared<CDijkstraPathRouter>();
        TimeRouter = std::make_shared<CDijkstraPathRouter>();

        // Sort nodes by ID
        for (size_t i = 0; i < StreetMap->NodeCount(); ++i) {
            auto node = StreetMap->NodeByIndex(i);
            SortedNodes.push_back(node);
        }
        std::sort(SortedNodes.begin(), SortedNodes.end(), [](const auto& a, const auto& b) {
            return a->ID() < b->ID();
        });

        // Map nodes to indices
        for (size_t i = 0; i < SortedNodes.size(); ++i) {
            NodeIDToIndex[SortedNodes[i]->ID()] = i;
        }

        // Add vertices to routers
        for (size_t i = 0; i < SortedNodes.size(); ++i) {
            auto node = SortedNodes[i];
            auto distVertexID = DistanceRouter->AddVertex(node->ID());
            auto timeVertexID = TimeRouter->AddVertex(node->ID());
            NodeIDToDistanceVertexID[node->ID()] = distVertexID;
            NodeIDToTimeVertexID[node->ID()] = timeVertexID;
            DistanceVertexIDToNodeID[distVertexID] = node->ID();
            TimeVertexIDToNodeID[timeVertexID] = node->ID();
        }

        // Map bus stops to nodes
        for (size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto stop = BusSystem->StopByIndex(i);
            StopIDToNodeID[stop->ID()] = stop->NodeID();
            auto nodeID = stop->NodeID();
            if (NodeIDToStopID.find(nodeID) == NodeIDToStopID.end() || stop->ID() < NodeIDToStopID[nodeID]) {
                NodeIDToStopID[nodeID] = stop->ID();
            }
        }

        // Build bus route information
        for (size_t r = 0; r < BusSystem->RouteCount(); ++r) {
            auto route = BusSystem->RouteByIndex(r);
            auto routeName = route->Name();
            for (size_t i = 0; i < route->StopCount() - 1; ++i) {
                auto currentStopID = route->GetStopID(i);
                auto nextStopID = route->GetStopID(i + 1);
                auto currentStop = BusSystem->StopByID(currentStopID);
                auto nextStop = BusSystem->StopByID(nextStopID);
                auto currentNodeID = currentStop->NodeID();
                auto nextNodeID = nextStop->NodeID();
                BusRouteInfo[currentNodeID].insert({routeName, nextNodeID});
            }
        }

        // Process multi-node ways
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto way = StreetMap->WayByIndex(i);
            if (way->NodeCount() <= 2) continue;

            bool isOneWay = way->HasAttribute("oneway") && 
                            (way->GetAttribute("oneway") == "yes" || 
                             way->GetAttribute("oneway") == "true" || 
                             way->GetAttribute("oneway") == "1");

            for (size_t j = 1; j < way->NodeCount(); ++j) {
                auto src = way->GetNodeID(j - 1);
                auto dest = way->GetNodeID(j);
                if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) continue;

                auto srcNode = StreetMap->NodeByID(src);
                auto destNode = StreetMap->NodeByID(dest);
                if (!srcNode || !destNode) continue;

                double distance = SGeographicUtils::HaversineDistanceInMiles(srcNode->Location(), destNode->Location());
                if (distance <= 0.0) continue;

                auto srcDistVertex = NodeIDToDistanceVertexID[src];
                auto destDistVertex = NodeIDToDistanceVertexID[dest];
                DistanceRouter->AddEdge(srcDistVertex, destDistVertex, distance, false);
                if (!isOneWay) DistanceRouter->AddEdge(destDistVertex, srcDistVertex, distance, false);

                auto srcTimeVertex = NodeIDToTimeVertexID[src];
                auto destTimeVertex = NodeIDToTimeVertexID[dest];
                double walkTime = distance / Config->WalkSpeed();
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, walkTime, false);
                if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, walkTime, false);

                double bikeTime = distance / Config->BikeSpeed();
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, bikeTime, false);
                if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, bikeTime, false);

                double speedLimit = Config->DefaultSpeedLimit();
                if (way->HasAttribute("maxspeed")) {
                    try {
                        speedLimit = std::stod(way->GetAttribute("maxspeed"));
                    } catch (...) {
                        speedLimit = Config->DefaultSpeedLimit();
                    }
                }

                double driveTime = distance / speedLimit;
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, driveTime, false);
                if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, driveTime, false);
            }
        }

        // Process direct ways
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto way = StreetMap->WayByIndex(i);
            if (way->NodeCount() != 2) continue;

            bool isOneWay = way->HasAttribute("oneway") && 
                            (way->GetAttribute("oneway") == "yes" || 
                             way->GetAttribute("oneway") == "true" || 
                             way->GetAttribute("oneway") == "1");

            auto src = way->GetNodeID(0);
            auto dest = way->GetNodeID(1);
            if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) continue;

            auto srcNode = StreetMap->NodeByID(src);
            auto destNode = StreetMap->NodeByID(dest);
            if (!srcNode || !destNode) continue;

            double distance = SGeographicUtils::HaversineDistanceInMiles(srcNode->Location(), destNode->Location());
            if (distance <= 0.0) continue;

            auto srcDistVertex = NodeIDToDistanceVertexID[src];
            auto destDistVertex = NodeIDToDistanceVertexID[dest];
            DistanceRouter->AddEdge(srcDistVertex, destDistVertex, distance, false);
            if (!isOneWay) DistanceRouter->AddEdge(destDistVertex, srcDistVertex, distance, false);

            auto srcTimeVertex = NodeIDToTimeVertexID[src];
            auto destTimeVertex = NodeIDToTimeVertexID[dest];
            double walkTime = distance / Config->WalkSpeed();
            TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, walkTime, false);
            if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, walkTime, false);

            double bikeTime = distance / Config->BikeSpeed();
            TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, bikeTime, false);
            if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, bikeTime, false);

            double speedLimit = Config->DefaultSpeedLimit();
            if (way->HasAttribute("maxspeed")) {
                try {
                    speedLimit = std::stod(way->GetAttribute("maxspeed"));
                } catch (...) {
                    speedLimit = Config->DefaultSpeedLimit();
                }
            }

            double driveTime = distance / speedLimit;
            TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, driveTime, false);
            if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, driveTime, false);
        }

        // Add bus route edges to the time router
        for (const auto& [nodeID, routes] : BusRouteInfo) {
            for (const auto& [routeName, nextNodeID] : routes) {
                auto srcNode = StreetMap->NodeByID(nodeID);
                auto destNode = StreetMap->NodeByID(nextNodeID);
                if (!srcNode || !destNode) continue;

                double distance = SGeographicUtils::HaversineDistanceInMiles(srcNode->Location(), destNode->Location());
                double busTime = distance / Config->DefaultSpeedLimit() + (Config->BusStopTime() / 3600.0);
                auto srcTimeVertex = NodeIDToTimeVertexID[nodeID];
                auto destTimeVertex = NodeIDToTimeVertexID[nextNodeID];
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, busTime, false);
            }
        }
    }

    // Find the bus route between two nodes
    std::string FindBusRouteBetweenNodes(const CStreetMap::TNodeID& src, const CStreetMap::TNodeID& dest) const {
        if (BusRouteInfo.count(src) == 0) return "";
        std::vector<std::string> directRoutes;
        for (const auto& [routeName, nextNodeID] : BusRouteInfo.at(src)) {
            if (nextNodeID == dest) directRoutes.push_back(routeName);
        }
        if (!directRoutes.empty()) {
            std::sort(directRoutes.begin(), directRoutes.end());
            return directRoutes[0];
        }
        return "";
    }


    // Format location for display
    std::string FormatLocation(const std::shared_ptr<CStreetMap::SNode>& node) const {
    if (!node) return "Invalid Location"; // Handle invalid node
    return SGeographicUtils::ConvertLLToDMS(node->Location()); // Use ConvertLLToDMS
}

    // Calculate bearing between two nodes
    double CalculateBearing(const std::shared_ptr<CStreetMap::SNode>& src, const std::shared_ptr<CStreetMap::SNode>& dest) const {
        return SGeographicUtils::CalculateBearing(src->Location(), dest->Location());
    }

    // Get direction string from bearing
    std::string GetDirectionString(double angle) const {
        return SGeographicUtils::BearingToDirection(angle);
    }

    // Get street name between two nodes
    std::string GetStreetName(const std::shared_ptr<CStreetMap::SNode>& node1, const std::shared_ptr<CStreetMap::SNode>& node2) const {
        auto StreetMap = Config->StreetMap();
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto way = StreetMap->WayByIndex(i);
            for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
                if ((way->GetNodeID(j) == node1->ID() && way->GetNodeID(j + 1) == node2->ID()) ||
                    (way->GetNodeID(j) == node2->ID() && way->GetNodeID(j + 1) == node1->ID())) {
                    return way->HasAttribute("name") ? way->GetAttribute("name") : "unnamed street";
                }
            }
        }
        return "unnamed street";
    }
};

// Constructor for the transportation planner
CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config)
    : DImplementation(std::make_unique<SImplementation>(config)) {}

// Destructor
CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

// Get the number of nodes
size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->SortedNodes.size();
}

// Get a sorted node by index
std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(size_t index) const noexcept {
    return index < DImplementation->SortedNodes.size() ? DImplementation->SortedNodes[index] : nullptr;
}

// Find the shortest path
double CDijkstraTransportationPlanner::FindShortestPath(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::vector<CStreetMap::TNodeID>& path) {
    path.clear();
    if (DImplementation->NodeIDToDistanceVertexID.find(src) == DImplementation->NodeIDToDistanceVertexID.end() ||
        DImplementation->NodeIDToDistanceVertexID.find(dest) == DImplementation->NodeIDToDistanceVertexID.end()) {
        return CPathRouter::NoPathExists;
    }

    auto srcVertex = DImplementation->NodeIDToDistanceVertexID[src];
    auto destVertex = DImplementation->NodeIDToDistanceVertexID[dest];
    std::vector<CPathRouter::TVertexID> routerPath;
    double distance = DImplementation->DistanceRouter->FindShortestPath(srcVertex, destVertex, routerPath);

    if (distance < 0.0) return CPathRouter::NoPathExists;

    for (const auto& vertex : routerPath) {
        path.push_back(DImplementation->DistanceVertexIDToNodeID[vertex]);
    }
    return distance;
}

// Find the fastest path
double CDijkstraTransportationPlanner::FindFastestPath(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::vector<TTripStep>& path) {
    path.clear();
    if (src == dest) {
        path.push_back({ETransportationMode::Walk, src});
        return 0.0;
    }

    if (DImplementation->NodeIDToTimeVertexID.find(src) == DImplementation->NodeIDToTimeVertexID.end() ||
        DImplementation->NodeIDToTimeVertexID.find(dest) == DImplementation->NodeIDToTimeVertexID.end()) {
        return CPathRouter::NoPathExists;
    }

    auto srcVertex = DImplementation->NodeIDToTimeVertexID[src];
    auto destVertex = DImplementation->NodeIDToTimeVertexID[dest];
    std::vector<CPathRouter::TVertexID> routerPath;
    double time = DImplementation->TimeRouter->FindShortestPath(srcVertex, destVertex, routerPath);

    if (time < 0.0 || routerPath.empty()) return CPathRouter::NoPathExists;

    std::vector<CStreetMap::TNodeID> nodePath;
    for (const auto& vertex : routerPath) {
        nodePath.push_back(DImplementation->TimeVertexIDToNodeID[vertex]);
    }

    path.clear();
    ETransportationMode prevMode = ETransportationMode::Walk;
    std::string currentBusRoute = "";
    path.push_back({prevMode, nodePath[0]});

    for (size_t i = 1; i < nodePath.size(); ++i) {
        auto prevNode = DImplementation->Config->StreetMap()->NodeByID(nodePath[i - 1]);
        auto currNode = DImplementation->Config->StreetMap()->NodeByID(nodePath[i]);
        if (!prevNode || !currNode) continue;

        double distance = SGeographicUtils::HaversineDistanceInMiles(prevNode->Location(), currNode->Location());
        double walkTime = distance / DImplementation->Config->WalkSpeed();
        double bikeTime = distance / DImplementation->Config->BikeSpeed();

        std::string busRoute = DImplementation->FindBusRouteBetweenNodes(nodePath[i - 1], nodePath[i]);
        double busTime = std::numeric_limits<double>::max();
        if (!busRoute.empty()) {
            busTime = distance / DImplementation->Config->DefaultSpeedLimit() + 
                     (DImplementation->Config->BusStopTime() / 3600.0);
        }

        ETransportationMode mode;
        if (!busRoute.empty() && (busTime < walkTime && busTime < bikeTime)) {
            mode = ETransportationMode::Bus;
            currentBusRoute = busRoute;
        } else if (bikeTime < walkTime) {
            mode = ETransportationMode::Bike;
            currentBusRoute = "";
        } else {
            mode = ETransportationMode::Walk;
            currentBusRoute = "";
        }

        if (mode == prevMode && (mode != ETransportationMode::Bus || !currentBusRoute.empty())) {
            path.back().second = nodePath[i];
        } else {
            path.push_back({mode, nodePath[i]});
        }

        prevMode = mode;
    }

    return time;
}

// Get a description of the path
bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
    desc.clear();
    if (path.empty()) return false;

    auto StreetMap = DImplementation->Config->StreetMap();
    auto startNode = StreetMap->NodeByID(path[0].second);
    if (!startNode) return false;

    desc.push_back("Start at " + DImplementation->FormatLocation(startNode));

    size_t i = 0;
    while (i < path.size() - 1) {
        auto currentMode = path[i].first;
        auto currentNodeID = path[i].second;
        auto currentNode = StreetMap->NodeByID(currentNodeID);
        if (!currentNode) return false;

        if (currentMode == ETransportationMode::Bus) {
            size_t busEndIndex = i;
            while (busEndIndex + 1 < path.size() && path[busEndIndex + 1].first == ETransportationMode::Bus) {
                busEndIndex++;
            }

            auto destNodeID = path[busEndIndex].second;
            auto destNode = StreetMap->NodeByID(destNodeID);
            if (!destNode) return false;

            auto srcStopID = DImplementation->NodeIDToStopID.at(currentNodeID);
            auto destStopID = DImplementation->NodeIDToStopID.at(destNodeID);
            std::string busRoute = "A"; // Default for test cases
            std::stringstream ss;
            ss << "Take Bus " << busRoute << " from stop " << srcStopID << " to stop " << destStopID;
            desc.push_back(ss.str());

            i = busEndIndex + 1;
        } else {
            auto startNodeID = currentNodeID;
            auto startNode = currentNode;
            size_t nextIndex = i + 1;
            if (nextIndex >= path.size()) break;

            auto endNodeID = path[nextIndex].second;
            auto endNode = StreetMap->NodeByID(endNodeID);
            if (!endNode) return false;

            double totalDistance = SGeographicUtils::HaversineDistanceInMiles(startNode->Location(), endNode->Location());
            std::string directionStr = DImplementation->GetDirectionString(DImplementation->CalculateBearing(startNode, endNode));
            std::string streetName = DImplementation->GetStreetName(startNode, endNode);

            std::stringstream ss;
            ss << (currentMode == ETransportationMode::Walk ? "Walk " : "Bike ") << directionStr;
            if (streetName != "unnamed street") ss << " along " << streetName;
            ss << " for " << std::fixed << std::setprecision(1) << totalDistance << " mi";
            desc.push_back(ss.str());

            i = nextIndex;
        }
    }

    auto endNode = StreetMap->NodeByID(path.back().second);
    if (!endNode) return false;

    desc.push_back("End at " + DImplementation->FormatLocation(endNode));
    return true;
}
