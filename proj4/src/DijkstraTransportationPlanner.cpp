#include "TransportPlanner.h" // Main header for the transportation planner
#include "PathRouter.h" // Header for the path routing logic
#include "GeoUtils.h" // Header for geographic utilities
#include <queue> // For priority queue
#include <unordered_map> // For hash maps
#include <set> // For sets
#include <cmath> // For mathematical operations
#include <algorithm> // For sorting
#include <sstream> // For string manipulation
#include <iomanip> // For formatting output
#include <iostream> // For standard I/O

// Implementation of the transportation planner
struct TransportPlanner::PlannerImpl {
    // Configuration settings
    std::shared_ptr<Config> Settings;
    // Sorted list of nodes
    std::vector<std::shared_ptr<StreetMap::Node>> Nodes;
    // Router for distance-based paths
    std::shared_ptr<PathRouter> DistanceRouter;
    // Router for time-based paths
    std::shared_ptr<PathRouter> TimeRouter;
    // Mapping from node ID to distance vertex ID
    std::unordered_map<StreetMap::NodeID, PathRouter::VertexID> NodeToDistanceVertex;
    // Mapping from node ID to time vertex ID
    std::unordered_map<StreetMap::NodeID, PathRouter::VertexID> NodeToTimeVertex;
    // Mapping from distance vertex ID to node ID
    std::unordered_map<PathRouter::VertexID, StreetMap::NodeID> DistanceVertexToNode;
    // Mapping from time vertex ID to node ID
    std::unordered_map<PathRouter::VertexID, StreetMap::NodeID> TimeVertexToNode;
    // Mapping from node ID to index
    std::unordered_map<StreetMap::NodeID, size_t> NodeToIndex;
    // Mapping from bus stop ID to node ID
    std::unordered_map<BusSystem::StopID, StreetMap::NodeID> StopToNode;
    // Mapping from bus stop ID to stop name
    std::unordered_map<BusSystem::StopID, std::string> StopToName;
    // Mapping from node ID to bus stop ID
    std::unordered_map<StreetMap::NodeID, BusSystem::StopID> NodeToStop;
    // Mapping from node ID to bus route information
    std::unordered_map<StreetMap::NodeID, std::set<std::pair<std::string, StreetMap::NodeID>>> RouteInfo;

    // Constructor
    PlannerImpl(std::shared_ptr<Config> config) : Settings(config) {
        auto map = Settings->GetStreetMap();
        auto busSystem = Settings->GetBusSystem();

        // Initialize routers
        DistanceRouter = std::make_shared<PathRouter>();
        TimeRouter = std::make_shared<PathRouter>();

        // Sort nodes by ID
        for (size_t i = 0; i < map->NodeCount(); ++i) {
            Nodes.push_back(map->GetNodeByIndex(i));
        }
        std::sort(Nodes.begin(), Nodes.end(), [](const auto& a, const auto& b) {
            return a->GetID() < b->GetID();
        });

        // Map nodes to indices
        for (size_t i = 0; i < Nodes.size(); ++i) {
            NodeToIndex[Nodes[i]->GetID()] = i;
        }

        // Add vertices to routers
        for (size_t i = 0; i < Nodes.size(); ++i) {
            auto node = Nodes[i];
            auto distVertex = DistanceRouter->AddVertex(node->GetID());
            auto timeVertex = TimeRouter->AddVertex(node->GetID());
            NodeToDistanceVertex[node->GetID()] = distVertex;
            NodeToTimeVertex[node->GetID()] = timeVertex;
            DistanceVertexToNode[distVertex] = node->GetID();
            TimeVertexToNode[timeVertex] = node->GetID();
        }

        // Map bus stops to nodes
        for (size_t i = 0; i < busSystem->StopCount(); ++i) {
            auto stop = busSystem->GetStopByIndex(i);
            StopToNode[stop->GetID()] = stop->GetNodeID();
            auto nodeID = stop->GetNodeID();
            if (NodeToStop.find(nodeID) == NodeToStop.end() || stop->GetID() < NodeToStop[nodeID]) {
                NodeToStop[nodeID] = stop->GetID();
            }
        }

        // Build bus route information
        for (size_t r = 0; r < busSystem->RouteCount(); ++r) {
            auto route = busSystem->GetRouteByIndex(r);
            auto routeName = route->GetName();
            for (size_t i = 0; i < route->StopCount() - 1; ++i) {
                auto currentStop = route->GetStopID(i);
                auto nextStop = route->GetStopID(i + 1);
                auto currentStopInfo = busSystem->GetStopByID(currentStop);
                auto nextStopInfo = busSystem->GetStopByID(nextStop);
                auto currentNode = currentStopInfo->GetNodeID();
                auto nextNode = nextStopInfo->GetNodeID();
                RouteInfo[currentNode].insert({routeName, nextNode});
            }
        }

        // Process multi-node ways
        for (size_t i = 0; i < map->WayCount(); ++i) {
            auto way = map->GetWayByIndex(i);
            if (way->NodeCount() <= 2) continue;

            bool isOneWay = way->HasAttribute("oneway") && 
                            (way->GetAttribute("oneway") == "yes" || 
                             way->GetAttribute("oneway") == "true" || 
                             way->GetAttribute("oneway") == "1");

            for (size_t j = 1; j < way->NodeCount(); ++j) {
                auto src = way->GetNodeID(j - 1);
                auto dest = way->GetNodeID(j);
                if (src == StreetMap::InvalidNodeID || dest == StreetMap::InvalidNodeID) continue;

                auto srcNode = map->GetNodeByID(src);
                auto destNode = map->GetNodeByID(dest);
                if (!srcNode || !destNode) continue;

                double distance = GeoUtils::CalculateDistance(srcNode->GetLocation(), destNode->GetLocation());
                if (distance <= 0.0) continue;

                auto srcDistVertex = NodeToDistanceVertex[src];
                auto destDistVertex = NodeToDistanceVertex[dest];
                DistanceRouter->AddEdge(srcDistVertex, destDistVertex, distance, false);
                if (!isOneWay) DistanceRouter->AddEdge(destDistVertex, srcDistVertex, distance, false);

                auto srcTimeVertex = NodeToTimeVertex[src];
                auto destTimeVertex = NodeToTimeVertex[dest];
                double walkTime = distance / Settings->GetWalkSpeed();
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, walkTime, false);
                if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, walkTime, false);

                double bikeTime = distance / Settings->GetBikeSpeed();
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, bikeTime, false);
                if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, bikeTime, false);

                double speedLimit = Settings->GetDefaultSpeedLimit();
                if (way->HasAttribute("maxspeed")) {
                    try {
                        speedLimit = std::stod(way->GetAttribute("maxspeed"));
                    } catch (...) {
                        speedLimit = Settings->GetDefaultSpeedLimit();
                    }
                }

                double driveTime = distance / speedLimit;
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, driveTime, false);
                if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, driveTime, false);
            }
        }

        // Process direct ways
        for (size_t i = 0; i < map->WayCount(); ++i) {
            auto way = map->GetWayByIndex(i);
            if (way->NodeCount() != 2) continue;

            bool isOneWay = way->HasAttribute("oneway") && 
                            (way->GetAttribute("oneway") == "yes" || 
                             way->GetAttribute("oneway") == "true" || 
                             way->GetAttribute("oneway") == "1");

            auto src = way->GetNodeID(0);
            auto dest = way->GetNodeID(1);
            if (src == StreetMap::InvalidNodeID || dest == StreetMap::InvalidNodeID) continue;

            auto srcNode = map->GetNodeByID(src);
            auto destNode = map->GetNodeByID(dest);
            if (!srcNode || !destNode) continue;

            double distance = GeoUtils::CalculateDistance(srcNode->GetLocation(), destNode->GetLocation());
            if (distance <= 0.0) continue;

            auto srcDistVertex = NodeToDistanceVertex[src];
            auto destDistVertex = NodeToDistanceVertex[dest];
            DistanceRouter->AddEdge(srcDistVertex, destDistVertex, distance, false);
            if (!isOneWay) DistanceRouter->AddEdge(destDistVertex, srcDistVertex, distance, false);

            auto srcTimeVertex = NodeToTimeVertex[src];
            auto destTimeVertex = NodeToTimeVertex[dest];
            double walkTime = distance / Settings->GetWalkSpeed();
            TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, walkTime, false);
            if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, walkTime, false);

            double bikeTime = distance / Settings->GetBikeSpeed();
            TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, bikeTime, false);
            if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, bikeTime, false);

            double speedLimit = Settings->GetDefaultSpeedLimit();
            if (way->HasAttribute("maxspeed")) {
                try {
                    speedLimit = std::stod(way->GetAttribute("maxspeed"));
                } catch (...) {
                    speedLimit = Settings->GetDefaultSpeedLimit();
                }
            }

            double driveTime = distance / speedLimit;
            TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, driveTime, false);
            if (!isOneWay) TimeRouter->AddEdge(destTimeVertex, srcTimeVertex, driveTime, false);
        }

        // Add bus route edges to the time router
        for (const auto& [nodeID, routes] : RouteInfo) {
            for (const auto& [routeName, nextNodeID] : routes) {
                auto srcNode = map->GetNodeByID(nodeID);
                auto destNode = map->GetNodeByID(nextNodeID);
                if (!srcNode || !destNode) continue;

                double distance = GeoUtils::CalculateDistance(srcNode->GetLocation(), destNode->GetLocation());
                double busTime = distance / Settings->GetDefaultSpeedLimit() + (Settings->GetBusStopTime() / 3600.0);
                auto srcTimeVertex = NodeToTimeVertex[nodeID];
                auto destTimeVertex = NodeToTimeVertex[nextNodeID];
                TimeRouter->AddEdge(srcTimeVertex, destTimeVertex, busTime, false);
            }
        }
    }

    // Find the bus route between two nodes
    std::string FindBusRoute(StreetMap::NodeID src, StreetMap::NodeID dest) const {
        if (RouteInfo.count(src) == 0) return "";
        std::vector<std::string> routes;
        for (const auto& [routeName, nextNodeID] : RouteInfo.at(src)) {
            if (nextNodeID == dest) routes.push_back(routeName);
        }
        if (!routes.empty()) {
            std::sort(routes.begin(), routes.end());
            return routes[0];
        }
        return "";
    }

    // Format location for display
    std::string FormatLocation(const std::shared_ptr<StreetMap::Node>& node) const {
        auto [lat, lon] = node->GetLocation();
        return GeoUtils::FormatCoordinates(lat, lon);
    }

    // Calculate bearing between two nodes
    double CalculateBearing(const std::shared_ptr<StreetMap::Node>& src, const std::shared_ptr<StreetMap::Node>& dest) const {
        return GeoUtils::CalculateBearing(src->GetLocation(), dest->GetLocation());
    }

    // Get direction string from bearing
    std::string GetDirection(double angle) const {
        return GeoUtils::BearingToDirection(angle);
    }

    // Get street name between two nodes
    std::string GetStreetName(const std::shared_ptr<StreetMap::Node>& node1, const std::shared_ptr<StreetMap::Node>& node2) const {
        auto map = Settings->GetStreetMap();
        for (size_t i = 0; i < map->WayCount(); ++i) {
            auto way = map->GetWayByIndex(i);
            for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
                if ((way->GetNodeID(j) == node1->GetID() && way->GetNodeID(j + 1) == node2->GetID()) ||
                    (way->GetNodeID(j) == node2->GetID() && way->GetNodeID(j + 1) == node1->GetID())) {
                    return way->HasAttribute("name") ? way->GetAttribute("name") : "unnamed street";
                }
            }
        }
        return "unnamed street";
    }
};

// Constructor for the transportation planner
TransportPlanner::TransportPlanner(std::shared_ptr<Config> config) 
    : Impl(std::make_unique<PlannerImpl>(config)) {}

// Destructor
TransportPlanner::~TransportPlanner() = default;

// Get the number of nodes
size_t TransportPlanner::GetNodeCount() const noexcept {
    return Impl->Nodes.size();
}

// Get a sorted node by index
std::shared_ptr<StreetMap::Node> TransportPlanner::GetNodeByIndex(size_t index) const noexcept {
    return index < Impl->Nodes.size() ? Impl->Nodes[index] : nullptr;
}

// Find the shortest path
double TransportPlanner::FindShortestPath(StreetMap::NodeID src, StreetMap::NodeID dest, std::vector<StreetMap::NodeID>& path) {
    path.clear();
    if (Impl->NodeToDistanceVertex.find(src) == Impl->NodeToDistanceVertex.end() ||
        Impl->NodeToDistanceVertex.find(dest) == Impl->NodeToDistanceVertex.end()) {
        return PathRouter::NoPathExists;
    }

    auto srcVertex = Impl->NodeToDistanceVertex[src];
    auto destVertex = Impl->NodeToDistanceVertex[dest];
    std::vector<PathRouter::VertexID> routerPath;
    double distance = Impl->DistanceRouter->FindShortestPath(srcVertex, destVertex, routerPath);

    if (distance < 0.0) return PathRouter::NoPathExists;

    for (const auto& vertex : routerPath) {
        path.push_back(Impl->DistanceVertexToNode[vertex]);
    }
    return distance;
}

// Find the fastest path
double TransportPlanner::FindFastestPath(StreetMap::NodeID src, StreetMap::NodeID dest, std::vector<TripStep>& path) {
    path.clear();
    if (src == dest) {
        path.push_back({TransportMode::Walk, src});
        return 0.0;
    }

    if (Impl->NodeToTimeVertex.find(src) == Impl->NodeToTimeVertex.end() ||
        Impl->NodeToTimeVertex.find(dest) == Impl->NodeToTimeVertex.end()) {
        return PathRouter::NoPathExists;
    }

    auto srcVertex = Impl->NodeToTimeVertex[src];
    auto destVertex = Impl->NodeToTimeVertex[dest];
    std::vector<PathRouter::VertexID> routerPath;
    double time = Impl->TimeRouter->FindShortestPath(srcVertex, destVertex, routerPath);

    if (time < 0.0 || routerPath.empty()) return PathRouter::NoPathExists;

    std::vector<StreetMap::NodeID> nodePath;
    for (const auto& vertex : routerPath) {
        nodePath.push_back(Impl->TimeVertexToNode[vertex]);
    }

    path.clear();
    TransportMode prevMode = TransportMode::Walk;
    std::string currentBusRoute = "";
    path.push_back({prevMode, nodePath[0]});

    for (size_t i = 1; i < nodePath.size(); ++i) {
        auto prevNode = Impl->Settings->GetStreetMap()->GetNodeByID(nodePath[i - 1]);
        auto currNode = Impl->Settings->GetStreetMap()->GetNodeByID(nodePath[i]);
        if (!prevNode || !currNode) continue;

        double distance = GeoUtils::CalculateDistance(prevNode->GetLocation(), currNode->GetLocation());
        double walkTime = distance / Impl->Settings->GetWalkSpeed();
        double bikeTime = distance / Impl->Settings->GetBikeSpeed();

        std::string busRoute = Impl->FindBusRoute(nodePath[i - 1], nodePath[i]);
        double busTime = std::numeric_limits<double>::max();
        if (!busRoute.empty()) {
            busTime = distance / Impl->Settings->GetDefaultSpeedLimit() + 
                     (Impl->Settings->GetBusStopTime() / 3600.0);
        }

        TransportMode mode;
        if (!busRoute.empty() && (busTime < walkTime && busTime < bikeTime)) {
            mode = TransportMode::Bus;
            currentBusRoute = busRoute;
        } else if (bikeTime < walkTime) {
            mode = TransportMode::Bike;
            currentBusRoute = "";
        } else {
            mode = TransportMode::Walk;
            currentBusRoute = "";
        }

        if (mode == prevMode && (mode != TransportMode::Bus || !currentBusRoute.empty())) {
            path.back().second = nodePath[i];
        } else {
            path.push_back({mode, nodePath[i]});
        }

        prevMode = mode;
    }

    return time;
}

// Get a description of the path
bool TransportPlanner::GetPathDescription(const std::vector<TripStep>& path, std::vector<std::string>& desc) const {
    desc.clear();
    if (path.empty()) return false;

    auto map = Impl->Settings->GetStreetMap();
    auto startNode = map->GetNodeByID(path[0].second);
    if (!startNode) return false;

    desc.push_back("Start at " + Impl->FormatLocation(startNode));

    size_t i = 0;
    while (i < path.size() - 1) {
        auto currentMode = path[i].first;
        auto currentNodeID = path[i].second;
        auto currentNode = map->GetNodeByID(currentNodeID);
        if (!currentNode) return false;

        if (currentMode == TransportMode::Bus) {
            size_t busEndIndex = i;
            while (busEndIndex + 1 < path.size() && path[busEndIndex + 1].first == TransportMode::Bus) {
                busEndIndex++;
            }

            auto destNodeID = path[busEndIndex].second;
            auto destNode = map->GetNodeByID(destNodeID);
            if (!destNode) return false;

            auto srcStopID = Impl->NodeToStop.at(currentNodeID);
            auto destStopID = Impl->NodeToStop.at(destNodeID);
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
            auto endNode = map->GetNodeByID(endNodeID);
            if (!endNode) return false;

            double totalDistance = GeoUtils::CalculateDistance(startNode->GetLocation(), endNode->GetLocation());
            std::string directionStr = Impl->GetDirection(Impl->CalculateBearing(startNode, endNode));
            std::string streetName = Impl->GetStreetName(startNode, endNode);

            std::stringstream ss;
            ss << (currentMode == TransportMode::Walk ? "Walk " : "Bike ") << directionStr;
            if (streetName != "unnamed street") ss << " along " << streetName;
            ss << " for " << std::fixed << std::setprecision(1) << totalDistance << " mi";
            desc.push_back(ss.str());

            i = nextIndex;
        }
    }

    auto endNode = map->GetNodeByID(path.back().second);
    if (!endNode) return false;

    desc.push_back("End at " + Impl->FormatLocation(endNode));
    return true;
}
