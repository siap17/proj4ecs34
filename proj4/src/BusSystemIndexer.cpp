#include "BusSystemIndexer.h"
#include "BusSystem.h"
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

//  impl for bus system indexer
struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> BusSystem;  // shared ptr to the bus system
    std::vector<std::shared_ptr<SStop>> SortedStops;  // holds sorted bus stops
    std::vector<std::shared_ptr<SRoute>> SortedRoutes;  // holds sorted bus routes
    std::unordered_map<TNodeID, std::shared_ptr<SStop>> NodeIDToStopMap;  // maps node ID to bus stop
    std::unordered_map<CBusSystem::TStopID, std::unordered_set<std::shared_ptr<SRoute>>> StopIDToRoutesMap;  // maps stop ID to routes

    // constructor that takes a bus system and precomputes needed info
    SImplementation(std::shared_ptr<CBusSystem> bussystem) : BusSystem(bussystem) {
        // loop thru all stops and add them to SortedStops
        for (std::size_t i = 0; i < BusSystem->StopCount(); ++i) {
            SortedStops.push_back(BusSystem->StopByIndex(i));
        }
        // sort stops by ID
        std::sort(SortedStops.begin(), SortedStops.end(), [](const auto& a, const auto& b) {
            return a->ID() < b->ID();
        });

        // loop thru all routes + add them to SortedRoutes
        for (std::size_t i = 0; i < BusSystem->RouteCount(); ++i) {
            SortedRoutes.push_back(BusSystem->RouteByIndex(i));
        }
        // sort routes by name
        std::sort(SortedRoutes.begin(), SortedRoutes.end(), [](const auto& a, const auto& b) {
            return a->Name() < b->Name();
        });

        // create map for node ID to stop
        for (const auto& stop : SortedStops) {
            NodeIDToStopMap[stop->NodeID()] = stop;
        }

        // create  map for stop ID to routes that go through that stop
        for (const auto& route : SortedRoutes) {
            for (std::size_t i = 0; i < route->StopCount(); ++i) {
                StopIDToRoutesMap[route->GetStopID(i)].insert(route);
            }
        }
    }

    // ret  number of stops
    std::size_t StopCount() const {
        return BusSystem->StopCount();
    }

    // returns  # of routes
    std::size_t RouteCount() const {
        return BusSystem->RouteCount();
    }

    // get sorted stop by its index
    std::shared_ptr<SStop> SortedStopByIndex(std::size_t index) const {
        return (index < SortedStops.size()) ? SortedStops[index] : nullptr;
    }

    // gets a sorted route by its index
    std::shared_ptr<SRoute> SortedRouteByIndex(std::size_t index) const {
        return (index < SortedRoutes.size()) ? SortedRoutes[index] : nullptr;
    }

    std::shared_ptr<SStop> StopByNodeID(TNodeID id) const { // finds  stop by its node ID
        auto it = NodeIDToStopMap.find(id);  // search map for the node ID
        return (it != NodeIDToStopMap.end()) ? it->second : nullptr;  // ret stop if found
    }

    // gets all routes that connect two nodes
    bool RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>>& routes) const {
        auto srcStop = StopByNodeID(src);  // find the start stop by node ID
        auto destStop = StopByNodeID(dest);  // find the dest stop by node ID
        if (!srcStop || !destStop) {  // if either stop is not found, ret false
            return false;
        }

        // routes for both stops
        const auto& srcRoutes = StopIDToRoutesMap.at(srcStop->ID());
        const auto& destRoutes = StopIDToRoutesMap.at(destStop->ID());

        
        for (const auto& route : srcRoutes) { // find common routes between the source and destination stops
            if (destRoutes.find(route) != destRoutes.end()) {
                routes.insert(route);
            }
        }

        
        return !routes.empty(); // if we found any common routes ret true
    }

    bool RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const { // checks if thers a route between two nodes
        std::unordered_set<std::shared_ptr<SRoute>> routes;
        return RoutesByNodeIDs(src, dest, routes);  // use the RoutesByNodeIDs method
    }
};

// CBusSystemIndexer constructor that creates the implementation
CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem) {
    DImplementation = std::make_unique<SImplementation>(bussystem);  // create the impl obj
}

// destructor
CBusSystemIndexer::~CBusSystemIndexer() = default;


std::size_t CBusSystemIndexer::StopCount() const noexcept { // ret # stops
    return DImplementation->StopCount();
}

std::size_t CBusSystemIndexer::RouteCount() const noexcept { // ret # routes
    return DImplementation->RouteCount();
}


std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedStopByIndex(index);
}


std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedRouteByIndex(index);
}


std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::StopByNodeID(TNodeID id) const noexcept {
    return DImplementation->StopByNodeID(id);
}


bool CBusSystemIndexer::RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>>& routes) const noexcept {
    return DImplementation->RoutesByNodeIDs(src, dest, routes);
}


bool CBusSystemIndexer::RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept { // checks theres a route between two node IDs
    return DImplementation->RouteBetweenNodeIDs(src, dest);
}
