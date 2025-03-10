#include "CSVBusSystem.h"
#include "BusSystem.h"
#include <vector>
#include <memory>
#include <unordered_map> 
#include <string> 
#include <iostream> 
#include <sstream> 


// Private Implementation
class CCSVBusSystem::SStop : public CBusSystem::SStop {     //Create a bus stop class 
    public: 
        TStopID DStopID; 
        CStreetMap::TNodeID NodeIDVal;                      //Initialized a NodeIDVal to retrieve the value of the Node and of the DStopID 

        TStopID ID() const noexcept override {             //Overriding everything to avoid abstraction from disrupting the virtual functions 
            return DStopID; 
        } 

        CStreetMap::TNodeID NodeID() const noexcept override {
            return NodeIDVal; 
        }
    }; 

class CCSVBusSystem::SRoute : public CBusSystem::SRoute {     //Created an SRoute Class to represent the bus route but it is essentially a nested class 
    public: 
        std::string DName;                   //This establishes the name of the route essentially 
        std::vector<TStopID> DStopIDs;       //This lists the Stop IDs that form the route 

        // This returns the name of the route hence why we call on override 
        std::string Name() const noexcept override{ 
            return DName; 
        }

        //This returns the size "aka" number of stopping points int he route 
        std::size_t StopCount() const noexcept override{ 
            return DStopIDs.size(); 
        }

        //This returns the stop ID to the corresponding index 
        TStopID GetStopID(std::size_t index) const noexcept override{
            if (index < DStopIDs.size()){
                return DStopIDs[index]; 
            }
            return CBusSystem::InvalidStopID;            //Otherwise it would return an invalid ID 
        }
    }; 

   struct CCSVBusSystem::SImplementation{                 //Esetablishes the private implementation; Helper structure to handle data 
        std::vector<std::shared_ptr<SStop>> DStops;       // DStops and Droutes hold the list of all stops and routes 
        std::vector<std::shared_ptr<SRoute>> DRoutes; 
        std::unordered_map<TStopID, std::shared_ptr<SStop>> DStopByIDMap;        //While DStopByIDMap and DRouteByNameMap map stopIDs and route names to their respective objects 
        std::unordered_map<std::string, std::shared_ptr<SRoute>> DRouteByNameMap;


        SImplementation(std::shared_ptr<CDSVReader> stopsrc, std::shared_ptr<CDSVReader>routesrc){ //This works as a CSV reader 

        }
    }; 

CCSVBusSystem::CCSVBusSystem(std::shared_ptr<CDSVReader> stopsrc, std::shared_ptr<CDSVReader> routesrc){
    DImplementation = std::make_unique<SImplementation>(stopsrc, routesrc); 
    std:: vector<std::string> stopRow; 

    // This works to read the stop data essentially 
    if (stopsrc){        
        while (stopsrc->ReadRow(stopRow)){                      //While stopsrc is reading the current row and if stoprow is less than or equal to 2 
            if (stopRow.size() >= 2){
                try {
                    auto stop = std::make_shared<SStop>();     // This reads every row of the stopsrc 
                    stop->DStopID=std::stoul(stopRow[0]); 
                    stop->NodeIDVal = std::stoul(stopRow[1]);  // This converst the string data to unsigned long aka std::stoul 
                    DImplementation->DStops.push_back(stop);   // We push back the stop and store it in Dstops and then DStopByIDMap so we can access it for lookups later 
                    DImplementation->DStopByIDMap[stop->DStopID] = stop;
                } catch (const std::exception& e){            // Handles error and exceptions 
                    std::cerr << "Caught an exception" << e.what() << "\n"; 
                }
            }
        }   
    }

    if (routesrc) {                                             // This functions reads the routes 
        std::unordered_map<std::string, std::shared_ptr<SRoute>> routeMap;    //This creates a temporary routeMap to store routes

        while (routesrc->ReadRow(stopRow)) {                    // This reads every line and we set stopRow.size() >= 2 
            if (stopRow.size() >= 2) {
                try {
                    std::string rName = stopRow[0];            //Indexing so routename starts at 0 and stop id starts after. We change Stop ID to unsigned 
                    TStopID stopID = std::stoul(stopRow[1]);
                    
                    auto& route = routeMap[rName];            //We group teh routes together here and 
                    if (!route) {                            //If a route does not exist then we just push the stopID 
                        route = std::make_shared<SRoute>();
                        route->DName = rName;
                    }
                    route->DStopIDs.push_back(stopID);
                } catch (const std::exception& e) {         //This handles the exceptions 
                    std::cerr << "Error processing route row: " << e.what() << "\n";
                }
            }
        }

        // Transfer routes to implementation
        for (const auto& pair : routeMap) {    
            DImplementation->DRouteByNameMap[pair.first] = pair.second;        //We transfer routes from routeMap list to DImplementation so we can actually use it 
            DImplementation->DRoutes.push_back(pair.second);      
        }
    }
}

// This is our destructor 
CCSVBusSystem::~CCSVBusSystem() = default;

//These return the number of stops and routes 
std::size_t CCSVBusSystem::StopCount() const noexcept { 
    return DImplementation->DStops.size(); 
}

std::size_t CCSVBusSystem::RouteCount() const noexcept { 
    return DImplementation->DRoutes.size(); 
}
//This retrieves the stops by size and returns a nullptr if else 
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept { 
    if (index < DImplementation->DStops.size()){
        return DImplementation->DStops[index]; 
    }
    return nullptr; 
}

//THis function returns a stop by id 
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept { 
    auto it = DImplementation->DStopByIDMap.find(id); 
    if (it != DImplementation ->DStopByIDMap.end()){
        return it->second; 
    }
    return nullptr; 
}

//This retrieves routes by index 
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept { 
    if (index < DImplementation->DRoutes.size()){
        return DImplementation->DRoutes[index];
    }
    return nullptr;
}

std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string &name) const noexcept { 
    
    auto it = DImplementation->DRouteByNameMap.find(name); 
    if (it != DImplementation->DRouteByNameMap.end()){
        return it->second; 
    }
    return nullptr; 
}

//This handles the operator overloading <<

std::ostream& operator<<(std::ostream& os, const CCSVBusSystem& busSystem) {
    os << "Bus System Details:\n";                         //Prints the Bus System Details 
    os << "Stop Count: " << busSystem.StopCount() << "\n"; //Prints the number of stops 
    for (std::size_t i = 0; i < busSystem.StopCount(); ++i) { //Loop through every stop
        auto stopPtr = busSystem.StopByIndex(i);              //Initialized StopPtr a shared pointer to SStop 
        if (stopPtr) { 
            os << "Stop " << i << ": ID = " << stopPtr->ID()  //If StopPtr is not null then we just print the Id and NodeId 
               << ", NodeID = " << stopPtr->NodeID() << "\n"; 
        }
    }

    os << "Route Count: " << busSystem.RouteCount() << "\n"; //Prints the Number of Routes 
    for (std::size_t i = 0; i < busSystem.RouteCount(); ++i) { //Loops through every route 
        auto routePtr = busSystem.RouteByIndex(i);             //Initialized a routepoointer similarly 
        if (routePtr) {                                        //If routePtr is not null then returns the routeName = routePtr->Name() and stopCount = routePTr->StopCount() 
            os << "Route " << i << ": Name = " << routePtr->Name()
               << ", StopCount = " << routePtr->StopCount() << "\n"; 
            os << "Stops: "; 

            for (std::size_t j = 0; j < routePtr->StopCount(); ++j) {    //Loop through the stop IDs in the route again and returns correspodning stop ID 
                os << routePtr->GetStopID(j); 
                if (j < routePtr->StopCount() - 1) {
                    os << ", "; 
                }
            }
            os << "\n"; 
        }
    }
    return os; 
}