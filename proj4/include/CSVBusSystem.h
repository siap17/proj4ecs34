#ifndef CSVBUSROUTE_H
#define CSVBUSROUTE_H

#include "BusSystem.h"
#include "DSVReader.h"
#include <memory> 
#include <unordered_map>
#include <vector> 
#include <string> 

class CCSVBusSystem : public CBusSystem{
    private:
        class SStop; 
        class SRoute; 
        struct SImplementation; 
        std::unique_ptr< SImplementation > DImplementation;
    public:
        CCSVBusSystem(std::shared_ptr< CDSVReader > stopsrc, std::shared_ptr< CDSVReader > routesrc);
        ~CCSVBusSystem();

        std::size_t StopCount() const noexcept override;
        std::size_t RouteCount() const noexcept override;
        std::shared_ptr<CBusSystem::SStop> StopByIndex(std::size_t index) const noexcept override;
        std::shared_ptr<CBusSystem::SStop> StopByID(TStopID id) const noexcept override;
        std::shared_ptr<CBusSystem::SRoute> RouteByIndex(std::size_t index) const noexcept override;
        std::shared_ptr<CBusSystem::SRoute> RouteByName(const std::string &name) const noexcept override;
};

std::ostream& operator<<(std::ostream& os, const CCSVBusSystem& busSystem); 
#endif