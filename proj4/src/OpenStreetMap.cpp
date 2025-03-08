#include "OpenStreetMap.h" 
#include "XMLReader.h" 
#include <memory> 
#include <vector> 
#include <string> 
#include <unordered_map> 

//  implementation structure first using COpenStreetMap
struct COpenStreetMap::SImplementation {
    // Forward declarations of implementation classes
    class MapNode;  // 
    class MapWay;   // forward declare the way class

    // storing ways and nodes here
    std::vector<std::shared_ptr<MapNode>> Nodes;  // list of all nodes
    std::vector<std::shared_ptr<MapWay>> Ways;    

    // helper method to handle attributes
    void ProcessAttributes(const std::vector<std::pair<std::string, std::string>>& attributes, 
                          std::unordered_map<std::string, std::string>& attributeMap) {
        for (const auto& attr : attributes) {  // loop through all attributes
            attributeMap[attr.first] = attr.second;  // add them to the map
        }
    }
};

// implementation classes using CStreetMap::SNode
class COpenStreetMap::SImplementation::MapNode : public CStreetMap::SNode {
public:
    
    TNodeID NodeID;  // unique ID for the node
    
    TLocation NodeLocation;  // latitude and longitude of the node
    
    std::unordered_map<std::string, std::string> Attributes;  // key-value pairs for attributes

   
    TNodeID ID() const noexcept override {
        return NodeID;  // return the node's ID
    }

    
    TLocation Location() const noexcept override {
        return NodeLocation;  // return the node's location
    }

    // # of attributes node has
    std::size_t AttributeCount() const noexcept override {
        return Attributes.size();  // return the number of attributes
    }

    // getting key of attribute through index
    std::string GetAttributeKey(std::size_t index) const noexcept override {
        if (index < Attributes.size()) {  // check if index is valid
            auto it = Attributes.begin();  
            std::advance(it, index);      // move to the right position
            return it->first;             
        }
        return "";  // if index is out of bounds, return empty string
    }

    //  see if the node has an attribute using key
    bool HasAttribute(const std::string &key) const noexcept override {
        return Attributes.find(key) != Attributes.end();  // look for the key
    }

    // retrieve the value of attribute if the node has an attribute
    std::string GetAttribute(const std::string &key) const noexcept override {
        auto it = Attributes.find(key);  // find the key
        if (it != Attributes.end()) {    // if found, return the value
            return it->second;
        }
        return "";  // if not found, return empty string
    }
};

// way class
class COpenStreetMap::SImplementation::MapWay : public CStreetMap::SWay {
public:
    
    TWayID WayID;  // unique ID for the way
    
    std::vector<TNodeID> NodeIDs;  // list of node IDs in this way
    
    std::unordered_map<std::string, std::string> Attributes;  // key-value pairs for attributes

    
    TWayID ID() const noexcept override {
        return WayID;  // return the way's ID
    }

    
    std::size_t NodeCount() const noexcept override {
        return NodeIDs.size();  // return the number of nodes in the way
    }

    // getting Node ID thru index
    TNodeID GetNodeID(std::size_t index) const noexcept override {
        if (index < NodeIDs.size()) {  // check if index is valid
            return NodeIDs[index];     
        }
        return CStreetMap::InvalidNodeID;  // if index is out of bounds, return invalid ID
    }

    // # of attributes way has
    std::size_t AttributeCount() const noexcept override {
        return Attributes.size();  // return the number of attributes
    }

    // getting key of attribute through index
    std::string GetAttributeKey(std::size_t index) const noexcept override {
        if (index < Attributes.size()) {  // check if index is valid
            auto it = Attributes.begin();  
            std::advance(it, index);      // move to the right position
            return it->first;        
        }
        return "";  // if index is out of bounds, return empty string
    }

    // check to see if the way has an attribute using key
    bool HasAttribute(const std::string &key) const noexcept override {
        return Attributes.find(key) != Attributes.end();  // look for the key
    }

    // get the value of attribute if the way has an attribute
    std::string GetAttribute(const std::string &key) const noexcept override {
        auto it = Attributes.find(key);  // find the key
        if (it != Attributes.end()) {    // if found, return the value
            return it->second;
        }
        return "";  // if not found, return empty string
    }
};

// initialize the implementation
COpenStreetMap::COpenStreetMap(std::shared_ptr<CXMLReader> src) {
    DImplementation = std::make_unique<SImplementation>();  // create the implementation

    SXMLEntity entity;  // temporary storage for XML elements
    std::shared_ptr<SImplementation::MapNode> currentNode = nullptr;  // current node being processed
    std::shared_ptr<SImplementation::MapWay> currentWay = nullptr;    // current way being processed

    // Parsing the XML file
    while (src->ReadEntity(entity)) {  // read the XML file line by line
        if (entity.DType == SXMLEntity::EType::StartElement) {  // if it's a start tag
            if (entity.DNameData == "node") {  // if it's a node
                currentNode = std::make_shared<SImplementation::MapNode>();  // create a new node
                currentWay = nullptr;  // reset the current way

                // Process node attributes
                for (const auto& attr : entity.DAttributes) {  // loop through attributes
                    if (attr.first == "id") {  // if it's the ID
                        currentNode->NodeID = std::stoull(attr.second);  
                    } else if (attr.first == "lat") {  // if it's latitude
                        currentNode->NodeLocation.first = std::stod(attr.second);  // store latitude
                    } else if (attr.first == "lon") {  // if it's longitude
                        currentNode->NodeLocation.second = std::stod(attr.second);  // store longitude
                    } else {  // if it's another attribute
                        currentNode->Attributes[attr.first] = attr.second;  // store it
                    }
                }
            } else if (entity.DNameData == "way") {  // if it's a way
                currentWay = std::make_shared<SImplementation::MapWay>();  // create a new way
                currentNode = nullptr;  // reset the current node

                // Process way attributes
                for (const auto& attr : entity.DAttributes) {  // loop through attributes
                    if (attr.first == "id") {  // if it's the ID
                        currentWay->WayID = std::stoull(attr.second);  // store the ID
                    } else {  // if it's another attribute
                        currentWay->Attributes[attr.first] = attr.second;  // store it
                    }
                }
            } else if (entity.DNameData == "nd" && currentWay) {  // if it's a node reference in a way
                for (const auto& attr : entity.DAttributes) {  // process the reference
                    if (attr.first == "ref") {  // if it's the node ID
                        currentWay->NodeIDs.push_back(std::stoull(attr.second));  // add it to the way
                    }
                }
            } else if (entity.DNameData == "tag") {  // if it's a tag (attribute)
                std::string key, value;  // temporary storage for key and value
                for (const auto& attr : entity.DAttributes) {  // process the tag
                    if (attr.first == "k") {  // if it's the key
                        key = attr.second;  // store the key
                    } else if (attr.first == "v") {  // if it's the value
                        value = attr.second;  // store the value
                    }
                }
                if (!key.empty()) {  // if the key is not empty
                    if (currentNode) {  // if we're processing a node
                        currentNode->Attributes[key] = value;  // add the attribute to the node
                    } else if (currentWay) {  // if we're processing a way
                        currentWay->Attributes[key] = value;  // add the attribute to the way
                    }
                }
            }
        } else if (entity.DType == SXMLEntity::EType::EndElement) {  // if it's an end tag
            if (entity.DNameData == "node" && currentNode) { 
                DImplementation->Nodes.push_back(currentNode);  // add the node to the list
                currentNode = nullptr;  // reset the current node
            } else if (entity.DNameData == "way" && currentWay) {  //end of way
                DImplementation->Ways.push_back(currentWay);  // add the way to the list
                currentWay = nullptr;  // reset the current way
            }
        }
    }
}

// destr
COpenStreetMap::~COpenStreetMap() = default;  // destructor (does nothing special)

// total count of nodes
std::size_t COpenStreetMap::NodeCount() const noexcept {
    return DImplementation->Nodes.size();  // return the number of nodes
}

// total count of ways
std::size_t COpenStreetMap::WayCount() const noexcept {
    return DImplementation->Ways.size();  // return the number of ways
}

// get node by index
std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->Nodes.size()) {  // check if index is valid
        return DImplementation->Nodes[index];  // return the node
    }
    return nullptr;  // if index is out of bounds, return null
}

// get node by ID
std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByID(TNodeID id) const noexcept {
    for (auto& node : DImplementation->Nodes) {  // loop through all nodes
        if (node->ID() == id) {  // if the ID matches
            return node;  // return the node
        }
    }
    return nullptr;  // if no match, return null
}

// get way by index
std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->Ways.size()) {  // check if index is valid
        return DImplementation->Ways[index];  // return the way
    }
    return nullptr;  // if index is out of bounds, return null
}

// get way by ID
std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByID(TWayID id) const noexcept {
    for (auto& way : DImplementation->Ways) {  // loop through all ways
        if (way->ID() == id) {  // if the ID matches
            return way;  // return the way
        }
    }
    return nullptr;  // if no match, return null
}
