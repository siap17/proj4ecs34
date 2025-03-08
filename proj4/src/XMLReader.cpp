#include "XMLReader.h"
#include <expat.h>
#include <queue>
#include <memory>
#include <vector>

struct CXMLReader::SImplementation { //implementation for the CXMLReader class 
    std::shared_ptr<CDataSource> InputSource; //this shares the pointer to data source 
    XML_Parser Parser;
    std::queue<SXMLEntity> EntityBuffer; //a queue to store XML entities 
    bool IsDataComplete;
    std::string CharacterBuffer; // a string buffer to accumulate character data 

    // this function handles start element events from Expat 
    static void HandleStartElement(void *data, const char *ele, const char **att) {
        auto *instance = static_cast<SImplementation *>(data); //This casts user data to SImplementation 
        instance->ProcessCharacterBuffer();
        SXMLEntity entity; //Entity created 
        entity.DType = SXMLEntity::EType::StartElement; //This forms a parsing 
        entity.DNameData = ele;

        // Parse attributes and pushes onto the the entity 
        if (att != nullptr) {
            for (int i = 0; att[i] != nullptr; i += 2) {
                if (att[i + 1] != nullptr) {
                    entity.DAttributes.emplace_back(att[i], att[i + 1]);
                }
            }
        }

        instance->EntityBuffer.push(entity);
    }

    //This function handles the ending element events from Expat 
    static void HandleEndElement(void *data, const char *ele) {
        auto *instance = static_cast<SImplementation *>(data); //This casts user data to SImplementation pointer 
        instance->ProcessCharacterBuffer();
        SXMLEntity entity; //We create an entity for the ending element 
        entity.DType = SXMLEntity::EType::EndElement;
        entity.DNameData = ele; //This forms the element 
        instance->EntityBuffer.push(entity); //This pushes the entity 
    }
    //This function works to append the character data to the CharacterData 
    static void HandleCharacterData(void *userData, const char *data, int length) {
        auto *instance = static_cast<SImplementation *>(userData);
        if (data != nullptr && length > 0) { 
            instance->CharacterBuffer.append(data, length);
        }
    }

    //Here, we just initiate the InputSource and set up an Expat parser 
    SImplementation(std::shared_ptr<CDataSource> source) : InputSource(std::move(source)), IsDataComplete(false) {
        Parser = XML_ParserCreate(nullptr); 
        XML_SetUserData(Parser, this);
        XML_SetElementHandler(Parser, HandleStartElement, HandleEndElement);
        XML_SetCharacterDataHandler(Parser, HandleCharacterData);
    }

    ~SImplementation() {
        XML_ParserFree(Parser);
    }

    //This accumulates the character data 
    void ProcessCharacterBuffer() {
        if (!CharacterBuffer.empty()) {
            SXMLEntity entity;
            entity.DType = SXMLEntity::EType::CharData; //Create a CharData entity 
            entity.DNameData = CharacterBuffer;
            EntityBuffer.push(entity); //Push the entity and then clear the CharacterBuffer
            CharacterBuffer.clear();
        }
    }

    //This fetches the SXMLEntity from EntityBuffer and reads more data 
    bool FetchEntity(SXMLEntity &entity, bool skipCharacterData) {
        while (EntityBuffer.empty() && !IsDataComplete) { //While the buffer is empty and data is not complete 
            std::vector<char> dataChunk(4096); // Use vector for dynamic buffer
            size_t byteread = 0; 
            while (byteread < dataChunk.size() && !InputSource->End()) {
                char byte;
                if (InputSource->Get(byte)) { //If we get the byte then we increment bytes read more data 
                    dataChunk[byteread++] = byte; 
                } else {
                    break;
                }
            }

            if (byteread == 0) { //If we have no bytes to read then end parsing as 
                // No more data to read
                IsDataComplete = true; 
                XML_Parse(Parser, nullptr, 0, 1); //This signals the parsing to end 
                break;
            }

            if (XML_Parse(Parser, dataChunk.data(), byteread, 0) == XML_STATUS_ERROR) {
                // This indicates a status error 
                return false;
            }
        }

        if (!EntityBuffer.empty()) { //This checks if the Entity buffer is full 
            entity = EntityBuffer.front(); //entity is the first element in the front 
            EntityBuffer.pop(); //This pops the entity buffer 

            // Skip character data if requested
            if (skipCharacterData && entity.DType == SXMLEntity::EType::CharData) { //This skips 
                return FetchEntity(entity, skipCharacterData); // Recursively skip character data
            }

            return true;
        }

        return false; // No more entities to read
    }
};

//This initializes the DImplementation and with a CDataSource 
CXMLReader::CXMLReader(std::shared_ptr<CDataSource> source)
    : DImplementation(std::make_unique<SImplementation>(std::move(source))) {} 

CXMLReader::~CXMLReader() = default; //Destruct defaulter since it does not require special handling 

bool CXMLReader::End() const { //This returns true if data is complete AND buffer is empty 
    return DImplementation->IsDataComplete && DImplementation->EntityBuffer.empty();
}

bool CXMLReader::ReadEntity(SXMLEntity &entity, bool skipCharacterData) { //Reads an entity by passing to FetchEntity in DImplementation 
    return DImplementation->FetchEntity(entity, skipCharacterData);
}