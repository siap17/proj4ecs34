#include "XMLWriter.h"
#include <stack>
#include <string>
#include <deque>

struct CXMLWriter::SImplementation {
    std::shared_ptr<CDataSink> OutputSink;  // holds the output sink for writing data
    std::deque<std::string> OpenElements;   // keeps track of open elements using a deque instead of a stack

    SImplementation(std::shared_ptr<CDataSink> sink)
        : OutputSink(sink) {
    }

    // str to the output sink, returns false if it fails
    bool WriteToSink(const std::string &content) {
        for (char ch : content) {
            if (!OutputSink->Put(ch)) {
                return false;
            }
        }
        return true;
    }

    // escapes special xml characters and writes them to the sink
    bool EscapeAndWrite(const std::string &text) {
        for (char ch : text) {
            switch (ch) {
                case '<': if (!WriteToSink("&lt;")) return false; break;  // escape <
                case '>': if (!WriteToSink("&gt;")) return false; break;  
                case '&': if (!WriteToSink("&amp;")) return false; break;  
                case '\'': if (!WriteToSink("&apos;")) return false; break;  // escape '
                case '"': if (!WriteToSink("&quot;")) return false; break;  // escape "
                default: if (!WriteToSink(std::string(1, ch))) return false; break;  // write normal chars
            }
        }
        return true;
    }

    // closes all open elements in the deque
    bool CloseAllElements() {
        while (!OpenElements.empty()) {
            if (!WriteToSink("</") || !WriteToSink(OpenElements.back()) || !WriteToSink(">")) {
                return false;  // write closing tag for each open element
            }
            OpenElements.pop_back();  // remove the element from the deque
        }
        return true;
    }

    // handles diff types of xml entities
    bool HandleEntity(const SXMLEntity &entity) {
        switch (entity.DType) {
            case SXMLEntity::EType::StartElement:
                if (!WriteToSink("<") || !WriteToSink(entity.DNameData)) {
                    return false;  // write start tag
                }
                for (const auto &attr : entity.DAttributes) {
                    if (!WriteToSink(" ") || !WriteToSink(attr.first) ||
                        !WriteToSink("=\"") || !EscapeAndWrite(attr.second) || !WriteToSink("\"")) {
                        return false;  // write attributes
                    }
                }
                if (!WriteToSink(">")) {
                    return false;  // close start tag
                }
                OpenElements.push_back(entity.DNameData);  // add element to open elements
                break;

            case SXMLEntity::EType::EndElement:
                if (!WriteToSink("</") || !WriteToSink(entity.DNameData) || !WriteToSink(">")) {
                    return false;  // write end tag
                }
                if (!OpenElements.empty() && OpenElements.back() == entity.DNameData) {
                    OpenElements.pop_back();  // remove element from open elements
                }
                break;

            case SXMLEntity::EType::CharData:
                if (!EscapeAndWrite(entity.DNameData)) {
                    return false;  // write escaped char data
                }
                break;

            case SXMLEntity::EType::CompleteElement:
                if (!WriteToSink("<") || !WriteToSink(entity.DNameData)) {
                    return false;  // write start tag for complete element
                }
                for (const auto &attr : entity.DAttributes) {
                    if (!WriteToSink(" ") || !WriteToSink(attr.first) ||
                        !WriteToSink("=\"") || !EscapeAndWrite(attr.second) || !WriteToSink("\"")) {
                        return false;  // write attributes
                    }
                }
                if (!WriteToSink("/>")) {
                    return false;  // close complete element
                }
                break;
        }
        return true;
    }
};

CXMLWriter::CXMLWriter(std::shared_ptr<CDataSink> sink)
    : DImplementation(std::make_unique<SImplementation>(sink)) {
}

CXMLWriter::~CXMLWriter() = default;

bool CXMLWriter::Flush() {
    return DImplementation->CloseAllElements();  // flush all open elements
}

bool CXMLWriter::WriteEntity(const SXMLEntity &entity) {
    return DImplementation->HandleEntity(entity);  // handle the entity
}
