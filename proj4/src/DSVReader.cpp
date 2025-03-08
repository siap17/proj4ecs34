#include "DSVReader.h"
#include <sstream>
#include <iostream>

struct CDSVReader::SImplementation {
    std::shared_ptr<CDataSource> InputSource;  // holds the input source for reading data
    char Separator;  // the delimiter used to separate values

    SImplementation(std::shared_ptr<CDataSource> src, char separator)
        : InputSource(std::move(src)), Separator(separator) {}
    
    // helper function to handle quoted sections
    bool handleQuotedSection(char ch, std::string &currentCell, bool &insideQuotes) {
        if (!InputSource->End()) {
            char nextChar;
            if (InputSource->Peek(nextChar) && nextChar == '"') {
                InputSource->Get(nextChar);  // escaped quote
                currentCell += '"';  // add  quote to the cell
            } else {
                insideQuotes = !insideQuotes;  //   quote state
            }
        } else {
            insideQuotes = !insideQuotes;  //  quote state at EOF
        }
        return true;
    }

    // helper function to handle end of row
    bool handleEndOfRow(char ch, std::vector<std::string> &row, std::string &currentCell) {
        if (!currentCell.empty() || !row.empty()) {
            row.push_back(currentCell);  // add  last cell to the row
        }
        // handle  \r\n line endings
        if (ch == '\r' && !InputSource->End()) {
            char nextChar;
            if (InputSource->Peek(nextChar) && nextChar == '\n') {
                InputSource->Get(nextChar);  // consume the '\n'
            }
        }
        return true;
    }

    // reads a row from the input source
    bool fetchRow(std::vector<std::string> &row) {
        row.clear();  // clear the row to start fresh
        
        std::string currentCell;  // holds the curr cell being read
        char ch;  // current char being processed
        bool insideQuotes = false;  // tracks if we r inside a quoted section
        bool hasData = false;  // tracks if we read any data
        
        while (!InputSource->End()) {
            if (!InputSource->Get(ch)) return false;  // read the next character
            hasData = true;  // mark that we've read data
            
            if (ch == '"') {
                if (!handleQuotedSection(ch, currentCell, insideQuotes)) {
                    return false;  // handle quoted sections
                }
            } else if (ch == Separator && !insideQuotes) {
                row.push_back(currentCell);  // end of cell, add to row
                currentCell.clear();  // clear the cell for the next value
            } else if ((ch == '\n' || ch == '\r') && !insideQuotes) {
                if (!handleEndOfRow(ch, row, currentCell)) {
                    return false;  // handle end of row
                }
                return true;  // row is complete
            } else {
                currentCell += ch;  // add the char to the curr cell
            }
        }
        
        // add the last cell if there was any data
        if (!currentCell.empty() || hasData) {
            row.push_back(currentCell);
        }
        
        return hasData;  // return true if data was read
    }
};

CDSVReader::CDSVReader(std::shared_ptr<CDataSource> src, char separator)
    : DImplementation(std::make_unique<SImplementation>(src, separator)) {}

CDSVReader::~CDSVReader() = default;

bool CDSVReader::End() const {
    return DImplementation->InputSource->End();  // check if  at  end of the input
}

bool CDSVReader::ReadRow(std::vector<std::string> &row) {
    return DImplementation->fetchRow(row);  // fetch the next row
}
