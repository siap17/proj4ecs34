#include "DSVWriter.h"
#include <sstream>
#include <iostream>
#include <vector>

// internal struct for managing writer state
struct CDSVWriter::SImplementation {
    std::shared_ptr<CDataSink> sink; // where data will be written
    char delimiter;  // char used to separate values
    bool quoteall;   // whether to always quote values

    // constructor initializes the sink, delimiter, and quoting option
    SImplementation(std::shared_ptr<CDataSink> s, char d, bool q)
        : sink(s), delimiter(d), quoteall(q) {}
};


CDSVWriter::CDSVWriter(std::shared_ptr<CDataSink> sink, char delimiter, bool quoteall) // constructor
    : DImplementation(std::make_unique<SImplementation>(sink, delimiter, quoteall)) {}


CDSVWriter::~CDSVWriter() = default; // destructor

// checks if a value needs to be quoted and escapes it if necessary
std::string EscapeAndQuote(const std::string &value, char delimiter, bool quoteall) {
    // determine if the value needs to be wrapped in quotes
    bool needs_quotes = quoteall || value.find(delimiter) != std::string::npos ||
                        value.find('"') != std::string::npos || value.find('\n') != std::string::npos;

    if (!needs_quotes) {
        return value; // ret same if no special characters are found
    }

    std::ostringstream escaped_value;
    escaped_value << '"';  // start with an opening quote

    // go thru each char in the value
    for (char c : value) {
        if (c == '"') {
            escaped_value << "\"\"";  // escape double quotes by doubling them
        } else {
            escaped_value << c;  // add normal chars as they are
        }
    }

    escaped_value << '"';  // end with closing quote
    return escaped_value.str();
}

// writes a row to the data sink
bool CDSVWriter::WriteRow(const std::vector<std::string> &row) {
    if (!DImplementation->sink) {
        return false; // cant write if there's no valid sink!!
    }

    std::ostringstream row_stream;

    // iterate over each field in the row
    for (size_t i = 0; i < row.size(); ++i) {
        if (i > 0) {
            row_stream << DImplementation->delimiter; // add delimiter between values
        }
        // escape and quote each field as needed before adding it to the stream
        row_stream << EscapeAndQuote(row[i], DImplementation->delimiter, DImplementation->quoteall);
    }

    row_stream << '\n'; // add newline at the end of row

    
    std::string row_str = row_stream.str(); // convert the constructed row string to vector<char> before writing
    std::vector<char> row_data(row_str.begin(), row_str.end());

    return DImplementation->sink->Write(row_data); // write to the sink ; return success status
}