#include "StringUtils.h"
#include "iostream"
#include <algorithm>
namespace StringUtils{

std::string Slice(const std::string &str, ssize_t start, ssize_t end) noexcept{
    if (end==0){
        end = str.size();
    }
    if (start<0){
        start = std::max(static_cast<ssize_t>(0), static_cast<ssize_t>(str.size()) + start); // if in case negative use ssize_t bc its signed
    }
    if (end<0){
        end = std::max(static_cast<ssize_t>(0), static_cast<ssize_t>(str.size()) + end); // same thing ssize_t so we get signed for neg. handling
    } 
    if (start > end) return "";  // for invalid range
    return str.substr(start, end - start);
}

std::string Capitalize(const std::string &str) noexcept{
    if (str.empty()){
        return str;
    }
    std::string res = str;
    res[0] = std::toupper(res[0]);
    std::transform(res.begin() + 1, res.end(), res.begin() + 1, ::tolower);
    return res;
}

std::string Upper(const std::string &str) noexcept{
    std::string res = str;
    std::transform(res.begin(),res.end(),res.begin(),::toupper);
    return res;
}

std::string Lower(const std::string &str) noexcept{
    std::string res = str;
    std::transform(res.begin(),res.end(),res.begin(),::tolower);
    return res;
}

std::string LStrip(const std::string &str) noexcept{
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return "";
    } else {
        return str.substr(start); // gives us start pos (first entry thats not a space) to end
    }
}

std::string RStrip(const std::string &str) noexcept{
    size_t end = str.find_last_not_of(" \t\r\n"); // returns last char in str thats not a space 
    if (end == std::string::npos) {
        return "";
    } else {
        return str.substr(0,end+1);
    }
}

std::string Strip(const std::string &str) noexcept{
    return LStrip(RStrip(str));
}

std::string Center(const std::string &str, int width, char fill) noexcept{
    if (str.size() >= static_cast<size_t>(width)) {
        return str;
    }
    int total_pad = width - str.size(); 
    int left_padding = total_pad / 2;
    int right_padding = total_pad-left_padding; 

    
    std::string res(left_padding, fill);
    res += str;
    res.append(right_padding, fill);
    return res;
}

std::string LJust(const std::string &str, int width, char fill) noexcept{
    if (str.size() >= static_cast<size_t>(width)) {
        return str;
    }
    std::string res = str; 
    int padding = width - str.size(); 
    res.append(padding, fill);
    return res;
}

std::string RJust(const std::string &str, int width, char fill) noexcept{
    if (str.size() >= static_cast<size_t>(width)) {
        return str;
    }
    int padding = width - str.size(); 
    
    std::string res(padding, fill);
    res += str;
    return res;
}

std::string Replace(const std::string &str, const std::string &old, const std::string &rep) noexcept{
    if (old.empty()){
        return str;
    }
    std::string res = str;
    size_t pos = 0;
    while ((pos = res.find(old, pos)) != std::string::npos){ 
        res.replace(pos,old.length(),rep);
        pos+=rep.length(); // move pos forward by length of rep to continue searching for  next occurrence of old after the newly replaced 
    }
    return res;
}

std::vector< std::string > Split(const std::string &str, const std::string &splt) noexcept{
    std::vector<std::string> res;
    size_t start = 0;
    size_t end;

    if (splt.empty()) {  // gonna split by whitespace 
        std::string word;
        for (char c : str) {
            if (std::isspace(c)) {
                if (!word.empty()) {
                    res.push_back(word);
                    word.clear();
                }
            } else {
                word += c;
            }
        }
        if (!word.empty()) {
            res.push_back(word); // if word not empty, add it to res
        }
    } else {  
        while ((end = str.find(splt, start)) != std::string::npos) {
            res.push_back(str.substr(start, end - start)); // end is where we find the splt
            start = end + splt.length();
        }
        res.push_back(str.substr(start));  // the last part that comes after the last splt occurence
    }

    return res;
}

std::string Join(const std::string &str, const std::vector< std::string > &vect) noexcept{
    if (vect.empty()){
        return "";
    }
    std::string res = vect[0]; // first element
    for (size_t i = 1; i < vect.size(); ++i) {
        res += str + vect[i]; // Append separator and next word
    }
    return res;
    
}

std::string ExpandTabs(const std::string &str, int tabsize) noexcept {
    std::string result;   // resulting string with expanded tabs
    
    size_t column = 0;   // curr column position in the string
    
    if (tabsize == 0) { // if so remove all tabs from the string
        for (char c : str) {
            if (c != '\t') {
            // append non-tab characters to the result
                result += c;  
            }
        }
        // return the string with tabs removed
        return result;  
    }
    for (char c : str) {
        if (c == '\t') {
            
            size_t spaces = tabsize - (column % tabsize); // calc t# of spaces needed to reach the next tab 
            result.append(spaces, ' '); 
            column += spaces;  
        } else {
            result += c;  // append non-tab characters to the res
            column++;     // increment the column position
        }
    }
    return result;  
}




int EditDistance(const std::string &left, const std::string &right, bool ignorecase) noexcept{
    // just like the dp table in ECS 122A
    // gettings lens of input strings and storing
    size_t len1 = left.size();
    size_t len2 = right.size();
    
    // make dp table to store the edit distances like in 122A
    std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1));

    for (size_t i = 0; i <= len1; ++i) { // initializing the first column-converting left to an empty string
        dp[i][0] = i; // cost of deleting all characters from left
    }
    
    for (size_t j = 0; j <= len2; ++j) { // initializing the first row-converting an empty string to right
        dp[0][j] = j; // cost of inserting all characters of right 
    }
    
    for (size_t i = 1; i <= len1; ++i) { // rest of the table
        for (size_t j = 1; j <= len2; ++j) {
            // obtaining curr characters from both strings
            char char1 = left[i - 1];
            char char2 = right[j - 1];
            
            if (ignorecase) { // if ignorecase is true convert to lowercase 
                char1 = std::tolower(char1);
                char2 = std::tolower(char2);
            }

            int cost; // getting cost of substitution
            if (char1 == char2) {
                cost = 0; // if chars are the same no cost
            } else {
                cost = 1; // if chars are diff cost is 1 
            }

            // calc min cost of three operations
            int deletion = dp[i - 1][j] + 1;      // deleting a char from left cost
            int insertion = dp[i][j - 1] + 1;     // inserting a char into left cost
            int substitution = dp[i - 1][j - 1] + cost; // substituting a char cost

            dp[i][j] = std::min({deletion, insertion, substitution}); // storing min cost in table
        }
    }
    return dp[len1][len2]; // bottom right corner val of dp table

}

}