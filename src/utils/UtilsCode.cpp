/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author	Sumit Jain
    Date    07/21/2011
*/

#include "UtilsCode.h"
using namespace std;


namespace utils{

    void tokenize(const string& str, vector<string>& tokens, const string& delimiters){
        // Skip delimiters at beginning.
        string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        // Find first "non-delimiter".
        string::size_type pos     = str.find_first_of(delimiters, lastPos);

        while (string::npos != pos || string::npos != lastPos)
        {
            // Found a token, add it to the vector.
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            // Skip delimiters.  Note the "not_of"
            lastPos = str.find_first_not_of(delimiters, pos);
            // Find next "non-delimiter"
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

    double strTodouble(const string& str) {
        stringstream ss(str);
        double value;
        ss >> value;
        return value;
    }



}   // namespace utils
