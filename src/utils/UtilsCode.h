/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author	Sumit Jain
    Date    07/21/2011
*/

#ifndef UTILS_UTILSCODE_H
#define UTILS_UTILSCODE_H

#include <string>
#include <vector>
#include <iterator>
#include <algorithm> // for copy
#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>

namespace utils {

    void tokenize(const std::string& _str, std::vector<std::string>& _tokens, const std::string& _delimiters = " ");    // breaks the _str string into a vector of _tokens using delimiters from input _delimiters

    template <class T>
    std::vector<T> stringToVec( const std::string &_str ){
        std::vector<T> myVec;
        std::istringstream iss (_str, std::istringstream::in);
        copy(std::istream_iterator<T>(iss), std::istream_iterator<T>(), back_inserter(myVec));
        //check http://www.sgi.com/tech/stl/istream_iterator.html
        return myVec;
    }


    double strTodouble(const std::string& str);

    template <typename T_POINTER> inline void swapPointers(T_POINTER *&_p1, T_POINTER *&_p2){
        T_POINTER *temp = _p2;
        _p2=_p1;
        _p1=temp;
    }

    
    inline bool isNaN(double _v){
        return _v != _v;
    }

} // namespace utils

#endif // #ifndef UTILS_UTILSMATH_H
