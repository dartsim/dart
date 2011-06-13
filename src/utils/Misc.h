/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_MISC_H
#define UTILS_MISC_H

// Epsilon definition
#ifndef EPSILON
#define EPSILON 1.0e-6
#endif

// PI definition
#ifndef M_PI
#define M_PI 3.141592653589793
#endif
#ifndef M_PI_2
#define M_PI_2 M_PI*0.5;
#endif

// Define a macro which returns the "pretty" name of function
// It should include names of class and functions
#ifdef __linux
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
#elif defined(__APPLE__)
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
//#error "Define your own FUNCTION_NAME() macro on Apple"
#elif defined(_WIN32)
#define FUNCTION_NAME() (__FUNCSIG__)
#else
#error "What's your operating system?"
#endif

// Safe Release Ptr
// Warning!!! You should put ";" after this macros!!!
#define SAFE_RELEASE_PTR(x) do{if(x) {delete x; x = NULL;}}while(0)

// For generate getter/setter method
#define GETSET(type, var)                       \
    public:                                     \
    type var() const {                          \
        return var##_;                          \
    }                                           \
    void set_##var(type _val) {                 \
        var##_ = _val;                          \
    }                                           \
private:                                        \
type var##_                 


#endif // #ifndef UTILS_MISC_H


