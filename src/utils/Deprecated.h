#ifdef __GNUC__ 
    #define DEPRECATED __attribute__ ((deprecated)) 
#elif defined(_MSC_VER) 
    #define DEPRECATED __declspec(deprecated) 
#endif