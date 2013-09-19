#ifdef __GNUC__ 
    #define DEPRECATED __attribute__ ((deprecated))
    #define FORCEINLINE __attribute__((always_inline))
#elif defined(_MSC_VER) 
    #define DEPRECATED __declspec(deprecated) 
    #define FORCEINLINE __forceinline
#else
    #define DEPRECATED(version) ()
    #define FORCEINLINE
#endif
