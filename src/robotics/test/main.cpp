
#include "robotics/ParserHUBO.h"

#include <iostream>

int main(int argc, char** argv)
{
    if( argc < 2 )
    {
        std::cerr << "Not enough arguments. Specify a filename" << std::endl;
        return -1;
    }

    ParseFile( argv[1] );

    return 0;
}
