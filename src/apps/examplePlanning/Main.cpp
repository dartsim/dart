#include "MyWindow.h"

int main(int argc, char* argv[])
{
    MyWindow window;
    
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Planning Example");
    glutMainLoop();

    return 0;
}
