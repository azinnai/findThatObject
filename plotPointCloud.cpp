#include "plotPointCloud.h"

std::string cloudPath;

void display (void) {
    std::ifstream indata;
    std::string line;
    std::vector<double> point(3);

    indata.open(cloudPath);

    if(!indata.is_open())
    {
        std::cout << "Failed to open " << cloudPath << std::endl;
        throw -1;
    }

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Clear the background of our window to red
    glClear(GL_COLOR_BUFFER_BIT); //Clear the colour buffer (more buffers later on)
    glLoadIdentity(); // Load the Identity Matrix to reset our drawing locations


    while (std::getline(indata, line)) {

        glColor3f(1,0,0);      // the r,g,b colour of the points
        glPointSize(1.0);     // where size indicates the size of the point
        glBegin(GL_POINTS);

        std::stringstream lineStream(line);
        std::string cell;
        int i = 0;
        while (std::getline(lineStream, cell, ' '))
        {

            point.at(i) = std::stod(cell.c_str());
            ++i;
        }
        glVertex3f(point.at(0), point.at(1), point.at(2));

    }
    glEnd();
    indata.close();

    glFlush(); // Flush the OpenGL buffers to the window
}

void draw (std::string& path) {
    int argc = 1;
    char *argv[1] = {(char*)"Something"};

    cloudPath = path;

    glutInit(&argc, argv); // Initialize GLUT
    glutInitDisplayMode (GLUT_SINGLE); // Set up a basic display buffer (only single buffered for now)
    glutInitWindowSize (500, 500); // Set the width and height of the window
    glutInitWindowPosition (100, 100); // Set the position of the window
    glutCreateWindow ("Your first OpenGL Window"); // Set the title for the window
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_CONTINUE_EXECUTION);

    glutDisplayFunc(display); // Tell GLUT to use the method "display" for rendering
    wait(3);
    glutLeaveMainLoop(); // Enter GLUT's main loop
    glutExit();
}
