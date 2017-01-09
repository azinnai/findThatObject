#ifndef PLOTPOINTCLOUD_H_INCLUDED
#define PLOTPOINTCLOUD_H_INCLUDED

#include <GL/glew.h> // Include the GLEW header file
#include <GL/freeglut.h> // Include the GLUT header file
#include <X11/extensions/XInput.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <random>



void display (void);
void draw (std::string& path);
//void draw(void);


#endif // PLOTPOINTCLOUD_H_INCLUDED
