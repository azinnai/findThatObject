#include "plotPointCloud.h"

std::string cloudPath;

// angle of rotation for the camera direction
float pitch = 0.0f, yaw = 0.0f;
// actual vector representing the camera's direction
float lx=0.0f, lz=-1.0f, ly=0.0f;
// XZ position of the camera
float x=0.0f, z=5.0f, y=0.0f;
// the key states. These variables will be zero
//when no key is being presses
float pitchRate = 0.0f, yawRate = 0.0f;
float deltaMoveX = 0, deltaMoveY = 0, deltaMoveZ = 0;

void changeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;
	float ratio =  w * 1.0 / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(90.0f, ratio, 0.001f, 100.0f);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
}

void drawSnowMan() {

	glColor3f(1.0f, 1.0f, 1.0f);

// Draw Body

	glTranslatef(0.0f ,0.75f, 0.0f);
	glutSolidSphere(0.75f,20,20);

// Draw Head
	glTranslatef(0.0f, 1.0f, 0.0f);
	glutSolidSphere(0.25f,20,20);

// Draw Eyes
	glPushMatrix();
	glColor3f(0.0f,0.0f,0.0f);
	glTranslatef(0.05f, 0.10f, 0.18f);
	glutSolidSphere(0.05f,10,10);
	glTranslatef(-0.1f, 0.0f, 0.0f);
	glutSolidSphere(0.05f,10,10);
	glPopMatrix();

// Draw Nose
	glColor3f(1.0f, 0.5f , 0.5f);
	glRotatef(0.0f,1.0f, 0.0f, 0.0f);
	glutSolidCone(0.08f,0.5f,10,2);
}

void computePos(float deltaMoveX, float deltaMoveY, float deltaMoveZ) {

	x += deltaMoveX * lx * 0.1f;
	z += deltaMoveZ * lz * 0.1f;
	y += deltaMoveY * 0.1f;
}

void computeDir(float yawRate, float pitchRate) {

	yaw += yawRate;
	pitch += pitchRate;
	lx = cos(yaw)* sin(pitch);
	ly = cos(pitch);
	lz = -sin(pitch) * sin(yaw);
}

void renderScene(void) {

	if (deltaMoveX || deltaMoveY || deltaMoveZ)
		computePos(deltaMoveX, deltaMoveY, deltaMoveZ);
	if (yawRate || pitchRate)
		computeDir(yawRate, pitchRate);

	// Clear Color and Depth Buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Reset transformations
	glLoadIdentity();
	// Set the camera
	std::cout << "x: " << x << "  y :" << y << "  z :" << z << "\n"<<
              "lx: " << lx << "  ly :" << ly << "  lz :" << lz << std::endl;
	gluLookAt(	x, y, z,
				x+lx, y+ly,  z+lz,
				0.0f, 1.0f,  0.0f);



    std::ifstream indata;
    std::string line;
    std::vector<double> point(3);

    indata.open(cloudPath);

    if(!indata.is_open())
    {
        std::cout << "Failed to open " << cloudPath << std::endl;
        throw -1;
    }


    glColor3f(1.0f,0.0f,0.0f);      // the r,g,b colour of the points
    glPointSize(1.0);     // where size indicates the size of the point

    glBegin(GL_POINTS);

    while (std::getline(indata, line)) {


        std::stringstream lineStream(line);
        std::string cell;
        int i = 0;
        while (std::getline(lineStream, cell, ' '))
        {

            point.at(i) = std::stod(cell);
            ++i;
        }
        //std::cout<< point.at(0) << std::endl;
        glVertex3f(point.at(0), point.at(1), point.at(2));


    }
    glEnd();

    indata.close();


	glutSwapBuffers();
}


void pressSpecialKey(int key, int xx, int yy) {

	switch (key) {
		case GLUT_KEY_LEFT : yawRate = 0.02f; break;
		case GLUT_KEY_RIGHT : yawRate = -0.02f; break;
		case GLUT_KEY_UP : pitchRate = 0.02f; break;
		case GLUT_KEY_DOWN : pitchRate = -0.02f; break;
	}
}

void releaseSpecialKey(int key, int x, int y) {

	switch (key) {
		case GLUT_KEY_LEFT :
		case GLUT_KEY_RIGHT : yawRate = 0.0f;break;
		case GLUT_KEY_UP :
		case GLUT_KEY_DOWN : pitchRate = 0.0f;break;
	}
}

void pressKey(unsigned char key, int xx, int yy) {

	switch (key) {
		case 'w' : deltaMoveZ = 0.1f; break;
		case 's' : deltaMoveZ = -0.1f; break;
		case 'a' : deltaMoveX = 0.1f; break;
		case 'd' : deltaMoveX = -0.1f; break;
		case 'p' : deltaMoveY = 0.1f; break;
		case 'l' : deltaMoveY = -0.1f; break;
	}
}
void releaseKey(unsigned char key, int x, int y) {

	switch (key) {
        case 'w' :
		case 's' : deltaMoveZ = 0.0f; break;
		case 'a' :
		case 'd' : deltaMoveX = 0.0f; break;
		case 'p' :
		case 'l' : deltaMoveY = 0.0f; break;
	}
}





void draw (std::string& path) {
    int argc = 1;
    char *argv[1] = {(char*)"Something"};

    cloudPath = path;

    glutInit(&argc, argv); // Initialize GLUT
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize (500, 500); // Set the width and height of the window
    glutInitWindowPosition (100, 100); // Set the position of the window
    glutCreateWindow ("Point cloud"); // Set the title for the window
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_CONTINUE_EXECUTION);

    glutDisplayFunc(renderScene);
    glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);

	glutSpecialFunc(pressSpecialKey);
	glutKeyboardFunc(pressKey);

	// here are the new entries
	glutIgnoreKeyRepeat(1);
	glutSpecialUpFunc(releaseSpecialKey);
	glutKeyboardUpFunc(releaseKey);

	// OpenGL init
	glEnable(GL_DEPTH_TEST);

	// enter GLUT event processing cycle
	glutMainLoop();

}


