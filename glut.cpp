#include "glut.h"
#include "main.h"
#include "Kinect.h"
#include "stdio.h"

void draw() {
   drawKinectData();
   glutSwapBuffers();
}

void execute() {
	
	glutMainLoop();
}

bool init(int argv, char** argc) {
    glutInit(&argv, argc);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width,height);
    glutCreateWindow("Kinect SDK Tutorial");
    glutDisplayFunc(draw);
    glutIdleFunc(draw);
	glewInit();
    return true;
}
