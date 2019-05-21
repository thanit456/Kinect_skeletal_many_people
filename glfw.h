#pragma once

#include <iostream>
#include "main.h"
#include <gl/glew.h>
#include <GLFW/glfw3.h>

/*
void draw(GLFWwindow* window) {
	drawKinectData();
	glfwSwapBuffers(window);
}

void updateInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
}

bool init() {
	glfwInit();
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);


	glfwInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	glutCreateWindow("Kinect SDK Tutorial");
	glutDisplayFunc(draw);
	glutIdleFunc(draw);
	glewInit();
	return true;
}
*/