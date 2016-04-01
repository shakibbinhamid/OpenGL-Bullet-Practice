/******************************************************************************\
 | OpenGL 4 Example Code.                                                       |
 | Accompanies written series "Anton's OpenGL 4 Tutorials"                      |
 | Email: anton at antongerdelan dot net                                        |
 | First version 27 Jan 2014                                                    |
 | Copyright Dr Anton Gerdelan, Trinity College Dublin, Ireland.                |
 | See individual libraries for separate legal notices                          |
 \******************************************************************************/
#ifndef _GL_UTIL_H_
#define _GL_UTIL_H_

#include <GL/glew.h> /* include GLEW and new version of GL on Windows */
#include <GLFW/glfw3.h> /* GLFW helper library */
#include <stdarg.h>
#include <stdbool.h>
#include "Camera.h"
//#define bool int // for visual studio
//#define true 1
//#define false 0

#define GL_LOG_FILE "/Users/shakib-binhamid/Desktop/gl.log"

extern GLuint WIDTH;
extern GLuint HEIGHT;
extern GLFWwindow* window;

extern Camera camera;
extern bool keys[];
extern GLfloat lastX, lastY;
extern bool firstMouse;

extern GLfloat deltaTime;
extern GLfloat lastFrame;

bool start_gl ();

void do_movement();

#endif
