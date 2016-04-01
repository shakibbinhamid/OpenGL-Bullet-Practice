/*****************************************************************************\
 | OpenGL                                                                      |
 |                                                                             |
 | Email: sh3g12 at soton dot ac dot uk                                        |
 | version 0.0.1                                                               |
 | Copyright Shakib Bin Hamid                                                  |
 |*****************************************************************************|
 |                                                                             |
 \*****************************************************************************/
#include "gl_util.hpp"
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <algorithm> // for std::find
#include <iterator> // for std::begin, std::end

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

const int qKeys[] = {GLFW_KEY_A, GLFW_KEY_B, GLFW_KEY_C, GLFW_KEY_D, GLFW_KEY_E};

//#define GL_LOG_FILE "/Users/shakib-binhamid/Desktop/OpenGL Test Ground/OpenGL TestGround 2/OpenGL TestGround 2/gl.log"

/*--------------------------------GLFW3 and GLEW-----------------------------*/
bool start_gl () {
    
    printf ("starting GLFW %s", glfwGetVersionString ());
    
    // Init GLFW
    if (!glfwInit ()) {
        fprintf (stderr, "ERROR: could not start GLFW3\n");
        return false;
    }
    
    // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    window = glfwCreateWindow (WIDTH, HEIGHT, "Window", nullptr, nullptr);
    
    if (!window) {
        fprintf (stderr, "ERROR: could not open window with GLFW3\n");
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent (window);
    glfwWindowHint (GLFW_SAMPLES, 4);
    // Set the required callback functions
    glfwSetKeyCallback(window, key_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, mouse_callback);
    
    /* start GLEW extension handler */
    glewExperimental = GL_TRUE;
    
    glewInit ();
    
    // Define the viewport dimensions
    glViewport(0, 0, WIDTH, HEIGHT);
    
    // Enable testing for depth so that stuff in the back are not drawn
    glEnable(GL_DEPTH_TEST);
    
    printf ("Renderer: %s\n", glGetString (GL_RENDERER));
    printf ("OpenGL version supported %s\n", glGetString (GL_VERSION));
    
    return true;
}

void setActive(int active){
    keys[active] = true;
    for(int i=0; i < 5; i++){
        if (active != qKeys[i])
            keys[qKeys[i]] = false;
    }
}

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode) {
    if((key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
    bool q = std::find(std::begin(qKeys), std::end(qKeys), key) != std::end(qKeys);
    if (q && action == GLFW_PRESS) setActive(key);
    else if (key >= 0 && key < 1024) {
        if(action == GLFW_PRESS) {
            keys[key] = true;
        } else if(!q && action == GLFW_RELEASE) {
            keys[key] = false;
        }
    }
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if(firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    
    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = lastY - ypos;  // Reversed since y-coordinates go from bottom to left
    
    lastX = xpos;
    lastY = ypos;
    
    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    camera.ProcessMouseScroll(yoffset);
}

void do_movement() {
    // Camera controls
    if(keys[GLFW_KEY_UP])
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if(keys[GLFW_KEY_DOWN])
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if(keys[GLFW_KEY_LEFT])
        camera.ProcessKeyboard(LEFT, deltaTime);
    if(keys[GLFW_KEY_RIGHT])
        camera.ProcessKeyboard(RIGHT, deltaTime);
}
