/*****************************************************************************\
 | OpenGL                                                                      |
 |                                                                             |
 | Email: sh3g12 at soton dot ac dot uk                                        |
 | version 0.0.1                                                               |
 | Copyright Shakib Bin Hamid                                                  |
 |*****************************************************************************|
 |                                                                             |
 \*****************************************************************************/
#define _USE_MATH_DEFINES
#define GLEW_STATIC

#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include "Shader.h"
#include "Camera.h"
#include "gl_util.hpp"
#include "World.h"

#include <btBulletDynamicsCommon.h>

// Window dimensions
GLuint WIDTH = 800, HEIGHT = 600;
GLFWwindow* window = nullptr;

// Camera
Camera camera(glm::vec3(0.0f, 25.0f, 70.0f));
bool keys[1024];
GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

GLint stacks = 100;
GLint slices = 100;
GLfloat radius = 1.0f;

glm::vec3 lightPos(0.0f, 0.0f, 100.0f);

std::vector<GLfloat> * generateSphere (std::vector<GLfloat> * vertices, std::vector<GLint> * indices, GLint Stacks, GLint Slices, GLfloat r);
void prepareVAO(GLuint * VAO, GLuint * VBO, GLuint * EBO,
                      std::vector<GLfloat> verts, std::vector<GLint> idx,
                      GLuint aCount, GLuint aLoc[], GLint size[], GLsizei vStride[], const void* vOffset[]);
void drawSphere(Shader * cubeShader, GLuint * container_VAO, Shader * sphereShader, GLuint * sphere_VAO, GLint sphere_idx_size, GLint cube_idx_size,
                GLint * objectColorLoc, GLint * lightColorLoc, GLint * lightPosLoc, GLint * viewPosLoc,
                GLuint count, glm::vec3 * locations, GLint * modelLoc, GLint * viewLoc, GLint * projLoc,
                btDiscreteDynamicsWorld * dynamicsWorld, btRigidBody * sphere1, btRigidBody * sphere2, btRigidBody * cube);

btRigidBody* makeSide(btDiscreteDynamicsWorld* dynamicsWorld, btVector3 norm, btVector3 pos, btScalar coe = COE){
    
    btCollisionShape* groundShape = new btStaticPlaneShape(norm, 0); // declare a plane shape at the right orientation
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), pos)); // no rotation, located at the right place
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0, 0, 0)); // the rigid body parameters
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI); // define the rigid body
    
    groundRigidBody->setRestitution(coe);
    dynamicsWorld->addRigidBody(groundRigidBody); // add to the world
    
    return groundRigidBody;
}

btRigidBody* makeSphere(btDiscreteDynamicsWorld* dynamicsWorld, btScalar mass, btScalar r, btVector3 pos, btScalar coe = COE){
    
    btCollisionShape* fallShape = new btSphereShape(r); // create a r radius sphere shape
    btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), pos)); // position it at right place, no rotation
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia); // calculate the inertia
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia); // rigid body params
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI); // create the rigid body
    
    fallRigidBody->setRestitution(coe);
    fallRigidBody->setLinearVelocity(btVector3(50, 0, 0));
    dynamicsWorld->addRigidBody(fallRigidBody);
    
    return fallRigidBody;
    
}

btRigidBody* makeCube(btDiscreteDynamicsWorld* dynamicsWorld, btScalar mass, btScalar d, btVector3 pos, btScalar coe = COE){
    
    btCollisionShape* cubeShape = new btBoxShape(btVector3(d, d, d)); // create a r radius sphere shape
    btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), pos)); // position it at right place, no rotation
    btVector3 fallInertia(0, 0, 0);
    cubeShape->calculateLocalInertia(mass, fallInertia); // calculate the inertia
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, cubeShape, fallInertia); // rigid body params
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI); // create the rigid body
    
    fallRigidBody->setRestitution(coe);
    fallRigidBody->setLinearVelocity(btVector3(50, 0, 0));
    dynamicsWorld->addRigidBody(fallRigidBody);
    
    return fallRigidBody;

}

// The MAIN function, from here we start the application and run the game loop
int main() {
    
    //////////////////////////////////////// BULLET ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // create the world parameters
    btBroadphaseInterface * broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0., GRAVITY, 0));
    
    // Set up the sides
    btRigidBody* sides [6];
    sides[0] = makeSide(dynamicsWorld, btVector3(0, 1, 0), btVector3(0, 0, 0));
    sides[1] = makeSide(dynamicsWorld, btVector3(1, 0, 0), btVector3(-20, 0, 0));
    sides[2] = makeSide(dynamicsWorld, btVector3(-1, 0, 0), btVector3(20, 0, 0));
    sides[3] = makeSide(dynamicsWorld, btVector3(0, -1, 0), btVector3(0, 40, 0));
    sides[4] = makeSide(dynamicsWorld, btVector3(0, 0, -1), btVector3(0, 0, 20));
    sides[5] = makeSide(dynamicsWorld, btVector3(0, 0, 1), btVector3(0, 0, -20));
    
    btRigidBody* sphere1 = makeSphere(dynamicsWorld, 1, 1, btVector3(0, 40, 0));
    btRigidBody* sphere2 = makeSphere(dynamicsWorld, 1, 1, btVector3(0, 40, 0));
    btRigidBody* cube = makeSphere(dynamicsWorld, 1, 1, btVector3(0, 35, 0));
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // start glfw and glew with default settings
    bool test = start_gl();

    assert(test);
    
    // Build and compile our shader program
    Shader sphereShader("shaders/shader.vs", "shaders/shader.frag");
    Shader cubeShader("shaders/shader.vs", "shaders/lamp.frag");
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    GLuint container_VAO, container_VBO, container_EBO;
    
    GLfloat verts[] = {
        -0.5f, -0.5f, 0.5f, // A 0
        -0.5f, -0.5f, -0.5f, // B 1
        0.5f, -0.5f, -0.5f, // C 2
        0.5f, -0.5f, 0.5f, // D 3
        -0.5f, 0.5f, 0.5f, // E 4
        -0.5f, 0.5f, -0.5f, // F 5
        0.5f, 0.5f, -0.5f, // G 6
        0.5f, 0.5f, 0.5f // H 7
    };

    GLuint idx[] = {
        0, 1, 2, 3, 0,
        4, 7, 3,
        2, 6, 7,
        6, 5, 4,
        5, 1, 0,
    };
    
    std::vector<GLfloat> container_verts = std::vector<GLfloat>(verts, verts + sizeof verts / sizeof verts[0]);
    std::vector<GLint> container_idx = std::vector<GLint>(idx, idx + sizeof idx / sizeof idx[0]);
    
    /////// Sphere vertices, normals and indices generation  //////////////////////////////////////////
    
    std::vector<GLfloat> sphere_verts;
    std::vector<GLint> sphere_idx;
    generateSphere( &sphere_verts, &sphere_idx, stacks, slices, radius);
    
    /////////////////  DECLARATIONS  ////////////////////////
    
    GLuint sphere_VBO, sphere_VAO, sphere_EBO;
    
    /////////////////  GET VAO READY  ////////////////////////
    GLuint aLoc[3] = {0, 1, 2};
    GLint size[3] = {3, 3, 2};
    GLsizei vStride[3] = {8 * sizeof(GLfloat), 8 * sizeof(GLfloat), 8 * sizeof(GLfloat)};
    const void* vOffset[3] = {(GLvoid*)0, (GLvoid*)(3 * sizeof(GLfloat)), (GLvoid*)(6 * sizeof(GLfloat))};
    
    prepareVAO(&sphere_VAO, &sphere_VBO, &sphere_EBO, sphere_verts, sphere_idx, 2, aLoc, size, vStride, vOffset);
    
    aLoc[0] = 0;
    size[0] = 3;
    vStride[0] = 3 * sizeof(GLfloat);
    vOffset[0] = (GLvoid*)0;
    
    prepareVAO(&container_VAO, &container_VBO, &container_EBO, container_verts, container_idx, 1, aLoc, size, vStride, vOffset);
    
    /////////////////  The positions for the spheres in q4  ////////////////////////
    // where the cubes will appear in the world space
    glm::vec3 cubePositions[] = {
        glm::vec3(1.5f, 0.0f, 0.0f),
        glm::vec3(1.0f, 0.0f, 0.0f)
    };
    
    /////////////////  Uniform variables for MVP in VS  ////////////////////////
    
    GLint modelLoc = glGetUniformLocation(sphereShader.Program, "model");
    GLint viewLoc = glGetUniformLocation(sphereShader.Program, "view");
    GLint projLoc = glGetUniformLocation(sphereShader.Program, "projection");
    
    // uniforms for lighting
    GLint objectColorLoc = glGetUniformLocation(sphereShader.Program, "objectColor");
    GLint lightColorLoc  = glGetUniformLocation(sphereShader.Program, "lightColor");
    GLint lightPosLoc = glGetUniformLocation(sphereShader.Program, "lightPos");
    GLint viewPosLoc = glGetUniformLocation(sphereShader.Program, "viewPos");
    
    ofstream myfile;
    myfile.open ("velocity.log");
    
    btTransform trans;
    // Main loop
    while (!glfwWindowShouldClose(window)) {
        GLfloat currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        // Check if any events have been activated (key pressed, mouse moved)
        glfwPollEvents();
        do_movement();
        
        // Clear the color buffer
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        drawSphere(&cubeShader, &container_VAO, &sphereShader, &sphere_VAO, (GLint)sphere_idx.size(), 17,
                   &objectColorLoc, &lightColorLoc, &lightPosLoc, &viewPosLoc,
                   2, cubePositions, &modelLoc, &viewLoc, &projLoc,
                   dynamicsWorld, sphere1, sphere2, cube);
        sphere1->getMotionState()->getWorldTransform(trans);
        myfile << trans.getOrigin().getX() << " " << trans.getOrigin().getY() <<  " " << trans.getOrigin().getZ() << ":";
        sphere2->getMotionState()->getWorldTransform(trans);
        myfile << trans.getOrigin().getX() << " " << trans.getOrigin().getY() <<  " " << trans.getOrigin().getZ() << ":";
        cube->getMotionState()->getWorldTransform(trans);
        myfile << trans.getOrigin().getX() << " " << trans.getOrigin().getY() <<  " " << trans.getOrigin().getZ() << "\n";
        
        // Swap the screen buffers
        glfwSwapBuffers(window);
    }
    // Deallocate
    glDeleteVertexArrays(1, &sphere_VAO);
    glDeleteBuffers(1, &sphere_VBO);
    glDeleteBuffers(1, &sphere_EBO);
    // Terminate GLFW
    glfwDestroyWindow(window);
    glfwTerminate();
    
    myfile.close();
    
    for(int i=0; i<6; i++){
        dynamicsWorld->removeRigidBody(sides[i]);
        delete sides[i]->getMotionState();
        delete sides[i];
    }
    
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
    
    return EXIT_SUCCESS;
}

void drawSphere(Shader * cubeShader, GLuint * container_VAO, Shader * sphereShader, GLuint * sphere_VAO, GLint sphere_idx_size, GLint cube_idx_size,
                GLint * objectColorLoc, GLint * lightColorLoc, GLint * lightPosLoc, GLint * viewPosLoc,
                GLuint count, glm::vec3 * locations, GLint * modelLoc, GLint * viewLoc, GLint * projLoc,
                btDiscreteDynamicsWorld * dynamicsWorld, btRigidBody * sphere1, btRigidBody * sphere2, btRigidBody * cube) {
    
    dynamicsWorld->stepSimulation(1 / 60.f, 10);
    
    btTransform trans;
    
    // draw the container now
    cubeShader->Use();
    
    // Pass the view and projection matrices to the shader
    glm::mat4 model = glm::translate(glm::scale(glm::mat4(), glm::vec3(40.0f)), glm::vec3(0.0f, 0.5f, 0.0f));
    glm::mat4 view = camera.GetViewMatrix(); // Camera/View transformation
    glm::mat4 projection = glm::perspective(45.0f, (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 100.0f); // Projection
    glUniformMatrix4fv(*modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(*viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(*projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    
    // bind the right vao and draw
    glBindVertexArray(*container_VAO);
    glDrawElements(GL_LINE_STRIP, cube_idx_size, GL_UNSIGNED_INT, 0);
    
    cube->getMotionState()->getWorldTransform(trans);
    model = glm::translate(glm::mat4(), glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    glUniformMatrix4fv(*modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glDrawElements(GL_LINE_STRIP, cube_idx_size, GL_UNSIGNED_INT, 0);
    
    glBindVertexArray(0);
    
    // Activate shader
    sphereShader->Use();
    
    glUniform3f(*viewPosLoc, camera.Position.x, camera.Position.y, camera.Position.z); // camera position for spec light
    glUniform3f(*lightColorLoc,  1.0f, 0.5f, 1.0f); // color of the light source

    // Pass the view and projection matrices to the shader
    sphere1->getMotionState()->getWorldTransform(trans);
    model = glm::translate(glm::mat4(), glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    glUniformMatrix4fv(*modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(*viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(*projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    
    // place the sphere in the right place
    glBindVertexArray(*sphere_VAO);
    glUniform3f(*lightPosLoc, lightPos.x, lightPos.y, lightPos.z);
    glUniform3f(*objectColorLoc, 1.0f, 0.5f, 0.31f);
    
    // draw big sphere
    glDrawElements(GL_TRIANGLES, sphere_idx_size, GL_UNSIGNED_INT, 0);
    
    sphere2->getMotionState()->getWorldTransform(trans);
    model = glm::mat4(); // model
    model = glm::translate(model, glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    glUniformMatrix4fv(*modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(*objectColorLoc, 1.0f, 1.0f, 0.0f);
    
    glDrawElements(GL_TRIANGLES, sphere_idx_size, GL_UNSIGNED_INT, 0);
    
    glBindVertexArray(0); // done drawing sphere, unload VAO
}

void prepareVAO(GLuint * VAO, GLuint * VBO, GLuint * EBO,
              std::vector<GLfloat> verts, std::vector<GLint> idx,
              GLuint aCount, GLuint aLoc[], GLint size[], GLsizei vStride[], const void* vOffset[]){
    
    // generate the vao's and vbo's
    glGenVertexArrays(1, VAO);
    glGenBuffers(1, VBO);
    if (EBO != nullptr) glGenBuffers(1, EBO);
    
    // bind the vao as current
    glBindVertexArray(*VAO);
    
    // bind VBO and load vertex data on it
    glBindBuffer(GL_ARRAY_BUFFER, *VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * verts.size(), &verts[0], GL_STATIC_DRAW);
    
    // if there is an ebo bind EBO and load index data on it
    if (EBO != nullptr) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLint) * idx.size(), &idx[0], GL_STATIC_DRAW);
    }
    
    // prepare the vao
    for (GLint i = 0; i < aCount; i++) {
        glVertexAttribPointer(i, size[i], GL_FLOAT, GL_FALSE, vStride[i], vOffset[i]);
        glEnableVertexAttribArray(i);
    }
    // Unbind vao as we're done pointing attributes
    glBindVertexArray(0);
}

/*
 Generates a sphere and populates the vertices, indices based on how many 'stacks' and 'slices' are needed.
 It is a UV sphere.
 vertices contain position, normal, texcord
 q2 verts just contain position, normal
 */
std::vector<GLfloat> * generateSphere (std::vector<GLfloat> * vertices, std::vector<GLint> * indices,
    const GLint Stacks, const GLint Slices, const GLfloat r){
    for (int i = 0; i <= Stacks; ++i){
        float V   = i / (float) Stacks;
        float phi = V * glm::pi <float> ();
        // Loop Through Slices
        for (int j = 0; j <= Slices; ++j){
            float U = j / (float) Slices;
            float theta = U * (glm::pi <float> () * 2);
            
            // Calc The Vertex Positions
            float x = r * cosf (theta) * sinf (phi);
            float y = r * cosf (phi);
            float z = r * sinf (theta) * sinf (phi);
            
            // vertices for sphere
            glm::vec3 v(x, y, z);
            glm::vec3 n(v + glm::normalize(v) * 0.05f);
            vertices->push_back (v.x); // vertex
            vertices->push_back (v.y);
            vertices->push_back (v.z);
            
            vertices->push_back (n.x); // normal
            vertices->push_back (n.y);
            vertices->push_back (n.z);
            
            vertices->push_back (U); // texcord
            vertices->push_back (V);
        }
    }
    
    for (int i = 0; i < Slices * Stacks + Slices; ++i){
        indices->push_back (i);
        indices->push_back (i + Slices + 1);
        indices->push_back (i + Slices);
        
        indices->push_back (i + Slices + 1);
        indices->push_back (i);
        indices->push_back (i + 1);
    }
    
    return vertices;
}