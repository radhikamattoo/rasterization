// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>

// For A x = b solver
#include <Eigen/Dense>

// IO Stream
#include <iostream>

// Timer
#include <chrono>

#include <cmath>

#include <math.h>
#define PI 3.14159265

using namespace std;
using namespace Eigen;

// VertexBufferObject wrapper
VertexBufferObject VBO;
VertexBufferObject VBO_C;

// Contains the vertex positions
//(numRows, numCols)
Eigen::MatrixXf V(2,6);

// Contains the per-vertex color
Eigen::MatrixXf C(3,3);
Matrix3f oldTranslatedColor(3,3);

// Holds # of clicks for triangle insertion
int insertClickCount = 0;

// If translationMode && clicked on a triangle && mouse is held down
bool translationPressed = false;
// If translationMode && clicked on a triangle && mouse is released
bool translationSelected = false;
// 1.2 booleans
bool clockwise = false;
bool counterclockwise = false;
bool scaleUp = false;
bool scaleDown = false;

// Index of selected vertex to color
int coloringVertex = -1;

// Number pressed while in color mode
int pressed = -1;

MatrixXf colors(3, 9);

// Holds the indices of the triangle in V being translated
int vertex_1_clicked = -1;
int vertex_2_clicked = -1;
int vertex_3_clicked = -1;

int vertex_1_deleted = -1;
int vertex_2_deleted = -1;
int vertex_3_deleted = -1;
// # of triangles that will be drawn
int numTriangles = 0;

// Am I currently creating a triangle?
bool insertionMode = false;

// Am I currently translating a triangle?
bool translationMode = false;

// Am I currently deleting a triangle?
bool deleteMode = false;

// Am I currently coloring a triangle vertex?
bool colorMode = false;

double currentX, currentY, previousX, previousY;

// Transformation matrices/vectors
Eigen::Matrix2f rotation(2,2);
Eigen::Matrix2f scaling(2,2);

// Contains the view transformation
Eigen::Matrix4f view(4,4);

void shiftVertical(bool down)
{
  if(down){
    view(1,3) -= .20;
  }else{
    view(1,3) += .20;
  }
}

void shiftHorizontal(bool right)
{
  // To give the appearance of moving the camera,
  // your OpenGL application must move the scene
  // with the inverse of the camera transformation
  if(right){
    view(0,3) += .20;
  }else{
    view(0,3) -= .20;
  }

}
void zoom(bool zoomIn)
{
  if(zoomIn)
  {
    view(0,0) += 0.20;
    view(1,1) += 0.20;
  }else{
    view(0,0) -= 0.20;
    view(1,1) -= 0.20;
  }
}

void colorVertex()
{
  cout << "Pressed this num in color mode:" << endl;
  cout << pressed << endl;
  if(colorMode && coloringVertex > -1)
  {
    int idx = pressed - 1 ; //off by 1 error
    C.col(coloringVertex) = colors.col(idx);
    VBO_C.update(C);
  }
}
// Returns x, y coordinates of selected traingle's barycenter
Vector2f calculateBarycenter()
{
  // Get coordinates
  float coord_1_x = V(0, vertex_1_clicked);
  float coord_1_y = V(1, vertex_1_clicked);

  float coord_2_x = V(0, vertex_2_clicked);
  float coord_2_y = V(1, vertex_2_clicked);

  float coord_3_x = V(0, vertex_3_clicked);
  float coord_3_y = V(1, vertex_3_clicked);

  // Calculate barycenter
  float barycenter_x = (coord_1_x + coord_2_x + coord_3_x) / 3;
  float barycenter_y = (coord_1_y + coord_2_y + coord_3_y) / 3;

  Vector2f barycenter(barycenter_x, barycenter_y);

  return barycenter;
}

void resetRotation()
{
  rotation <<
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1;
}

void resetScaling()
{
  scaling <<
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1;
}

void resetTranslationVariables()
{
  if(vertex_1_clicked > -1)
  {
    C.col(vertex_1_clicked) = oldTranslatedColor.col(0);
    C.col(vertex_2_clicked) = oldTranslatedColor.col(1);
    C.col(vertex_3_clicked) = oldTranslatedColor.col(2);

    vertex_1_clicked = -1;
    vertex_2_clicked = -1;
    vertex_3_clicked = -1;

    translationSelected = false;

    VBO_C.update(C);
  }

}

void scaleTriangle(bool scaleUp)
{
  if(translationMode && translationSelected)
  {
    Vector2f barycenter = calculateBarycenter();

    float scale;
    if(scaleUp)
    {
      scale = 1 + 0.25;
    }else{
      scale = 1 - 0.25;
    }

    scaling <<
    scale, 0,
    0, scale;

    // Apply rotation
    for(int i = vertex_1_clicked; i <= vertex_3_clicked; i++)
    {
      V.col(i) -= barycenter;
      V.col(i) = scaling * V.col(i);
      V.col(i) += barycenter;
    }
    VBO.update(V);
  }
}


void rotateTriangle(bool clockwise)
{
  if(translationMode && translationSelected)
  {
    Vector2f barycenter = calculateBarycenter();

    float degree;
    if(clockwise)
    {
      degree = -10 * (PI / 180);
    }else{
      degree = 10 * (PI / 180);
    }

    rotation <<
    cos(degree), -sin(degree),
    sin(degree), cos(degree);

    // Apply rotation
    for(int i = vertex_1_clicked; i <= vertex_3_clicked; i++)
    {
      V.col(i) -= barycenter;
      V.col(i) = rotation * V.col(i);
      V.col(i) += barycenter;
    }
    VBO.update(V);
  }
}

// Translates triangle based on mouse movement
void translateTriangle()
{
  // Compare previousX and currentX, etc. and figure out translation
  float x_difference;
  float y_difference;
  if(currentX > previousX) // mouse moved right
  {
    x_difference = currentX - previousX;
    V(0, vertex_1_clicked) += x_difference;
    V(0, vertex_2_clicked) += x_difference;
    V(0, vertex_3_clicked) += x_difference;
  }else{ // mouse moved left
    x_difference = previousX - currentX;
    V(0, vertex_1_clicked) -= x_difference;
    V(0, vertex_2_clicked) -= x_difference;
    V(0, vertex_3_clicked) -= x_difference;
  }

  if(currentY > previousY) // mouse moved up
  {
    y_difference = currentY - previousY;
    V(1, vertex_1_clicked) += y_difference;
    V(1, vertex_2_clicked) += y_difference;
    V(1, vertex_3_clicked) += y_difference;
  }else{ //mouse moved down
    y_difference = previousY - currentY;
    V(1, vertex_1_clicked) -= y_difference;
    V(1, vertex_2_clicked) -= y_difference;
    V(1, vertex_3_clicked) -= y_difference;
  }
}

// Uses barycentric interpolation to see if mouse clicked on triangle
bool clickedOnTriangle(double mousex, double mousey, float coord1_x, float coord1_y, float coord2_x, float coord2_y, float coord3_x, float coord3_y)
{
  Matrix3f A_;
  Vector3f b_;
  A_ << coord1_x, coord2_x, coord3_x, coord1_y, coord2_y, coord3_y, 1, 1, 1;
  b_ << mousex, mousey, 1;

  Vector3f sol = A_.colPivHouseholderQr().solve(b_);
  float alpha = sol[0];
  float beta = sol[1];
  float gamma = sol[2];

  if(alpha > 0 && beta > 0 && gamma > 0){
    std::cout << "Clicked on a triangle!!" << std::endl;
    return true;
  }
  return false;

}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
  // Get the size of the window
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Convert screen position to world coordinates
  Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
  Eigen::Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,0,1);
  Eigen::Vector4f p_world = view.inverse()*p_canonical;

  double xworld = p_world[0];
  double yworld = p_world[1];

  // Keep track of mouse positions
  if(!previousX && previousY)
  {
    previousX = xworld;
    previousY = yworld;
  }else{
    previousX = currentX;
    previousY = currentY;

    currentX = xworld;
    currentY = yworld;
  }
  if(insertionMode && insertClickCount > 0)
  {
    // Store coordinates in V and send to GPU
    if(insertClickCount == 1){
      V.col( (numTriangles * 3) + 1) << xworld, yworld;
    }else if(insertClickCount == 2){
      V(0, (numTriangles * 3) + 3) = xworld;
      V(1, (numTriangles * 3) + 3) = yworld;
      V(0, (numTriangles * 3) + 5) = xworld;
      V(1, (numTriangles * 3) + 5) = yworld;
    }
    VBO.update(V);
  }

  if(translationPressed)
  {
    translateTriangle();
    VBO.update(V);
  }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
    Eigen::Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,0,1);
    Eigen::Vector4f p_world = view.inverse()*p_canonical;

    double xworld = p_world[0];
    double yworld = p_world[1];

    if(insertionMode && action == GLFW_RELEASE) // creating a triangle out of line segments!
    {
      // Update insertClickCount
      std::cout << "Mouse clicked in INSERTION mode" << std::endl;
      if(insertClickCount == 0) // First click
      {
        V.conservativeResize(2, (numTriangles * 3) + 6);
        C.conservativeResize(3, (numTriangles * 3) + 6);

        // Set the color of new lines to 0
        C.col(numTriangles * 3) << 0.0, 0.0, 0.0;
        C.col((numTriangles * 3) + 1) << 0.0, 0.0, 0.0;
        C.col((numTriangles * 3) + 2) << 0.0, 0.0, 0.0;
        C.col((numTriangles * 3) + 3) << 0.0, 0.0, 0.0;
        C.col((numTriangles * 3) + 4) << 0.0, 0.0, 0.0;
        C.col((numTriangles * 3) + 5) << 0.0, 0.0, 0.0;

        // Clicking for the first time, so set the 'line' to the same coords
        V(0, ( numTriangles * 3) ) = xworld;
        V(0, (( numTriangles * 3) + 1) ) = xworld;
        V(0, (( numTriangles * 3) + 2) ) = xworld;
        V(0, (( numTriangles * 3) + 3) ) = xworld;
        V(0, (( numTriangles * 3) + 4) ) = xworld;
        V(0, (( numTriangles * 3) + 5) ) = xworld;
        V(1, ( numTriangles * 3) ) = yworld;
        V(1, (( numTriangles * 3) + 1) ) = yworld;
        V(1, (( numTriangles * 3) + 2) ) = yworld;
        V(1, (( numTriangles * 3) + 3) ) = yworld;
        V(1, (( numTriangles * 3) + 4) ) = yworld;
        V(1, (( numTriangles * 3) + 5) ) = yworld;

        // std::cout << "\t" << V << std::endl;
        VBO.update(V);
        VBO_C.update(C);

      }else if(insertClickCount == 1){ // Second click
        // std::cout << "\t Changing V for multiple dynamic lines" << std::endl;
        V.col(( numTriangles * 3 ) + 1) << xworld, yworld;
        V.col( (numTriangles * 3) + 2) << V( 0, ( numTriangles * 3 )), V( 1, ( numTriangles * 3 ));
        V.col( (numTriangles * 3) + 3) << xworld, yworld;
        V.col( (numTriangles * 3) + 4) << xworld, yworld;
        V.col( (numTriangles * 3) + 5) << xworld, yworld;

        VBO.update(V);
        // std::cout << '\t' << V << std::endl;
      }else if(insertClickCount == 2){

        Eigen::MatrixXf V_alt(2, (numTriangles * 3) + 3);
        MatrixXf C_alt(3, (numTriangles * 3) + 3);

        int start_column = numTriangles * 3;
        // copy everything before start column into new matrix
        for(int i = 0; i < start_column; i++)
        {
          V_alt.col(i) = V.col(i);
          C_alt.col(i) = C.col(i);
        }

        V_alt.col(start_column) << V(0, start_column), V(1, start_column);
        V_alt.col(start_column + 1) << V(0, (start_column + 1)), V(1, (start_column + 1));
        V_alt.col(start_column + 2) << xworld, yworld;

        C_alt.col(start_column) << 0.0, 0.0, 0.0;
        C_alt.col(start_column + 1) << 0.0, 0.0, 0.0;
        C_alt.col(start_column + 2) << 0.0, 0.0, 0.0;

        numTriangles++;

        // Update vertices matrix V
        V.resize(2, (numTriangles * 3));
        V = V_alt;
        VBO.update(V);

        // Update color matrix C
        C.resize(3, (numTriangles * 3));
        C = C_alt;
        VBO_C.update(C);
      }
      insertClickCount++;
      std::cout << "\t insertClickCount: " << insertClickCount << std::endl;
    }
    else if(translationMode)
    {
      if(action == GLFW_PRESS)
      {
        // See if cursor position is within or on the border of a triangle
        for(int i = 0; i < V.cols(); i += 3) // 3 vertices per triangle
        {
          float coord1_x = V(0,i);
          float coord1_y = V(1, i);

          float coord2_x = V(0, i + 1);
          float coord2_y = V(1, i + 1);

          float coord3_x = V(0, i + 2);
          float coord3_y = V(1, i + 2);

          translationPressed = clickedOnTriangle(xworld, yworld, coord1_x, coord1_y, coord2_x, coord2_y, coord3_x, coord3_y);
          // If it's clicked on, save V indices of selected triangle for mouse movement
          if(translationPressed)
          {
            // Reset the old color of previously selected triangle
            cout << "Vertex 1 inside translation mouse press: " << vertex_1_clicked << endl;
            cout << "i inside translation mouse press: " << i << endl;
            if(vertex_1_clicked != i){
              if(vertex_1_clicked > -1){
                // Swapping selected triangles
                cout << "Reseting old triangle color" << endl;
                C.col(vertex_1_clicked) = oldTranslatedColor.col(0);
                C.col(vertex_2_clicked) = oldTranslatedColor.col(1);
                C.col(vertex_3_clicked) = oldTranslatedColor.col(2);
              }
              vertex_1_clicked = i;
              vertex_2_clicked = i + 1;
              vertex_3_clicked = i + 2;

              // Hold the old color for when it's unselected
              cout << "Storing old color" << endl;
              oldTranslatedColor.col(0) = C.col(vertex_1_clicked);
              oldTranslatedColor.col(1) = C.col(vertex_2_clicked);
              oldTranslatedColor.col(2) = C.col(vertex_3_clicked);

              // Make selected triangle white
              cout << "Setting new color to white" << endl;
              C.col(vertex_1_clicked) << 1.0, 1.0, 1.0;
              C.col(vertex_2_clicked) << 1.0, 1.0, 1.0;
              C.col(vertex_3_clicked) << 1.0, 1.0, 1.0;

              VBO_C.update(C);
            }

            break;
          }
        }

      }else if(action == GLFW_RELEASE && translationPressed)
      {
        translationPressed = false;
        translationSelected = true;
        VBO_C.update(C);
      }

    }
    else if(deleteMode && action == GLFW_RELEASE)
    {
      // See if cursor position is within or on the border of a triangle
      for(int i = 0; i < V.cols(); i += 3) // 3 vertices per triangle
      {
        float coord1_x = V(0,i);
        float coord1_y = V(1, i);

        float coord2_x = V(0, i + 1);
        float coord2_y = V(1, i + 1);

        float coord3_x = V(0, i + 2);
        float coord3_y = V(1, i + 2);

        bool shouldDelete = clickedOnTriangle(xworld, yworld, coord1_x, coord1_y, coord2_x, coord2_y, coord3_x, coord3_y);

        // If it's clicked on, delete it
        if(shouldDelete)
        {
          // Get triangle information & remove it
          numTriangles--;
          vertex_1_deleted = i;
          vertex_2_deleted = i + 1;
          vertex_3_deleted = i + 2;

          // cout <<"Num triangles:" <<numTriangles << endl;
          Eigen::MatrixXf V_alt(2, (numTriangles * 3));
          MatrixXf C_alt(3, (numTriangles * 3));
          // C.conservativeResize(3, (numTriangles * 3));
          int alt_counter = 0;
          // Copy non-clicked columns of V into V_alt
          for(int cols = 0; cols < V.cols(); cols += 1)
          {
            if(cols == vertex_1_deleted || cols == vertex_2_deleted || cols == vertex_3_deleted)
            {
              continue;
            }else{
              V_alt.col(alt_counter) = V.col(cols);
              C_alt.col(alt_counter) = C.col(cols);
              alt_counter++;
            }
          } // inner V for

          cout << "DELETED tringle" << endl;
          V = V_alt;
          C = C_alt;
          break;
        } //end shouldDelete if
      } //end V for
      VBO.update(V);
      VBO_C.update(C);

    } //end deleteMode
    else if(colorMode && action == GLFW_RELEASE)
    {
      // Find closest vertex using Euclidean distance
      float smallestDistance = 1000.;
      for(int i = 0; i < V.cols(); i++)
      {
        float x_distance = xworld - V(0,i);
        float y_distance = yworld - V(1, i);
        float distance = sqrt(pow(x_distance, 2) + pow(y_distance, 2));
        if(distance < smallestDistance)
        {
          cout << "Found closest vertex: " << distance <<  " is less than: " << smallestDistance << endl;
          smallestDistance = distance;
          coloringVertex = i;
        }
      }
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Set boolean variables if I, O, or P is pressed
    if(action == GLFW_RELEASE)
    {
      switch (key)
      {
          case GLFW_KEY_I:
              cout << "INSERTION mode" << endl;
              insertionMode = true;
              translationMode = false;
              deleteMode = false;
              colorMode = false;
              resetTranslationVariables();
              break;
          case GLFW_KEY_O:
              cout << "TRANSLATION mode"  <<endl;
              insertionMode = false;
              translationMode = true;
              deleteMode = false;
              colorMode = false;
              break;
          case  GLFW_KEY_P:
              cout << "DELETE mode" << endl;
              insertionMode = false;
              translationMode = false;
              deleteMode = true;
              colorMode = false;
              resetTranslationVariables();
              break;
          case  GLFW_KEY_C:
              cout << "COLOR mode" << endl;
              insertionMode = false;
              translationMode = false;
              deleteMode = false;
              colorMode = true;
              resetTranslationVariables();
              break;
          case  GLFW_KEY_H:
              cout << "Rotate Clockwise by 10 degrees"  << endl;
              rotateTriangle(true);
              break;
          case  GLFW_KEY_J:
              cout << "Rotate Counter-Clockwise by 10 degrees"  << endl;
              rotateTriangle(false);
              break;
          case  GLFW_KEY_K:
              cout << "Scale up by 25 percent" << endl;
              scaleTriangle(true);
              break;
          case  GLFW_KEY_L:
              cout << "Scale down by 25 percent" << endl;
              scaleTriangle(false);
              break;
          case GLFW_KEY_W:
              cout << "Shifting DOWN" << endl;
              shiftVertical(true);
              break;
          case GLFW_KEY_A:
            cout << "Shifting RIGHT " << endl;
            shiftHorizontal(true);
            break;
          case GLFW_KEY_S:
            cout << "Shifting UP" << endl;
            shiftVertical(false);
            break;
          case GLFW_KEY_D:
            cout << "Shifting LEFT" << endl;
            shiftHorizontal(false);
            break;
          case GLFW_KEY_MINUS:
            cout << "ZOOM OUT" << endl;
            zoom(false);
            break;
          case GLFW_KEY_EQUAL:
            cout << "ZOOM IN" << endl;
            zoom(true);
            break;
          case GLFW_KEY_1:
            pressed = 1;
            colorVertex();
            break;
          case GLFW_KEY_2:
            pressed = 2;
            colorVertex();
            break;
          case GLFW_KEY_3:
            pressed = 3;
            colorVertex();
            break;
          case GLFW_KEY_4:
            pressed = 4;
            colorVertex();
            break;
          case GLFW_KEY_5:
            pressed = 5;
            colorVertex();
            break;
          case GLFW_KEY_6:
            pressed = 6;
            colorVertex();
            break;
          case GLFW_KEY_7:
            pressed = 7;
            colorVertex();
            break;
          case GLFW_KEY_8:
            pressed = 8;
            colorVertex();
            break;
          case GLFW_KEY_9:
            pressed = 9;
            colorVertex();
            break;
          case GLFW_KEY_ESCAPE:
              cout << "Reset to default mode" << endl;
              insertionMode = false;
              translationMode = false;
              deleteMode = false;
              colorMode = false;
              resetTranslationVariables();
          default:
              break;
      } // End switch
    }

    // Upload the change to the GPU
    // VBO.update(V);
}

int main(void)
{
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

        // On apple we have to load a core profile with forward compatibility
    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    // Create a windowed mode window and its OpenGL context
    /**
    The fourth parameter should be set to NULL for windowed mode and glfwGetPrimaryMonitor() for fullscreen mode.
    The last parameter allows you to specify an existing OpenGL context to share resources like textures with
    **/
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex
    // attributes are stored in a Vertex Buffer Object (or VBO). This means that
    // the VAO is not the actual object storing the vertex data,
    // but the descriptor of the vertex data.
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    // Initialize the VBO with the vertices data
    // A VBO is a data container that lives in the GPU memory
    VBO.init();
    V << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    VBO.update(V);

    // Second VBO for colors
    VBO_C.init();
    C <<
    0, 0, 0,
    0, 0, 0,
    0, 0, 0;
    VBO_C.update(C);

    // Set translation matrices for shader
    scaling <<
    1, 0,
    0, 1;

    rotation <<
    1, 0,
    0, 1;

    colors <<
    0.33, 0.66, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0,  0.33, 0.66, 0.99, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.33, 0.66, 0.99;

    view <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;


    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec2 position;"
                    "uniform mat4 view;"
                    "in vec3 color;"
                    "out vec3 f_color;"
                    "void main()"
                    "{"
                    "    gl_Position = view * vec4(position, 0.0, 1.0);"
                    "    f_color = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec3 f_color;"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(f_color, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    // The vertex shader wants the position of the vertices as an input.
    // The following line connects the VBO we defined above with the position "slot"
    // in the vertex shader
    program.bindVertexAttribArray("position",VBO);
    program.bindVertexAttribArray("color",VBO_C);

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse click callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Register the mouse movement callback
    glfwSetCursorPosCallback(window, cursor_pos_callback);

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // // Set the uniform value depending on the time difference
        // auto t_now = std::chrono::high_resolution_clock::now();
        // float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
        // glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 1.0f) / 2.0f, 0.0f, 0.0f);

        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());


        // INSERTION STATE DRAWING
        if(insertClickCount == 1){
          glDrawArrays(GL_LINES, (numTriangles*3), 2);
        }else if(insertClickCount ==  2){ //need to trace out 3 lines
          glDrawArrays(GL_LINES, (numTriangles*3), 6);
        }else if(insertClickCount == 3){
          insertClickCount = 0;
          glDrawArrays( GL_TRIANGLES, 0, (numTriangles * 3));
        }
        // TRANSLATION STATE DRAWING
        if(translationMode && (translationPressed || translationSelected)){
           // Make the selected triangle white, everything else black
          for(int i = 0; i < V.cols(); i+= 3)
          {
            if(i == vertex_1_clicked)
            {
              glUniform3f(program.uniform("triangleColor"), 1.0f, 1.0f, 1.0f);
              // CHANGE ROTATION/SCALING MATRIX & REUPLOAD
              glDrawArrays(GL_TRIANGLES, i, 3);
              glUniform3f(program.uniform("triangleColor"), 0.0f, 0.0f, 0.0f);
            }else{
              glDrawArrays(GL_TRIANGLES, i, 3);
            }
          }
        }
        // NORMAL STATE DRAWING
        else{
          // Draw everything black
          glUniform3f(program.uniform("triangleColor"), 0.0f, 0.0f, 0.0f);
          glDrawArrays(GL_TRIANGLES, 0, (numTriangles * 3));
        }


        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();

    }

    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
