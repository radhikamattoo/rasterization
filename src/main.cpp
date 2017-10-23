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

using namespace std;
using namespace Eigen;

// VertexBufferObject wrapper
VertexBufferObject VBO;

// Contains the vertex positions
//(numRows, numCols)
Eigen::MatrixXf V(2,6);

// Holds # of clicks for triangle insertion
int insertClickCount = 0;

// If translationMode && clicked on a triangle && mouse is held down
bool translationPressed = false;

// Holds the indices of the triangle in V being translated
int vertex_1_clicked = -1;
int vertex_2_clicked = -1;
int vertex_3_clicked = -1;

// # of triangles that will be drawn
int numTriangles = 0;

// Am I currently creating a triangle?
bool insertionMode = false;

// Am I currently translating a triangle?
bool translationMode = false;

// Am I currently deleting a triangle?
bool deleteMode = false;

double currentX, currentY, previousX, previousY;

void translateTriangle()
{
  // Shift the values of the triangle in V based on the difference
  float coord_1_x = V(0, vertex_1_clicked);
  float coord_1_y = V(1, vertex_1_clicked);

  float coord_2_x = V(0, vertex_2_clicked);
  float coord_2_y = V(1, vertex_2_clicked);

  float coord_3_x = V(0, vertex_3_clicked);
  float coord_3_y = V(1, vertex_3_clicked);

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
// Returns bool describing if the mouse clicked on a triangle
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
  double xworld = ((xpos/double(width))*2)-1;
  double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

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
      // std::cout << "After second click: " << std::endl;
      // std::cout << V << std::endl;
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

    if(insertionMode && action == GLFW_RELEASE) // creating a triangle out of line segments!
    {
      // Get the position of the mouse in the window
      double xpos, ypos;
      glfwGetCursorPos(window, &xpos, &ypos);

      // Get the size of the window
      int width, height;
      glfwGetWindowSize(window, &width, &height);

      // Convert screen position to world coordinates
      double xworld = ((xpos/double(width))*2)-1;
      double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

      // Update insertClickCount
      std::cout << "Mouse clicked in INSERTION mode" << std::endl;
      if(insertClickCount == 0) // First click
      {
        V.conservativeResize(2, (numTriangles*3)+6);

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

        std::cout << "\t" << V << std::endl;
        VBO.update(V);
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
        int start_column = numTriangles * 3;
        // copy everything before start column into new matrix
        for(int i = 0; i < start_column; i++)
        {
          V_alt.col(i) = V.col(i);
        }
        V_alt.col(start_column) << V(0, start_column), V(1, start_column);
        V_alt.col(start_column + 1) << V(0, (start_column + 1)), V(1, (start_column + 1));
        V_alt.col(start_column + 2) << xworld, yworld;
        V.resize(2, ((numTriangles+1) * 3));
        V = V_alt;
        std::cout << "\t" << V << std::endl;
        VBO.update(V);
        numTriangles++;

      }
      insertClickCount++;
      std::cout << "\t insertClickCount: " << insertClickCount << std::endl;


    }

    else if(translationMode)
    {
      // Get the position of the mouse in the window
      double xpos, ypos;
      glfwGetCursorPos(window, &xpos, &ypos);

      // Get the size of the window
      int width, height;
      glfwGetWindowSize(window, &width, &height);

      // Convert screen position to world coordinates
      double xworld = ((xpos/double(width))*2)-1;
      double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

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
            vertex_1_clicked = i;
            vertex_2_clicked = i + 1;
            vertex_3_clicked = i + 2;
            break;
          }
        }

      }else if(action == GLFW_RELEASE)
      {
        // TODO: Update VBO?
        translationPressed = false;
      }

    }else if(deleteMode && action == GLFW_RELEASE)
    {
      // See if cursor position is within or on the border of a triangle
      // If it is, remove the 3 vertices from V
      // Decrement numTriangles counter
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Set boolean variables if I, O, or P is pressed
    if(action == GLFW_RELEASE)
    {
      switch (key)
      {
          case  GLFW_KEY_I:
              std::cout << "INSERTION mode: " << !insertionMode << std::endl;
              insertionMode = !insertionMode;
              translationMode = false;
              deleteMode = false;
              break;
          case GLFW_KEY_O:
              std::cout << "TRANSLATION mode"  << !translationMode << std::endl;
              insertionMode = false;
              translationMode = !translationMode;
              deleteMode = false;
              break;
          case  GLFW_KEY_P:
            std::cout << "DELETE mode"  << !deleteMode  << std::endl;
              insertionMode = false;
              translationMode = false;
              deleteMode = !deleteMode;
              break;
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

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec2 position;"
                    "void main()"
                    "{"
                    "    gl_Position = vec4(position, 0.0, 1.0);"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(triangleColor, 1.0);"
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


        // CREATING TRAINGLES
        if(insertClickCount == 1){
          glDrawArrays(GL_LINES, (numTriangles*3), 2);
        }else if(insertClickCount ==  2){ //need to trace out 3 lines
          glDrawArrays(GL_LINES, (numTriangles*3), 6);
        }else if(insertClickCount == 3){
          insertClickCount = 0;
        }

        // DRAWING TRIANGLES
        if(numTriangles > 0){
          glDrawArrays( GL_TRIANGLES, 0, (numTriangles * 3));
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
