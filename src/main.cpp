// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>

// IO Stream
#include <iostream>

// Timer
#include <chrono>

// VertexBufferObject wrapper
VertexBufferObject VBO;

// Contains the vertex positions
//(numRows, numCols)
Eigen::MatrixXf V(2,6);

// Holds # of clicks for triangle creation, or is NULL
int clickCount = -1;

// Change from line segments to a triangle
bool readyForTriangle = false;

// Am I currently creating a triangle? (1.1)
bool insertionMode = false;

// Am I currently translating a triangle?
bool translationMode = false;

// Am I currently deleting a triangle?
bool deleteMode = false;

double currentX, currentY, previousX, previousY;

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
  if(insertionMode && clickCount > 0)
  {
    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    double xworld = ((xpos/double(width))*2)-1;
    double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

    // Store coordinates in V and send to GPU
    if(clickCount == 1){
      V.col(clickCount) << xworld, yworld;
    }else if(clickCount == 2){
      V(0,3) = xworld;
      V(1,3) = yworld;
      V(0,5) = xworld,
      V(1,5) = yworld;
    }

    VBO.update(V);
  }
}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{

    if(insertionMode && action == GLFW_RELEASE) // creating a triangle out of line segments!
    {
      // Reset counter
      if(clickCount == -1) clickCount = 0;

      // Get the position of the mouse in the window
      double xpos, ypos;
      glfwGetCursorPos(window, &xpos, &ypos);

      // Get the size of the window
      int width, height;
      glfwGetWindowSize(window, &width, &height);

      // Convert screen position to world coordinates
      double xworld = ((xpos/double(width))*2)-1;
      double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

      // Update clickCount
      std::cout << "Mouse clicked in INSERTION mode" << std::endl;
      if(clickCount == 0) // First click
      {
        // Clicking for the frist time, so set the 'line' to the same coords
        V << xworld, xworld, yworld, yworld, xworld, yworld, xworld, yworld, xworld, yworld, xworld, yworld;
        std::cout << "\t" << V << std::endl;
        VBO.update(V);
      }else if(clickCount == 1){ // Second click
        std::cout << "\t Changing V for multiple dynamic lines" << std::endl;
        V.col(clickCount) << xworld, yworld;
        V.col(clickCount+1) << V(0,0), V(1,0);
        V.col(clickCount+2) << xworld, yworld;
        V.col(clickCount+3) << V(0,1), V(1,1);
        V.col(clickCount+4) << xworld, yworld;
        VBO.update(V);
        std::cout << '\t' << V << std::endl;
      }else if(clickCount == 2){
        insertionMode = false;
        Eigen::MatrixXf V_final(2,3);
        V_final.col(0) << V(0,0), V(1,0);
        V_final.col(1) << V(0,1), V(1,1);
        V_final.col(2) << xworld, yworld;
        VBO.update(V_final);

      }
      clickCount++;
      std::cout << "\t clickCount: " << clickCount << std::endl;


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

        // Draw a line
        if(clickCount == 1){
          // std::cout << "Drawing a single line " << std::endl;
          glDrawArrays(GL_LINES, 0, 2);
        }else if(clickCount ==  2){ //need to trace out 3 lines
          std::cout << "Drawing several lines " << std::endl;
          glDrawArrays(GL_LINES, 0, 6);
        }else if(clickCount == 3){
          std::cout << "Drawing a triangle " << std::endl;
          glDrawArrays(GL_TRIANGLES, 0, 3);
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
