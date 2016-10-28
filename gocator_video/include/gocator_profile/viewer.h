#ifndef _VIEWER_H
#define _VIEWER_H

#include <string>
#include <map>

//We must include glew first before glfw3
#include <GL/glew.h>
#include <GLFW/glfw3.h>

//prototypes
void context(std::vector<float> xvec, std::vector<float> zvec, unsigned int validPointCount, unsigned int arrayIndex);
int width;
int height;

typedef struct
{
	double x;	// x-coordinate in engineering units (mm) - position along laser line
	double y;
	double z;	// z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
}ProfilePoint;

struct profileVertices
{
	float x;
	float y;
	float z;
};

//Global variables
ProfilePoint* profileBuffer = NULL;
ProfilePoint **surfaceBuffer = NULL;
k32u surfaceBufferHeight = 0;

void error_callback(int error, const char* description)
{
    ROS_INFO_STREAM("Error in context callback" << description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

//vertex shader in opengl shading language
const GLchar* vertexShaderSource = "#version 330 core \n"
		  
"layout (location = 0) in vec3 position;"
"void main()"
"{"
    "gl_Position = vec4(position.x, position.y, position.z, 1.0);"
"}";
const GLchar* fragmentShaderSource = "#version 330 core\n"
    "out vec4 color;\n"
    "void main()\n"
    "{\n"
    "color = vec4(1.0f, 0.1f, 0.0f, 1.0f);\n"
    "}\n\0";
void context(std::vector<float> xvec, std::vector<float> zvec, unsigned int validPointCount, unsigned int arrayIndex)
{
	glfwSetErrorCallback(error_callback);

	if(!glfwInit())
	{	
		ROS_FATAL("Could not initialize opengl window");
	}

	else 
		ROS_INFO( "Starting GLFW context, OpenGL 3.2");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	//set up opengl window and its context
	GLFWwindow* window = glfwCreateWindow(800, 600, "Profile Map", NULL, NULL);
	/*GLFWwindow* window =  glfwCreateWindow(800, 600, "OpenGL", \
							glfwGetPrimaryMonitor(), nullptr); 			//Fullscreen
	*/
	if(!window)
	{
		glfwTerminate();
		ROS_FATAL("Terminating glfw window");
	}

	glfwSetKeyCallback(window, key_callback);

	//Make Windows context current
	glfwMakeContextCurrent(window);

	glewExperimental = GL_TRUE;

	if (glewInit() != GLEW_OK)   //to setup the OpenGL Function pointers
	{
	    ROS_FATAL("Failed to initialize GLEW");
	}

	glViewport(0, 0, 800, 600);

	GLint vertexShader;
	vertexShader = glCreateShader(GL_VERTEX_SHADER);

	//attach shader source code to shader object and compile shader
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);

	//check for compile time errors
	GLint success;
	GLchar infoLog[512];  //storage container for error message
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

	if(!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		ROS_ERROR_STREAM("ERROR Shader Vertex Compilation Fail out!" << infoLog);
	}
	// Fragment shader
	GLint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);
	// Check for compile time errors
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}
	// Link shaders
	GLint shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	// Check for linking errors
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
	    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		   std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	unsigned int q;

	/*create vertex buffer objects (vbo's)*/
	GLuint VBO, VAO, EBO;
	for(q = 0; q <  xvec.size(); ++q)
	{
		/*std::cout << "(x, z):" << "\t (" << xvec.at(q) <<
					 ", " 	  << zvec.at(q) << ")" << std::endl;*/
		GLfloat vertices[]	= {	xvec.at(q), zvec.at(q), 0.0f};		
		GLuint indices []	= { q };

	

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(VAO);
	//bind the new buffer to the GL_Array_buffer target
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	//copy the previously defined vertex data into the buffer's memory
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STREAM_DRAW);			//cause profile will change with each iterarion

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
	//to normalize the data, i.e. map between 0 and 1 since actual range is -350:350
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 3 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind

	glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO
	}
	
	glfwGetFramebufferSize(window, &width, &height);
	
	while(!glfwWindowShouldClose(window))
	{
		/*Poll for and process events*/
		glfwPollEvents();
		/*This is where we do the mothertrucking render*/

		//clear buffer in each iteration
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT); 	//clear buffer bit
		glClear(GL_STENCIL_BUFFER_BIT);  //see http://www.learnopengl.com/#!Getting-Started/Hello-Window

		// Draw our first triangle
		glUseProgram(shaderProgram);
		glBindVertexArray(VAO);
		// glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		glDrawElements(GL_LINE_LOOP, validPointCount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		glfwSwapInterval(0.02);
		/*Swap front/back buffers*/
		glfwSwapBuffers(window);		
	}
	// Properly de-allocate all resources once they've outlived their purpose
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);

	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}


#endif