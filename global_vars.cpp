#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

// vector: eventually remove this
// only using it for shader creation
#include <vector> // shader creation
#include <algorithm> // file saving
#include <fstream> // file saving
#include <cstdlib> // i dont know

#include <SDL2/SDL.h>

#define GLEW_STATIC
#include <GL/glew.h>
#include <SDL2/SDL_opengl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "b_memory.h"
#include "b_vec.h"

#define DEBUG_BUILD

using namespace std;

struct PRIMITIVE
{
	GLuint verts;
	GLuint colors;
	GLuint indices;
	GLuint uv_coords;
} plane; 

struct BASIC_TEXTURE_SHADER {
	GLuint program;
	GLint uniform_model;
	GLint uniform_view;
	GLint uniform_proj;
	GLint uniform_scale;
	GLint uniform_tex_source;
	GLint uniform_tex_offset;
	GLint attribute_coord3d;
	GLint attribute_tex_coord2d;
} basic_texture;

struct SGL
{
	SDL_Window* window;
	s4 width;
	s4 height;
} sgl;

struct RENDER_STATE {
	glm::mat4 view;
	glm::mat4 projection;
	vec3 rotation;
	vec3 world;
	vec3 scale;
	GLuint texture;
};

struct IMAGE {
	unsigned char* data;
	int x;
	int y;
	int n;
} image_font;

GLuint texture_font;