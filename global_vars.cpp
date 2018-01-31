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

#include <stdio.h> // objloader.cpp
#include <string> // objloader.cpp
#include <cstring> // objloader.cpp

#include <SDL2/SDL.h>

#define GLEW_STATIC
#include <GL/glew.h>
#include <SDL2/SDL_opengl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "memory.h"
#include "math3D.h"

#define DEBUG_BUILD

using namespace std;

#include "objectloader.cpp"

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
};

struct Library 
{
    struct Texture {
        string name;
        GLuint id;
    };
    Texture* textures = 0;
    u4 texture_count = 0;

    /* ---------- */

    struct Mesh {
        string name;
        vector<glm::vec3> vertices;
        vector<glm::vec2> uvs;
        vector<glm::vec3> normals;
        GLuint vertex_buffer;
        GLuint uv_buffer;
    };
    Mesh* meshes = 0;
    u4 mesh_count = 0;
} library;

struct RigidBody 
{
    vec3 velocity;  /* LOCAL */
    vec3 pos;       /* GLOBAL */
    vec3 prev_pos;  /* GLOBAL */

    f4 radius = 0.1f;
    f4 mass = 10.0f;
    f4 restitution = 0.75f;
    f4 friction = 0.75f;

    vec3 user_force;    /* LOCAL */

    /* Collision information */
    vec3 PoC;           /* LOCAL */
    vec3 PoC_on_radius; /* LOCAL */
};

struct Entity 
{
    /* Physics */
    RigidBody body;

    /* Rendering */
    u4 mesh;
    GLuint texture;
    vec3 scale;
    vec3 rotation;
};