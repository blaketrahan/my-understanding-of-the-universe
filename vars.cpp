#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

// vector: eventually remove this
// only using it for shader creation
#include <vector> // shader creation

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
    GLint uniform_rotation;
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
    vec3 world;
    vec3 scale;
    GLuint texture;
    mat3x3 orient;
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
    f4 radius;
    f4 density;
    f4 volume;
    f4 mass;
    f4 gravity;
    f4 one_over_mass;
    f4 coefficient_restitution;

    f4 width;
    f4 height;
    f4 depth;

    u4 type;

    mat3x3 MoI_local;
    mat3x3 inverse_MoI_local;
    mat3x3 inverse_MoI_world;

    vec3 prev_pos;
    vec3 pos;
    vec3 future_pos;
    mat3x3 orientation;
    vec3 angular_momentum;
    vec3 velocity;
    vec3 angular_velocity;

    vec3 force;
    vec3 torque;

    f4 collision_time;
    f4 remaining_velocity;
    vec3 collision_pos;
    vec3 collision_normal;
    vec3 PoC;
};

struct Entity 
{
    /* Physics */
    RigidBody body;

    /* Rendering */
    u4 mesh;
    GLuint texture;
    vec3 scale;
};

// input
struct SinglePress
{
    b4 pressed = false;
    b4 released = true;
};
inline b4 single_press(SinglePress &key)
{
    u4 state = key.released & key.pressed;
    key.released = !state & !key.pressed; 
    return state;
}
inline void poll_events();
struct Keys {
    b4 left, right, up, down;
    SinglePress w,a,s,d;
    SinglePress i,j,k,l;
    b4 quit_app;
} key;