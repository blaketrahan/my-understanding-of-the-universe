attribute vec3 coord3d;
attribute vec2 tex_coord2d;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform vec3 scale;
uniform mat4 rotation;

// fragment
uniform sampler2D tex_source;
varying vec2 UV;

void main(void)
{
    vec3 scaled_vertex = coord3d * scale;

    gl_Position = proj * view * model * rotation * vec4(scaled_vertex.x,scaled_vertex.y,scaled_vertex.z, 1.0);

    UV = tex_coord2d;
}