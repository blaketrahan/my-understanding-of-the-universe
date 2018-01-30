#ifndef B_VEC_234_H
#define B_VEC_234_H

#include <iostream>
#include <iomanip>
#include "math.h"

#ifndef M_DEFINE_ANGLES
#define M_DEFINE_ANGLES

#define M_PI32 3.14159265359f
#define M_DEGTORAD32 M_PI32 / 180.0f
#define M_RADTODEG32 180.0f / M_PI32
#define M_PI64 3.1415926535897932384626433832795028841971693993751
#define M_DEGTORAD64 M_PI64 / 180.0
#define M_RADTODEG64 180.0 / M_PI64

f4 identity[] =
{
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
};

f4 rotation_matrix[4][16] = 
{
    { // 0 degrees (identity)
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    },
    { // 90 degrees
        (f4)cos( M_PI32/2.0f ), (f4)-sin( M_PI32/2.0f ), 0.0f, 0.0f,
        (f4)sin( M_PI32/2.0f ), (f4) cos( M_PI32/2.0f ), 0.0f, 0.0f, // 
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    },
    { // 180 degrees
        (f4)cos( M_PI32 ), (f4)-sin( M_PI32 ), 0.0f, 0.0f,
        (f4)sin( M_PI32 ), (f4) cos( M_PI32 ), 0.0f, 0.0f, // top
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    },
    { // 270 degrees
        (f4)cos( 3.0f*M_PI32/2.0f ), (f4)-sin( 3.0f*M_PI32/2.0f ), 0.0f, 0.0f,
        (f4)sin( 3.0f*M_PI32/2.0f ), (f4) cos( 3.0f*M_PI32/2.0f ), 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    }
};
#endif

typedef struct vec3
{
    f4 x,y,z;

    vec3 (f4 mx = 0.0f, f4 my = 0.0f, f4 mz = 0.0f)
        : x(mx),y(my), z(mz) {}

    inline vec3 operator + ( const vec3 &v )
    {
        vec3 result;
        result.x = x + v.x;
        result.y = y + v.y;
        result.z = z + v.z;
        return result;
    }
    inline vec3 operator - ( const vec3 &v )
    {
        vec3 result;
        result.x = x - v.x;
        result.y = y - v.y;
        result.z = z - v.z;
        return result;
    }
    inline vec3 operator * ( f4 scalar )
    {
        vec3 result;
        result.x = x * scalar;
        result.y = y * scalar;
        result.z = z * scalar;
        return result;
    }
    inline vec3 operator * ( vec3 v )
    {
        vec3 result;
        result.x = x * v.x;
        result.y = y * v.y;
        result.z = z * v.z;
        return result;
    }
    inline b4 operator == ( vec3 v )
    {
        return (v.x == x && v.y == y && v.z == z);
    }
    inline b4 operator != ( vec3 v )
    {
        return v.x != x || v.y != y || v.z != z;
    }
    inline vec3 normal ()
    {
        f4 magnitude = sqrt((x * x) + (y * y) + (z * z));
        if (magnitude == 0.0f)
        {
            return vec3(0.0f,0.0f,0.0f);
        }
        else {
            return vec3(x / magnitude, y / magnitude, z / magnitude);
        }
    }
    inline f4 length ()
    {
        return sqrt((x * x) + (y * y) + (z * z));
    }
    inline f4 dot (vec3 v)
    {
        return (x * v.x) + (y * v.y) + (z * v.z);
    }
    inline vec3 lerp (vec3 target, f4 alpha)
    {
        return (target * alpha) + (vec3(x,y,z) * (1.0f - alpha));
    }
    inline b4 has_length () {
        return x != 0.0f || y != 0.0f || z != 0.0f;
    }
    inline vec3 set (f4 v) {
        return vec3(v,v,v);
    }
} vec3;

inline void print(vec3 v)
{
    std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
}

inline f4 distance_from_origin (vec3 v) {
    return sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

inline f4 distance_between (vec3 a, vec3 b) {
    return sqrt(((b.x - a.x) * (b.x - a.x)) + ((b.y - a.y) * (b.y - a.y)) + ((b.z - a.z) * (b.z - a.z)));
}
/* collisions */

inline vec3 reflect_circle_line ( vec3 circle_velocity, vec3 PoC, vec3 surface_normal )
{
    /* Assumes unit circle */
    
    // https://stackoverflow.com/questions/573084/how-to-calculate-bounce-angle
    f4 scalar_product = circle_velocity.dot(surface_normal);

    vec3 U = surface_normal * scalar_product;
    vec3 W = circle_velocity - U;

    return W - U;
}

#endif