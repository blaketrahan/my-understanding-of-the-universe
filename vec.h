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

    vec3(f4 mx = 0.0f, f4 my = 0.0f, f4 mz = 0.0f)
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
    inline vec3 normal()
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
    inline f4 dot(vec3 v)
    {
        return (x * v.x) + (y * v.y) + (z * v.z);
    }
    inline vec3 lerpto(vec3 target, f4 alpha)
    {
        return (target * alpha) + (vec3(x,y,z) * (1.0f - alpha));
    }
} vec3;

inline void print(vec3 v)
{
    std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
}

#endif