#ifndef B_MATH3D_H
#define B_MATH3D_H

#include <iostream>
#include <iomanip>
#include "math.h"

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

/*
    3D Vector
*/
struct vec3
{
    f4 x = 0.0f;
    f4 y = 0.0f;
    f4 z = 0.0f;
};
inline vec3 operator + (vec3 a, vec3 b)
{
    vec3 r;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    return r;
}
inline vec3 operator - (vec3 a, vec3 b)
{
    vec3 r;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    return r;
}
inline vec3 operator * (vec3 v, f4 scalar)
{
    vec3 r;
    r.x = v.x * scalar;
    r.y = v.y * scalar;
    r.z = v.z * scalar;
    return r;
}
inline vec3 operator * (vec3 a, vec3 b)
{
    vec3 r;
    r.x = a.x * b.x;
    r.y = a.y * b.y;
    r.z = a.z * b.z;
    return r;
}
inline vec3 lerp (vec3 a, vec3 b, f4 alpha)
{
    vec3 r;
    f4 t = 1.0f - alpha;
    r.x = (b.x * alpha) + (a.x * t);
    r.y = (b.y * alpha) + (a.y * t);
    r.z = (b.z * alpha) + (a.z * t);
    return r;
}
inline vec3 operator / (vec3 v, f4 scalar)
{
    vec3 r;
    r.x = v.x / scalar;
    r.y = v.y / scalar;
    r.z = v.z / scalar;
    return r;
}
inline f4 dot (vec3 a, vec3 b)
{
    return ( (a.x * b.x) + (a.y * b.y) + (a.z * b.z) );
}
inline f4 length (vec3 v)
{
    return sqrtf(dot(v,v));
}
inline vec3 normal (vec3 v)
{
    vec3 r;
    f4 magnitude = length(v);
    r = v / magnitude;
    return r;
}
inline f4 length_squared (vec3 v)
{
    return dot(v,v);
}
inline f4 has_length (vec3 v)
{
    return length_squared(v);
}
inline vec3 setv (f4 scalar)
{
    vec3 r;
    r.x = scalar;
    r.y = scalar;
    r.z = scalar;
    return r;
}
inline vec3 setv (f4 x, f4 y, f4 z)
{
    vec3 r;
    r.x = x;
    r.y = y;
    r.z = z;
    return r;
}
inline vec3 weighted_average (vec3 a, vec3 b, f4 weight)
{
    vec3 r;
    r.x = ((a.x * (weight - 1)) + b.x) / weight;
    r.y = ((a.y * (weight - 1)) + b.y) / weight;
    r.z = ((a.z * (weight - 1)) + b.z) / weight;
    return r;
}
/*
    Quaternion

    Source:
        http://sjbrown.co.uk/2002/05/01/representing-rotations-in-quaternion-arithmetic/
        http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
        https://www.youtube.com/watch?v=SDS5gLSiLg0&t=1934s
        https://github.com/HandmadeMath/Handmade-Math/blob/master/HandmadeMath.
        http://quaternions.online/
        https://www.3dgep.com/understanding-quaternions/
        https://www.youtube.com/playlist?list=PLi1nGcVzHwvNCML2OoO5ZPvnLN1mdfvS4
*/
struct quat
{
    f4 w = 1.0f;
    f4 x = 0.0f;
    f4 y = 0.0f;
    f4 z = 0.0f;
};
inline f4 dot (quat a, quat b)
{
    return ((a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w));
}
inline f4 length (quat q)
{
    return sqrtf(dot(q,q));
}
inline quat operator / (quat q, f4 scalar)
{
    quat r;
    r.w = q.w / scalar;
    r.x = q.x / scalar;
    r.y = q.y / scalar;
    r.z = q.z / scalar;
    return r;
}
inline quat normal (quat q)
{   
    quat r;
    f4 magnitude = length(q);
    r = q / magnitude;
    return r;
}
inline quat operator * (quat a, quat b)
{
    quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
    r.z = a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x;
    return r;
}
inline quat operator * (quat q, f4 scalar)
{
    quat r;
    r.w = q.w * scalar;
    r.x = q.x * scalar;
    r.y = q.y * scalar;
    r.z = q.z * scalar;
    return r;
}
glm::mat4 glm_matrix (quat q)
{
    /*
        @todo: learn more about this, verify this is correct
    */
    quat qn = normal(q);
    
    f4    XX, YY, ZZ,
          XY, XZ, YZ,
          WX, WY, WZ;

    XX = qn.x * qn.x;
    YY = qn.y * qn.y;
    ZZ = qn.z * qn.z;
    XY = qn.x * qn.y;
    XZ = qn.x * qn.z;
    YZ = qn.y * qn.z;
    WX = qn.w * qn.x;
    WY = qn.w * qn.y;
    WZ = qn.w * qn.z;

    glm::mat4 m(    
        1.0f - 2.0f * (YY + ZZ), 2.0f * (XY - WZ),        2.0f * (XZ + WY),        0.0f,
        2.0f * (XY + WZ),        1.0f - 2.0f * (XX + ZZ), 2.0f * (YZ - WX),        0.0f,
        2.0f * (XZ - WY),        2.0f * (YZ + WX),        1.0f - 2.0f * (XX + YY), 0.0f,
        0.0f,                    0.0f,                    0.0f,                    1.0f
    );

    return m;
}
quat quat_from_axis (vec3 axis, f4 angle)
{
    /*
        @todo: learn more about this, verify this is correct
    */
    quat r;

    angle = angle * M_DEGTORAD32;
    
    f4 axis_length = length(axis);
    f4 sine = sinf(angle / 2.0f);

    vec3 rotated_vector = axis * sine;
    rotated_vector = rotated_vector / axis_length;

    r.w = cosf(angle / 2.0f);
    r.x = rotated_vector.x;
    r.y = rotated_vector.y;
    r.z = rotated_vector.z;

    return r;
}
quat quat_axis_z (f4 angle)
{
    quat r;
    angle = angle * M_DEGTORAD32;

    r.w = cosf(angle / 2.0f);
    r.x = 0.0f;
    r.y = 0.0f;
    r.z = 1.0f * sinf(angle / 2.0f);

    return r;
}
quat quat_axis_x (f4 angle)
{
    quat r;
    angle = angle * M_DEGTORAD32;

    r.w = cosf(angle / 2.0f);
    r.x = 1.0f * sinf(angle / 2.0f);
    r.y = 0.0f;
    r.z = 0.0f;

    return r;
}
quat quat_axis_y (f4 angle)
{
    quat r;
    angle = angle * M_DEGTORAD32;

    r.w = cosf(angle / 2.0f);
    r.x = 0.0f;
    r.y = 1.0f * sinf(angle / 2.0f);
    r.z = 0.0f;

    return r;
}
#endif