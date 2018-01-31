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

typedef struct mat4
{
    f4 e[4][4];

    inline mat4 identity ()
    {
        mat4 r;
        f4 d = 1.0f;
        r.e[0][0] =   d; r.e[1][0] = .0f; r.e[2][0] = .0f; r.e[3][0] = .0f;
        r.e[0][1] = .0f; r.e[1][1] =   d; r.e[2][1] = .0f; r.e[3][1] = .0f;
        r.e[0][2] = .0f; r.e[1][2] = .0f; r.e[2][2] =   d; r.e[3][2] = .0f;
        r.e[0][3] = .0f; r.e[1][3] = .0f; r.e[2][3] = .0f; r.e[3][3] =   d;
        return r;
    }

} mat4;

/*
    3D Vector
*/
typedef struct vec3
{
    f4 x,y,z;

    vec3 (f4 mx = 0.0f, f4 my = 0.0f, f4 mz = 0.0f)
        : x(mx), y(my), z(mz) {}

    inline vec3 operator + ( const vec3 &v )
    {
        vec3 r;
        r.x = x + v.x;
        r.y = y + v.y;
        r.z = z + v.z;
        return r;
    }
    inline vec3 operator - ( const vec3 &v )
    {
        vec3 r;
        r.x = x - v.x;
        r.y = y - v.y;
        r.z = z - v.z;
        return r;
    }
    inline vec3 operator * ( f4 scalar )
    {
        vec3 r;
        r.x = x * scalar;
        r.y = y * scalar;
        r.z = z * scalar;
        return r;
    }
    inline vec3 operator * ( vec3 v )
    {
        vec3 r;
        r.x = x * v.x;
        r.y = y * v.y;
        r.z = z * v.z;
        return r;
    }
    inline vec3 operator / ( f4 scalar )
    {
        vec3 r;
        r.x = x / scalar;
        r.y = y / scalar;
        r.z = z / scalar;
        return r;
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
    inline b4 has_length ()
    {
        return x != 0.0f || y != 0.0f || z != 0.0f;
    }
    inline vec3 set (f4 v)
    {
        return vec3(v,v,v);
    }
    inline void print ()
    {
        std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
    }
} vec3;

/*
    Quaternion

    Source:
        http://sjbrown.co.uk/2002/05/01/representing-rotations-in-quaternion-arithmetic/
        http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
        https://www.youtube.com/watch?v=SDS5gLSiLg0&t=1934s
        https://github.com/HandmadeMath/Handmade-Math/blob/master/HandmadeMath.
        http://quaternions.online/
*/
typedef struct quaternion
{
    f4 w,x,y,z;

    quaternion (f4 mw = 0.0f, f4 mx = 0.0f, f4 my = 0.0f, f4 mz = 0.0f)
        : w(mw), x(mx), y(my), z(mz) {}

    inline f4 length ()
    {
        return sqrt((w * w) + (x * x) + (y * y) + (z * z));
    }

    inline f4 length_squared ()
    {
        return ((w * w) + (x * x) + (y * y) + (z * z));
    }

    inline quaternion normal ()
    {   
        f4 magnitude = sqrt((w * w) + (x * x) + (y * y) + (z * z));
        if (magnitude == 0.0f)
        {
            return quaternion(0.0f, 0.0f,0.0f,0.0f);
        }
        else {
            return quaternion(w / magnitude, x / magnitude, y / magnitude, z / magnitude);
        }
    }

    inline quaternion operator + (quaternion q)
    {
        quaternion r;
        r.w = w + q.w;
        r.x = x + q.x;
        r.y = y + q.y;
        r.z = z + q.z;
        return r;
    }

    inline quaternion operator - (quaternion q)
    {
        quaternion r;
        r.w = w - q.w;
        r.x = x - q.x;
        r.y = y - q.y;
        r.z = z - q.z;
        return r;
    }

    inline quaternion operator * (quaternion q)
    {
        quaternion r;
        r.w = w * q.w - x * q.x - y * q.y - z * q.z;
        r.x = w * q.x + x * q.w + y * q.z - z * q.y;
        r.y = w * q.y + y * q.w + z * q.x - x * q.z;
        r.z = w * q.z + z * q.w + x * q.y - y * q.x;
        return r;
    }

    inline quaternion operator * (f4 scalar)
    {
        quaternion r;
        r.w = w * scalar;
        r.x = x * scalar;
        r.y = y * scalar;
        r.z = z * scalar;
        return r;
    }

    inline quaternion operator / (f4 scalar)
    {
        quaternion r;
        r.w = w / scalar;
        r.x = x / scalar;
        r.y = y / scalar;
        r.z = z / scalar;
        return r;
    }

    inline f4 dot (quaternion q)
    {
        return ((x * q.x) + (y * q.y) + (z * q.z) + (w * q.w));
    }

    inline quaternion lerp (quaternion target, f4 alpha)
    {
        quaternion r;
        
        r.w = (target.w * alpha) + (w * (1.0f - alpha));
        r.x = (target.x * alpha) + (x * (1.0f - alpha));
        r.y = (target.y * alpha) + (y * (1.0f - alpha));
        r.z = (target.z * alpha) + (z * (1.0f - alpha));

        return r.normal();
    }
    quaternion inverse ()
    {
        /*
            @todo: learn more about the inverse
        */
        quaternion conjugate;
        quaternion r;

        f4 length = 0;
        f4 length_squared = 0;

        conjugate.w =  w;
        conjugate.x = -x;
        conjugate.y = -y;
        conjugate.z = -z;

        length_squared = quaternion(w,x,y,z).length_squared();

        r.w = conjugate.w / length_squared;
        r.x = conjugate.x / length_squared;
        r.y = conjugate.y / length_squared;
        r.z = conjugate.z / length_squared;

        return r;
    }

    mat4 matrix ()
    {
        /*
            @todo: learn more about this, verify this is correct
        */
        mat4 r;
        r = r.identity();

        quaternion qn = quaternion(w,x,y,z).normal();
        
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

        r.e[0][0] = 1.0f - 2.0f * (YY + ZZ);
        r.e[0][1] = 2.0f * (XY + WZ);
        r.e[0][2] = 2.0f * (XZ - WY);

        r.e[1][0] = 2.0f * (XY - WZ);
        r.e[1][1] = 1.0f - 2.0f * (XX + ZZ);
        r.e[1][2] = 2.0f * (YZ + WX);

        r.e[2][0] = 2.0f * (XZ + WY);
        r.e[2][1] = 2.0f * (YZ - WX);
        r.e[2][2] = 1.0f - 2.0f * (XX + YY);

        return r;
    }

    quaternion from_axis_angle (vec3 axis, f4 angle)
    {
        /*
            @todo: learn more about this, verify this is correct
        */
        quaternion r;
        
        f4 axis_length = axis.length();
        f4 sine = sin(angle / 2.0f);

        vec3 rotated_vector = axis * sine;
        rotated_vector = rotated_vector / axis_length;

        r.w = cos(angle / 2.0f);
        r.x = rotated_vector.x;
        r.y = rotated_vector.y;
        r.z = rotated_vector.z;

        return r;
    }

} quaternion;

#endif