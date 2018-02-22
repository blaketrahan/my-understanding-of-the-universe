#ifndef B_MATH3D_H
#define B_MATH3D_H

#include <iostream>
#include <iomanip>
#include "math.h"

#define PI64 3.1415926535897932384626433832795028841971693993751
#define DEGTORAD64 PI64 / 180.0
#define RADTODEG64 180.0 / PI64

#define PI 3.14159265359f
#define DEGTORAD PI / 180.0f
#define RADTODEG 180.0f / PI

inline f4 degtorad (f4 degrees)
{
    return degrees * DEGTORAD;
}
struct mat3x3
{
    f4 e[9];
    f4& operator [] (u4 index)
    {
        return e[index];
    }
};
struct vec3
{
    f4 x = 0.0f;
    f4 y = 0.0f;
    f4 z = 0.0f;
};
struct quat
{
    f4 w = 1.0f;
    f4 x = 0.0f;
    f4 y = 0.0f;
    f4 z = 0.0f;
};
/*
    Matrix 3x3

    For more study:
        https://math.stackexchange.com/questions/24456/matrix-multiplication-interpreting-and-understanding-the-process/24469#24469
        https://www.khanacademy.org/math/linear-algebra/matrix-transformations/composition-of-transformations/v/compositions-of-linear-transformations-1
*/
inline mat3x3 identity ()
{
    mat3x3 r;
    r[0] = 1.0f; r[1] = 0.0f; r[2] = 0.0f;
    r[3] = 0.0f; r[4] = 1.0f; r[5] = 0.0f;
    r[6] = 0.0f; r[7] = 0.0f; r[8] = 1.0f;
    return r;
}
inline mat3x3 operator * (mat3x3 a, mat3x3 b)
{
    mat3x3 r;

    r[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    r[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    r[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

    r[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    r[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    r[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

    r[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    r[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    r[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

    return r;
}
inline mat3x3 operator * (mat3x3 m, f4 scalar)
{
    mat3x3 r;

    r[0] = m[0] * scalar;
    r[1] = m[1] * scalar;
    r[2] = m[2] * scalar;

    r[3] = m[3] * scalar;
    r[4] = m[4] * scalar;
    r[5] = m[5] * scalar;

    r[6] = m[6] * scalar;
    r[7] = m[7] * scalar;
    r[8] = m[8] * scalar;

    return r;
}
inline mat3x3 transpose (mat3x3 m)
{
    mat3x3 r;

    r[0] = m[0];
    r[1] = m[3];
    r[2] = m[6];

    r[3] = m[1];
    r[4] = m[4];
    r[5] = m[7];

    r[6] = m[2];
    r[7] = m[5];
    r[8] = m[8];

    return r;
}
mat3x3 inverse (mat3x3 m)
{
    // matrix of minors with cofactors
    mat3x3 r;
    r[0] =  m[4]*m[8] - m[5]*m[7];
    r[1] = (m[3]*m[8] - m[5]*m[6]) * -1.0f;
    r[2] =  m[3]*m[7] - m[4]*m[6];

    r[3] = (m[1]*m[8] - m[2]*m[7]) * -1.0f;
    r[4] =  m[0]*m[8] - m[2]*m[6];
    r[5] = (m[0]*m[7] - m[1]*m[6]) * -1.0f;

    r[6] =  m[1]*m[5] - m[2]*m[4];
    r[7] = (m[0]*m[5] - m[2]*m[3]) * -1.0f;
    r[8] =  m[0]*m[4] - m[1]*m[3];

    // adjugate
    r = transpose(r);

    // determinant
    f4  d =  m[0] * (m[4]*m[8] - m[5]*m[7]);
        d -= m[1] * (m[3]*m[8] - m[5]*m[6]);
        d += m[3] * (m[3]*m[7] - m[4]*m[6]);

    d = 1.0f / d;

    // adjugate * one over the determinant
    r = r * d;

    return r;
}
mat3x3 from_axis_angle (vec3 axis, f4 angle)
{
    /*
        Source:
            http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
    */
    f4 c = cosf(angle);
    f4 s = sinf(angle);
    f4 t = 1.0f - c;
    f4 x = axis.x;
    f4 y = axis.y;
    f4 z = axis.z;

    f4 xx = x*x;
    f4 xy = x*y;
    f4 xz = x*z;
    f4 yy = y*y;
    f4 yz = y*z;
    f4 zz = z*z;

    f4 xs = x * s;
    f4 ys = y * s;
    f4 zs = z * s;

    mat3x3 r;
    r[0] = t * xx + c;
    r[1] = t * xy - zs;
    r[2] = t * xz + ys;

    r[3] = t * xy + zs;
    r[4] = t * yy + c;
    r[5] = t * yz - xs;

    r[6] = t * xz - ys;
    r[7] = t * yz + xs;
    r[8] = t * zz + c;

    return r;
}
inline vec3 skew_symmetric (mat3x3 m, vec3 v)
{
    // tilde operator
    vec3 r;

    r.x = v.x * m[0] + v.y * m[3] + v.z * m[6];
    r.y = v.x * m[1] + v.y * m[4] + v.z * m[7];
    r.z = v.x * m[2] + v.y * m[5] + v.z * m[8];

    return r;
}
glm::mat4 glm_matrix (mat3x3 m)
{
    glm::mat4 r(    
        m[0], m[1], m[2], 0.0f,
        m[3], m[4], m[5], 0.0f,
        m[6], m[7], m[8], 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
    return r;
}
void print (mat3x3 m, const char* name)
{
    printf("%s : mat3x3\n[ %f  %f  %f ]\n[ %f  %f  %f ]\n[ %f  %f  %f ]\n\n",name,m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
}
/*
    3D Vector
*/
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
inline vec3 operator * (f4 scalar, vec3 v)
{
    return v * scalar;
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
    return f4( (a.x * b.x) + (a.y * b.y) + (a.z * b.z) );
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
inline f4 distance_between (vec3 a, vec3 b) {
    return sqrtf(((b.x - a.x) * (b.x - a.x)) + ((b.y - a.y) * (b.y - a.y)) + ((b.z - a.z) * (b.z - a.z)));
}
inline void print(vec3 v, const char* name = "") {
    printf("%s : vec3(%f, %f, %f) \n", name, v.x, v.y, v.z);
}
inline vec3 crossproduct (vec3 a, vec3 b)
{
    vec3 r;
    //    a Y   b Z - a Z   b Y
    r.x = a.y * b.z - a.z * b.y;
    //    a Z   b X - a X   b Z
    r.y = a.z * b.x - a.x * b.z;
    //    a X   b Y - a Y   b X
    r.z = a.x * b.y - a.y * b.x;
    return r;
};
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

    angle = degtorad(angle);
    
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
    angle = degtorad(angle);

    r.w = cosf(angle / 2.0f);
    r.x = 0.0f;
    r.y = 0.0f;
    r.z = 1.0f * sinf(angle / 2.0f);

    return r;
}
quat quat_axis_x (f4 angle)
{
    quat r;
    angle = degtorad(angle);

    r.w = cosf(angle / 2.0f);
    r.x = 1.0f * sinf(angle / 2.0f);
    r.y = 0.0f;
    r.z = 0.0f;

    return r;
}
quat quat_axis_y (f4 angle)
{
    quat r;
    angle = degtorad(angle);

    r.w = cosf(angle / 2.0f);
    r.x = 0.0f;
    r.y = 1.0f * sinf(angle / 2.0f);
    r.z = 0.0f;

    return r;
}
#endif