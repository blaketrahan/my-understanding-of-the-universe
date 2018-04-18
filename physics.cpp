
enum BODY_TYPES {
    TYPE_SPHERE = 0,
    TYPE_CUBOID,
};

struct Plane {
    /*
        Counter Clockwise
        Top Left -> Bottom Left -> Bottom Right -> Top Right
    */
    vec3 pos; // local to rigidbody center
    vec3 normal;
    vec3 p[4]; // local to pos
};

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

    // Collision info from last detected collision
    f4 collision_time;
    f4 remaining_velocity;
    vec3 collision_pos;
    vec3 collision_normal;
    vec3 PoC;

    Plane* planes = 0;
    u4 num_plane = 0;
};

struct Entity 
{
    /* Physics */
    RigidBody body;

    /* Rendering */
    u4 mesh;
    GLuint texture;
};

struct BodyInfo {
    f4 restitution = 1.0f;
    f4 radius = 1.0f; 
    f4 density = 0.01f;
    vec3 pos;
    u4 type = 0;
    b4 dynamic = true;

    f4 width = 1.0f;
    f4 height = 1.0f;
    f4 depth = 1.0f;
};

void step (RigidBody &body)
{
    // Source: Chris Hecker pdf

    body.prev_pos = body.pos;
    body.collision_time = 0.0f;
    body.remaining_velocity = 1.0f;
    body.collision_pos = body.pos;

    // --

    body.force.z = body.force.z + body.gravity;

    // body.torque = crossproduct ( setv( 0, 0, -body.radius ), setv( 0, -0.00001f, 0 ) );

    // --

    body.velocity = body.velocity + (body.one_over_mass * body.force);
    body.velocity = body.velocity * 0.95f;

    body.future_pos = body.pos + body.velocity;

    body.orientation = body.orientation + (skew_symmetric(body.angular_velocity) * body.orientation);

    body.angular_momentum = body.angular_momentum + body.torque;

    body.orientation = orthonormalize(body.orientation);

    // --

    body.inverse_MoI_world = body.orientation * body.inverse_MoI_local * transpose(body.orientation);

    body.angular_velocity = body.angular_momentum * body.inverse_MoI_world;

    // -- clear forces
    body.torque = setv();
    body.force = setv();
}

void resolve_dynamic_static (RigidBody &A, RigidBody &B)
{
    // Rather than equal and opposite, body A should reflect off B
}

void resolve_dynamic_dynamic (RigidBody &A, RigidBody &B)
{
    // Source: Chris Hecker pdf

    vec3 r = A.PoC;
    vec3 rb = B.PoC;

    vec3 v1 = A.velocity + crossproduct(A.angular_velocity, r);
    vec3 v2 = B.velocity + crossproduct(B.angular_velocity, rb);
    vec3 v = v1 - v2;

    f4 ImpulseNumerator = -(1.0f + ((A.coefficient_restitution + B.coefficient_restitution) * 0.5f)) * dot( v, A.collision_normal);

    vec3 inertia_vector_normal = crossproduct(crossproduct(r, A.collision_normal) * A.inverse_MoI_world, r) + crossproduct(crossproduct(rb, A.collision_normal) * B.inverse_MoI_world, rb);
    f4 ImpulseDenominator = (A.one_over_mass + B.one_over_mass) + dot(inertia_vector_normal, A.collision_normal);

    vec3 Impulse = (ImpulseNumerator / ImpulseDenominator) * A.collision_normal;

    A.velocity = A.velocity + A.one_over_mass * Impulse;
    A.angular_momentum = A.angular_momentum + crossproduct(r, Impulse);
    // compute affected auxiliary quantities
    A.angular_velocity = A.angular_momentum * A.inverse_MoI_world;

    // -- equal and opposite
    B.velocity = B.velocity + (B.one_over_mass * Impulse * -1.0f);
    B.angular_momentum = B.angular_momentum + (crossproduct(rb, Impulse) * -1.0f);
    // compute affected auxiliary quantities
    B.angular_velocity = B.angular_momentum * B.inverse_MoI_world;

    // calculate new future position
    A.future_pos = A.future_pos + A.velocity * A.remaining_velocity;
    B.future_pos = B.future_pos + B.velocity * B.remaining_velocity;
}

b4 collide_sphere_sphere (RigidBody &A, RigidBody &B)
{
    /*
        Detect collision and correct velocities of:
            1) one moving circle against one stationary circle
            2) two moving circles

        Position the circles so that they are touching but not penetrating.

        Source: https://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php
        
        See references/circle-circle-collision.jpg
            references/circle-circle-collision-B.jpg
    */

    /*
        subtract velocity B from A, so that B is treated as stationary
        @todo: does this work if A is stationary and B is moving?

        @todo: doesn't handle contact without collision well.
               place two spheres running parallel where sides graze the other.
    */
    vec3 A_combined_velocity = A.velocity - B.velocity;

    f4 length_combined = length(A_combined_velocity);

    if (!length_combined) return false;

    /*
        is velocity less than distance between A and B
    */
    f4 dist = distance_between(B.pos, A.pos);
    f4 r = (B.radius + A.radius);
    dist -= r;
    if (length_combined < dist) return false;

    /*
        @todo: I don't understand why the normal rather than velocity is used.
    */
    vec3 N = normal(A_combined_velocity);

    /*
        vector C: A center to B center
    */
    vec3 C = B.pos - A.pos;

    /* 
        dot product: project vector N onto C, returns scalar value
        describing how much of N projects onto C
        if the value is <= 0, then N does not project onto C at all
        aka, the N is not moving toward C
        D = N . C = |C| * cos(angle between N and C)
    */
    f4 D = dot(N, C);
    if (D <= 0) { return false; }

    /* 
        F = |C| - dot product 
        F = Distance to C - Distance on C that A will travel
        Square these values so not to use squareroot
        Cannot collide if the difference is greater than their radii
    */
    f4 length_c = length(C);
    f4 F = (length_c * length_c) - (D * D);
    f4 rr = r * r;
    if (F >= rr) { return false; }

    /*
        F and rr make two sides of a right triangle. 
        The third side (T) being parallel to A_combined_velocity
        (if the collision exists)
    */
    f4 T = rr - F;

    /*
        If there is no such right triangle with sides length of 
        rr and sqrtf(F), T will probably be less than 0.  
    */
    if (T < 0) { return false; }

    /*
        Therefore the distance the circle has to travel along 
        A_combined_velocity is D - sqrtf(T)

        Finally, the corrected velocity vector must be shorter
        than the original velocity vector.
    */
    f4 distance = D - sqrtf(T);
    if (length_combined < distance) { return false; }

    /*
        Calculate the position of contact for each sphere
    */
    vec3 fixed_combined_velocity = N * distance;
    f4 fixed_length = length(fixed_combined_velocity);

    f4 collision_time = fixed_length / length_combined;

    A.future_pos = A.prev_pos;
    B.future_pos = B.prev_pos;

    A.future_pos = A.future_pos + (A.velocity * collision_time);
    B.future_pos = B.future_pos + (B.velocity * collision_time);
    
    A.collision_time = collision_time;
    B.collision_time = collision_time;

    A.remaining_velocity = 1.0f - collision_time;
    B.remaining_velocity = 1.0f - collision_time;

    A.collision_pos = A.future_pos;
    B.collision_pos = B.future_pos;

    A.collision_normal = normal(A.future_pos - B.future_pos);
    B.collision_normal = normal(B.future_pos - A.future_pos);

    A.PoC = A.collision_normal * -A.radius;
    B.PoC = B.collision_normal * -B.radius;

    return true;
}
void init_body (RigidBody &body, BodyInfo info)
{
    body.density = info.density;
    body.radius = info.radius;
    body.width = info.width;
    body.height = info.height;
    body.depth = info.depth;
    body.type = info.type;
    
    switch (info.type)
    {
        case TYPE_SPHERE:
            body.volume = (4.0f/3.0f) * PI * info.radius * info.radius * info.radius;
            break;

        case TYPE_CUBOID:
            body.volume = info.width * info.height * info.depth;
            break;
    }

    body.mass = body.density * body.volume;

    body.coefficient_restitution = info.restitution;

    body.one_over_mass = 1.0f / body.mass;

    const f4 gravity = 0.0f;//0.01f;
    body.gravity = -gravity / body.one_over_mass;

    body.force = setv();
    body.torque = setv();

    // https://en.wikipedia.org/wiki/List_of_moments_of_inertia

    switch (info.type)
    {
        case TYPE_SPHERE:
        {
            f4 moment_of_inertia = (2.0f/5.0f) * body.mass * info.radius * info.radius;
            body.MoI_local = identity() * moment_of_inertia;
        }
        break;

        case TYPE_CUBOID:
        {
            f4 f = 1.0f/12.0f * body.mass;
            f4 Ih = f * (body.width * body.width + body.depth * body.depth);
            f4 Iw = f * (body.depth * body.depth + body.height * body.height);
            f4 Id = f * (body.width * body.width + body.height * body.height);

            body.MoI_local = identity();
            body.MoI_local[0] = Iw;
            body.MoI_local[4] = Ih;
            body.MoI_local[8] = Id;
        }
        break;
    }

    body.inverse_MoI_local = inverse(body.MoI_local); // |I^-1 CM

    body.pos = info.pos; // r CM
    body.prev_pos = info.pos; 
    body.velocity = setv(); // v CM
    body.angular_velocity = setv();
    body.orientation = from_axis_angle(setv(1.0f,0.0f,0.0f), degtorad(0.0f)); // A
    body.angular_momentum = setv(); // L CM

    body.inverse_MoI_world = body.orientation * body.inverse_MoI_local * transpose(body.orientation); // I^-1 CM
}

/*

    Utility functions

*/

void add_planar_body (RigidBody &body, u4 total_plane)
{
    body.planes = (Plane*)alloc(memory, sizeof(plane) * total_plane);
    body.num_plane = 0;
}

void add_plane (RigidBody &body, vec3 center, vec3 p1, vec3 p2, vec3 p3, vec3 p4)
{
    u4 i = body.num_plane;

    body.planes[i].pos = center;
    body.planes[i].p[0] = p1;
    body.planes[i].p[1] = p2;
    body.planes[i].p[2] = p3;
    body.planes[i].p[3] = p4;

    vec3 dir = crossproduct((p2 - p1), (p3 - p1));
    body.planes[i].normal = dir / length(dir);

    body.num_plane++;
}

void build_planes_from_cuboid (RigidBody &body)
{
    /*
        Builds an AABB cube from planes to fit the dimensions of the RigidBody
    */

    add_planar_body(body, 6);

    f4 hw = body.width  * .5f;
    f4 hh = body.height * .5f;
    f4 hd = body.depth  * .5f;

    // X+
    add_plane(body,
        setv(   hw,  .0f,  .0f),
        setv(  .0f,  -hh,   hd ),
        setv(  .0f,  -hh,  -hd ),
        setv(  .0f,   hh,  -hd ),
        setv(  .0f,   hh,   hd ));
    // X-
    add_plane(body,
        setv(  -hw,  .0f,  .0f),
        setv(  .0f,   hh,   hd ),
        setv(  .0f,   hh,  -hd ),
        setv(  .0f,  -hh,  -hd ),
        setv(  .0f,  -hh,   hd ));
    // Y+
    add_plane(body,
        setv(  .0f,   hh,  .0f),
        setv(   hw,  .0f,   hd ),
        setv(   hw,  .0f,  -hd ),
        setv(  -hw,  .0f,  -hd ),
        setv(  -hw,  .0f,   hd ));
    // Y-
    add_plane(body,
        setv(  .0f,   hh,  .0f),
        setv(  -hw,  .0f,   hd ),
        setv(  -hw,  .0f,  -hd ),
        setv(   hw,  .0f,  -hd ),
        setv(   hw,  .0f,   hd ));
    // Z+
    add_plane(body,
        setv(  .0f,  .0f,   hd),
        setv(   hw,   hh,  .0f ),
        setv(  -hw,   hh,  .0f ),
        setv(  -hw,  -hh,  .0f ),
        setv(   hw,  -hh,  .0f ));
    // Z-
    add_plane(body,
        setv(  .0f,  .0f,  -hd),
        setv(   hw,  -hh,  .0f ),
        setv(  -hw,  -hh,  .0f ),
        setv(  -hw,   hh,  .0f ),
        setv(   hw,   hh,  .0f ));
}