
inline f4 distance_between (vec3 a, vec3 b) {
    return sqrt(((b.x - a.x) * (b.x - a.x)) + ((b.y - a.y) * (b.y - a.y)) + ((b.z - a.z) * (b.z - a.z)));
}

inline vec3 reflect_circle_line ( vec3 circle_velocity, vec3 surface_normal )
{
    /* Assumes unit circle */
    
    // https://stackoverflow.com/questions/573084/how-to-calculate-bounce-angle
    f4 scalar_product = dot(circle_velocity, surface_normal);

    vec3 U = surface_normal * scalar_product;
    vec3 W = circle_velocity - U;

    return W - U;
}

void reflect_circle_circle (RigidBody &A, RigidBody &B)
{
    /*
        2D or 3D

        Collision resolution between:
            1) Two circles, regardless of motion.

        Source: https://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php
    */
    /*
        Normalized vector N from pos A to pos B
    */
    vec3 N = B.pos - A.pos;
    N = normal(N);
    /*
        Find the length of the component of each velocity vector
        along the N (the normal running through the collision point)
    */
    f4 component_length_A = dot(A.velocity, N);
    f4 component_length_B = dot(B.velocity, N);

    /*
        Using the optimized version, 
        optimizedP =  2(a1 - a2)
                     ------------
                     mass1 + mass2
        P = magnitude of the change in the momentums of the circles before and after
    */
    f4 P = (2.0f * (component_length_A - component_length_B)) / (A.mass + B.mass);

    /*
        Calculate new velocites 
    */    
    vec3 v1_after = A.velocity - (N * (P * B.mass));
    vec3 v2_after = B.velocity + (N * (P * A.mass));

    /*
        Apply changes.
    */
    A.velocity = v1_after;
    B.velocity = v2_after;
}

b4 detect_collision_circle_circle_stationary (RigidBody A, RigidBody B)
{
    /*
        2D

        Detect collision only of:
            1) two stationary circles

        Source: https://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php
    */
    f8 x = A.pos.x - B.pos.x;
    x *= x;
    f8 y = A.pos.y - B.pos.y;
    y *= y;

    f8 r = A.radius + B.radius; 
    r *= r;

    return (x + y <= r);
}

b4 detect_and_apply_collision_circle_circle (RigidBody &A, RigidBody &B)
{
    /*
        2D or 3D

        Detect collision and correct velocities of:
            1) one moving circle against one stationary circle
            2) two moving circles

        Source: https://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php
        
        See references/circle-circle-collision.jpg
            references/circle-circle-collision-B.jpg
    */

    /*
        subtract velocity B from A, so that B is treated as stationary
    */
    vec3 A_combined_velocity = A.velocity - B.velocity;

    /*
        is velocity less than distance between A and B
    */
    f4 dist = distance_between(B.pos, A.pos);
    f4 r = (B.radius + A.radius);
    dist -= r;
    if (length(A_combined_velocity) < dist) return false;

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
        rr and sqrt(F), T will probably be less than 0.  
    */
    if (T < 0) { return false; }

    /*
        Therefore the distance the circle has to travel along 
        A_combined_velocity is D - sqrt(T)
    */
    f4 distance = D - sqrt(T);

    /*
        Get the length of the original velocity vector.
        Finally, the corrected velocity vector must be shorter
        than the original velocity vector.
    */
    f4 mag = length(A_combined_velocity);
    if (mag < distance) { return false; }

    /*
        If the two bodies should not reflect, then
        calculate the collision time and apply changes.

        if (has_length(A.velocity) && has_length(B.velocity))
        {
            // 2 moving circles
            f4 collision_time = length(A_combined_velocity) / length(A.velocity);
            A.velocity = A.velocity * collision_time;
            B.velocity = B.velocity * collision_time;
        }
        else {
            // 1 moving circle 1 stationary circle
            A.velocity = A_combined_velocity.normal() * distance;
        }
        
        Else, reflect the two bodies.
    */
    reflect_circle_circle(A,B);

    return true;
}

f4 calculate_kinetic_energy (RigidBody* bodies, u4 count)
{
    /*
        Calculate kinetic energy in system.
        Don't forget to remove friction when checking for conservation.
    */
    f4 joules = 0.0f;
    for (u4 i = 0; i < count; i++)
    {
        // KE = 0.5f * (M * V)^2
        f4 mv = bodies[i].mass * length(bodies[i].velocity);
        joules += 0.5f * (mv * mv);
    }
    return joules;
}

b4 calculate_PoC_circle_in_circle_minkowski_difference (RigidBody &B, RigidBody &A)
{
    if (!has_length(B.velocity)) return false;
    /*
        B is smaller inner circle
        A is larger containing circle

        C is B's next position

        This calculates B's point of contact on A along it's velocity vector,
        but does not alter velocity.

        Sources:
            Casey Muratori, "Implementing GJK - 2006", 7 min 55 sec
            https://www.youtube.com/watch?v=Qupqu1xe7Io
            
            Sam Hocevar
            https://gamedev.stackexchange.com/questions/29650/circle-inside-circle-collision

        Additional reading:
            https://en.wikipedia.org/wiki/Minkowski_addition#Convex_hulls_of_Minkowski_sums
            https://wildbunny.co.uk/blog/2011/04/20/collision-detection-for-dummies/
    */
    f4 R = A.radius;
    f4 r = B.radius;

    vec3 C_pos = B.pos + B.velocity;
    
    vec3 AC = C_pos - A.pos;

    if (length(AC) < R - r) {
        return false;
    }

    /*
        @todo: Understand this series of equations better.
    */

    vec3 AB = B.pos - A.pos;
    vec3 BC = C_pos - B.pos; 

    f4 BC2 = length(BC);
    BC2 *= BC2;

    f4 AB2 = length(AB);
    AB2 *= AB2;

    f4 Rr2 = (R - r) * (R - r);

    f4 b = ( dot(AB,BC) / BC2 ) * -1.0f;
    f4 c = (AB2 - Rr2) / BC2;
    f4 d = b * b - c;
    f4 k = b - sqrtf(d);
    if (k < 0)
        k = b + sqrtf(d);
    if (k < 0)
    {
        // cout << "No solution: " << endl;
    }
    else
    {
        B.PoC = (B.velocity * k);
        B.PoC_on_radius = B.PoC + (normal(AB) * B.radius);
    }

    return true;
}

void reflect_circle_within_cirle (RigidBody &A, vec3 container_position)
{
    /*
        2D or 3D
        Assumes intersection has already been found.
        Source: Gareth Rees,
            https://stackoverflow.com/questions/573084/how-to-calculate-bounce-angle

        @todo: How does the container friction and restitution into this

        @todo: Change this to reflect vector off line.
            reflect_vector_off_line (&velocity, line.pos, line.normal, friction, restitution)
    */
    vec3 V = A.velocity;

    /*
        N : Normal of surface with length of 1
            at the position of contact
    */
    vec3 N = normal(container_position - (A.pos + A.PoC));

    /*  
        Split velocity into two components:
        U : perpendicular to the surface
        W : parallel with surface
    */
    vec3 U = N * dot(V,N);
    vec3 W = V - U;

    /*
        Calcuate the reflected vector with friction and restitution
    */
    V = (W * A.friction) - (U * A.restitution);

    /*
        @todo: Clean this up, get rid of all the sqrt()
        Calculate energy lost and new velocity.
    */
    f4 traveled = length(A.PoC);
    f4 needs_to_travel = length(A.velocity);
    f4 remaining = needs_to_travel - traveled;

    f4 energy_remaining = length(V) / needs_to_travel;

    remaining *= energy_remaining;

    A.pos = A.pos + A.PoC + (normal(V) * remaining);

    A.velocity = normal(V) * (length(A.velocity) * energy_remaining);
}