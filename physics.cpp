struct RigidBody {
    vec3 velocity;
    vec3 pos = vec3(0.5f,0.0f,0.0f); /* GLOBAL */
    vec3 prev_pos = vec3(0.5f,0.0f,0.0f); /* GLOBAL */

    f4 radius = 0.1f;

    f4 mass = 10.0f;
    f4 restitution = 0.75f;
    f4 friction = 0.75f;

    /*
        Collision information
    */
    vec3 PoC; /* LOCAL */
    vec3 PoC_on_radius; /* LOCAL */
} marble, poolball, container;

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
    N = N.normal();
    /*
        Find the length of the component of each velocity vector
        along the N (the normal running through the collision point)
    */
    f4 component_length_A = A.velocity.dot(N);
    f4 component_length_B = B.velocity.dot(N);

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
        2D or 3D

        Detect collision only of:
            1) two stationary circles

        Source: https://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php
    */
    f8 x = A.pos.x - B.pos.x;
    x *= x;
    f8 z = A.pos.z - B.pos.z;
    z *= z;

    f8 r = A.radius + B.radius; 
    r *= r;

    return (x + z <= r);
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
    if (A_combined_velocity.length() < dist) return false;

    /*
        @todo: I don't understand why the normal rather than velocity is used.
    */
    vec3 N = A_combined_velocity.normal();

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
    f4 D = N.dot(C);
    if (D <= 0) { return false; }

    /* 
        F = |C| - dot product 
        F = Distance to C - Distance on C that A will travel
        Square these values so not to use squareroot
        Cannot collide if the difference is greater than their radii
    */
    f4 length_c = C.length();
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
    f4 mag = A_combined_velocity.length();
    if (mag < distance) { return false; }

    /*
        If the two bodies should not reflect, then
        calculate the collision time and apply changes.

        if (A.velocity.has_length() && B.velocity.has_length())
        {
            // 2 moving circles
            f4 collision_time = A_combined_velocity.length() / A.velocity.length();
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
        f4 mv = bodies[i].mass * bodies[i].velocity.length();
        joules += 0.5f * (mv * mv);
    }
    return joules;
}

b4 calculate_PoC_circle_in_circle_minkowski_difference (RigidBody &B, RigidBody &A)
{
    if (B.velocity.has_length() == false) return false;
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

    if (AC.length() < R - r) {
        return false;
    }

    /*
        @todo: Understand this series of equations better.
    */

    vec3 AB = B.pos - A.pos;
    vec3 BC = C_pos - B.pos; 

    f4 BC2 = BC.length();
    BC2 *= BC2;

    f4 AB2 = AB.length();
    AB2 *= AB2;

    f4 Rr2 = (R - r) * (R - r);

    f4 b = ( AB.dot(BC) / BC2 ) * -1.0f;
    f4 c = (AB2 - Rr2) / BC2;
    f4 d = b * b - c;
    f4 k = b - sqrt(d);
    if (k < 0)
        k = b + sqrt(d);
    if (k < 0)
    {
        // cout << "No solution: " << endl;
    }
    else
    {
        B.PoC = (B.velocity * k);
        B.PoC_on_radius = B.PoC + (AB.normal() * B.radius);
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
    vec3 N = vec3(container_position - (A.pos + A.PoC)).normal();

    /*  
        Split velocity into two components:
        U : perpendicular to the surface
        W : parallel with surface
    */
    vec3 U = N * V.dot(N);
    vec3 W = V - U;

    /*
        Calcuate the reflected vector with friction and restitution
    */
    V = (W * A.friction) - (U * A.restitution);

    /*
        @todo: Clean this up, get rid of all the sqrt()
        Calculate energy lost and new velocity.
    */
    f4 traveled = A.PoC.length();
    f4 needs_to_travel = A.velocity.length();
    f4 remaining = needs_to_travel - traveled;

    f4 energy_remaining = V.length() / needs_to_travel;

    remaining *= energy_remaining;

    A.pos = A.pos + A.PoC + (V.normal() * remaining);

    A.velocity = V.normal() * (A.velocity.length() * energy_remaining);
}