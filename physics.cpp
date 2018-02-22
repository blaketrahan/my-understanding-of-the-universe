void apply_impulses_2D (RigidBody &A, RigidBody &B)
{

    /*
        source:
            http://chrishecker.com/images/e/e7/Gdmphys3.pdf

        @todo: understand this better
    */
    auto Perp = [] (vec3 v) {
        vec3 r;
        r.x = 0.0f;
        r.y = -v.z;
        r.z = v.y;
        return r;
    };
    vec3 relative_velocity = A.velocity - B.velocity;

    vec3 N = A.collision_normal;

    f4 impulse_numerator = -(1.0f + A.coefficient_restitution) * dot(relative_velocity, A.collision_normal);
    f4 impulse_denominator = dot(A.collision_normal,A.collision_normal) * (A.one_over_mass + B.one_over_mass);
    
    f4 A_perpdot = dot(Perp(A.PoC), relative_velocity);
    
    impulse_denominator += A_perpdot * A_perpdot * A.one_over_CM_moment_of_inertia;
    impulse_denominator += A_perpdot * A_perpdot * B.one_over_CM_moment_of_inertia;
    
    f4 J = impulse_numerator / impulse_denominator;

    auto do_impulse = [N, Perp] (f4 J, RigidBody &body)
    {
        body.velocity = body.velocity + (N * body.one_over_mass * J);

        body.AngularVelocity = body.AngularVelocity + dot(Perp(body.PoC), N * J) * body.one_over_CM_moment_of_inertia;
    };

    do_impulse( J, A);
    do_impulse(-J, B);
}

void apply_impulses_3D (RigidBody &A, RigidBody &B)
{

    /*
        source:
            http://chrishecker.com/images/b/bb/Gdmphys4.pdf
            http://www.euclideanspace.com/maths/algebra/matrix/functions/skew/index.htm

        @todo: understand this better
    */
    auto Perp = [] (vec3 v) {
        vec3 r;
        r.x = 0.0f;
        r.y = -v.z;
        r.z = v.y;
        return r;
    };
    vec3 relative_velocity = A.velocity - B.velocity;

    vec3 N = A.collision_normal;

    f4 impulse_numerator = -(1.0f + A.coefficient_restitution) * dot(relative_velocity, A.collision_normal);
    f4 impulse_denominator = dot(A.collision_normal,A.collision_normal) * (A.one_over_mass + B.one_over_mass);
    
    f4 A_perpdot = dot(Perp(A.PoC), relative_velocity);
    
    impulse_denominator += A_perpdot * A_perpdot * A.one_over_CM_moment_of_inertia;
    impulse_denominator += A_perpdot * A_perpdot * B.one_over_CM_moment_of_inertia;
    
    f4 J = impulse_numerator / impulse_denominator;

    auto do_impulse = [N, Perp] (f4 J, RigidBody &body)
    {
        body.velocity = body.velocity + (N * body.one_over_mass * J);

        body.AngularVelocity = body.AngularVelocity + dot(Perp(body.PoC), N * J) * body.one_over_CM_moment_of_inertia;
    };

    do_impulse( J, A);
    do_impulse(-J, B);
}

b4 collision_circle_circle (RigidBody &A, RigidBody &B)
{
    /*
        2D or 3D

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

    A.pos = A.pos + (A.velocity * collision_time);
    B.pos = B.pos + (B.velocity * collision_time);
    
    A.collision_time = collision_time;
    B.collision_time = collision_time;

    A.remaining_velocity = 1.0f - collision_time;
    B.remaining_velocity = 1.0f - collision_time;

    A.collision_pos = A.pos;
    B.collision_pos = B.pos;

    A.collision_normal = normal(A.pos - B.pos);
    B.collision_normal = normal(B.pos - A.pos);

    A.PoC = A.collision_normal * -A.radius;
    B.PoC = B.collision_normal * -B.radius;

    return true;
}

void reflect_circle_circle (RigidBody &A, RigidBody &B)
{
    /*
        @todo: currently assumes constant velocity over the time of the step.
               in reality, each body has lost some of its velocity mid-step,
               unless the collision time is at exactly 0.
    */
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
        @todo: Don't understand this
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
    A.velocity = v1_after * (1.0f - A.collision_time);
    B.velocity = v2_after * (1.0f - B.collision_time);
}