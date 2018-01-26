/*
	-Blake Trahan
	-2018
*/
#include "global_vars.cpp"
#include "sgl_functions.cpp"
#include "render_functions.cpp"
#include "input.cpp"

struct RigidBody {
    vec3 velocity;
    vec3 pos = vec3(0.5f,0.0f,0.0f);
    vec3 prev_pos = vec3(0.5f,0.0f,0.0f);

    f4 radius = 0.1f;

    f4 mass = 10.0f;

    /*
        Collision information
    */
    vec3 PoC;
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
        // cout << "K: " << k << endl;;
        // print(B.pos + (BC * k)); 
        B.PoC = (B.velocity * k) + (B.velocity.normal() * B.radius); 
    }

    return true;
}

int main(int argc, char* argv[]) {
	initialize_memory(memory, 8);
	
	if (!create_sgl()) 
    { 
        cout << "ERROR: failed to create sdl or opengl" << endl;
    }
	
	create_plane();

	if (!create_basic_texture_shader()) return 0;

	field_texture = 0;
	field_image.data = stbi_load("media/circle.png", &field_image.x, &field_image.y, &field_image.n, 4);
	field_texture = my_create_texture(256,256,true,field_image.data,false,field_image.n);

    marble_texture = 0;
    marble_image.data = stbi_load("media/marble.png", &marble_image.x, &marble_image.y, &marble_image.n, 4);
    marble_texture = my_create_texture(256,256,true,marble_image.data,false,marble_image.n);

	// loop
	const f4 RENDER_MS = 1.0f/60.0f;
	const f4 PHYSICS_MS = 1.0f/60.0f;
	f4 render_dt = 0.0f;
	f4 physics_dt = 0.0f;
	u4 time_physics_prev = SDL_GetTicks();
	f4 camera_angle = M_PI32/-2.0f;
	f4 cam_radius = 8.0f;
	vec3 camera_pos = vec3(0.0f,0.0f,0.0f);

    container.radius = 1.0f;
    container.pos.x = 0.0f;
    container.prev_pos.x = 0.0f;

    poolball.pos.x = -0.5f;
    poolball.prev_pos.x = -0.5f;
    poolball.pos.z = -0.05f;
    poolball.prev_pos.z = -0.05f;
    poolball.velocity.x = 0.025f;
	
    u4 step = 0;

	while(!input.quit_app)
	{
		u4 time_physics_curr = SDL_GetTicks();
		f4 frame_time = ((f4)(time_physics_curr - time_physics_prev)) / 1000.0f;
		time_physics_prev = time_physics_curr;

        if (frame_time >= 0.064f) {
            frame_time = 0.064f;
        }

        // frame_time *= 0.15f;

		poll_events();

		physics_dt += frame_time;
		while (physics_dt >= PHYSICS_MS)
		{
			physics_dt -= PHYSICS_MS;

			f4 W = 20.0f;
			camera_pos.x = ((camera_pos.x * (W - 1)) + 0) / W;
			camera_pos.y = ((camera_pos.y * (W - 1)) + 0) / W;
			camera_pos.z = ((camera_pos.z * (W - 1)) + 0) / W;
            
            // apply friction
            const f4 ground_friction = 1.0f; //0.985f;

            struct Force {
                vec3 dir;
                f4 length = 1.0f * 0.075f;
            } marble_force, poolball_force;

            // get user input
            if (single_press(input.d)) { marble_force.dir.x =  1.0f; }
            if (single_press(input.a)) { marble_force.dir.x = -1.0f; }
            if (single_press(input.w)) { marble_force.dir.z =  1.0f; }
            if (single_press(input.s)) { marble_force.dir.z = -1.0f; }

            marble_force.dir = marble_force.dir.normal();
            poolball_force.length = 0.0f;

            auto step = [ground_friction] (RigidBody &body, Force force) {
                body.prev_pos = body.pos;

                // calculate velocity
                body.velocity = body.velocity + (force.dir * force.length);

                // apply friction
                body.velocity = body.velocity * ground_friction;
            };

            step(marble, marble_force);
            step(poolball, poolball_force);

            // b4 intersected = detect_collision_circle_circle_minkowski(marble, poolball);

            // b4 intersected = detect_and_apply_collision_circle_circle(marble, poolball);

            b4 intersected = calculate_PoC_circle_in_circle_minkowski_difference (marble, container);
            b4 intersected2 = calculate_PoC_circle_in_circle_minkowski_difference (poolball, container);

            /*
                REFLECT POINT OFF LINE (aka inner circle off outer circle tangent) WALL
            */
            if (intersected) {
                /*
                    https://stackoverflow.com/questions/573084/how-to-calculate-bounce-angle
                */
            }

            // apply changes
            marble.pos = marble.pos + marble.velocity; 
            poolball.pos = poolball.pos + poolball.velocity;
            if (intersected2) {
                poolball.velocity = vec3();
            }
		}

        f4 alpha = physics_dt / PHYSICS_MS;

		render_dt += frame_time;
		// if (render_dt >= RENDER_MS)
		{
			render_dt = 0;
		
			glm::vec3 up_axis(0, 0, 1); 

			f4 posX = cam_radius * cos(camera_angle); 
    		f4 posY = cam_radius * sin(camera_angle); 

    		vec3 pp = camera_pos; // center of world

			glm::mat4 view = glm::lookAt(
				glm::vec3(pp.x,pp.y-3,pp.z), // pos
				glm::vec3(pp.x,pp.y,pp.z), // lookat
				glm::vec3(0.0f, 0.0f, 1.0f)); // up
			glm::mat4 projection = glm::perspective(45.0f, 1.0f*sgl.width/sgl.height, 0.1f, 100.0f);

		   	glBindFramebuffer(GL_FRAMEBUFFER,0);
			glClearColor(0.23f,0.47f,0.58f,1.0f); 
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			RENDER_STATE state;
			state.view = view;
			state.projection = projection;
			state.world = pp;
			state.scale = vec3(1.0f,1.0f,1.0f);
			state.rotation.x = 90.0f * M_DEGTORAD32;

            state.world = container.prev_pos.lerp(container.pos, alpha);
            state.scale = vec3(container.radius,container.radius,1.0f);
            state.texture = field_texture;
			render_plane(state);

            state.world = poolball.pos;//poolball.prev_pos.lerp(poolball.pos, alpha);
            state.scale = vec3(poolball.radius,poolball.radius,1.0f);
            state.texture = marble_texture;
            render_plane(state);

            state.world = marble.prev_pos.lerp(marble.pos, alpha);
            state.scale = vec3(marble.radius,marble.radius,1.0f);
            state.texture = marble_texture;
            render_plane(state);

            // state.world = vec3(0.99861, 0, -0.05);
            // // state.world = vec3(0.89861, 0, -0.05);
            // state.scale = vec3(0.01f,0.01f,1.0f);
            // state.texture = marble_texture;
            // render_plane(state);

			SDL_GL_SwapWindow(sgl.window);
		}
		memory.transient_current = 0;
	}
	
	stbi_image_free(field_image.data);
	glDeleteTextures(1,&field_texture);
    stbi_image_free(marble_image.data);
    glDeleteTextures(1,&marble_texture);
	
	glDeleteProgram(basic_texture.program);
	glDeleteBuffers(1, &plane.verts);
	glDeleteBuffers(1, &plane.colors);
	glDeleteBuffers(1, &plane.indices);
	glDeleteBuffers(1, &plane.uv_coords);
	SDL_DestroyWindow( sgl.window );
	SDL_Quit();

	free(memory.TransientStorage);
	free(memory.PermanentStorage);

	return 0;
}