/*
	-Blake Trahan
	-2018
*/
#include "global_vars.cpp"
#include "sgl_functions.cpp"
#include "render_functions.cpp"
#include "input.cpp"

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
	vec3 camera_pos = vec3(0,0,0);

    struct Ball {
        vec3 pos;
        vec3 prev_pos;
        vec3 dir;
        f4 radius;
        f4 vel;
        b4 is_reflecting;
        f4 time_of_contact;
        vec3 point_of_contact;
    } ball;

    ball.radius = 0.1f;
    ball.pos = vec3(0.0f,0.0f,0.0f);
    ball.prev_pos = ball.pos;
    ball.dir = vec3(0.0f,0.0f,0.0f);
    ball.vel = 0.0f;
    ball.point_of_contact = vec3(0.0f,0.0f,0.0f);
    ball.is_reflecting = false;
    ball.time_of_contact = 0.0f;
            
    auto get_magnitude = [] (vec3 v) {
        return sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
    };

    auto get_normal = [&] (vec3 v) {
        f4 magnitude = get_magnitude(v);
        if (magnitude == 0.0f) 
            return vec3(0.0f,0.0f,0.0f);
        return vec3(v.x / magnitude, v.y / magnitude, v.z / magnitude);
    };

    auto get_normal2 = [] (vec3 v, f4 magnitude) {
        if (magnitude == 0.0f) 
            return vec3(0.0f,0.0f,0.0f);
        return vec3(v.x / magnitude, v.y / magnitude, v.z / magnitude);
    };

    auto scalar_product = [] (vec3 X, vec3 Y) {
        return (X.x * Y.x) + (X.y * Y.y) + (X.z * Y.z);
    };

    auto apply_coefficient = [] (f4 C, vec3 V) {
        return vec3(V.x * C, V.y * C, V.z * C);
    };
	
    u4 step = 0;

	while(!input.quit_app)
	{
		u4 time_physics_curr = SDL_GetTicks();
		f4 frame_time = ((f4)(time_physics_curr - time_physics_prev)) / 1000.0f;
		time_physics_prev = time_physics_curr;

        if (frame_time >= 0.064f) {
            frame_time = 0.064f;
        }

        // frame_time *= 0.25f;

		poll_events();

		physics_dt += frame_time;
		while (physics_dt >= PHYSICS_MS)
		{
			physics_dt -= PHYSICS_MS;

			f4 W = 20.0f;
			camera_pos.x = ((camera_pos.x * (W - 1)) + 0) / W;
			camera_pos.y = ((camera_pos.y * (W - 1)) + 0) / W;
			camera_pos.z = ((camera_pos.z * (W - 1)) + 0) / W;

            // existing forces
            ball.prev_pos = ball.pos;

            f4   inertial_velocity = ball.vel;
            vec3 inertial_direction = ball.dir;
            vec3 next_pos = ball.pos;

            // user forces
            f4 applied_velocity = 0.0f;
            vec3 applied_direction = vec3(0.0f,0.0f,0.0f);

            // get user input
            if (single_press(input.d)) { applied_direction.x =  1.0f; }
            if (single_press(input.a)) { applied_direction.x = -1.0f; }
            if (single_press(input.w)) { applied_direction.z =  1.0f; }
            if (single_press(input.s)) { applied_direction.z = -1.0f; }

            // get user velocity
            const f4 PUSH_STRENGTH = 1.0f * 0.075f; // 1.0 == radius of cylinder in one timestep
            if (applied_direction.x != 0.0f || applied_direction.z != 0.0f)
            {
                applied_velocity = PUSH_STRENGTH;
            }

            // add user force to existing force
            inertial_direction = apply_coefficient(inertial_velocity, inertial_direction);
            applied_direction = apply_coefficient(applied_velocity, applied_direction);
            inertial_direction += applied_direction;

            // get new velocity and direction
            inertial_velocity = get_magnitude(inertial_direction);
            inertial_direction = get_normal(inertial_direction);

            if (inertial_velocity > PUSH_STRENGTH) {
                inertial_velocity = PUSH_STRENGTH;
            }

            // apply friction
            const f4 ground_friction = 0.985f;
            inertial_velocity *= ground_friction;

            // calculate new position
            vec3 delta = apply_coefficient(inertial_velocity, inertial_direction);
            next_pos += delta;

            // collision check with wall
            f4 distance_from_center = sqrt((next_pos.x * next_pos.x) + (next_pos.z * next_pos.z));
            if (distance_from_center >= 1.0f - ball.radius) // hit wall
            {
                // 1.0/d returns the percent of the old to new pos needed to move
                // aka point of collision
                vec3 point_of_contact = vec3(next_pos.x, 0.0f, next_pos.z); // dont use Y
                point_of_contact.x *= ((1.0f - ball.radius) / distance_from_center);
                point_of_contact.z *= ((1.0f - ball.radius) / distance_from_center);

                // reflect off tangent
                // u = (v · n / n · n) n
                // w = v − u
                vec3 V = delta;
                // surface normal of tangent
                // because unit circle, it is also the normal
                vec3 N = get_normal(vec3(-point_of_contact.x, point_of_contact.y, -point_of_contact.z));

                f4 SP = scalar_product(delta,N);

                vec3 U = apply_coefficient(SP,N);
                vec3 W = delta - U;

                // const f4 F = 1.0f; // wall friction, 1 == frictionless
                // const f4 R = 1.0f;  // wall elasticity, 1 == perfect elastic reaction
                
                // get new delta from point of contact
                // delta = (apply_coefficient(R,W)) - (apply_coefficient(F,U));
                delta = W - U;

                /*  
                    Get the velocity that would've gone through the wall.
                    Then get ratio of it to the reflected velocity to 
                    calculate the velocity in the next time step
                */
                f4 reflected_velocity = get_magnitude(delta);
                
                inertial_velocity *= 0.95; /* Fake friction and restitution */
                inertial_direction = get_normal2(delta, reflected_velocity);
                
                // ball.point_of_contact = point_of_contact;
                // ball.time_of_contact = get_magnitude(point);
                // ball.is_reflecting = true;

                // get new ball pos
                next_pos = point_of_contact + delta;
                
                // angular momentum
                // deformation
                // https://gafferongames.com/post/physics_in_3d/
            }
            else
            {
                ball.is_reflecting = false;
            }

            // apply changes
            ball.pos = next_pos;
            ball.dir = inertial_direction;
            ball.vel = inertial_velocity;
            step++;
		}

        f4 alpha = physics_dt / PHYSICS_MS;

        // integrate old physics state with new physics state
        auto integrate_vector = [&] (vec3 A, vec3 B) {
            return apply_coefficient(alpha, B) + apply_coefficient(1.0f - alpha, A);
        };

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
			state.texture = field_texture;
			state.scale = vec3(1.0f,1.0f,1.0f);
			state.rotation.x = 90.0f * M_DEGTORAD32;
			render_plane(state);

            state.world = integrate_vector(ball.prev_pos, ball.pos);
            state.scale = vec3(ball.radius,ball.radius,1.0f);
            state.texture = marble_texture;
            render_plane(state);

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