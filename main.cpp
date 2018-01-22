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
	vec3 camera_pos = vec3(0.0f,0.0f,0.0f);

    struct RigidBody {
        vec3 velocity;
        vec3 pos = vec3(0.5f,0.0f,0.0f);
        vec3 prev_pos = vec3(0.5f,0.0f,0.0f);

        f4 radius = 0.1f;

        f4 momentum; /* how to use */
        f4 mass = 10.0f;
    } body;
	
    u4 step = 0;

	while(!input.quit_app)
	{
		u4 time_physics_curr = SDL_GetTicks();
		f4 frame_time = ((f4)(time_physics_curr - time_physics_prev)) / 1000.0f;
		time_physics_prev = time_physics_curr;

        if (frame_time >= 0.064f) {
            frame_time = 0.064f;
        }

        // frame_time *= 0.5f;

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
            const f4 ground_friction = 0.985f;

            struct Force {
                vec3 dir;
                f4 magnitude = 1.0f * 0.075f;
            } force;

            // get user input
            if (single_press(input.d)) { force.dir.x =  1.0f; }
            if (single_press(input.a)) { force.dir.x = -1.0f; }
            if (single_press(input.w)) { force.dir.z =  1.0f; }
            if (single_press(input.s)) { force.dir.z = -1.0f; }

            force.dir = force.dir.normal();

            body.prev_pos = body.pos;

            // calculate velocity
            body.velocity = body.velocity + (force.dir * force.magnitude);

            // apply friction
            body.velocity = body.velocity * ground_friction;

            // next pos
            vec3 future_pos = body.pos + body.velocity;

            // contained circle vs outer circle collision
            f4 dist = sqrt((future_pos.x * future_pos.x) + (future_pos.z * future_pos.z));

            // @todo: convert this to general case
            // reflect_circle_off_line(Velocity, PoC, LineNormal)
            // https://stackoverflow.com/questions/573084/how-to-calculate-bounce-angle

            if (dist >= 1.0f - body.radius)
            {
                // Point of Contact: shorten future position by the amount it exceeded the outer circle
                vec3 PoC = future_pos;
                PoC.x *= ((1.0f - body.radius) / dist);
                PoC.z *= ((1.0f - body.radius) / dist);

                // ( pointing inward to (0,0) )
                vec3 N = vec3(-PoC.x, PoC.y, -PoC.z).normal();

                f4 SP = body.velocity.dot(N);

                vec3 U = N * SP;
                vec3 W = body.velocity - U;

                // get new delta from point of contact
                body.velocity = W - U;
                future_pos = body.pos + body.velocity;
            }

            // apply changes
            body.pos = future_pos;
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
			state.texture = field_texture;
			state.scale = vec3(1.0f,1.0f,1.0f);
			state.rotation.x = 90.0f * M_DEGTORAD32;
			render_plane(state);

            state.world = body.prev_pos.lerpto(body.pos, alpha);
            state.scale = vec3(body.radius,body.radius,1);
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