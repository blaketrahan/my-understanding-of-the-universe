/*
	-Blake Trahan
	-2018
*/
#include "global_vars.cpp"
#include "sgl.cpp"
#include "shaders.cpp"
#include "render_functions.cpp"
#include "input.cpp"
#include "physics.cpp"

int main(int argc, char* argv[])
{
	initialize_memory(memory, 8, 2);
	
	if (!create_sdl_opengl_window()) 
    { 
        cout << "ERROR: failed to create sdl or opengl" << endl;
    }

    create_plane();

    if (!create_basic_texture_shader()) return 0;

    library.texture_count = 20;
    library.textures = (Library::Texture*)alloc(memory, sizeof(Library::Texture) * library.texture_count);
    library.mesh_count = 20;
    library.meshes = (Library::Mesh*)alloc(memory, sizeof(Library::Mesh) * library.mesh_count);

    load_texture("media/circle.png", 256, 256, 4);
    load_texture("media/rabbit.tga", 1024, 1024, 3);
    load_texture("media/tamanegi.png", 1024, 1024, 4);
    load_texture("media/garlic.png", 1024, 1024, 3);

    load_mesh("media/tamanegi.obj");
    load_mesh("media/rabbit.obj");

    /*
        Entities
    */
    Entity Tamanegi;
    assign_mesh(Tamanegi, "media/tamanegi.obj");
    assign_texture(Tamanegi, "media/tamanegi.png");
    Tamanegi.body.radius = 0.1f;
    Tamanegi.body.pos = vec3(0.0f,0.0f,-0.5f);
    Tamanegi.body.prev_pos = Tamanegi.body.pos;

    Entity Garlic;
    assign_mesh(Garlic, "media/tamanegi.obj");
    assign_texture(Garlic, "media/garlic.png");
    Garlic.body.radius = 0.1f;
    Garlic.body.pos = vec3(0.5f,0.0f,-0.5f);
    Garlic.body.prev_pos = Garlic.body.pos;

    Entity Rabbit;
    assign_mesh(Rabbit, "media/rabbit.obj");
    assign_texture(Rabbit, "media/rabbit.tga");

	// loop
	const f4 RENDER_MS = 1.0f/60.0f;
	const f4 PHYSICS_MS = 1.0f/60.0f;
	f4 render_dt = 0.0f;
	f4 physics_dt = 0.0f;
	u4 time_physics_prev = SDL_GetTicks();
	f4 camera_angle = 0.0f;
	f4 cam_radius = 3.0f;
	vec3 camera_pos = vec3();

    RigidBody container;
    container.radius = 1.0f;
    container.pos = vec3(0.0f,0.0f,-1.0f);
    container.prev_pos = container.pos;
    GLuint container_texture = get_texture("media/circle.png");

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

            if (input.right) { camera_angle -= M_PI32 * 0.01f; }
            if (input.left)  { camera_angle += M_PI32 * 0.01f; }

            if (input.up)    { cam_radius -= M_PI32 * 0.01f; }
            if (input.down)  { cam_radius += M_PI32 * 0.01f; }

			f4 W = 20.0f;
			camera_pos.x = ((camera_pos.x * (W - 1)) + 0) / W;
			camera_pos.y = ((camera_pos.y * (W - 1)) + 0) / W;
			camera_pos.z = ((camera_pos.z * (W - 1)) + 0) / W;
            
            // apply friction
            const f4 ground_friction = 0.985f;

            struct Force {
                vec3 dir;
                f4 length = 1.0f * 0.075f;
            } marble_force, poolball_force;

            // get user input
            if (single_press(input.d)) { marble_force.dir.y = -1.0f; }
            if (single_press(input.a)) { marble_force.dir.y =  1.0f; }
            if (single_press(input.w)) { marble_force.dir.x =  1.0f; }
            if (single_press(input.s)) { marble_force.dir.x = -1.0f; }

            marble_force.dir = marble_force.dir.normal();
            poolball_force.length = 0.0f;

            auto step = [ground_friction] (RigidBody &body, Force force) {
                // store last position
                body.prev_pos = body.pos;

                // calculate velocity
                body.velocity = body.velocity + (force.dir * force.length);

                // apply friction
                body.velocity = body.velocity * ground_friction;
            };

            // move everything
            step(Tamanegi.body, marble_force);
            step(Garlic.body, poolball_force);

            // Tamanegi.body + Garlic.body
            detect_and_apply_collision_circle_circle(Tamanegi.body, Garlic.body);

            // Tamanegi.body + container
            if (calculate_PoC_circle_in_circle_minkowski_difference (Tamanegi.body, container))
            {
                reflect_circle_within_cirle(Tamanegi.body, container.pos);
            }
            else
            {
                Tamanegi.body.pos = Tamanegi.body.pos + Tamanegi.body.velocity;
            }

            // Garlic.body + container
            if (calculate_PoC_circle_in_circle_minkowski_difference (Garlic.body, container))
            {
                reflect_circle_within_cirle(Garlic.body, container.pos);
            }
            else
            {
                Garlic.body.pos = Garlic.body.pos + Garlic.body.velocity;
            }
		}

        f4 alpha = physics_dt / PHYSICS_MS;

		render_dt += frame_time;
		// if (render_dt >= RENDER_MS)
		{
			render_dt = 0;

			f4 posX = cam_radius * cos(camera_angle);
    		f4 posY = cam_radius * sin(camera_angle);

            // X+ left, Y+ left, Z+ up
            vec3 pp = camera_pos + vec3(-posX, -posY, cam_radius);
            vec3 lookat = camera_pos;

			glm::mat4 view = glm::lookAt(
				glm::vec3(pp.x,pp.y,pp.z),
				glm::vec3(lookat.x, lookat.y, lookat.z),
				glm::vec3(0.0f, 0.0f, 1.0f));

			glm::mat4 projection = glm::perspective(45.0f, 1.0f*sgl.width/sgl.height, 0.1f, 100.0f);

		   	glBindFramebuffer(GL_FRAMEBUFFER,0);
			glClearColor(0.23f,0.47f,0.58f,1.0f); 
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			RENDER_STATE state;
			state.view = view;
			state.projection = projection;
			state.world = pp;
			state.scale = vec3(1.0f,1.0f,1.0f);

            // container
            state.world = container.pos;
            state.scale = vec3().set(container.radius);
            state.texture = container_texture;
            state.rotation = vec3();
			render_plane(state);

            // rabbit
            state.world = container.pos + vec3(0.0f,0.0f,container.radius);
            state.scale = vec3().set(container.radius);
            state.texture = Rabbit.texture;
            state.rotation.x = 0.0f;
            state.rotation.y = 90.0f * M_DEGTORAD32;
            state.rotation.z = 90.0f * M_DEGTORAD32;
            render_mesh(state, library.meshes[Rabbit.mesh]);

            // tamanegi
            state.world = Tamanegi.body.prev_pos.lerp(Tamanegi.body.pos, alpha);
            state.scale = vec3().set(Tamanegi.body.radius);
            state.rotation = vec3();
            state.texture = Tamanegi.texture;
            render_mesh(state, library.meshes[Tamanegi.mesh]);

            // garlic
            state.world = Garlic.body.prev_pos.lerp(Garlic.body.pos, alpha);
            state.scale = vec3().set(Garlic.body.radius);
            state.rotation = vec3();
            state.texture = Garlic.texture;
            render_mesh(state, library.meshes[Garlic.mesh]);

			SDL_GL_SwapWindow(sgl.window);
		}
		memory.transient_current = 0;
	}
	
    // Texture data
    for (u4 i = 0; i < library.texture_count; i++)
    {
        glDeleteTextures(1,&library.textures[i].id);
    }

	// Mesh data
    for (u4 i = 0; i < library.mesh_count; i++)
    {
        glDeleteBuffers(1, &library.meshes[i].vertex_buffer);
        glDeleteBuffers(1, &library.meshes[i].uv_buffer);
    }

    // Shader
	glDeleteProgram(basic_texture.program);
	
    // Primitives
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