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
#include "objectloader.cpp"

int main(int argc, char* argv[])
{
	initialize_memory(memory, 8, 2);
	
	if (!create_sdl_opengl_window()) 
    { 
        cout << "ERROR: failed to create sdl or opengl" << endl;
    }
	
	create_plane();

    GLuint field_texture;
    GLuint marble_texture; 
    GLuint future_texture;
    GLuint mesh_texture;
    IMAGE field_image;
    IMAGE marble_image;
    IMAGE future_image;
    IMAGE mesh_image;

    /*
        OBJ LOADING 
    */
    if (!loadOBJ("media/rabbit.obj", vertices, uvs, normals))
    {
        cout << "Failed  to load obj." << endl;
        return 0;
    }
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

    mesh_texture = 0;
    mesh_image.data = stbi_load("media/rabbit.tga", &mesh_image.x, &mesh_image.y, &mesh_image.n, 3);
    mesh_texture = my_create_texture(1024,1024,true,mesh_image.data,false,mesh_image.n);

    /*
        OBJ LOADING
    */

	if (!create_basic_texture_shader()) return 0;

	field_texture = 0;
	field_image.data = stbi_load("media/circle.png", &field_image.x, &field_image.y, &field_image.n, 4);
	field_texture = my_create_texture(256,256,true,field_image.data,false,field_image.n);

    marble_texture = 0;
    marble_image.data = stbi_load("media/marble.png", &marble_image.x, &marble_image.y, &marble_image.n, 4);
    marble_texture = my_create_texture(256,256,true,marble_image.data,false,marble_image.n);

    future_texture = 0;
    future_image.data = stbi_load("media/future.png", &future_image.x, &future_image.y, &future_image.n, 4);
    future_texture = my_create_texture(256,256,true,future_image.data,false,future_image.n);

	// loop
	const f4 RENDER_MS = 1.0f/60.0f;
	const f4 PHYSICS_MS = 1.0f/60.0f;
	f4 render_dt = 0.0f;
	f4 physics_dt = 0.0f;
	u4 time_physics_prev = SDL_GetTicks();
	f4 camera_angle = M_PI32/-2.0f;
	f4 cam_radius = 3.0f;
	vec3 camera_pos = vec3();

    container.radius = 1.0f;
    container.pos.x = 0.0f;
    container.prev_pos.x = 0.0f;

    poolball.pos.x = -0.5f;
    poolball.prev_pos.x = -0.5f;
    poolball.pos.z = -0.05f;
    poolball.prev_pos.z = -0.05f;
    poolball.velocity.x = 0.025f;

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
            if (single_press(input.d)) { marble_force.dir.x =  1.0f; }
            if (single_press(input.a)) { marble_force.dir.x = -1.0f; }
            if (single_press(input.w)) { marble_force.dir.z =  1.0f; }
            if (single_press(input.s)) { marble_force.dir.z = -1.0f; }

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

            step(marble, marble_force);
            step(poolball, poolball_force);

            // marble + poolball
            detect_and_apply_collision_circle_circle(marble, poolball);

            // marble + container
            if (calculate_PoC_circle_in_circle_minkowski_difference (marble, container))
            {
                reflect_circle_within_cirle(marble, container.pos);
            }
            else
            {
                marble.pos = marble.pos + marble.velocity;
            }

            // poolball + container
            if (calculate_PoC_circle_in_circle_minkowski_difference (poolball, container))
            {
                reflect_circle_within_cirle(poolball, container.pos);
            }
            else
            {
                poolball.pos = poolball.pos + poolball.velocity;
            }
		}
        // cout << "  " << marble.velocity.length() << endl;
        f4 alpha = physics_dt / PHYSICS_MS;

		render_dt += frame_time;
		// if (render_dt >= RENDER_MS)
		{
			render_dt = 0;

			f4 posX = cam_radius * cos(camera_angle);
    		f4 posZ = cam_radius * sin(camera_angle);

            vec3 pp = camera_pos + vec3(posX, cam_radius, posZ);
            vec3 lookat = camera_pos;

			glm::mat4 view = glm::lookAt(
				glm::vec3(pp.x,pp.y,pp.z),
				glm::vec3(lookat.x, lookat.y, lookat.z),
				glm::vec3(0.0f, 1.0f, 0.0f));

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

            // container
   //          state.world = container.pos;
   //          state.scale = vec3(container.radius,container.radius,1.0f);
   //          state.texture = field_texture;
			// render_plane(state);

   //          // poolball
   //          state.world = poolball.prev_pos.lerp(poolball.pos, alpha);
   //          state.scale = vec3(poolball.radius,poolball.radius,1.0f);
   //          state.texture = marble_texture;
   //          render_plane(state);

   //          // marble
   //          state.world = marble.prev_pos.lerp(marble.pos, alpha);
   //          state.scale = vec3(marble.radius,marble.radius,1.0f);
   //          state.texture = marble_texture;
   //          render_plane(state);

            // mesh
            state.world = container.pos;
            state.scale = vec3(container.radius,container.radius,1.0f);
            state.texture = mesh_texture;
            state.rotation.x = 0.0f;
            render_mesh(state);

			SDL_GL_SwapWindow(sgl.window);
		}
		memory.transient_current = 0;
	}
	
    // Image data, Texture data
	stbi_image_free(field_image.data);
	glDeleteTextures(1,&field_texture);
    stbi_image_free(marble_image.data);
    glDeleteTextures(1,&marble_texture);
    stbi_image_free(future_image.data);
    glDeleteTextures(1,&future_texture);
    stbi_image_free(mesh_image.data);
    glDeleteTextures(1,&mesh_texture);
	
    // Shader
	glDeleteProgram(basic_texture.program);
	
    // Primitives
    glDeleteBuffers(1, &plane.verts);
	glDeleteBuffers(1, &plane.colors);
	glDeleteBuffers(1, &plane.indices);
	glDeleteBuffers(1, &plane.uv_coords);

    // OBJ LOADING
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &uvbuffer);
	
    SDL_DestroyWindow( sgl.window );
	SDL_Quit();

	free(memory.TransientStorage);
	free(memory.PermanentStorage);

	return 0;
}