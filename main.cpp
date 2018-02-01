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

    if (!create_basic_texture_shader()) return 0;

    library.texture_count = 20;
    library.textures = (Library::Texture*)alloc(memory, sizeof(Library::Texture) * library.texture_count);
    library.mesh_count = 20;
    library.meshes = (Library::Mesh*)alloc(memory, sizeof(Library::Mesh) * library.mesh_count);

    load_texture("media/circle.png", 256, 256, 4);
    load_texture("media/rabbit.tga", 1024, 1024, 3);
    load_texture("media/steel.png", 1024, 1024, 3);
    load_texture("media/aluminum.png", 1024, 1024, 3);
    load_texture("media/pusher.png", 256, 256, 4);
    load_texture("media/floor.png", 1024, 1024, 3);

    load_mesh("media/tamanegi.obj");
    load_mesh("media/rabbit.obj");
    load_mesh("media/pusher.obj");

    /*
        Entities
    */
    Entity Tamanegi;
    Tamanegi.mesh = get_mesh("media/tamanegi.obj");
    Tamanegi.texture = get_texture("media/steel.png");
    Tamanegi.body.radius = 0.1f;
    Tamanegi.body.pos = setv(0.0f,0.0f,-0.5f);
    Tamanegi.body.prev_pos = Tamanegi.body.pos;
    Tamanegi.scale = setv(Tamanegi.body.radius);

    Entity Pusher;
    Pusher.mesh = get_mesh("media/pusher.obj");
    Pusher.texture = get_texture("media/pusher.png");
    Pusher.body.radius = 0.1f;
    Pusher.scale = setv(Pusher.body.radius);

    Entity Garlic;
    Garlic.mesh = get_mesh("media/tamanegi.obj");
    Garlic.texture = get_texture("media/aluminum.png");
    Garlic.body.radius = 0.1f;
    Garlic.body.mass = 5.5f;
    Garlic.body.pos = setv(0.5f,0.0f,-0.5f);
    Garlic.body.prev_pos = Garlic.body.pos;
    Garlic.scale = setv(Garlic.body.radius);

    Entity Rabbit;
    Rabbit.mesh = get_mesh("media/rabbit.obj");
    Rabbit.texture = get_texture("media/rabbit.tga");
    Rabbit.body.radius = 1.0f;
    Rabbit.body.pos.y = 1.5f;
    Rabbit.body.prev_pos = Rabbit.body.pos;
    Rabbit.scale = setv(Rabbit.body.radius);
    Rabbit.orient = quat_axis_z(-90.0f) * quat_axis_y(-90.0f);

    Entity Floor;
    Floor.body.radius = 1.0f;
    Floor.body.pos = setv(0.0f,0.0f,-0.75f);
    Floor.body.prev_pos = Floor.body.pos;
    Floor.texture = get_texture("media/floor.png");
    Floor.scale = setv(Floor.body.radius);

    RigidBody container;
    container.radius = 1.0f;
    container.pos = setv(0.0f,0.0f,-1.0f);
    container.prev_pos = container.pos;
    GLuint container_texture = get_texture("media/circle.png");

	// loop
	const f4 RENDER_MS = 1.0f/60.0f;
	const f4 PHYSICS_MS = 1.0f/60.0f;
	f4 render_dt = 0.0f;
	f4 physics_dt = 0.0f;
	u4 time_physics_prev = SDL_GetTicks();
	f4 camera_angle = 0.0f;
	f4 cam_radius = 3.0f;
	vec3 camera_pos;
    vec3 camera_pos_on_radius;

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

            /*
                Camera
            */
            if (input.right) {
                camera_angle -= M_PI32 * 0.01f;
                camera_angle = camera_angle < 0.0f ? camera_angle = M_PI32 * 2.0f + camera_angle : camera_angle;
            }
            if (input.left) {
                camera_angle += M_PI32 * 0.01f;
                camera_angle = camera_angle >  M_PI32 * 2.0f ? camera_angle - (M_PI32 * 2.0f) : camera_angle;
            }
            if (input.up) { 
                cam_radius -= M_PI32 * 0.01f;
            }
            if (input.down) {
                cam_radius += M_PI32 * 0.01f;
            }

            camera_pos = weighted_average(camera_pos, Rabbit.body.pos, 20.0f);

            f4 cPosX = cam_radius * cos(camera_angle);
            f4 cPosY = cam_radius * sin(camera_angle);
            camera_pos_on_radius = setv(-cPosX, -cPosY, cam_radius);
            
            /*
                Physics
            */
            // apply friction
            const f4 ground_friction = 0.985f;

            const vec3 gravity = setv(0.0f,0.0f,-0.0025f);

            // user input
            vec3 push_direction = normal(camera_pos_on_radius * setv(-1.0f,-1.0f,0.0f));
            f4 push_length = 0.0f;

            if (single_press(input.w)) { push_length = 1.0f * 0.075f; }

            Tamanegi.body.user_force = Tamanegi.body.user_force + (push_direction * push_length);

            auto step = [ground_friction, gravity] (RigidBody &body) {
                // store last position
                body.prev_pos = body.pos;

                // calculate velocity
                body.velocity = body.velocity + body.user_force;

                // apply friction
                body.velocity = body.velocity * ground_friction;

                // add gravity
                body.velocity = body.velocity + gravity;

                // erase user forces
                body.user_force = setv(0.0f);
            };

            // move everything
            step(Tamanegi.body);
            step(Garlic.body);

            // Tamanegi.body + Garlic.body
            detect_and_apply_collision_circle_circle(Tamanegi.body, Garlic.body);

            /*
                https://gafferongames.com/post/physics_in_3d/
            */

            auto calculate_PoC_sphere_plane = [] (RigidBody &Sphere, vec3 plane_normal)
            {
                vec3 P = Sphere.pos;
                vec3 N = plane_normal;
                vec3 V = Sphere.velocity;
                f4 d;

                /*
                    Ray - Plane intersection
                    Source:
                        https://www.cs.princeton.edu/courses/archive/fall00/cs426/lectures/raycast/sld017.htm
                        https://www.siggraph.org/education/materials/HyperGraph/raytrace/rayplane_intersection.htm
                */
                f4 t = ( (dot(P, N) + d) * -1.0f) / dot(V, N);
                vec3 PoC = P + (V * t);
            };

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
		if (render_dt >= RENDER_MS)
		{
			render_dt = 0;

            // @todo: lerp camera position

            // X+ left, Y+ left, Z+ up
			glm::mat4 view = glm::lookAt(
				glmv(camera_pos + camera_pos_on_radius),
				glmv(camera_pos),
				glm::vec3(0.0f, 0.0f, 1.0f));

			glm::mat4 projection = glm::perspective(45.0f, 1.0f*sgl.width/sgl.height, 0.1f, 100.0f);

		   	glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glClearColor(0.23f, 0.47f, 0.58f, 1.0f); 
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            auto render = [alpha, view, projection] (Entity &entity)
            {
                RENDER_STATE state;
                state.view = view;
                state.projection = projection;
                state.world = lerp(entity.body.prev_pos, entity.body.pos, alpha);
                state.scale = setv(entity.body.radius);
                state.texture = entity.texture;
                state.rotation = entity.rotation;
                state.orient = entity.orient;
                render_mesh(state, library.meshes[entity.mesh]);
            };

            render(Tamanegi);
            render(Garlic);
            render(Pusher);
            render(Rabbit);

			SDL_GL_SwapWindow(sgl.window);
		}
		memory.transient_current = 0;
	}
	
    // Texture data
    for (u4 i = 0; i < library.texture_count; i++)
    {
        glDeleteTextures(1, &library.textures[i].id);
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