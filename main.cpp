
/*
    http://www.feynmanlectures.caltech.edu/I_toc.html
    http://chrishecker.com/Physics_References
    http://chrishecker.com/Rigid_Body_Dynamics  
*/

/*
	-Blake Trahan
	-2018
*/
#include "vars.cpp"
#include "sgl.cpp"
#include "shaders.cpp"
#include "render_functions.cpp"
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

    // load_texture("media/circle.png", 256, 256, 4);
    // load_texture("media/rabbit.tga", 1024, 1024, 3);
    load_texture("media/steel.png", 1024, 1024, 3);
    load_texture("media/aluminum.png", 1024, 1024, 3);
    // load_texture("media/pusher.png", 256, 256, 4);
    // load_texture("media/floor.png", 1024, 1024, 3);
    // load_texture("media/cube2.png", 512, 512, 3);

    load_mesh("media/tamanegi.obj");
    // load_mesh("media/rabbit.obj");
    // load_mesh("media/cube2.obj");

    auto init_sphere_body = [] (RigidBody &body, f4 radius, f4 density, f4 restitution, vec3 pos)
    {
        body.density = density;
        body.radius = radius;
        body.volume = (4.0f/3.0f) * PI * radius * radius * radius; // sphere
        
        body.mass = body.density * body.volume;

        body.coefficient_restitution = restitution;

        body.one_over_mass = 1.0f / body.mass;

        const f4 gravity = 0.0f;//0.01f;
        body.gravity = -gravity / body.one_over_mass;

        // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        // solid sphere
        f4 moment_of_inertia = (2.0f/5.0f) * body.mass * radius * radius;
        body.one_over_CM_moment_of_inertia = 1.0f / moment_of_inertia;

        body.inverse_inertia_tensor = identity();
        body.inverse_inertia_tensor = body.inverse_inertia_tensor * (moment_of_inertia);
        body.inverse_inertia_tensor = inverse(body.inverse_inertia_tensor);

        body.orientation = from_axis_angle(setv(1.0f,0.0f,0.0f), degtorad(0.0f));
        body.AngularVelocity = 0.0f;
        body.torque = 0.0f;
        body.velocity = setv(0.0f, 0.0f, 0.0f);
        body.CMForce = setv(0.0f,0.0f,0.0f);
        body.pos = pos;

        body.prev_pos = pos;
    };

    /*
        Entities
    */

    Entity Ball;
    Ball.mesh = get_mesh("media/tamanegi.obj");
    Ball.texture = get_texture("media/steel.png");
    init_sphere_body(Ball.body, 1.0f, 0.01f, 1.0f, setv(0.0f,0.0f,0.0f));

    Entity Garlic;
    Garlic.mesh = get_mesh("media/tamanegi.obj");
    Garlic.texture = get_texture("media/aluminum.png");
    init_sphere_body(Garlic.body, 1.0f, 0.01f, 1.0f, setv(0.0f,-10.0f,0.0f));
    Garlic.body.velocity = setv(0.0f,0.0f,0.0f);

    // Ball.body.AngularVelocity = 10.0f;
    // Garlic.body.AngularVelocity = 10.0f;

	// loop
	const f4 RENDER_MS = 1.0f/120.0f;
	const f4 PHYSICS_MS = 1.0f/60.0f;
	f4 render_dt = 0.0f;
	f4 physics_dt = 0.0f;
	u4 time_physics_prev = SDL_GetTicks();
	f4 camera_angle = 0.0f;
	f4 cam_radius = 20.0f;
	vec3 prev_camera_pos;
    vec3 camera_pos;
    vec3 camera_pos_on_radius;

	while(!key.quit_app)
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
            if (key.right) {
                camera_angle -= PI * 0.01f;
                camera_angle = camera_angle < 0.0f ? camera_angle = PI * 2.0f + camera_angle : camera_angle;
            }
            if (key.left) {
                camera_angle += PI * 0.01f;
                camera_angle = camera_angle >  PI * 2.0f ? camera_angle - (PI * 2.0f) : camera_angle;
            }
            if (key.up) { 
                cam_radius -= PI * 0.5f;
            }
            if (key.down) {
                cam_radius += PI * 0.5f;
            }

            prev_camera_pos = camera_pos;
            camera_pos = weighted_average(camera_pos, camera_pos, 20.0f);

            f4 cPosX = cam_radius * cos(camera_angle);
            f4 cPosY = cam_radius * sin(camera_angle);
            camera_pos_on_radius = setv(-cPosX, -cPosY, 0.0f);
            
            /*
                Physics
            */

            /* user input */
            const f4 push = 0.025f;
            if (single_press(key.a)) Garlic.body.CMForce.y += push;
            if (single_press(key.w)) Garlic.body.CMForce.z += push;
            if (single_press(key.s)) Garlic.body.CMForce.z -= push;
            if (single_press(key.d)) Garlic.body.CMForce.y -= push;
            if (single_press(key.j)) Ball.body.CMForce.y += push;
            if (single_press(key.i)) Ball.body.CMForce.z += push;
            if (single_press(key.k)) Ball.body.CMForce.z -= push;
            if (single_press(key.l)) Ball.body.CMForce.y -= push;

            auto step = [] (RigidBody &body)
            {
                body.prev_pos = body.pos;
                body.collision_time = 0.0f;
                body.remaining_velocity = 1.0f;
                body.collision_pos = body.pos;

                // add forces
                // body.CMForce = body.CMForce + forces
                body.CMForce.z = body.CMForce.z + body.gravity;

                // calculate new velocities
                body.velocity = body.velocity + (body.one_over_mass * body.CMForce);

                body.AngularVelocity = body.AngularVelocity + body.one_over_CM_moment_of_inertia * body.torque;

                body.velocity = body.velocity * 0.85f; /* @todo: remove this */
                body.AngularVelocity = body.AngularVelocity;
                
                // clear forces
                body.torque = 0.0f;
                body.CMForce = setv(0.0f,0.0f,0.0f);
            };

            // move everything
            step(Ball.body);
            step(Garlic.body);

            // Ball.body + Garlic.body
            if (collision_circle_circle(Ball.body, Garlic.body))
            {
                // apply_impulses_3D(Ball.body, Garlic.body);
                apply_impulses_3D(Ball.body, Garlic.body);
            }

            auto apply = [] (RigidBody &body)
            {
                body.pos = body.pos + body.velocity * body.remaining_velocity;
                // body.Orientation = body.Orientation + body.AngularVelocity;
            };

            apply(Ball.body);
            apply(Garlic.body);
		}

        f4 alpha = physics_dt / PHYSICS_MS;

		render_dt += frame_time;
		if (render_dt >= RENDER_MS)
		{
			render_dt = 0;

            // X+ forward, Y+ left, Z+ up
			glm::mat4 view = glm::lookAt(
				glmv(lerp(prev_camera_pos,camera_pos,alpha) + camera_pos_on_radius),
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
                if (entity.body.collision_time != 0.0f)
                {
                    if (alpha <= entity.body.collision_time)
                    {
                        state.world = lerp(entity.body.prev_pos, entity.body.collision_pos, (alpha/entity.body.collision_time));
                    }
                    else {
                        state.world = lerp(entity.body.collision_pos, entity.body.pos, (alpha - entity.body.collision_time) / (1.0f - entity.body.collision_time));
                    }
                }
                else
                {
                    state.world = lerp(entity.body.prev_pos, entity.body.pos, alpha);
                }
                state.scale = setv(entity.body.radius);
                state.texture = entity.texture;
                state.orient = entity.body.orientation;
                render_mesh(state, library.meshes[entity.mesh]);
            };

            render(Ball);
            render(Garlic);

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

inline void poll_events()
{
    SDL_Event e;        
    while( SDL_PollEvent( &e ) != 0 )
    {
        if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
        {
            b4 val = e.type == SDL_KEYDOWN ? true : false;
            switch( e.key.keysym.sym )
            { 
                case SDLK_w:        key.w.pressed = val; break;
                case SDLK_a:        key.a.pressed = val; break;
                case SDLK_s:        key.s.pressed = val; break;
                case SDLK_d:        key.d.pressed = val; break;
                case SDLK_i:        key.i.pressed = val; break;
                case SDLK_j:        key.j.pressed = val; break;
                case SDLK_k:        key.k.pressed = val; break;
                case SDLK_l:        key.l.pressed = val; break;
                case SDLK_RIGHT:    key.right = val; break;
                case SDLK_LEFT:     key.left = val; break;
                case SDLK_UP:       key.up = val; break;
                case SDLK_DOWN:     key.down = val; break;
                case SDLK_ESCAPE:   key.quit_app = val; break;
            }
        }
        else if ( e.type == SDL_QUIT )
        {
            key.quit_app = true;
        }
    }
}