/*
    http://www.feynmanlectures.caltech.edu/I_toc.html
    http://chrishecker.com/Physics_References
    http://chrishecker.com/Rigid_Body_Dynamics

    Things I would like to implement:
    - energy calculator: beginning and end should be equal
        - graph it (2 graphs, blue potential, red kinetic)
        - running graph, centered dot with trailing line
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

    library.textures = (Library::Texture*)alloc(memory, sizeof(Library::Texture) * 20);
    library.meshes = (Library::Mesh*)alloc(memory, sizeof(Library::Mesh) * 20);

    load_texture("media/steel.png", 1024, 1024, 3);
    load_texture("media/aluminum.png", 1024, 1024, 3);
    load_mesh("media/tamanegi.obj");
    load_mesh("media/cube.obj");

    struct BodyInfo {
        f4 restitution = 1.0f;
        f4 radius = 1.0f; 
        f4 density = 0.01f;
        vec3 pos;
        u4 type = 0;
        b4 dynamic = true;

        f4 width = 1.0f;
        f4 height = 1.0f;
        f4 depth = 1.0f;
    };
    enum BODY_TYPES {
        TYPE_SPHERE = 0,
        TYPE_CUBOID,
    };
    auto init_body = [] (RigidBody &body, BodyInfo info)
    {
        body.density = info.density;
        body.radius = info.radius;
        body.width = info.width;
        body.height = info.height;
        body.depth = info.depth;
        body.type = info.type;
        
        switch (info.type)
        {
            case TYPE_SPHERE:
                body.volume = (4.0f/3.0f) * PI * info.radius * info.radius * info.radius;
                break;

            case TYPE_CUBOID:
                body.volume = info.width * info.height * info.depth;
                break;
        }

        body.mass = body.density * body.volume;

        body.coefficient_restitution = info.restitution;

        body.one_over_mass = 1.0f / body.mass;

        const f4 gravity = 0.0f;//0.01f;
        body.gravity = -gravity / body.one_over_mass;

        body.force = setv();
        body.torque = setv();

        // https://en.wikipedia.org/wiki/List_of_moments_of_inertia

        switch (info.type)
        {
            case TYPE_SPHERE:
            {
                f4 moment_of_inertia = (2.0f/5.0f) * body.mass * info.radius * info.radius;
                body.MoI_local = identity() * moment_of_inertia;
            }
            break;

            case TYPE_CUBOID:
            {
                f4 f = 1.0f/12.0f * body.mass;
                f4 Ih = f * (body.width * body.width + body.depth * body.depth);
                f4 Iw = f * (body.depth * body.depth + body.height * body.height);
                f4 Id = f * (body.width * body.width + body.height * body.height);

                body.MoI_local = identity();
                body.MoI_local[0] = Iw;
                body.MoI_local[4] = Ih;
                body.MoI_local[8] = Id;
            }
            break;
        }

        body.inverse_MoI_local = inverse(body.MoI_local); // |I^-1 CM

        body.pos = info.pos; // r CM
        body.prev_pos = info.pos; 
        body.velocity = setv(); // v CM
        body.angular_velocity = setv();
        body.orientation = from_axis_angle(setv(1.0f,0.0f,0.0f), degtorad(0.0f)); // A
        body.angular_momentum = setv(); // L CM
    
        body.inverse_MoI_world = body.orientation * body.inverse_MoI_local * transpose(body.orientation); // I^-1 CM
    };

    /*
        Entities
    */
    BodyInfo info;

    Entity Ball;
        Ball.mesh = get_mesh("media/tamanegi.obj");
        Ball.texture = get_texture("media/steel.png");
        info.dynamic = true;
        info.type = TYPE_SPHERE;
        info.radius = 1.0f;
        info.width = info.radius;
        info.height = info.radius;
        info.depth = info.radius;
    init_body(Ball.body, info);

    Entity Garlic;
        Garlic.mesh = get_mesh("media/tamanegi.obj");
        Garlic.texture = get_texture("media/aluminum.png");
        info.pos.y = -10.0f;
        info.dynamic = true;
        info.type = TYPE_SPHERE;
        info.radius = 1.0f;
        info.width = info.radius;
        info.height = info.radius;
        info.depth = info.radius;
    init_body(Garlic.body, info);

    Entity Cuboid;
        Cuboid.mesh = get_mesh("media/cube.obj");
        Cuboid.texture = get_texture("media/aluminum.png");
        info.pos = setv(0.0f, 0.0f, -5.0f);
        info.dynamic = false;
        info.type = TYPE_CUBOID;
        info.width = 5.0f;
        info.height = 5.0f;
        info.depth = 0.25f;
    init_body(Cuboid.body, info);

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

            // user input
            const f4 push = 0.025f;
            if (single_press(key.a)) Garlic.body.force.y += push;
            if (single_press(key.w)) Garlic.body.force.z += push;
            if (single_press(key.s)) Garlic.body.force.z -= push;
            if (single_press(key.d)) Garlic.body.force.y -= push;
            if (single_press(key.j)) Ball.body.force.y += push;
            if (single_press(key.i)) Ball.body.force.z += push;
            if (single_press(key.k)) Ball.body.force.z -= push;
            if (single_press(key.l)) Ball.body.force.y -= push;

            // apply everything but new position.
            step(Ball.body);
            step(Garlic.body);
            step(Cuboid.body);

            if (collide_sphere_sphere(Ball.body, Garlic.body))
            {
                resolve_dynamic_dynamic(Ball.body, Garlic.body);
            }

            if (collide_sphere_cuboid_AABB(Ball.body, Cuboid.body))
            {
                resolve_dynamic_static(Ball.body, Garlic.body);
            }

            // apply new position
            auto post_step = [] (RigidBody &body) {
                body.pos = body.future_pos;
            };
            post_step(Ball.body);
            post_step(Garlic.body);
            post_step(Cuboid.body);
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
                state.scale = setv(entity.body.width, entity.body.height, entity.body.depth);
                state.texture = entity.texture;
                state.orient = entity.body.orientation;
                render_mesh(state, library.meshes[entity.mesh]);
            };

            render(Ball);
            render(Garlic);
            render(Cuboid);

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