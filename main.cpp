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

    /*
        Entities
    */
    BodyInfo info;

    Entity Ball;
        Ball.mesh = get_mesh("media/tamanegi.obj");
        Ball.texture = get_texture("media/steel.png");
        info.pos = setv(0.0f,5.0f,0.0f);
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
        info.pos = setv(0.0f,-5.0f,0.0f);
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
        info.width = 10.0f;
        info.height = 10.0f;
        info.depth = .05f;
    init_body(Cuboid.body, info);

    build_planes_from_cuboid(Cuboid.body);

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

            auto collide_sphere_planar_body = [] (RigidBody &A, RigidBody &B, u4* indices, u4 num_index)
            {
                // todo: capture which planes to test against before this function is called
                // todo: rotate planes to match body rotation
                // todo: test if point is within plane
                vec3 p = A.pos;

                /*
                    Loop through each plane of interest until finding collision
                */
                for (u4 i = 0; i < num_index; i++)
                {
                    Plane plane = B.planes[indices[i]];

                    // print(plane.normal);
                    vec3 world_pos = plane.pos + B.pos;

                    vec3 V = A.pos - world_pos;

                    f4 D = dot(V, plane.normal);

                    if (D < A.radius) return true;
                }
                // vec3 pt_on_plane = p - (plane.normal * D);
                return false;
            };

            u4 num_index = 1;
            u4 indices[1];
            indices[0] = 4; // just the Z+ plane
            if (collide_sphere_planar_body(Ball.body, Cuboid.body, indices, num_index)) {
                // 
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