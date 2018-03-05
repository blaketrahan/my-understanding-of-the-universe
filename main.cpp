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

    library.texture_count = 2;
    library.textures = (Library::Texture*)alloc(memory, sizeof(Library::Texture) * library.texture_count);
    library.mesh_count = 1;
    library.meshes = (Library::Mesh*)alloc(memory, sizeof(Library::Mesh) * library.mesh_count);

    load_texture("media/steel.png", 1024, 1024, 3);
    load_texture("media/aluminum.png", 1024, 1024, 3);
    load_mesh("media/tamanegi.obj");

    struct BodyInfo {
        f4 restitution = 1.0f;
        f4 radius = 1.2f;
        f4 density = 0.01f;
        vec3 pos;
    };

    auto init_sphere_body = [] (RigidBody &body, BodyInfo info)
    {
        body.density = info.density;
        body.radius = info.radius;
        body.volume = (4.0f/3.0f) * PI * info.radius * info.radius * info.radius; // sphere
        body.mass = body.density * body.volume;

        body.coefficient_restitution = info.restitution;

        body.one_over_mass = 1.0f / body.mass;

        const f4 gravity = 0.0f;//0.01f;
        body.gravity = -gravity / body.one_over_mass;

        body.force = setv();
        body.torque = setv();

        // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        // solid sphere
        body.moment_of_inertia = (2.0f/5.0f) * body.mass * info.radius * info.radius;
        body.MoI_local = identity() * body.moment_of_inertia;
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
    init_sphere_body(Ball.body, info);

    Entity Garlic;
    Garlic.mesh = get_mesh("media/tamanegi.obj");
    Garlic.texture = get_texture("media/aluminum.png");
    info.pos.y = -10.0f;
    init_sphere_body(Garlic.body, info);

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
            if (single_press(key.a)) Garlic.body.force.y += push;
            if (single_press(key.w)) Garlic.body.force.z += push;
            if (single_press(key.s)) Garlic.body.force.z -= push;
            if (single_press(key.d)) Garlic.body.force.y -= push;
            if (single_press(key.j)) Ball.body.force.y += push;
            if (single_press(key.i)) Ball.body.force.z += push;
            if (single_press(key.k)) Ball.body.force.z -= push;
            if (single_press(key.l)) Ball.body.force.y -= push;

            auto step = [] (RigidBody &body)
            {
                body.prev_pos = body.pos;
                body.collision_time = 0.0f;
                body.remaining_velocity = 1.0f;
                body.collision_pos = body.pos;

                // --

                body.force.z = body.force.z + body.gravity;

                // body.torque = crossproduct ( setv( 0, 0, -body.radius ), setv( 0, -0.00001f, 0 ) );

                // --

                body.velocity = body.velocity + (body.one_over_mass * body.force);
                body.velocity = body.velocity * 0.95f;

                body.future_pos = body.pos + body.velocity;

                body.orientation = body.orientation + (skew_symmetric(body.angular_velocity) * body.orientation);

                body.angular_momentum = body.angular_momentum + body.torque;

                body.orientation = orthonormalize(body.orientation);

                // --

                body.inverse_MoI_world = body.orientation * body.inverse_MoI_local * transpose(body.orientation);

                body.angular_velocity = body.angular_momentum * body.inverse_MoI_world;

                // -- clear forces
                body.torque = setv();
                body.force = setv();
            };

            // apply everything but new position.
            step(Ball.body);
            step(Garlic.body);

            // Ball.body + Garlic.body
            if (collision_circle_circle(Ball.body, Garlic.body))
            {
                auto apply_impulses = [] (RigidBody &A, RigidBody &B)
                {
                    vec3 r = A.PoC;
                    vec3 rb = B.PoC;

                    vec3 v1 = A.velocity + crossproduct(A.angular_velocity, r);
                    vec3 v2 = B.velocity + crossproduct(B.angular_velocity, rb);
                    vec3 v = v1 - v2;

                    f4 ImpulseNumerator = -(1.0f + ((A.coefficient_restitution + B.coefficient_restitution) * 0.5f)) * dot( v, A.collision_normal);

                    vec3 inertia_vector_normal = crossproduct(crossproduct(r, A.collision_normal) * A.inverse_MoI_world, r) + crossproduct(crossproduct(rb, A.collision_normal) * B.inverse_MoI_world, rb);
                    f4 ImpulseDenominator = (A.one_over_mass + B.one_over_mass) + dot(inertia_vector_normal, A.collision_normal);

                    vec3 Impulse = (ImpulseNumerator / ImpulseDenominator) * A.collision_normal;

                    A.velocity = A.velocity + A.one_over_mass * Impulse;
                    A.angular_momentum = A.angular_momentum + crossproduct(r, Impulse);
                    // compute affected auxiliary quantities
                    A.angular_velocity = A.angular_momentum * A.inverse_MoI_world;

                    // -- equal and opposite
                    B.velocity = B.velocity + (B.one_over_mass * Impulse * -1.0f);
                    B.angular_momentum = B.angular_momentum + (crossproduct(rb, Impulse) * -1.0f);
                    // compute affected auxiliary quantities
                    B.angular_velocity = B.angular_momentum * B.inverse_MoI_world;

                    // calculate new future position
                    A.future_pos = A.future_pos + A.velocity * A.remaining_velocity;
                    B.future_pos = B.future_pos + B.velocity * B.remaining_velocity;
                };
                apply_impulses(Ball.body, Garlic.body);
            }

            auto post_step = [] (RigidBody &body) {
                body.pos = body.future_pos;
            };
            post_step(Ball.body);
            post_step(Garlic.body);
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