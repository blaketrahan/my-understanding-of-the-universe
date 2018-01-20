/*
	-Blake Trahan
	-2017
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
	texture_font = 0;
	image_font.data = stbi_load("circle.png", &image_font.x, &image_font.y, &image_font.n, 4);
	texture_font = my_create_texture(256,256,true,image_font.data,false,image_font.n);

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
        vec3 dir;
        f4 radius;
        f4 vel;
    } ball;

    ball.radius = 0.1f;
    ball.dir = vec3(1.0f,0.0f,0.0f);
    ball.vel = 0.0f;
	
	while(!input.quit_app)
	{
		u4 time_physics_curr = SDL_GetTicks();
		f4 frameTime = ((f4)(time_physics_curr - time_physics_prev)) / 1000.0f;
		time_physics_prev = time_physics_curr;

		poll_events();

		physics_dt += frameTime;
		if (physics_dt >= PHYSICS_MS)
		{
			physics_dt -= PHYSICS_MS;

			f4 W = 20.0f;
			camera_pos.x = ((camera_pos.x * (W - 1)) + 0) / W;
			camera_pos.y = ((camera_pos.y * (W - 1)) + 0) / W;
			camera_pos.z = ((camera_pos.z * (W - 1)) + 0) / W;

            f4 vel = ball.vel;
            vec3 dir = ball.dir;

            b4 push = false;

            vec3 user_dir = vec3(0.0f,0.0f,0.0f);

            if (single_press(input.d)) {user_dir.x =  1.0f; push=true;}
            if (single_press(input.a)) {user_dir.x = -1.0f; push=true;}
            if (single_press(input.w)) {user_dir.z =  1.0f; push=true;}
            if (single_press(input.s)) {user_dir.z = -1.0f; push=true;}

            if (push) { dir = user_dir; vel = 0;}
            
            auto get_length = [] (vec3 v) {
                return sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
            };

            auto get_normal = [] (vec3 v) {
                f4 length = sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
                return vec3(v.x / length, v.y / length, v.z / length);
            };

            auto scalar_product = [] (vec3 X, vec3 Y) {
                return (X.x * Y.x) + (X.y * Y.y) + (X.z * Y.z);
            };

            auto vector_coefficient = [] (f4 C, vec3 V) {
                return vec3(V.x * C, V.y * C, V.z * C);
            };

            // get direction normal
            dir = get_normal(dir);

            // apply velocity
            const f4 PUSH_STRENGTH = 1.0f * 0.035f; // 1.0 == radius of cylinder in one timestep
            if (push) vel += (PUSH_STRENGTH);
            
            // add friction
            const f4 ground_friction = 0.95;
            vel *= ground_friction;

            // new position
            vec3 pos = ball.pos;
            vec3 diff = vec3(dir.x * vel, dir.y * vel, dir.z * vel);
            pos += diff;

            // collision check with wall
            f4 d = sqrt((pos.x * pos.x) + (pos.z * pos.z));
            if (d >= 1.0) // hit wall
            {
                // 1.0/d returns the percent of the old to new pos needed to move
                // aka point of collision
                vec3 point_of_contact;
                point_of_contact.x = pos.x * (1.0/d); 
                point_of_contact.z = pos.z * (1.0/d);

                // reflect off tangent
                // https://stackoverflow.com/questions/573084/how-to-calculate-bounce-angle
                // u = (v · n / n · n) n 
                // w = v − u
                vec3 V = pos - ball.pos; // velocity
                vec3 N = get_normal(vec3(-pos.x,pos.y,-pos.z)); // surface normal of circle tangent

                f4 SP = (scalar_product(V,N));

                vec3 U = vector_coefficient(SP,N);
                vec3 W = V - U;

                const f4 F = 0.9; // wall friction, 1 == frictionless
                const f4 R = 0.75;  // wall elasticity, 1 == perfect elastic reaction
                vec3 new_vel = (vector_coefficient(R,W)) - (vector_coefficient(F,U));
                pos = point_of_contact + new_vel;

                vel = get_length(new_vel); // after friction, restitution, this is the new vel

                dir = get_normal(pos - point_of_contact); // new dir

                // angular momentum
                // deformation
                // https://gafferongames.com/post/physics_in_3d/
            }

            // apply changes
            ball.pos = pos;
            ball.dir = dir;
            ball.vel = vel;
		}

		render_dt += frameTime;
		if (render_dt >= RENDER_MS)
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
			state.world = vec3(pp.x,pp.y,pp.z);
			state.texture = texture_font;
			state.scale = vec3(1.0f,1.0f,1.0f);
			state.rotation.x = 90.0f * M_DEGTORAD32;
			render_plane(state);

            state.world = vec3(ball.pos.x,ball.pos.y,ball.pos.z);
            state.scale = vec3(ball.radius,ball.radius,1.0f);
            render_plane(state);

			SDL_GL_SwapWindow(sgl.window);
		}
		memory.transient_current = 0;
	}
	
	stbi_image_free(image_font.data);
	glDeleteTextures(1,&texture_font);
	
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