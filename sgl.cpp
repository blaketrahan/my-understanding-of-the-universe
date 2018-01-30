b4 create_sdl_opengl_window ()
{
    sgl.width = 800;
    sgl.height = 600;

    SDL_Init(SDL_INIT_VIDEO);
    
    sgl.window = SDL_CreateWindow("Circle",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        sgl.width, sgl.height,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);
    
    if (sgl.window == NULL)
    {
        cerr << "Error: can't create window: " << SDL_GetError() << endl;
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    if (SDL_GL_CreateContext(sgl.window) == NULL)
    {
        cerr << "Error: SDL_GL_CreateContext: " << SDL_GetError() << endl;
        return false;
    }

    //Initialize GLEW
    glewExperimental = GL_TRUE;
    GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK)
    {
        cerr << "Error: glewInit: " << glewGetErrorString(glew_status) << endl;
        return false;
    }
    if (!GLEW_VERSION_2_0)
    {
        cerr << "Error: your graphic card does not support OpenGL 2.0" << endl;
        return false;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    /* Enable depth test */
    glEnable(GL_DEPTH_TEST);
    /* Accept fragment if it closer to the camera than the former one */
    glDepthFunc(GL_LESS); 
    /* Cull triangles which normal is not towards the camera */
    glEnable(GL_CULL_FACE);
    
    // SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    // SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    // SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    // SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    // SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
    // SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    // SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);



    return true;
}

void create_plane () {
    GLfloat plane_vertices[] = {
        -1.0f, -1.0f, 0.0f, //VO - 0
         1.0f, -1.0f, 0.0f, //V1 - 1
        -1.0f,  1.0f, 0.0f, //V2 - 2
         1.0f,  1.0f, 0.0f, //V3 - 3
    };
    glGenBuffers(1, &plane.verts);
    glBindBuffer(GL_ARRAY_BUFFER, plane.verts);
    glBufferData(GL_ARRAY_BUFFER, sizeof(plane_vertices), plane_vertices, GL_STATIC_DRAW);
    
    GLfloat plane_colors_black[] = {
        // top colors
        0.06, 0.52, 0.15,
        0.06, 0.52, 0.15,
        0.06, 0.52, 0.15,
        0.06, 0.52, 0.15,
    };
    glGenBuffers(1, &plane.colors);
    glBindBuffer(GL_ARRAY_BUFFER, plane.colors);
    glBufferData(GL_ARRAY_BUFFER, sizeof(plane_colors_black), plane_colors_black, GL_STATIC_DRAW);
    
    GLushort plane_elements[] = {
        0,1,2, 1,3,2,
    };
    glGenBuffers(1, &plane.indices);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, plane.indices);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(plane_elements), plane_elements, GL_STATIC_DRAW);

    GLfloat plane_uv[] = {
        0.0f, 1.0f, //0
        1.0f, 1.0f, //1
        0.0f, 0.0f, //2
        1.0f, 0.0f, //3
    };
    glGenBuffers(1, &plane.uv_coords);
    glBindBuffer(GL_ARRAY_BUFFER, plane.uv_coords);
    glBufferData(GL_ARRAY_BUFFER, sizeof(plane_uv), plane_uv, GL_STATIC_DRAW);
}



GLuint my_create_texture (s4 sw, s4 sh, b4 alpha, unsigned char* image_data = 0, b4 is_render_target = false, int comp = 0)
{
    // @TODO: figure texture size
    unsigned int width = sw;
    unsigned int height = sh;

    // Create one OpenGL texture
    GLuint textureID;
    glGenTextures(1, &textureID);
    
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    if (is_render_target)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
        return textureID;
    }

    // Give the image to OpenGL
    if (image_data == 0)
    {
        if (alpha) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, 0);
        }
        else { 
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, 0); 
            // @TODO: test if necessary
        }
    }
    else
    {
        if(comp == 3)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
        else if(comp == 4)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    }
    return textureID;
}

void load_texture (const char *filename, u4 w, u4 h, u4 components)
{
    IMAGE image;
    image.data = stbi_load(filename, &image.x, &image.y, &image.n, components);
    if (image.data == NULL)
    {
        cout << "Failed to load texture: " << filename << endl;
        return;
    }
    GLuint texture_id = my_create_texture(w, h, true, image.data, false, image.n);

    stbi_image_free(image.data);

    /*
        add to texture list
    */
    library.textures[library.texture_count].name = filename;
    library.textures[library.texture_count].id = texture_id;
    library.texture_count++;
}

void assign_texture (Entity &entity, const char* filename)
{
    b4 found = false;
    for (u4 i = 0; i < library.texture_count; i++)
    {
        if (library.textures[i].name == filename)
        {
            entity.texture = library.textures[i].id;
            found = true;
            break;
        }
    }
    if (!found)
    {
        cout << "Failed to assign texture: " << filename << endl;
    }
}

GLuint get_texture (const char* filename)
{
    GLuint texture_id = 0;
    b4 found = false;
    for (u4 i = 0; i < library.texture_count; i++)
    {
        if (library.textures[i].name == filename)
        {
            texture_id = library.textures[i].id;
            found = true;
            break;
        }
    }
    if (!found)
    {
        cout << "Failed to get texture: " << filename << endl;
    }
    return texture_id;
}

void assign_mesh (Entity &entity, const char* filename)
{
    b4 found = false;
    for (u4 i = 0; i < library.mesh_count; i++)
    {
        if (library.meshes[i].name == filename)
        {
            entity.mesh = i;
            found = true;
            break;
        }
    }
    if (!found)
    {
        cout << "Failed to assign mesh: " << filename << endl;
    }
}

void load_mesh (const char * filename)
{
    /*
        add to mesh list
    */
    u4 c = library.mesh_count;
    library.meshes[c].name = filename;

    if (!loadOBJ(filename, library.meshes[c].vertices, library.meshes[c].uvs, library.meshes[c].normals))
    {
        cout << "Failed  to load mesh: " << filename << endl;
        return;
    }
    glGenBuffers(1, &library.meshes[c].vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, library.meshes[c].vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, 
        library.meshes[c].vertices.size() * sizeof(glm::vec3), 
        &library.meshes[c].vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &library.meshes[c].uv_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, library.meshes[c].uv_buffer);
    glBufferData(GL_ARRAY_BUFFER, 
        library.meshes[c].uvs.size() * sizeof(glm::vec2), 
        &library.meshes[c].uvs[0], GL_STATIC_DRAW);

    library.mesh_count++;
};

glm::vec3 glmv(vec3 v) {
    return glm::vec3(v.x,v.y,v.z);
}