void render_mesh(RENDER_STATE &rs, Library::Mesh &mesh)
{
    /* Bind texture */
    glBindTexture(GL_TEXTURE_2D, rs.texture);

    /* Select program */
    glUseProgram(basic_texture.program);

    /* View matrix */
    glUniformMatrix4fv(basic_texture.uniform_view, 1, GL_FALSE, glm::value_ptr(rs.view));

    /* Projection matrix */
    glUniformMatrix4fv(basic_texture.uniform_proj, 1, GL_FALSE, glm::value_ptr(rs.projection));
    
    /* Translation matrix */
    glm::vec3 translation;
    translation.x = rs.world.x;
    translation.y = rs.world.y;
    translation.z = rs.world.z;
    glm::mat4 model = glm::translate(glm::mat4(1.0f), translation);
    glUniformMatrix4fv(basic_texture.uniform_model, 1, GL_FALSE, glm::value_ptr(model));

    /* Rotation matrix */
    glm::mat4 rotation = glm_matrix(rs.orient);
    glUniformMatrix4fv(basic_texture.uniform_rotation, 1, GL_FALSE, glm::value_ptr(rotation));

    /* Scale vector */
    glUniform3f(basic_texture.uniform_scale,rs.scale.x,rs.scale.y,rs.scale.z);

    /* Vertex buffer */
    glEnableVertexAttribArray(basic_texture.attribute_coord3d);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vertex_buffer);
    glVertexAttribPointer(
            basic_texture.attribute_coord3d, // attribute
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

    /* UV buffer */
    glEnableVertexAttribArray(basic_texture.attribute_tex_coord2d);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.uv_buffer);
        glVertexAttribPointer(
            basic_texture.attribute_tex_coord2d, // attribute
            2,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            0,                                // stride
            (void*)0                          // array buffer offset
        );

    /* Draw */
    glDrawArrays(GL_TRIANGLES, 0, mesh.vertices.size() );

    /* Disable buffers */
    glDisableVertexAttribArray(basic_texture.attribute_coord3d);
    glDisableVertexAttribArray(basic_texture.attribute_tex_coord2d);
}