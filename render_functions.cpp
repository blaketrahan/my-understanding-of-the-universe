void render_plane(RENDER_STATE &rs)
{
	glBindTexture(GL_TEXTURE_2D, rs.texture);

	glUseProgram(basic_texture.program);
	glUniformMatrix4fv(basic_texture.uniform_view, 1, GL_FALSE, glm::value_ptr(rs.view));
	glUniformMatrix4fv(basic_texture.uniform_proj, 1, GL_FALSE, glm::value_ptr(rs.projection));
	glUniform3f(basic_texture.uniform_scale,1.0f,1.0f,1.0f);
	
	glm::vec3 translation;
	translation.x = rs.world.x;
	translation.y = rs.world.y;
	translation.z = rs.world.z;
	glm::mat4 model = glm::translate(glm::mat4(1.0f), translation);

	model = glm::rotate(model, rs.rotation.x, glm::vec3(1, 0, 0));
	model = glm::rotate(model, rs.rotation.y, glm::vec3(0, 1, 0));
	model = glm::rotate(model, rs.rotation.z, glm::vec3(0, 0, 1));

	glUniformMatrix4fv(basic_texture.uniform_model, 1, GL_FALSE, glm::value_ptr(model));

	glUniform3f(basic_texture.uniform_scale,rs.scale.x,rs.scale.y,rs.scale.z);

	glEnableVertexAttribArray(basic_texture.attribute_coord3d);
	glBindBuffer(GL_ARRAY_BUFFER, plane.verts);
	glVertexAttribPointer( basic_texture.attribute_coord3d, 3, GL_FLOAT, GL_FALSE, 0, 0 );

	glEnableVertexAttribArray(basic_texture.attribute_tex_coord2d);
	glBindBuffer(GL_ARRAY_BUFFER, plane.uv_coords);
	glVertexAttribPointer( basic_texture.attribute_tex_coord2d, 2, GL_FLOAT, GL_FALSE, 0, 0 );

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, plane.indices);
	int size; 
	glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &size);

	glDrawElements(GL_TRIANGLES, size/sizeof(GLushort), GL_UNSIGNED_SHORT, 0);

	glDisableVertexAttribArray(basic_texture.attribute_coord3d);
	glDisableVertexAttribArray(basic_texture.attribute_tex_coord2d);
}

void render_mesh(RENDER_STATE &rs)
{
    glBindTexture(GL_TEXTURE_2D, rs.texture);

    glUseProgram(basic_texture.program);
    glUniformMatrix4fv(basic_texture.uniform_view, 1, GL_FALSE, glm::value_ptr(rs.view));
    glUniformMatrix4fv(basic_texture.uniform_proj, 1, GL_FALSE, glm::value_ptr(rs.projection));
    glUniform3f(basic_texture.uniform_scale,1.0f,1.0f,1.0f);
    
    glm::vec3 translation;
    translation.x = rs.world.x;
    translation.y = rs.world.y;
    translation.z = rs.world.z;
    glm::mat4 model = glm::translate(glm::mat4(1.0f), translation);

    model = glm::rotate(model, rs.rotation.x, glm::vec3(1, 0, 0));
    model = glm::rotate(model, rs.rotation.y, glm::vec3(0, 1, 0));
    model = glm::rotate(model, rs.rotation.z, glm::vec3(0, 0, 1));

    glUniformMatrix4fv(basic_texture.uniform_model, 1, GL_FALSE, glm::value_ptr(model));

    glUniform3f(basic_texture.uniform_scale,rs.scale.x,rs.scale.y,rs.scale.z);


    glEnableVertexAttribArray(basic_texture.attribute_coord3d);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(
            basic_texture.attribute_coord3d, // attribute
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
    glEnableVertexAttribArray(basic_texture.attribute_tex_coord2d);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
            basic_texture.attribute_tex_coord2d, // attribute
            2,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            0,                                // stride
            (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, vertices.size() );

    glDisableVertexAttribArray(basic_texture.attribute_coord3d);
    glDisableVertexAttribArray(basic_texture.attribute_tex_coord2d);
}