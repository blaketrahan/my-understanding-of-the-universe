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

	glUniform2f(basic_texture.uniform_tex_offset, 0 , 0);
	glDrawElements(GL_TRIANGLES, size/sizeof(GLushort), GL_UNSIGNED_SHORT, 0);

	glDisableVertexAttribArray(basic_texture.attribute_coord3d);
	glDisableVertexAttribArray(basic_texture.attribute_tex_coord2d);
}


void render_mesh(RENDER_STATE &rs, OBJ obj)
{
    glEnableVertexAttribArray(basic.attribute_coord3d);
    glBindBuffer(GL_ARRAY_BUFFER, obj.verts);
    glVertexAttribPointer( basic.attribute_coord3d, 3, GL_FLOAT, GL_FALSE, 0, 0 );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obj.indices);
    int size;  glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &size);

    glUseProgram(basic.program);
    glUniformMatrix4fv(basic.uniform_view, 1, GL_FALSE, glm::value_ptr(rs.view));
    glUniformMatrix4fv(basic.uniform_proj, 1, GL_FALSE, glm::value_ptr(rs.projection));
    glUniform3f(basic.uniform_scale,0.3f,0.3f,0.3f);

    glm::vec3 translation;
    translation.x = rs.world.x;
    translation.y = rs.world.y;
    translation.z = rs.world.z;
    glm::mat4 model = glm::translate(glm::mat4(1.0f), translation);

    model = glm::rotate(model, rs.rotation.x, glm::vec3(1, 0, 0));
    model = glm::rotate(model, rs.rotation.y, glm::vec3(0, 1, 0));
    model = glm::rotate(model, rs.rotation.z, glm::vec3(0, 0, 1));

    glUniformMatrix4fv(basic.uniform_model, 1, GL_FALSE, glm::value_ptr(model));
    
    
    glUniform3f(basic.uniform_color, 1.0f, 1.0f, 1.0f);

    glUniform1f(basic.uniform_alpha, 1.0f);

    glDrawElements(GL_TRIANGLES, size/sizeof(GLushort), GL_UNSIGNED_SHORT, 0);

    glDisableVertexAttribArray(basic.attribute_coord3d);
}