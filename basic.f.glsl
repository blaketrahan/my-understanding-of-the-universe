varying vec3 f_color;
uniform float in_alpha;

void main(void) {
  gl_FragColor = vec4(f_color.x, f_color.y, f_color.z, in_alpha);
}
