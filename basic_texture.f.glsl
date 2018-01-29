varying vec2 UV;
uniform sampler2D tex_source;

void main(void)
{
  gl_FragColor = texture2D( tex_source, UV ).rgba;
}