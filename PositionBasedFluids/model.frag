#version 400 core
in vec3 Position;
in vec3 Normal;
in vec2 TexCoords;

out vec4 color;
//uniform sampler2D texture_diffuse1;

uniform float shininess;
uniform vec4 default_color;
//uniform sampler2D texture_reflection1;
//uniform samplerCube skybox;
//uniform vec3 viewPos;
void main()
{
	//vec4 diffuseColor=texture(texture_diffuse1,TexCoords);
	//vec4 reflectColor=vec4(0.0);
	//vec3 I=normalize(Position-viewPos);
	//vec3 R=reflect(I,normalize(Normal));
	//float reflect_intensity=texture(texture_reflection1,TexCoords).r;
	//if(reflect_intensity>0.1){
		//reflectColor=texture(skybox,R)*reflect_intensity;
	//}
	//color=diffuseColor+reflectColor;
	color=default_color;
	//color=vec4(1.0,0,0,1);
}