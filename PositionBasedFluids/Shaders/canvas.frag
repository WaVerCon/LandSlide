#version 400 core

in vec2 coord;

uniform sampler2D tex;

out vec4 fragColor;

void main() {
	fragColor=texture(tex,coord);
	//fragColor=vec4(vec3(0.5f),1.0f);
}