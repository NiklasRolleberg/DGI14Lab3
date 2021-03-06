#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::vec2;
using glm::ivec2;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
vector<Triangle> triangles;

vec3 cameraPos(0, 0,-3.001);
mat3 R; //rotation matrix;
float yaw = 0; // Yaw angle controlling camera rotation around y-axis
float cameraAngle; // angle controlling camera rotation around x-axis
float tilt; // angle controlling camera rotation around z-axis
float focalLength = SCREEN_HEIGHT;
vec3 currentColor;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
void VertexShader(const vec3& v, ivec2& p);
void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result);
void DrawLineSDL(SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(const vector<vec3>& vertices);
void RotateVec(vec3& in);

void ComputePolygonRows(const vector<ivec2>& vertexPixels,
	vector<ivec2>& leftPixels,
	vector<ivec2>& rightPixels);
void DrawRows(const vector<ivec2>& leftPixels,const vector<ivec2>& rightPixels);
void DrawPolygon(const vector<vec3>& vertices);
void DrawPolygonRows(vector<ivec2>leftPixels, vector<ivec2>rightPixels);

int main(int argc, char* argv[])
{
	LoadTestModel(triangles);
	screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);
	t = SDL_GetTicks();	// Set start value for timer.


	//////////////////////////////////////////////
	/*
	vector<ivec2> vertexPixels(3);
	vertexPixels[0] = ivec2(10, 5);
	vertexPixels[1] = ivec2(5, 10);
	vertexPixels[2] = ivec2(15, 15);
	vector<ivec2> leftPixels;
	vector<ivec2> rightPixels;
	ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
	for (int row = 0; row<leftPixels.size(); ++row)
	{
		cout << "Start: ("
			<< leftPixels[row].x << ","
			<< leftPixels[row].y << "). "
			<< "End: ("
			<< rightPixels[row].x << ","
			<< rightPixels[row].y << "). " << endl;
	}

	//Draw();
	//SDL_Delay(100000);
	*/
	//////////////////////////////////////////////
	while (NoQuitMessageSDL())
	{
		Update();
		Draw();
	}

	//SDL_SaveBMP(screen, "screenshot.bmp");
	return 0;
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2 - t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

	Uint8* keystate = SDL_GetKeyState(0);

	if (keystate[SDLK_UP])
		yaw+=0.01;

	if (keystate[SDLK_DOWN])
		yaw -= 0.01;

	if (keystate[SDLK_RIGHT])
		cameraAngle += 0.01;;

	if (keystate[SDLK_LEFT])
		cameraAngle -= 0.01;;

	if (keystate[SDLK_RSHIFT])
		tilt += 0.01;
		
	if (keystate[SDLK_RCTRL])
		tilt -= 0.01;
	
	if (keystate[SDLK_w])
		cameraPos.y -= 0.01;
		
	if (keystate[SDLK_s])
		cameraPos.y += 0.01;
		
	if (keystate[SDLK_d])
		cameraPos.x += 0.01;
		
	if (keystate[SDLK_a])
		cameraPos.x -= 0.01;
		
	if (keystate[SDLK_e])
		cameraPos.z += 0.01;
		
	if (keystate[SDLK_q])
		cameraPos.z -= 0.01;
	
	int dx;
	int dy;
	SDL_GetRelativeMouseState(&dx, &dy);

	//SDL_PumpEvents();
	if (SDL_GetMouseState(NULL, NULL)&SDL_BUTTON(1)) 
	{
		cameraAngle -= (float)dx / 100;
		yaw += (float)dy / 100;
		/*
		vec3 v(dx, dy, 0);
		// Rotate around z axis
		vec3 row1 = vec3(cos(tilt), -sin(tilt), 0);
		vec3 row2 = vec3(sin(tilt), cos(tilt), 0);
		//vec3 row3 = vec3(0, 0, 1);

		float x = glm::dot(row1, v);
		float y = glm::dot(row2, v);
		//float z = glm::dot(row3, v);

		cameraAngle -= (float)x / 100;
		yaw += (float)y / 100;
		*/
	}
		
}

void Draw()
{
	SDL_FillRect(screen, 0, 0);
	if (SDL_MUSTLOCK(screen))
		SDL_LockSurface(screen);
	for (int i = 0; i<triangles.size(); ++i)
	{
		vector<vec3> vertices(3);
		vertices[0] = triangles[i].v0;
		vertices[1] = triangles[i].v1;
		vertices[2] = triangles[i].v2;

		RotateVec(vertices[0]);
		RotateVec(vertices[1]);
		RotateVec(vertices[2]);

		currentColor = triangles[i].color;
		//DrawPolygonEdges(vertices);
		DrawPolygon(vertices);
	}
	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_UpdateRect(screen, 0, 0, 0, 0);
}

void VertexShader(const vec3& v, ivec2& p)
{
	//�p'= (P-C)*R
	vec3 c = (v - cameraPos);//*R;
	p.x = focalLength*(c.x / c.z) + (SCREEN_WIDTH / 2);
	p.y = focalLength*(c.y / c.z) + (SCREEN_HEIGHT / 2);
}

void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result)
{
	int N = result.size();
	vec2 step = vec2(b-a) / float(glm::max(N-1, 1));
	vec2 current(a);
	for (int i = 0; i<N; ++i)
	{
		result[i] = current;
		current += step;
	}
}

void DrawLineSDL(SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color)
{
	ivec2 delta = glm::abs(a-b);
	int pixels = glm::max(delta.x, delta.y) + 1;

	vector<ivec2> line(pixels);
	Interpolate(a, b, line);

	for (int i = 0; i < line.size(); i++) {
		PutPixelSDL(screen, line[i].x, line[i].y, color);
	}
}

void DrawPolygonEdges(const vector<vec3>& vertices)
{
	int V = vertices.size();
	// Transform each vertex from 3D world position to 2D image position :
	vector<ivec2> projectedVertices(V);
	for (int i = 0; i<V; ++i)
	{
		VertexShader(vertices[i], projectedVertices[i]);
	}
	// Loop over all vertices and draw the edge from it to the next vertex:
	for (int i = 0; i<V; ++i)
	{
		int j = (i + 1) % V; //The next vertex
			DrawLineSDL(screen, projectedVertices[i], projectedVertices[j], currentColor);
	}
}

void RotateVec(vec3& in)
{

	// Rotate around y axis
	vec3 row1(cos(cameraAngle), 0, sin(cameraAngle));
	vec3 row2(0, 1, 0);
	vec3 row3(-sin(cameraAngle), 0, cos(cameraAngle));

	float x = glm::dot(row1, in);
	float y = glm::dot(row2, in);
	float z = glm::dot(row3, in);

	in.x = x;
	in.y = y;
	in.z = z;

	// Rotate around x axis
	row1 = vec3(1, 0, 0);
	row2 = vec3(0, cos(yaw), -sin(yaw));
	row3 = vec3(0, sin(yaw), cos(yaw));

	x = glm::dot(row1, in);
	y = glm::dot(row2, in);
	z = glm::dot(row3, in);

	in.x = x;
	in.y = y;
	in.z = z;

	// Rotate around z axis
	row1 = vec3(cos(tilt), -sin(tilt), 0);
	row2 = vec3(sin(tilt), cos(tilt),0);
	row3 = vec3(0, 0, 1);

	x = glm::dot(row1, in);
	y = glm::dot(row2, in);
	z = glm::dot(row3, in);

	in.x = x;
	in.y = y;
	in.z = z;
}

void DrawPolygon(const vector<vec3>& vertices)
{
	int V = vertices.size();
	vector<ivec2> vertexPixels(V);
	for (int i = 0; i<V; ++i)
		VertexShader(vertices[i], vertexPixels[i]);
	vector<ivec2> leftPixels;
	vector<ivec2> rightPixels;
	ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
	DrawPolygonRows(leftPixels, rightPixels);
}

void ComputePolygonRows(const vector<ivec2>& vertexPixels,
	vector<ivec2>& leftPixels,
	vector<ivec2>& rightPixels)
{
	// 1. Find max and min y-value of the polygon
	// and compute the number of rows it occupies.
	int max = numeric_limits<int>::min();
	int min = numeric_limits<int>::max();
	
	for (ivec2 p : vertexPixels) 
	{
		min = SDL_min(p.y, min);
		max = SDL_max(p.y, max);
	}

	//cout << "Steg1 klart\n" << endl;
	
	// 2. Resize leftPixels and rightPixels
	// so that they have an element for each row.
	leftPixels.resize(max - min + 1);
	rightPixels.resize(max - min + 1);

	//cout << "Steg2 klart! size: " << leftPixels.size() << "\n" << endl;
	
	// 3. Initialize the x-	coordinates in leftPixels
	// to some really large value and the x-coordinates
	// in rightPixels to some really small value.
	for (int i = 0; i < leftPixels.size(); i++)
	{
		leftPixels[i] = ivec2(SCREEN_WIDTH, min + i);
		rightPixels[i] = ivec2(0, min + i);
	}
	
	//cout << "Steg3 klart\n" << endl;
	
	// 4. Loop through all edges of	the polygon and use
	// linear interpolation to find the x-coordinate for
	// each row it occupies. Update the corresponding
	// values in rightPixels and leftPixels.

	//cout << "Steg4 Startar" << endl;
	
	for (int i = 0; i<vertexPixels.size(); i++) 
	{
		ivec2 p1 = vertexPixels[i];
		ivec2 p2 = vertexPixels[(i + 1) % vertexPixels.size()];

		int steps = abs(p1.y-p2.y)+1;
		vector<ivec2> edge = vector<ivec2>(steps);
		Interpolate(p1, p2, edge);

		for (ivec2 p : edge)
		{
			int i = p.y-min;
			if (p.x < leftPixels[i].x)
				leftPixels[i].x = p.x;
			if (p.x > rightPixels[i].x)
				rightPixels[i].x = p.x;
			/*else
			{
				cout << "\nNu blev det fel...\nIndex =" << i << endl;
				cout << "min = " << min << "  max = " << max << endl;
				cout << "p.x = " << p.x << " p.y= " << p.y << endl;
				cout << "p1: (" << p1.x << "," << p1.y << ") p2: (" << p2.x << "," << p2.y << ")" << endl;
				cout << "dx:" << abs(p1.x - p2.x) << " dy:" << abs(p1.y - p2.y) << endl;

				SDL_Delay(20000);
			}*/
		}
	}
	//cout << "Steg4 klart\n" << endl;

}

void DrawPolygonRows(vector<ivec2>leftPixels, vector<ivec2>rightPixels){
	for (int i = 0; i < leftPixels.size(); i++) {
		DrawLineSDL(screen, leftPixels[i], rightPixels[i], currentColor);
	}

}