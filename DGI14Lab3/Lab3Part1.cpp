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
		;

	if (keystate[SDLK_RCTRL])
		tilt -= 0.01;
	;

	if (keystate[SDLK_w])
		cameraPos.y -= 0.01;
		;

	if (keystate[SDLK_s])
		cameraPos.y += 0.01;
		;

	if (keystate[SDLK_d])
		cameraPos.x += 0.01;
		;

	if (keystate[SDLK_a])
		cameraPos.x -= 0.01;
		;

	if (keystate[SDLK_e])
		cameraPos.z += 0.01;
		;

	if (keystate[SDLK_q])
		cameraPos.z -= 0.01;
		;

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
		
		//SDL_UpdateRect(screen, 0, 0, 0, 0);
		//SDL_Delay(200);
	}
	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_UpdateRect(screen, 0, 0, 0, 0);
}

void VertexShader(const vec3& v, ivec2& p)
{
	//´p'= (P-C)*R
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
			//vec3 color(1, 1, 1);
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
	//cout << vertexPixels.size() << endl;

	// 1. Find max and min y-value of the polygon
	// and compute the number of rows it occupies.
	int maxY = -numeric_limits<int>::max();
	int minY = +numeric_limits<int>::max();
	int maxIndex;
	int minIndex;
	int mittenIndex;

	for (int i = 0; i < vertexPixels.size(); i++) {
		if (vertexPixels[i].y > maxY)
		{
			maxY = vertexPixels[i].y;
			maxIndex = i;
		}

		if (vertexPixels[i].y < minY)
		{
			minY = vertexPixels[i].y;
			minIndex = i;
		}
	}
	int rows = maxY - minY;


	if (maxIndex == 0 && minIndex == 1)
		mittenIndex = 2;
	else if (maxIndex == 0 && minIndex == 2)
		mittenIndex = 1;
	else if (maxIndex == 1 && minIndex == 0)
		mittenIndex = 2;
	else if (maxIndex == 1 && minIndex == 2)
		mittenIndex = 0;
	else if (maxIndex == 2 && minIndex == 0)
		mittenIndex = 1;
	else if (maxIndex == 2 && minIndex == 1)
		mittenIndex = 2;
	else
		cout << "nu glömde jagnågot fall" << "  Max: " << maxIndex << "  Min: " << maxIndex << "  Mitten: " << maxIndex << endl;


	// 2. Resize leftPixels and rightPixels
	// so that they have an element for each row.

	leftPixels = vector<ivec2>(rows);
	rightPixels = vector<ivec2>(rows);

	Interpolate(vertexPixels[maxIndex], vertexPixels[minIndex], leftPixels);
	if (vertexPixels[maxIndex] != vertexPixels[mittenIndex]){
		Interpolate(vertexPixels[maxIndex], vertexPixels[mittenIndex], rightPixels);
	}
	else{
		Interpolate(vertexPixels[minIndex], vertexPixels[mittenIndex], rightPixels);
	}


	// 3. Initialize the x-	coordinates in leftPixels
	// to some really large value and the x-coordinates
	// in rightPixels to some really small value.
	//for (int i = 0; i < rows; i++)
	//{
		//leftPixels[i].y = minY+i;
		//rightPixels[i].y = minY+i;

		//leftPixels[i].x = +numeric_limits<int>::max();
		//rightPixels[i].x = -numeric_limits<int>::max();
	//}

	// 4. Loop through all edges of	the polygon and use
	// linear interpolation to find the x-coordinate for
	// each row it occupies. Update the corresponding
	// values in rightPixels and leftPixels.

	//if (maxIndex == mittenIndex || maxIndex == mittenIndex || mittenIndex == minIndex) {
		//cout << "nu är ett par index samma" << endl;
		//currentColor = vec3(1, 1, 1);
	//}

	//Interpolate(vertexPixels[maxIndex], vertexPixels[minIndex], leftPixels);

	//vector<ivec2>temp = vector<ivec2>(vertexPixels[maxIndex].y - vertexPixels[mittenIndex].y);
	//Interpolate(vertexPixels[maxIndex], vertexPixels[mittenIndex], temp);

	//vector<ivec2>temp2 = vector<ivec2>(vertexPixels[mittenIndex].y - vertexPixels[minIndex].y);
	//Interpolate(vertexPixels[mittenIndex], vertexPixels[minIndex], temp2);

	//for (int i = 0; i < rows; i++)
	//{
	//	if (i < temp.size())
			//rightPixels[i] = temp[i];
	//	else
	//		rightPixels[i] = temp2[i - temp.size()];
	//}
	//Interpolate(vertexPixels[1], vertexPixels[2], rightPixels);
}

void DrawPolygonRows(vector<ivec2>leftPixels, vector<ivec2>rightPixels){
	for (int i = 0; i < leftPixels.size(); i++) {
		DrawLineSDL(screen, leftPixels[i], rightPixels[i], currentColor);
	}

}