#pragma once

#include <SDL.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cmath>
#include <Windows.h>
#include "Texture.h"
#include "Vectors.h"
#include "Sky.h"
#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>

#define _USE_MATH_DEFINES
#include <math.h>

//Position of camera, absolute coordiantes are with respect to the centre of the Earth
const SphVec POS = SphVec(6360.1, 0, 0);
//Normalised vector pointing in the direction the camera is looking
const SphVec VIEW = SphVec(1, (M_PI / 2) - (12 * M_PI / 80), 0);//(M_PI / 2) - (10 * M_PI / 80)
//Normalised vector pointing at the sun, we take the sun to be 'very far away' (much, much further than the variations in viewpoints) and so this vector is constant for all viewpoints
SphVec SUN = SphVec(1, ((75 - 50) / (50.0)) * (M_PI / 4) + (3 * M_PI / 8), 0);//(1, (M_PI / 2) - (0 * M_PI / 40), 0);

bool LoadVariables(std::string, Variables & outVariables);
void SetVariableFromData(std::string, std::string, Variables & outVariables);

SDL_Window* gWindow = NULL;
SDL_Renderer* gRenderer = NULL;

Variables Loaded_Variables;

//Wrapper for SDL_texture
Texture Image;
int RENDER_WIDTH;
int RENDER_HEIGHT;

//For waking display update upon completeing frame render
std::condition_variable frame_ready;
std::mutex frame_wait_mutex;
std::atomic<int> frames_rendered;

const std::string VariablesFileName = "Variables.txt";

bool LoadVariables(std::string filename, Variables & outVariables)
{
	std::ifstream File(filename);
	std::string Line;
	bool success;
	if (File)
	{
		while (getline(File, Line))
		{
			std::istringstream stream(Line);
			std::string name;
			std::string value;

			std::getline(stream, name, '=');
			std::getline(stream, value, ';');
			SetVariableFromData(name, value, outVariables);
		}
		File.close();
		success = true;
	}
	else
	{
		success = false;
	}
	return success;
}
void SetVariableFromData(std::string VariableName, std::string VariableValue, Variables & outVariables)
{
	if (VariableName == "SCREEN_WIDTH")
	{
		outVariables.SCREEN_WIDTH = atoi(VariableValue.c_str());
	}
	else if (VariableName == "SCREEN_HEIGHT")
	{
		outVariables.SCREEN_HEIGHT = atoi(VariableValue.c_str());
	}
	else if (VariableName == "FOV")
	{
		outVariables.FOV = atoi(VariableValue.c_str());
	}
	else if (VariableName == "EARTH_RADIUS")
	{
		outVariables.EARTH_RADIUS = atof(VariableValue.c_str());
	}
	else if (VariableName == "ATMO_RADIUS")
	{
		outVariables.ATMO_RADIUS = atof(VariableValue.c_str());
	}
	else if (VariableName == "SUN_INTENSITY")
	{
		outVariables.SUN_INTENSITY = atof(VariableValue.c_str());
	}
	else if (VariableName == "DIRECT_SUN_INTENSITY")
	{
		outVariables.DIRECT_SUN_INTENSITY = atof(VariableValue.c_str());
	}
	else if (VariableName == "EARTH_RED_RATIO")
	{
		outVariables.EARTH_RED_RATIO = atof(VariableValue.c_str());
	}
	else if (VariableName == "EARTH_BLUE_RATIO")
	{
		outVariables.EARTH_BLUE_RATIO = atof(VariableValue.c_str());
	}
	else if (VariableName == "EARTH_GREEN_RATIO")
	{
		outVariables.EARTH_GREEN_RATIO = atof(VariableValue.c_str());
	}
	else if (VariableName == "MIE_COEFF")
	{
		outVariables.MIE_COEFF = atof(VariableValue.c_str());
	}
	else if (VariableName == "MIE_SCALE_HEIGHT")
	{
		outVariables.MIE_SCALE_HEIGHT = atof(VariableValue.c_str());
	}
	else if (VariableName == "MIE_MEAN_COSINE")
	{
		outVariables.MIE_MEAN_COSINE = atof(VariableValue.c_str());
	}
	else if (VariableName == "CLOUD_MIE_COEFF")
	{
		outVariables.CLOUD_MIE_COEFF = atof(VariableValue.c_str());
	}
	else if (VariableName == "RAYLEIGH_BLUE")
	{
		outVariables.RAYLEIGH_BLUE = atof(VariableValue.c_str());
	}
	else if (VariableName == "RAYLEIGH_RED")
	{
		outVariables.RAYLEIGH_RED = atof(VariableValue.c_str());
	}
	else if (VariableName == "RAYLEIGH_GREEN")
	{
		outVariables.RAYLEIGH_GREEN = atof(VariableValue.c_str());
	}
	else if (VariableName == "RAYLEIGH_SCALE_HEIGHT")
	{
		outVariables.RAYLEIGH_SCALE_HEIGHT = atof(VariableValue.c_str());
	}
	else if (VariableName == "RED_MAP_SCALE")
	{
		outVariables.RED_MAP_SCALE = atof(VariableValue.c_str());
	}
	else if (VariableName == "GREEN_MAP_SCALE")
	{
		outVariables.GREEN_MAP_SCALE = atof(VariableValue.c_str());
	}
	else if (VariableName == "BLUE_MAP_SCALE")
	{
		outVariables.BLUE_MAP_SCALE = atof(VariableValue.c_str());
	}
	else if (VariableName == "INTEGRAL_STEPS")
	{
		outVariables.INTEGRAL_STEPS = atof(VariableValue.c_str());
	}
	else if (VariableName == "NUM_THREADS")
	{
		outVariables.NUM_THREADS = atoi(VariableValue.c_str());
		if (outVariables.NUM_THREADS < 1)
		{
			outVariables.NUM_THREADS = 1;
		}
	}
	else if (VariableName == "RES_SCALING")
	{
		outVariables.RES_SCALING = atoi(VariableValue.c_str());
		if (outVariables.RES_SCALING < 1)
		{
			outVariables.RES_SCALING = 1;
		}
	}
}

//Initialize SDL and texture
bool Init()
{
	bool success;
	Loaded_Variables = Variables();
	if (LoadVariables(VariablesFileName, Loaded_Variables))
	{
		SDL_Init(SDL_INIT_VIDEO);
		RENDER_HEIGHT = Loaded_Variables.SCREEN_HEIGHT * Loaded_Variables.RES_SCALING;
		RENDER_WIDTH = Loaded_Variables.SCREEN_WIDTH * Loaded_Variables.RES_SCALING;

		gWindow = SDL_CreateWindow("Sky Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, RENDER_WIDTH, RENDER_HEIGHT, SDL_WINDOW_SHOWN);
		gRenderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED);
		Image.Create(RENDER_WIDTH, RENDER_HEIGHT, gRenderer, gWindow);
		success = true;
	}
	else
	{
		printf("Failed to load variables!\n");
		success = false;
	}
	return success;
}
//close and free SDL resources
void Close()
{
	Image.Free();

	SDL_DestroyRenderer(gRenderer);
	SDL_DestroyWindow(gWindow);

	gWindow = NULL;
	gRenderer = NULL;

	SDL_Quit();
}

void RenderAllFrames(std::shared_ptr<Pixel>* frames, int num_frames, Simulator* Sim)
{
	for (int i = 0; i < num_frames; i++)
	{
		//A variable sun position based on the sun counter to produce a moving animation
		SUN = SphVec(1, ((i - (num_frames / 2.0)) / (num_frames / 2.0)) * (M_PI / 4) + (3 * M_PI / 8), 0); //((i - (FRAMES / 2.0)) / (FRAMES / 2.0)) * (M_PI / 4) + (3 * M_PI / 8)

		if (frames == nullptr || Sim == nullptr) { break; }
		frames[i] = Sim->ProduceImage(POS, VIEW, SUN);
		{
			std::lock_guard<std::mutex> set_frames_rendered(frame_wait_mutex);
			frames_rendered.store(i + 1);
		}
		frame_ready.notify_all();
	}
}

int main(int argc, char* args[])
{
	//Start up SDL
	if (Init())
	{
		//Event handler
		SDL_Event e;

		//The simulator object itself, it will load the required parameters from the file at VariablesFileName
		Simulator Sim = Simulator(Loaded_Variables);

		//Variable for keeping track of time
		Uint32 lasttime = 0;
		//Number of frames to be rendered and stored for playing continuously
		const int FRAMES = 60;
		//Current frame counter
		int i = 0;
		//Frames per second
		double FPS = 10;
		//Array of Pixel arrays that each store one produced frame
		std::shared_ptr<Pixel> temp[FRAMES];

		//Pointer to the pixel format of the screen
		SDL_PixelFormat * pixform = SDL_GetWindowSurface(gWindow)->format;

		//Set all frames rendering
		std::thread render(RenderAllFrames, temp, FRAMES, &Sim);

		bool quit = false;
		while (!quit)
		{
			//Check if the user has quit
			while (SDL_PollEvent(&e) != 0)
			{
				if (e.type == SDL_QUIT)
				{
					quit = true;
				}
			}
			//Checks if the set frame time has passed since the last frame was produced/displayed
			if (SDL_GetTicks() - lasttime > (1000.0 / FPS))
			{
				if (temp[i].get() == NULL)
				{	//wait for frame i to be rendered
					frame_ready.wait(std::unique_lock<std::mutex>(frame_wait_mutex), [=]()->bool { return frames_rendered.load() >= i + 1; });
				}

				//Set pixels of texture to be displayed, passing array of pixel struct, screen pixel format and the byte width of the texture
				Image.SetPixels(temp[i], pixform, 4 * RENDER_WIDTH);

				//clear, render and show the entire texture
				SDL_RenderClear(gRenderer);
				SDL_RenderCopy(gRenderer, Image.GetTexture(), NULL, NULL);
				SDL_RenderPresent(gRenderer);

				//update time
				lasttime = SDL_GetTicks();

				//increment frame counter and loop round if the sequence has reached the end
				i += 1;
				if (i == FRAMES)
				{
					i = 0;
				}
			}
		}

		//Free resources and close SDL
		Close();
		render.join();
	}
	return 0;
}

//TODO: Separate threads for rendering / displaying to screen.
//TODO: Better sun? Better mapping?
//TODO: Clouds - My own model? Keep reading?