/*
Title: Mass Spring Softbody (3D)
File Name: main.cpp
Copyright © 2015
Original authors: Nicholas Gallagher, Benjamin Evans
Written under the supervision of David I. Schwartz, Ph.D., and
supported by a professional development seed grant from the B. Thomas
Golisano College of Computing & Information Sciences
(https://www.rit.edu/gccis) at the Rochester Institute of Technology.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This is a demonstration of using mass spring systems to simulate soft body physics.
The demo contains a blue structure made of a 6x10x6 grid of masses with springs connecting them.
The top of the structure is pinned in place and the rest is hanging freely.

Each physics timestep the mass spring system is solved to determine the force on each
individual point mass in the system. This is done using Hooke's law. The springs also contain
dampening forces to help relax the system upon purterbation.

The user can apply forces to the bottom edge of the cloth.
Hold the left mouse button to apply a force along the positive X axis.
Hold the right mouse button to apply a force along the negative X axis.
Hold Left shift to switch the axis to the Y axis.
Press the up arrow to increase the rigidness of the structure.
Press the down arrow to decrease the rigidness of the structure.

References:
Game Physics by David Eberly
NGenVS by Nicholas Gallagher
PhysicsTimestep by Brockton Roth
Base by Srinivasan Thiagarajan
*/


#include "GLRender.h"
#include "Structs.h"



// This is a reference to your uniform MVP matrix in your vertex shader




struct Mesh* lattice;

struct SoftBody* body;

double time = 0.0;
double timebase = 0.0;
double accumulator = 0.0;
double physicsStep = 0.012; // This is the number of milliseconds we intend for the physics to update.

#pragma endregion Base_data								  

							// Functions called only once every time the program is executed.

///
//Performs second order euler integration for linear motion
//
//Parameters:
//	dt: The timestep
//	body: The rigidbody being integrated
void IntegrateLinear(float dt, RigidBody &body)
{
	//Calculate the current acceleration
	body.acceleration = body.inverseMass * body.netForce;

	//Calculate new position with
	//	X = X0 + V0*dt + (1/2) * A * dt^2
	glm::vec3 v0dT = dt * body.velocity;						//Solve for the first degree term
	glm::vec3 aT2 = (0.5f) * body.acceleration * powf(dt, 2);	//Solve for the second degree term
	body.position += v0dT + aT2;								//Take sum

																//determine the new velocity
	body.velocity += dt * body.acceleration + body.inverseMass * body.netImpulse;

	//Zero the net impulse and net force!
	body.netForce = body.netImpulse = glm::vec3(0.0f);
}

// This runs once every physics timestep.
void update(float dt)
{

	//This is the external force we will apply based on which keys are pressed.
	glm::vec3 gravity = glm::vec3(0.0f, -1.0f, 0.0f);
	glm::vec3 externalForce = glm::vec3(0.0f, 0.0f, 0.0f);


	bool left = true;
	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
	{
		left = false;
		if (glfwGetMouseButton(window, 0) == GLFW_PRESS)
		{
			externalForce.y = 100.0f;
		}
		if (glfwGetMouseButton(window, 1) == GLFW_PRESS)
		{
			externalForce.y = -100.0f;
		}
	}
	else
	{

		if (glfwGetMouseButton(window, 0) == GLFW_PRESS)
		{
			externalForce.x = 100.0f;
		}
		if (glfwGetMouseButton(window, 1) == GLFW_PRESS)
		{
			externalForce.x = -100.0f;
		}
	}

	glm::vec3 displacement;	//The displacement between nodes
	glm::vec3 direction;	//The direction of the displacement
	float mag;				//The magnitude of the dispplacement

							//Apply forces to each rigidbody making up the softbody
	for (int i = 0; i < body->subdivisionsZ; ++i)
	{
		for (int j = 0; j < body->subdivisionsY; ++j)
		{

			for (int k = 0; k < body->subdivisionsX; ++k)
			{
				glm::vec3 neighborForce(0.0f);

				//If there is a rigidbody behind this one, calculate teh spring force between this rigidbody and the rigidbody behind
				if (i > 0)
				{
					displacement = body->bodies[i - 1][j][k].position - body->bodies[i][j][k].position;
					mag = glm::length(displacement);
					if (mag > FLT_EPSILON)
					{
						direction = glm::normalize(displacement);
					}
					else
					{
						direction = glm::vec3(0.0f);
						mag = 0.0f;
					}
					neighborForce += body->coefficient * (mag - body->restDepth) * direction - body->bodies[i][j][k].velocity * body->dampening;
				}

				//If there is a rigidbody in front of this one, calculate the spring force between this rigidbody and the rigidbody in front
				if (i < body->subdivisionsZ - 1)
				{
					displacement = body->bodies[i + 1][j][k].position - body->bodies[i][j][k].position;
					mag = glm::length(displacement);
					if (mag > FLT_EPSILON)
					{
						direction = glm::normalize(displacement);
					}
					else
					{
						direction = glm::vec3(0.0f);
						mag = 0.0f;
					}
					neighborForce += body->coefficient * (mag - body->restDepth) * direction - body->bodies[i][j][k].velocity * body->dampening;
				}

				//If there is a rigidbody above this one, calculate the spring force between this rigidbody and the rigidbody above
				if (j > 0)
				{
					//Get displacement from rigidBody[i-1] to rigidBody[i]
					displacement = body->bodies[i][j - 1][k].position - body->bodies[i][j][k].position;
					//Extract the direction and the magnitude from this displacement
					mag = glm::length(displacement);
					if (mag > FLT_EPSILON)
					{
						direction = glm::normalize(displacement);
					}
					else
					{
						direction = glm::vec3(0.0f);
						mag = 0.0f;
					}

					//Calculate and Add the applied force according the Hooke's law
					//Fspring = -k(dX)
					//And from that we must add the dampening force:
					//Fdamp = -V * C 
					//Where C is the dampening constant
					neighborForce += body->coefficient * (mag - body->restHeight) * direction - body->bodies[i][j][k].velocity * body->dampening;
				}

				//If there is a rigidbody below this one, calculate the spring force between this rigidbody and the one below
				if (j < body->subdivisionsY - 1)
				{
					displacement = body->bodies[i][j + 1][k].position - body->bodies[i][j][k].position;
					mag = glm::length(displacement);
					if (mag > FLT_EPSILON)
					{
						direction = glm::normalize(displacement);
					}
					else
					{
						direction = glm::vec3(0.0f);
						mag = 0.0f;
					}
					neighborForce += body->coefficient * (mag - body->restHeight) * direction - body->bodies[i][j][k].velocity * body->dampening;
				}

				//If there is a rigidbody left of this one, calculate the spring force between this rigidbody and the one below
				if (k > 0)
				{
					displacement = body->bodies[i][j][k - 1].position - body->bodies[i][j][k].position;
					mag = glm::length(displacement);
					if (mag > FLT_EPSILON)
					{
						direction = glm::normalize(displacement);
					}
					else
					{
						direction = glm::vec3(0.0f);
						mag = 0.0f;
					}
					neighborForce += body->coefficient * (mag - body->restWidth) * direction - body->bodies[i][j][k].velocity * body->dampening;
				}

				//If there is a rigidbody right of this one, calculate the spring force between this rigidbody and the one below
				if (k < body->subdivisionsX - 1)
				{
					displacement = body->bodies[i][j][k + 1].position - body->bodies[i][j][k].position;
					mag = glm::length(displacement);
					if (mag > FLT_EPSILON)
					{
						direction = glm::normalize(displacement);
					}
					else
					{
						direction = glm::vec3(0.0f);
						mag = 0.0f;
					}
					neighborForce += body->coefficient * (mag - body->restWidth) * direction - body->bodies[i][j][k].velocity * body->dampening;
				}

				//If the vertex is on the bottom row, apply the external force
				//if(i == body->subdivisionsZ - 1)

				if (j != body->subdivisionsY - 1)
				{
					body->bodies[i][j][k].netForce += neighborForce + gravity;
					if (j == 0)
					{
						body->bodies[i][j][k].netForce += externalForce;
					}

				}
			}
		}
	}


	//Update the rigidbodies & mesh
	//For each rigidbody
	for (int i = 0; i < body->subdivisionsZ; ++i)
	{
		for (int j = 0; j < body->subdivisionsY; ++j)
		{
			for (int k = 0; k < body->subdivisionsX; k++)
			{
				int numVertex = i * body->subdivisionsY * body->subdivisionsX + j * body->subdivisionsX + k;
				//Integrate kinematics
				IntegrateLinear(dt, body->bodies[i][j][k]);
				//And change the mesh's vertices to match this rigidbodies position
				lattice->vertices[numVertex].x = body->bodies[i][j][k].position.x;
				lattice->vertices[numVertex].y = body->bodies[i][j][k].position.y;
				lattice->vertices[numVertex].z = body->bodies[i][j][k].position.z;
			}
		}
	}

}

// This runs once every frame to determine the FPS and how often to call update based on the physics step.
void checkTime()
{
	// Get the current time.
	time = glfwGetTime();

	// Get the time since we last ran an update.
	double dt = time - timebase;

	// If more time has passed than our physics timestep.
	if (dt > physicsStep)
	{

		timebase = time; // set new last updated time

						 // Limit dt
		if (dt > 0.25)
		{
			dt = 0.25;
		}
		accumulator += dt;

		// Update physics necessary amount
		while (accumulator >= physicsStep)
		{
			update(physicsStep);
			accumulator -= physicsStep;
		}
	}
}



// This function runs every frame
void renderScene()
{
	// Clear the color buffer and the depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Clear the screen to white
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glLineWidth(1.0f);

	// Tell OpenGL to use the shader program you've created.
	glUseProgram(program);

	//Set hue uniform
	glUniformMatrix4fv(uniHue, 1, GL_FALSE, glm::value_ptr(hue));

	//Refresh the lattice vertices
	lattice->RefreshData();
	// Draw the Gameobjects
	lattice->Draw();
}

void OnKeyPress(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		if (key == GLFW_KEY_UP)
		{
			body->dampening += 0.1f;
			if (body->dampening > 20.0f) body->dampening = 20.0f;
			printf("\rRigidness:\t%f", body->dampening);
		}
		else if (key == GLFW_KEY_DOWN)
		{
			body->dampening -= 0.1f;
			if (body->dampening < 0.5f) body->dampening = 0.5f;
			printf("\rRigidness:\t%f", body->dampening);
		}
	}
}

#pragma endregion util_Functions



void main()
{
	glfwInit();

	// Create a window
	window = glfwCreateWindow(800, 800, "Mass Spring Softbody (3D)", nullptr, nullptr);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);

	glfwSetKeyCallback(window, OnKeyPress);

	// Initializes most things needed before the main loop
	init();

	const int subX = 6, subY = 10, subZ = 6;

	//Generate the rope mesh
	float latticeArr[subZ * subY * subX * (sizeof(struct Vertex) / sizeof(float))];
	for (int i = 0; i < subZ; i++)
	{
		for (int j = 0; j < subY; j++)
		{
			for (int k = 0; k < subX; k++)
			{
				int offset = sizeof(struct Vertex) / sizeof(float);
				int index = (i * subX * subY + j * subX + k) * offset;
				Vertex* v = (Vertex*)(latticeArr + index);
				v->x = (1.0f / subX) * (float)k - (1.0f / 2.0f);
				v->y = (1.0f / subY) * (float)j - (1.0f / 2.0f);
				v->z = (1.0f / subZ) * (float)i - (1.0f / 2.0f);

				v->r = 0.0f;
				v->g = 1.0f;
				v->b = 1.0f;
				v->a = 1.0f;
			}
		}
	}

	Vertex *latticeVerts = (Vertex*)latticeArr;

	const int numIndices = subX * subY * subZ;
	GLuint latticeElems[numIndices];
	for (int i = 0; i < subZ; i++)
	{
		for (int j = 0; j < subY; j++)
		{
			for (int k = 0; k < subX; k++)
			{
				int index = (i * subX * subY + j * subX + k);
				latticeElems[index] = index;


			}
		}
	}

	//lattice creation
	lattice = new struct Mesh(subX * subY * subZ, latticeVerts, numIndices, latticeElems, GL_LINES);

	//Scale the lattice
	lattice->scale = glm::scale(lattice->scale, glm::vec3(1.0f));

	//Set spring constant, rest length, and dampening constant
	float coeff = 100.0f;
	float damp = 1.0f;

	//Generate the softbody
	body = new SoftBody(1.0f, 1.0f, 1.0f, subX, subY, subZ, coeff, damp);

	//Print controls
	printf("Controls:\nPress and hold the left mouse button to cause a positive constant force\nalong the selected axis.\n");
	printf("Press and hold the right mouse button to cause a negative constant force\nalong the selected axis.\n");
	printf("The selected axis by default is the X axis\n");
	printf("Hold Left Shift to change the selected axis to the Y axis\n");
	printf("All forces will be applied along the bottom of the structure.\n");
	printf("Press the up arrow to increase rigidness of the structure.\n");
	printf("Press the down arrow to decrease rigidness of the structure.\n");


	// Enter the main loop.
	while (!glfwWindowShouldClose(window))
	{
		//Check time will update the programs clock and determine if & how many times the physics must be updated
		checkTime();

		// Call the render function.
		renderScene();

		// Swaps the back buffer to the front buffer
		// Remember, you're rendering to the back buffer, then once rendering is complete, you're moving the back buffer to the front so it can be displayed.
		glfwSwapBuffers(window);

		// Checks to see if any events are pending and then processes them.
		glfwPollEvents();
	}

	// After the program is over, cleanup your data!
	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);
	glDeleteProgram(program);
	// Note: If at any point you stop using a "program" or shaders, you should free the data up then and there.

	delete lattice;
	delete body;


	// Frees up GLFW memory
	glfwTerminate();
}