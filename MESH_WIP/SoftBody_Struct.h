#ifndef _SOFTBODY_STRUCT_H
#define _SOFTBODY_STRUCT_H


#include "GLIncludes.h"


//A struct for 1D Mass-Spring softbody physics
struct SoftBody
{
	int subdivisionsX;
	int subdivisionsY;
	int subdivisionsZ;

	float restHeight;
	float restWidth;
	float restDepth;

	//The rigidbodies of the point masses which make up the softbody mass-spring system
	unsigned int numRigidBodies;
	struct RigidBody*** bodies;

	float coefficient;	//The spring coefficients between the point masses in the system
						//float restLength;	//The resting length of the springs
	float dampening;	//The dampening coefficient of the springs

	SoftBody::SoftBody()
	{
		numRigidBodies = 0;
		coefficient = 0.0f;
		//restLength = 0.0f;
		dampening = 0.0f;

		subdivisionsX = subdivisionsY = 0;
		restHeight = restWidth = 0;
	}

	SoftBody::SoftBody(
		float width, float height, float depth,
		int subX, int subY, int subZ,
		float coeff, float damp
	)
	{
		subdivisionsX = subX;
		subdivisionsY = subY;
		subdivisionsZ = subZ;

		numRigidBodies = subX * subY * subZ;
		coefficient = coeff;
		//restLength = rest;
		dampening = damp;

		float startWidth = -width / 2.0f;
		float widthStep = width / subdivisionsX;

		restWidth = widthStep;

		float startHeight = -height / 2.0f;
		float heightStep = height / subdivisionsY;

		restHeight = heightStep;

		float startDepth = -depth / 2.0f;
		float depthStep = depth / subdivisionsZ;
		restDepth = depthStep;
		bodies = new struct RigidBody**[subZ];
		for (int i = 0; i < subdivisionsZ; i++)
		{
			bodies[i] = new struct RigidBody*[subY];
			for (int j = 0; j < subdivisionsY; ++j)
			{
				bodies[i][j] = new struct RigidBody[subX];
				for (int k = 0; k < subdivisionsX; ++k)
				{
					bodies[i][j][k] = struct RigidBody(glm::vec3(startWidth + widthStep * k, startHeight + heightStep * j, startDepth + depthStep * i), glm::vec3(0.0f), glm::vec3(0.0f), 1.0f);
				}
			}
		}


	}

	SoftBody::~SoftBody()
	{
		for (int i = 0; i < subdivisionsZ; ++i)
		{
			for (int j = 0; j < subdivisionsY; ++j)
			{
				delete[] bodies[i][j];
			}
			delete[] bodies[i];

		}
		delete[] bodies;
	}
};
#endif _SOFTBODY_STRUCT_H