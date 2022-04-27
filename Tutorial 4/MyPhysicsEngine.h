#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger (PxTriggerPair* pairs, PxU32 count);

		///Method called when the contact by the filter shader is detected.
		virtual void onContact (const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs);

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Cloth* cloth;
		Box* box;
		MySimulationEventCallback* my_callback;

		vector<Domino*> dominoes;

		Sphere* ball;
		Platform* ramp;

		Balancer* seesaw;

		Platform* platform1;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};

		///A custom scene class
		void SetVisualisation ();

		//Custom scene initialisation
		virtual void CustomInit ();

		//Custom udpate function
		virtual void CustomUpdate ();

		/// An example use of key release handling
		void ExampleKeyReleaseHandler ();

		/// An example use of key presse handling
		void ExampleKeyPressHandler ();

	private:
		void Spiral (PxReal distance, const int numTiles, PxTransform &pose);

		PxTransform DrawBox (int width, int height, PxReal spacing, PxTransform& center, bool fill = false);
		PxTransform Line (int length, PxVec3& direction, PxReal spacing, PxTransform& start, PxVec3& colour = PxVec3 (1.f, 1.f, 1.f));
	};

	enum Materials : PxU32 {
		DEFAULT,
		ICE,
		METAL
	};
}
