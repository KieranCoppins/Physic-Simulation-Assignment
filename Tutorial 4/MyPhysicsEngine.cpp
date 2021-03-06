#include "MyPhysicsEngine.h"

namespace PhysicsEngine
{
	using namespace std;

	bool over = false;
	bool particles = false;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = { PxVec3 (46.f / 255.f,9.f / 255.f,39.f / 255.f),PxVec3 (217.f / 255.f,0.f / 255.f,0.f / 255.f),
		PxVec3 (255.f / 255.f,45.f / 255.f,0.f / 255.f),PxVec3 (255.f / 255.f,140.f / 255.f,54.f / 255.f),PxVec3 (4.f / 255.f,117.f / 255.f,111.f / 255.f) };

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0 = (1 << 0),
			ACTOR1 = (1 << 1),
			ACTOR2 = (1 << 2)
			//add more if you need
		};
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader (PxFilterObjectAttributes attributes0, PxFilterData filterData0,
											 PxFilterObjectAttributes attributes1, PxFilterData filterData1,
											 PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		// let triggers through
		if (PxFilterObjectIsTrigger (attributes0) || PxFilterObjectIsTrigger (attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags ();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;


		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags ();
	};

	void MySimulationEventCallback::onTrigger(PxTriggerPair* pairs, PxU32 count)
	{
		//you can read the trigger information here
		for (PxU32 i = 0; i < count; i++)
		{
			//filter out contact with the planes
			if (pairs[i].otherShape->getGeometryType () != PxGeometryType::ePLANE)
			{
				//check if eNOTIFY_TOUCH_FOUND trigger
				if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					//Check if the geometry is of a box type - chances are this is the the domino
					if (pairs[i].otherShape->getGeometryType () == PxGeometryType::eBOX) {
						//If we have triggered a domino, display the domino show is over
						if (!over) {
							cout << "Domino show is over!" << endl;

							//TODO PUT SOME KIND OF VISUAL THAT THE DOMINO SHOW IS OVER

							over = true;
						}

					}
					cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
					trigger = true;
				}
				//check if eNOTIFY_TOUCH_LOST trigger
				if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
					trigger = false;
				}
			}
		}
	}

	void MySimulationEventCallback::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
	{
		cerr << "Contact found between " << pairHeader.actors[0]->getName () << " " << pairHeader.actors[1]->getName () << endl;

		//check all pairs
		for (PxU32 i = 0; i < nbPairs; i++)
		{
			//check eNOTIFY_TOUCH_FOUND
			if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
			{
				cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
			}
			//check eNOTIFY_TOUCH_LOST
			if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
			{
				cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
			}
		}
	}

	void MyScene::SetVisualisation()
	{
		px_scene->setVisualizationParameter (PxVisualizationParameter::eSCALE, 1.0f);
		px_scene->setVisualizationParameter (PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

		//cloth visualisation
		px_scene->setVisualizationParameter (PxVisualizationParameter::eCLOTH_HORIZONTAL, 1.0f);
		px_scene->setVisualizationParameter (PxVisualizationParameter::eCLOTH_VERTICAL, 1.0f);
		px_scene->setVisualizationParameter (PxVisualizationParameter::eCLOTH_BENDING, 1.0f);
		px_scene->setVisualizationParameter (PxVisualizationParameter::eCLOTH_SHEARING, 1.0f);

		px_scene->setVisualizationParameter (PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.f);
		px_scene->setVisualizationParameter (PxVisualizationParameter::eJOINT_LIMITS, 1.f);
	}

	void MyScene::CustomInit()
	{
		SetVisualisation ();

		GetMaterial ()->setDynamicFriction (.2f);

		///Initialise and set the customised event callback
		my_callback = new MySimulationEventCallback ();
		px_scene->setSimulationEventCallback (my_callback);

		//Create Materials in enum order
		CreateMaterial (0.1f, 0.02f);			//ICE
		CreateMaterial (1.1f, 1.35f);			//METAL
		CreateMaterial (1.1f, 1.35f, .8f);		//METALBALL
		CreateMaterial (0.5f, 0.5f, .02f);		//WOOD
		CreateMaterial (0.3f, 0.3f, .02f);		//DOMINO


		plane = new Plane ();
		plane->Color (PxVec3 (210.f / 255.f, 210.f / 255.f, 210.f / 255.f));
		Add (plane);

		/*
		cloth = new Cloth (PxTransform (PxVec3 (-4.f, 9.f, 0.f)), PxVec2 (8.f, 8.f), 40, 40);
		cloth->Color (color_palette[2]);
		Add (cloth);

		box = new Box (PxTransform (PxVec3 (0.f, 2.f, 0.f)), PxVec3 (2.f, 2.f, 2.f));
		box->Color (color_palette[3]);
		Add (box);
		*/

		/*
		//Place a line of dominoes
		const int dominoesNum = 50;
		PxReal dominoSpacing = .5f;
		PxTransform start = PxTransform (PxVec3 (10.f, 0.f, 0.f),
										PxQuat (90.f * (PxPi / 180.f), PxVec3 (0.f, 0.f, 1.f)));
		PxTransform lastPoint = Line (50, PxVec3 (0.f, 0.f, -1.f), dominoSpacing, start);
		lastPoint.p += PxVec3 (0.f, 0.f, -1.f) * dominoSpacing;
		lastPoint = DrawBox (50, 20, -dominoSpacing, lastPoint, true);
		lastPoint = Line (20, PxVec3 (-0.5, 0.f, -0.5f), dominoSpacing, lastPoint);


		ramp = new Platform (PxTransform (PxVec3 (10.f, 2.f, 5.f), PxQuat (60.f * (PxPi / 180.f), PxVec3 (1.f, 0.f, 0.f))), PxVec3(2.f, 3.f, 0.1f));
		Add (ramp);

		ball = new Sphere (PxTransform(PxVec3(10.f, 5.f, 5.f), PxQuat(PxIdentity)),.25f, 5000.f);
		Add (ball);

		*/

		platform1 = new Platform (PxTransform (PxVec3 (0.f, 0.f, 0.f), PxQuat(PxIdentity)), PxVec3 (10.f, 0.1f, 10.f));
		platform1->Color (PxVec3 (.0f, .0f, .0f));
		platform1->Material (GetMaterial (Materials::METAL));
		Add (platform1);

		Platform* platform2 = new Platform (PxTransform (PxVec3 (20.f, 0.f, 0.f), PxQuat (PxIdentity)), PxVec3 (10.f, 0.1f, 10.f));
		platform2->Material (GetMaterial (Materials::ICE));
		platform2->Color (PxVec3 (.0f, .0f, .0f));
		Add (platform2);

		Platform* platform3 = new Platform (PxTransform (PxVec3 (20.f, 0.f, -20.f), PxQuat (PxIdentity)), PxVec3 (10.f, 0.1f, 10.f));
		platform3->Material (GetMaterial (Materials::WOOD));
		platform3->Color (PxVec3 (.0f, .0f, .0f));
		Add (platform3);

		Platform* platform4 = new Platform (PxTransform (PxVec3 (0.f, 0.f, -20.f), PxQuat (PxIdentity)), PxVec3 (10.f, 0.1f, 10.f));
		platform4->Material (GetMaterial (Materials::ICE));
		platform4->Color (PxVec3 (.0f, .0f, .0f));
		Add (platform4);

		PxReal dominoSpacing = .5f;

		//Creates a see saw
		//seesaw = new Balancer (PxTransform (PxVec3 (0.f, 0.f, 0.f), PxQuat(90.0f * (PxPi/180), PxVec3 (0.f, 1.f, 0.f))), PxVec3 (2.f, 1.f, 5.f));
		//seesaw->AddToScene (this);

		PxTransform start = PxTransform (PxVec3 (2.3f, .12f, -6.f),
										 PxQuat (90.f * (PxPi / 180.f), PxVec3 (0.f, 0.f, 1.f)));

		//PxTransform lastPoint = Line (20, PxVec3 (1.f, 0.f, 0.f), dominoSpacing, start, PxVec3 (1.f, 0.f, 0.f));
		//lastPoint = Line (20, PxVec3 (.5f, 0.f, .5f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));
		//lastPoint = Line (20, PxVec3 (0.f, 0.f, -1.f), dominoSpacing, lastPoint, PxVec3(1.f, 0.f, 0.f));

		//Create Newton Cradle to start the domino falling
		newtonCradle = new NewtonCradle (PxTransform (PxVec3 (0.f, 4.f, -6.f), PxQuat (90.f * (PxPi / 180.f), PxVec3 (0.f, 1.f, 0.f))));
		newtonCradle->SetMaterial (GetMaterial (Materials::METALBALL));
		newtonCradle->AddToScene (this);

		PxTransform lastPoint = Line (10, PxVec3 (1.f, 0.f, 0.f), dominoSpacing, start, PxVec3 (1.f, 0.f, 0.f));
		lastPoint = DrawBend (10, lastPoint, dominoSpacing * 0.6f, PxVec3 (1.f, 0.f, 0.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = Line (5, PxVec3 (0.f, 0.f, 1.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));
		lastPoint = DrawBend (10, lastPoint, dominoSpacing * 0.6f, PxVec3(0.f, 0.f, 1.f), - 90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = Line (15, PxVec3 (-1.f, 0.f, 0.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));
		lastPoint = DrawBend (20, lastPoint, dominoSpacing * 0.6f, PxVec3 (-1.f, 0.f, 0.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = DrawBend (20, lastPoint, dominoSpacing * 0.6f, PxVec3 (0.f, 0.f, -1.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = Line (20, PxVec3 (1.f, 0.f, 0.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));
		//lastPoint = DrawBend (20, lastPoint, dominoSpacing * 0.6f, PxVec3 (1.f, 0.f, 0.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		//lastPoint = Line (30, PxVec3 (0.f, 0.f, 1.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));

		stairs = new Stairs (PxTransform (PxVec3(lastPoint.p.x + dominoSpacing / 2.f, lastPoint.p.y, lastPoint.p.z), PxQuat(0.f * (PxPi/180.f), PxVec3 (0.f, 1.f, 0.f))),
							 12, 0.12f);
		stairs->Material (GetMaterial (Materials::WOOD));
		stairs->AddToScene (this);
		lastPoint = stairs->getEndPoint ();
		Platform* bridge = new Platform (PxTransform (PxVec3 (lastPoint.p.x + 4.f + 0.12f, lastPoint.p.y - 0.24f, lastPoint.p.z), PxQuat (90.f * (PxPi / 180.f), PxVec3 (0.f, 1.f, 0.f))),
										 PxVec3 (stairs->getStairWidth () / 2.f, 0.12f / 2.f, 4.f));
		bridge->Material (GetMaterial (Materials::WOOD));
		Add (bridge);


		cloth = new Cloth (PxTransform (PxVec3 (lastPoint.p.x + 2.f, lastPoint.p.y - 0.30f, lastPoint.p.z), PxQuat(0.f, PxVec3(0.f, 1.f, 0.f))),
						   PxVec2 (5.f, 1.2f), 
						   20, 20);
		((PxCloth*) cloth->Get ())->setSolverFrequency (120.f);
		((PxCloth*) cloth->Get ())->setFrictionCoefficient (.25f);
		Add (cloth);

		lastPoint = Line (16, PxVec3 (1.f, 0.f, 0.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));

		lastPoint.p = PxVec3 (lastPoint.p.x + (12 * 0.24f), lastPoint.p.y - (13 * 0.12f), lastPoint.p.z);

		Stairs* stairs1 = new Stairs (PxTransform (lastPoint.p, PxQuat (180.f * (PxPi / 180.f), PxVec3 (0.f, 1.f, 0.f))),
									  12, 0.12f);
		stairs1->Material (GetMaterial (Materials::WOOD));
		stairs1->AddToScene (this);
		stairs1->DominoMaterial (GetMaterial (Materials::DOMINO));

		lastPoint.p.y += 0.12f;

		lastPoint = Line (8, PxVec3 (1.f, 0.f, 0.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));


		SpinningBlade* sBlade = new SpinningBlade (PxTransform (lastPoint.p + PxVec3 (0.f, .1f, 2.5f)), 8, PxVec3(0.2f, 0.01f, 0.6f), 30.f);
		sBlade->Material (GetMaterial (Materials::METAL));
		sBlade->DriveVelocity (2.f);
		sBlade->AddToScene (this);

		lastPoint = DrawBend (10, lastPoint, dominoSpacing * 0.6f, PxVec3 (1.f, 0.f, 0.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = DrawBend (10, lastPoint, dominoSpacing * 0.6f, PxVec3 (0.f, 0.f, 1.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = Line (16, PxVec3 (-1.f, 0.f, 0.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));
		lastPoint = DrawBend (10, lastPoint, dominoSpacing * 0.6f, PxVec3 (-1.f, 0.f, 0.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = Line (8, PxVec3 (0.f, 0.f, -1.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));
		lastPoint.p.y += 0.12f;
		lastPoint = DrawBend (10, lastPoint, dominoSpacing * 0.6f, PxVec3 (0.f, 0.f, -1.f), 90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint.p.x -= dominoSpacing / 2.f;
		lastPoint.p.y -= 0.24f;
		Stairs* stairs2 = new Stairs (PxTransform (lastPoint.p, PxQuat (180.f * (PxPi / 180.f), PxVec3 (0.f, 1.f, 0.f))),
									  8, 0.12f);
		stairs2->Material (GetMaterial (Materials::WOOD));
		stairs2->DominoMaterial (GetMaterial (Materials::DOMINO));
		stairs2->AddToScene (this);

		lastPoint = stairs2->getEndPoint ();

		lastPoint.p.y -= 8 * 0.12f;

		lastPoint.p.x -= .5f;


		//Draws a spiral out of bends
		PxTransform spiralStart = lastPoint;
		spiralStart.p.z -= .1f;
		spiralStart.p.y += .12f;

		PxVec3 direction = PxVec3 (-1.f, 0.f, 0.f);
		for (int i = 20; i > 0; i--) {
			spiralStart = DrawBend (5 + i, spiralStart, dominoSpacing * 0.6f, direction, -90.f, PxVec3 (0.f, 1.f, 0.f));
			direction = PxQuat (-(PxPi / 2), PxVec3 (0.f, 1.f, 0.f)).rotate (direction);
		}

		lastPoint.p.z += .1f;



		lastPoint = DrawBend (5, lastPoint, dominoSpacing * 0.6f, PxVec3 (-1.f, 0.f, 0.f), 90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = DrawBend (5, lastPoint, dominoSpacing * 0.6f, PxVec3 (0.f, 0.f, 1.f), 90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = DrawBend (5, lastPoint, dominoSpacing * 0.6f, PxVec3 (1.f, 0.f, 0.f), -90.f, PxVec3 (0.f, 1.f, 0.f));
		lastPoint = Line (8, PxVec3 (0.f, 0.f, 1.f), dominoSpacing, lastPoint, PxVec3 (1.f, 0.f, 0.f));

		lastPoint.p.z += dominoSpacing / 2.f;
		lastPoint.p.y -= 0.12f;

		Stairs* stairs3 = new Stairs (PxTransform (lastPoint.p, PxQuat (-90.f * (PxPi / 180.f), PxVec3 (0.f, 1.f, 0.f))),
									  18, 0.12f, PxVec3(0.f, 0.f, 1.f));
		stairs3->Material (GetMaterial (Materials::WOOD));
		stairs3->DominoMaterial (GetMaterial (Materials::DOMINO));
		stairs3->AddToScene (this);

		lastPoint = stairs3->getEndPoint ();
		lastPoint.p.y -= 18 * 0.12f;


		SpinningBlade* sBlade2 = new SpinningBlade (PxTransform (lastPoint.p + PxVec3 (0.f, .1f, 1.6f)), 8, PxVec3 (0.2f, 0.01f, 0.6f), 60.f);
		sBlade2->Material (GetMaterial (Materials::METAL));
		sBlade2->DriveVelocity (10.f);
		sBlade2->AddToScene (this);

		//lastPoint.p.z += .8f;
		lastPoint.p.y += .4f;
		lastPoint.p.x += 10.f;
		lastPoint.p.z += 10.f;

		trigger = new Box (PxTransform (PxVec3 (lastPoint.p)),
						   PxVec3 (.5f, .5f, 10.f));
		trigger->SetTrigger (true);
		trigger->Get ()->setActorFlag (PxActorFlag::eDISABLE_GRAVITY, true);
		Add (trigger);

		//setting custom cloth parameters
		//((PxCloth*)cloth->Get())->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(1.f));
	}

	void MyScene::CustomUpdate () {
		if (over && !particles) {
			SpawnParticles ();
			particles = true;
			Over = true;
		}
	}

	void MyScene::ExampleKeyReleaseHandler (PxVec3 dir, PxVec3 pos) {
		MetalBall* ball = ShootObject<MetalBall> (dir, pos, 30000.f, GetMaterial(Materials::METALBALL));
	}

	void MyScene::ExampleKeyPressHandler () {

	}
	PxParticleSystem* MyScene::getParticleSystem ()
	{
		return ps;
	}
	void MyScene::SpawnParticles ()
	{
		/// ---------------------------------
		/// DEPRECIATED PARTICLE SYSTEM
		/// ---------------------------------
		ps = GetPhysics ()->createParticleSystem (2000, false);
		ps->setActorFlag (PxActorFlag::eVISUALIZATION, true);
		ps->setParticleBaseFlag (PxParticleBaseFlag::eENABLED, true);
		ps->setParticleBaseFlag (PxParticleBaseFlag::eCOLLISION_WITH_DYNAMIC_ACTORS, true);
		ps->setParticleBaseFlag (PxParticleBaseFlag::eGPU, true);
		ps->setParticleMass (.07f);
		ps->setMaxMotionDistance (.6f);
		Get()->addActor (*ps);

		const int numParticles = 2000;
		PxU32 particleIndicies[numParticles];
		PxVec3 particlePositions[numParticles];
		for (int i = 0; i < numParticles; i++) {
			particleIndicies[i] = i;
			particlePositions[i] = PxVec3 (10.f, 10.f, -10.f);
		}

		PxParticleCreationData data;
		data.numParticles = numParticles;
		data.indexBuffer = PxStrideIterator<const PxU32> (particleIndicies);
		data.positionBuffer = PxStrideIterator<const PxVec3> (particlePositions);
		ps->createParticles (data);

		// Scatter particles
		PxVec3 forces[numParticles];
		PxReal particleForce = 50.f;
		for (int i = 0; i < numParticles; i++) {
			float randomX = (rand () % 100 - 50) / 100.f;
			float randomY = (rand () % 100) / 100.f;
			float randomZ = (rand () % 100 - 50) / 100.f;
			float randomForce = (rand () % 100) / 100.f;
			forces[i] = PxVec3 (randomX, randomY, randomZ).getNormalized () * (particleForce * randomForce);
		}

		ps->addForces (numParticles,
					   PxStrideIterator<const PxU32> (particleIndicies),
					   PxStrideIterator<const PxVec3> (forces),
					   PxForceMode::eFORCE);
	}
	/// <summary>
	/// Creates a box of dominoes of width and height and out puts the ending position of the dominoes - VERY LAGGY
	/// </summary>
	/// <param name="width"></param>
	/// <param name="depth"></param>
	/// <param name="spacing"></param>
	/// <param name="center"></param>
	/// <param name="fill"></param>
	/// <returns></returns>
	PxTransform MyScene::DrawBox (int width, int depth, PxReal spacing, PxTransform& start, bool fill)
	{
		if (fill) {
			PxTransform pose;
			for (int x = 0; x < width; x++) {
				for (int z = 0; z < depth; z++) {
					PxQuat rot = start.q;
					PxVec3 pos = PxVec3 (start.p.x + (x * (spacing * 0.6f)), start.p.y, start.p.z + (z * (spacing * 0.5f)));
					rot *= PxQuat (45.0f * (PxPi / 180.f), PxVec3 (1.f, 0.f, 0.f));
					pose = PxTransform (pos, rot);
					Domino* d = new Domino (pose);
					d->Material (GetMaterial (Materials::DOMINO));
					Add (d);
				}
				if (x == width - 1) {
					return pose;

				}
			}
		}
		return PxTransform (PxVec3(0.f, 100.f, 0.f));
	}

	PxTransform MyScene::Line (int length, PxVec3& direction, PxReal spacing, PxTransform& start, PxVec3& colour)
	{
		PxTransform pose;
		for (int i = 0; i < length; i++) {
			pose = start;
			pose.p += (((i + 1) * spacing) * direction);
			float angle = PxVec3 (1.f, 0.f, 0.f).dot (direction) * (PxPi / 2);
			if (direction.z < 0)
				angle *= -1;
			pose.q *= PxQuat (angle, PxVec3 (1.f, 0.f, 0.f));
			Domino* d = new Domino (pose);
			d->Material (GetMaterial (Materials::DOMINO));
			d->Color (colour);
			dominoes.push_back (d);
			Add (d);
		}
		return PxTransform(pose.p, start.q);
	}

	PxTransform MyScene::DrawBend (int length, PxTransform& start, PxReal spacing, PxVec3 dir, PxReal degrees, PxVec3& colour)
	{
		PxTransform pose;
		for (int i = 0; i <= length; i++) {
			PxReal rotationAngle = (((degrees / length) * i) / 2.f);
			pose = start;
			PxVec3 direction = (PxQuat (rotationAngle * (PxPi / 180), PxVec3 (0.f, 1.f, 0.f)).rotate (dir)).getNormalized();
			pose.p += ((i + 1) * spacing) * direction;
			pose.q *= PxQuat ((((rotationAngle * 2) + (90.f * dir.x)) * (PxPi / 180)), PxVec3 (1.f, 0.f, 0.f));
			Domino* d = new Domino (pose);
			d->Material (GetMaterial (Materials::DOMINO));
			d->Color (colour);
			dominoes.push_back (d);
			Add (d);
		}
		return PxTransform (pose.p, start.q);
	}


}