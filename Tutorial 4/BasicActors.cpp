#include "BasicActors.h"

namespace PhysicsEngine
{
	Plane::Plane(PxVec3 normal, PxReal distance)
		: StaticActor (PxTransformFromPlaneEquation (PxPlane (normal, distance)))
	{
		CreateShape (PxPlaneGeometry ());
	}

	Sphere::Sphere(const PxTransform& pose, PxReal radius, PxReal density)
		: DynamicActor (pose)
	{
		CreateShape (PxSphereGeometry (radius), density);
	}

	Box::Box(const PxTransform& pose, PxVec3 dimensions, PxReal density)
		: DynamicActor (pose)
	{
		CreateShape (PxBoxGeometry (dimensions), density);
	}



	Capsule::Capsule(const PxTransform& pose, PxVec2 dimensions, PxReal density)
		: DynamicActor (pose)
	{
		CreateShape (PxCapsuleGeometry (dimensions.x, dimensions.y), density);
	}

	ConvexMesh::ConvexMesh(const std::vector<PxVec3>& verts, const PxTransform& pose, PxReal density)
		: DynamicActor (pose)
	{
		PxConvexMeshDesc mesh_desc;
		mesh_desc.points.count = (PxU32) verts.size ();
		mesh_desc.points.stride = sizeof (PxVec3);
		mesh_desc.points.data = &verts.front ();
		mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
		mesh_desc.vertexLimit = 256;

		CreateShape (PxConvexMeshGeometry (CookMesh (mesh_desc)), density);
	}

	PxConvexMesh* ConvexMesh::CookMesh(const PxConvexMeshDesc& mesh_desc)
	{
		PxDefaultMemoryOutputStream stream;

		if (!GetCooking ()->cookConvexMesh (mesh_desc, stream))
			throw new Exception ("ConvexMesh::CookMesh, cooking failed.");

		PxDefaultMemoryInputData input (stream.getData (), stream.getSize ());

		return GetPhysics ()->createConvexMesh (input);
	}

	TriangleMesh::TriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose)
		: StaticActor (pose)
	{
		PxTriangleMeshDesc mesh_desc;
		mesh_desc.points.count = (PxU32) verts.size ();
		mesh_desc.points.stride = sizeof (PxVec3);
		mesh_desc.points.data = &verts.front ();
		mesh_desc.triangles.count = (PxU32) trigs.size () / 3;
		mesh_desc.triangles.stride = 3 * sizeof (PxU32);
		mesh_desc.triangles.data = &trigs.front ();

		CreateShape (PxTriangleMeshGeometry (CookMesh (mesh_desc)));
	}

	PxTriangleMesh* TriangleMesh::CookMesh (const PxTriangleMeshDesc& mesh_desc)
	{
		PxDefaultMemoryOutputStream stream;

		if (!GetCooking ()->cookTriangleMesh (mesh_desc, stream))
			throw new Exception ("TriangleMesh::CookMesh, cooking failed.");

		PxDefaultMemoryInputData input (stream.getData (), stream.getSize ());

		return GetPhysics ()->createTriangleMesh (input);
	}

	DistanceJoint::DistanceJoint (Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
	{
		PxRigidActor* px_actor0 = 0;
		if (actor0)
			px_actor0 = (PxRigidActor*) actor0->Get ();

		joint = (PxJoint*) PxDistanceJointCreate (*GetPhysics (), px_actor0, localFrame0, (PxRigidActor*) actor1->Get (), localFrame1);
		joint->setConstraintFlag (PxConstraintFlag::eVISUALIZATION, true);
		((PxDistanceJoint*) joint)->setDistanceJointFlag (PxDistanceJointFlag::eSPRING_ENABLED, true);
		Damping (1.f);
		Stiffness (1.f);
	}

	void DistanceJoint::Stiffness (PxReal value)
	{
		((PxDistanceJoint*) joint)->setStiffness (value);
	}
	
	PxReal DistanceJoint::Stiffness ()
	{
		return ((PxDistanceJoint*) joint)->getStiffness ();
	}

	void DistanceJoint::Damping (PxReal value)
	{
		((PxDistanceJoint*) joint)->setDamping (value);
	}

	PxReal DistanceJoint::Damping ()
	{
		return ((PxDistanceJoint*) joint)->getDamping ();
	}

	RevoluteJoint::RevoluteJoint (Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
	{
		PxRigidActor* px_actor0 = 0;
		if (actor0)
			px_actor0 = (PxRigidActor*) actor0->Get ();

		joint = PxRevoluteJointCreate (*GetPhysics (), px_actor0, localFrame0, (PxRigidActor*) actor1->Get (), localFrame1);
		joint->setConstraintFlag (PxConstraintFlag::eVISUALIZATION, true);
	}

	void RevoluteJoint::DriveVelocity (PxReal value)
	{
		//wake up the attached actors
		PxRigidDynamic* actor_0, * actor_1;
		((PxRevoluteJoint*) joint)->getActors ((PxRigidActor*&) actor_0, (PxRigidActor*&) actor_1);
		if (actor_0)
		{
			if (actor_0->isSleeping ())
				actor_0->wakeUp ();
		}
		if (actor_1)
		{
			if (actor_1->isSleeping ())
				actor_1->wakeUp ();
		}
		((PxRevoluteJoint*) joint)->setDriveVelocity (value);
		((PxRevoluteJoint*) joint)->setRevoluteJointFlag (PxRevoluteJointFlag::eDRIVE_ENABLED, true);
	}

	PxReal RevoluteJoint::DriveVelocity ()
	{
		return ((PxRevoluteJoint*) joint)->getDriveVelocity ();
	}

	void RevoluteJoint::SetLimits (PxReal lower, PxReal upper)
	{
		((PxRevoluteJoint*) joint)->setLimit (PxJointAngularLimitPair (lower, upper));
		((PxRevoluteJoint*) joint)->setRevoluteJointFlag (PxRevoluteJointFlag::eLIMIT_ENABLED, true);
	}

	Cloth::Cloth (PxTransform pose, const PxVec2& size, PxU32 width, PxU32 height, bool fix_top)
	{
		//prepare vertices
		PxReal w_step = size.x / width;
		PxReal h_step = size.y / height;

		PxClothParticle* vertices = new PxClothParticle[(width + 1) * (height + 1) * 4];
		PxU32* quads = new PxU32[width * height * 4];

		for (PxU32 j = 0; j < (height + 1); j++)
		{
			for (PxU32 i = 0; i < (width + 1); i++)
			{
				PxU32 offset = i + j * (width + 1);
				vertices[offset].pos = PxVec3 (w_step * i, 0.f, h_step * j);
				if (fix_top && (j == 0)) //fix the top row of vertices
					vertices[offset].invWeight = 0.f;
				else
					vertices[offset].invWeight = 1.f;
			}

			for (PxU32 j = 0; j < height; j++)
			{
				for (PxU32 i = 0; i < width; i++)
				{
					PxU32 offset = (i + j * width) * 4;
					quads[offset + 0] = (i + 0) + (j + 0) * (width + 1);
					quads[offset + 1] = (i + 1) + (j + 0) * (width + 1);
					quads[offset + 2] = (i + 1) + (j + 1) * (width + 1);
					quads[offset + 3] = (i + 0) + (j + 1) * (width + 1);
				}
			}
		}

		//init cloth mesh description
		mesh_desc.points.data = vertices;
		mesh_desc.points.count = (width + 1) * (height + 1);
		mesh_desc.points.stride = sizeof (PxClothParticle);

		mesh_desc.invMasses.data = &vertices->invWeight;
		mesh_desc.invMasses.count = (width + 1) * (height + 1);
		mesh_desc.invMasses.stride = sizeof (PxClothParticle);

		mesh_desc.quads.data = quads;
		mesh_desc.quads.count = width * height;
		mesh_desc.quads.stride = sizeof (PxU32) * 4;

		//create cloth fabric (cooking)
		PxClothFabric* fabric = PxClothFabricCreate (*GetPhysics (), mesh_desc, PxVec3 (0, -1, 0));

		//create cloth
		actor = (PxActor*) GetPhysics ()->createCloth (pose, *fabric, vertices, PxClothFlags ());
		//collisions with the scene objects
		((PxCloth*) actor)->setClothFlag (PxClothFlag::eSCENE_COLLISION, true);

		colors.push_back (default_color);
		actor->userData = new UserData (&colors.back (), &mesh_desc);
	}

	Cloth::~Cloth()
	{
		delete (UserData*) actor->userData;
	}

	Platform::Platform (const PxTransform& pose, PxVec3 dimensions) : StaticActor (pose)
	{
		CreateShape (PxBoxGeometry (dimensions), 1.f);
	}

	Domino::Domino (const PxTransform& pose, PxVec3 dimensions, PxReal density)
		: DynamicActor(pose)
	{
		CreateShape (PxBoxGeometry (dimensions), density);
	}


	Balancer::Balancer (const PxTransform& pose, PxVec3 dimensions, PxReal density, PxReal thickness) : DynamicActor(pose)
	{
		body = new Box (PxTransform (PxVec3 (pose.p.x, 5.f, pose.p.z), pose.q), PxVec3 (dimensions.x, thickness, dimensions.z));

		joint = new RevoluteJoint (nullptr, PxTransform (PxVec3 (pose.p.x, 1.f, pose.p.z), pose.q), 
								   body, PxTransform (PxVec3 (0.f, -.1f, 0.f)));
		joint->Get ()->setConstraintFlag (PxConstraintFlag::eVISUALIZATION, true);

	}

	void Balancer::AddToScene (Scene* scene)
	{
		scene->Add (body);
	}
	NewtonCradle::NewtonCradle (const PxTransform& pose, PxReal ballRadius, PxU32 ballCount, PxReal density) : DynamicActor(pose)
	{
		for (int i = 0; i < ballCount; i++) {
			//Get the relative position foreach ball in the cradle.
			//Offset the position larger than the contact offset (which defaults to 0.02)
			//This allows for momentum to be calculated some-what accurately between each ball.
			//If the cradle is set to be touching, conservation of momentum isn't calculated correctly.
			PxVec3 relativePos = PxVec3 (pose.p.x, pose.p.y, pose.p.z + (ballRadius * i * 2) + (0.1f) * i) - pose.p;
			//PxVec3 relativePos = PxVec3 (pose.p.x, pose.p.y, pose.p.z + (ballRadius * i * 2)) - pose.p;
			relativePos = pose.q.rotate (relativePos);
			PxVec3 pos = relativePos + pose.p;

			//Create the ball for the cradle with the relative position - 
			//spawn the first ball in an offset so that it  drops to start the cradles reaction
			Sphere* ball;
			if (i == 0) {
				ball = new Sphere (PxTransform (PxVec3(pos.x, pos.y, pos.z) + pose.q.rotate(PxVec3(0.f, ballRadius * 2.f * 10.f, -ballRadius * 2.f * 10.f)), pose.q), ballRadius, density);
			}
			else {
				ball = new Sphere (PxTransform (pos, pose.q), ballRadius, density);
			}

			//Create a joint that connects to the world, I.E is stationary
			//Future improvements could be to create a physically accurate frame.
			/*
			RevoluteJoint* joint;
			if (i == 0) {
				joint = new RevoluteJoint (nullptr,
											PxTransform (PxVec3 (pos.x, pos.y, pos.z), pose.q),
											ball,
											PxTransform (PxVec3 (0.f, -(ballRadius * 2.0f * 6.0f), 0.f)));
			}
			else {
				joint = new RevoluteJoint (nullptr,
										PxTransform (PxVec3 (pos.x, pos.y, pos.z), pose.q),
										ball,
										PxTransform (PxVec3 (0.f, ballRadius * 2.0f * 6.0f, 0.f)));
			}
			*/

			//To recreate similarity to a real world newtons cradle, two lengths of string of equal length
			//are attached to the ball.
			DistanceJoint* leftJoint = new DistanceJoint (nullptr,
														 PxTransform (PxVec3 (pos.x, pos.y, pos.z) + pose.q.rotate(PxVec3 (-2.f, 0.f, 0.f)), pose.q),
														 ball,
														 PxTransform (PxVec3 (0.f, 0.f, 0.f)));

			DistanceJoint* rightJoint = new DistanceJoint (nullptr,
														  PxTransform (PxVec3 (pos.x, pos.y, pos.z) + pose.q.rotate(PxVec3(2.f, 0.f, 0.f)), pose.q),
														  ball,
														  PxTransform (PxVec3 (0.f, 0.f, 0.f)));

			//Allow for a max distance to be set
			((PxDistanceJoint*) leftJoint->Get ())->setDistanceJointFlag (PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
			((PxDistanceJoint*) rightJoint->Get ())->setDistanceJointFlag (PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);

			//Disable the defaulted spring of the distance joint
			((PxDistanceJoint*) rightJoint->Get ())->setDistanceJointFlag (PxDistanceJointFlag::eSPRING_ENABLED, false);
			((PxDistanceJoint*) leftJoint->Get ())->setDistanceJointFlag (PxDistanceJointFlag::eSPRING_ENABLED, false);

			//Set the max distance
			((PxDistanceJoint*) rightJoint->Get ())->setMaxDistance (ballRadius * 2.0f * 10.0f);
			((PxDistanceJoint*) leftJoint->Get ())->setMaxDistance (ballRadius * 2.0f * 10.0f);


			//Enable visualisation of the joints
			leftJoint->Get ()->setConstraintFlag (PxConstraintFlag::eVISUALIZATION, true);
			rightJoint->Get ()->setConstraintFlag (PxConstraintFlag::eVISUALIZATION, true);

			//Save the joints and balls in a vector
			joints.push_back (leftJoint);
			joints.push_back (rightJoint);
			balls.push_back (ball);

		}
	}

	NewtonCradle::~NewtonCradle ()
	{
		for (int i = 0; i < balls.size (); i++) {
			delete joints[i];
			delete balls[i];
		}
	}

	void NewtonCradle::AddToScene (Scene* scene)
	{
		for (int i = 0; i < balls.size (); i++) {
			scene->Add (balls[i]);
		}
	}

	void NewtonCradle::SetMaterial (PxMaterial* material)
	{
		for (int i = 0; i < balls.size (); i++) {
			balls[i]->Material (material);
		}
	}

	MetalBall::MetalBall (const PxTransform& pose, PxReal radius, PxReal density) : DynamicActor(pose)
	{
		CreateShape (PxSphereGeometry (radius), density);
	}


}