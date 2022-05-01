#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	///Plane class
	class Plane : public StaticActor
	{
	public:
		//A plane with default paramters: XZ plane centred at (0,0,0)
		Plane (PxVec3 normal = PxVec3 (0.f, 1.f, 0.f), PxReal distance = 0.f);
	};

	///Sphere class
	class Sphere : public DynamicActor
	{
	public:
		//a sphere with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m
		// - denisty: 1kg/m^3
		Sphere (const PxTransform& pose = PxTransform (PxIdentity), PxReal radius = 1.f, PxReal density = 1.f);
	};

	///Box class
	class Box : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Box (const PxTransform& pose = PxTransform (PxIdentity), PxVec3 dimensions = PxVec3 (.5f, .5f, .5f), PxReal density = 1.f);
	};

	//Platform class (basically just a static box)
	class Platform : public StaticActor {
	public:
		Platform (const PxTransform& pose = PxTransform (PxIdentity), PxVec3 dimensions = PxVec3 (.5f, .5f, .5f));
	};

	class Capsule : public DynamicActor
	{
	public:
		Capsule (const PxTransform& pose = PxTransform (PxIdentity), PxVec2 dimensions = PxVec2 (1.f, 1.f), PxReal density = 1.f);
	};

	///The ConvexMesh class
	class ConvexMesh : public DynamicActor
	{
	public:
		//constructor
		ConvexMesh (const std::vector<PxVec3>& verts, const PxTransform& pose = PxTransform (PxIdentity), PxReal density = 1.f);

		//mesh cooking (preparation)
		PxConvexMesh* CookMesh (const PxConvexMeshDesc& mesh_desc);
	};

	///The TriangleMesh class
	class TriangleMesh : public StaticActor
	{
	public:
		//constructor
		TriangleMesh (const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose = PxTransform (PxIdentity));

		//mesh cooking (preparation)
		PxTriangleMesh* CookMesh (const PxTriangleMeshDesc& mesh_desc);
	};

	//Distance joint with the springs switched on
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint (Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1);

		void Stiffness (PxReal value);

		PxReal Stiffness ();

		void Damping (PxReal value);

		PxReal Damping ();
	};

	///Revolute Joint
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint (Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1);

		void DriveVelocity (PxReal value);

		PxReal DriveVelocity ();

		void SetLimits (PxReal lower, PxReal upper);
	};

	class Cloth : public Actor
	{
		PxClothMeshDesc mesh_desc;

	public:
		//constructor
		Cloth (PxTransform pose = PxTransform (PxIdentity), const PxVec2& size = PxVec2 (1.f, 1.f), PxU32 width = 1, PxU32 height = 1, bool fix_top = true);

		~Cloth ();
	};

	class Domino : public DynamicActor {
	public:
		/// <summary>
		/// Constructor for a domino with accurate default real life dimensions
		/// </summary>
		/// <param name="pose"></param>
		/// <param name="dimensions">Defaults to 48mm x 24mm x 7.5mm</param>
		/// <param name="density">Defaults to 1144.781kg/m^3</param>
		Domino (const PxTransform& pose = PxTransform (PxIdentity), PxVec3 dimensions = PxVec3 (.24f, .12f, 0.0375f), PxReal density = 1144.781f);
	};

	class Balancer : public DynamicActor {
	private:
		Box* body;
		RevoluteJoint* joint;
	public:
		Balancer (const PxTransform& pose = PxTransform (PxIdentity), PxVec3 dimensions = PxVec3 (1.f, 1.f, 1.f), PxReal density = 1.f, PxReal thickness = .1f);

		void AddToScene (Scene* scene);
	};

	class WindMill : public DynamicActor {
	private:
		Box* blades[4];
		RevoluteJoint* joint;
	public:
		WindMill (const PxTransform& pose = PxTransform (PxIdentity), PxReal bladeSpan = 3.f, PxReal density = 1.f);

		void AddToScene (Scene* scene);
	};

	class NewtonCradle : public DynamicActor {
	private:
		//vector<RevoluteJoint*> joints;
		vector<DistanceJoint*> joints;
		vector<Sphere*> balls;

	public:
		NewtonCradle (const PxTransform& pose = PxTransform (PxIdentity), PxReal ballRadius = 0.2f, PxU32 ballCount = 5, PxReal density = 656.52f);
		~NewtonCradle ();

		void AddToScene (Scene* scene);

		void SetMaterial (PxMaterial* material);
	};

	class MetalBall : public DynamicActor {
	public:
		MetalBall (const PxTransform& pose = PxTransform (PxIdentity), PxReal radius = .2f, PxReal density = 656.52f);
	};
}