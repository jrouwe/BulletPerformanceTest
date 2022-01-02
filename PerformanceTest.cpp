// Bullet includes
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"

static_assert(sizeof(btScalar) == sizeof(float), "Assuming single precision numbers");

// STL includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

using namespace std;

// Options
#define RECORD_PER_FRAME

btDefaultCollisionConfiguration* gCollisionConfiguration = nullptr;
btITaskScheduler* gTaskScheduler = nullptr;
btDispatcher* gDispatcher = nullptr;
btBroadphaseInterface* gOverlappingPairCache = nullptr;
btConstraintSolverPoolMt* gSolverPool = nullptr;
btConstraintSolver* gSolver = nullptr;
btDynamicsWorld* gDynamicsWorld = nullptr;
btAlignedObjectArray<btCollisionShape*> gCollisionShapes;
btVector3* gVertices = nullptr;
short* gIndices = nullptr;
btAlignedObjectArray<btRigidBody*> gRigidBodies;

void createTerrain()
{
	const int n = 100;
	const float cell_size = 3.0f;
	const float max_height = 5.0f;
	float center = n * cell_size / 2;

	// Create vertices
	const int num_vertices = (n + 1) * (n + 1);
	gVertices = new btVector3[num_vertices];
	for (int x = 0; x <= n; ++x)
		for (int z = 0; z <= n; ++z)
		{
			float height = sin(float(x) * 50.0f / n) * cos(float(z) * 50.0f / n);
			gVertices[z * (n + 1) + x] = btVector3(cell_size * x, max_height * height, cell_size * z);
		}

	// Create regular grid of triangles
	const int num_triangles = n * n * 2;
	gIndices = new short[num_triangles * 3];
	short *next = gIndices;
	for (int x = 0; x < n; ++x)
		for (int z = 0; z < n; ++z)
		{
			int start = (n + 1) * z + x;

			*next++ = start;
			*next++ = start + n + 1;
			*next++ = start + 1;

			*next++ = start + 1;
			*next++ = start + n + 1;
			*next++ = start + n + 2;
		}

	// Create mesh shape
	btTriangleIndexVertexArray* mesh_interface = new btTriangleIndexVertexArray();	
	btIndexedMesh part;
	part.m_vertexBase = (const unsigned char *)gVertices;
	part.m_vertexStride = sizeof(btVector3);
	part.m_numVertices = num_vertices;
	part.m_triangleIndexBase = (const unsigned char *)gIndices;
	part.m_triangleIndexStride = sizeof(short) * 3;
	part.m_numTriangles = num_triangles;
	part.m_indexType = PHY_SHORT;
	mesh_interface->addIndexedMesh(part, PHY_SHORT);
	btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(mesh_interface, true);
	gCollisionShapes.push_back(shape);

	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(-center, max_height, -center));

	// Create body
	btDefaultMotionState* motion_state = new btDefaultMotionState(trans);
	btRigidBody::btRigidBodyConstructionInfo cinfo(0.0f, motion_state, shape, btVector3(0, 0, 0));
	btRigidBody* body = new btRigidBody(cinfo);
	body->setFriction(btScalar(0.5));
	body->setRestitution(btScalar(0.6));
	gDynamicsWorld->addRigidBody(body);
	gRigidBodies.push_back(body);
}

void createDynamic()
{
	// Create convex mesh
	const btVector3 convex_points[] = { btVector3(0, 1, 0), btVector3(1, 0, 0), btVector3(-1, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, -1) };
	btConvexHullShape* convex_hull_shape = new btConvexHullShape((const btScalar *)convex_points, 5);

	// Create shapes
	btCollisionShape *shapes[] = { 
		new btBoxShape(btVector3(0.5f, 0.75f, 1.0f)),
		new btSphereShape(0.5f),
		new btCapsuleShape(0.5f, 0.75f),
		convex_hull_shape
	};
	const int num_shapes = 4;
	for (int i = 0; i < num_shapes; ++i)
		gCollisionShapes.push_back(shapes[i]);

	// Precalculated masses when density is 1000
	float shape_masses[] = {
		3000.0f,
		523.598816f,
		1701.69604f,
		666.666687f,
	};

	// Inner radius of shape (smallest distance from center of mass to surface)
	float shape_inner_radius[] = {
		0.5f,
		0.5f,
		0.5f,
		0.25f
	};

	// Construct bodies
	for (int x = -10; x <= 10; ++x)
		for (int y = 0; y < num_shapes; ++y)
			for (int z = -10; z <= 10; ++z)
			{
				btCollisionShape *shape = shapes[y];
				float mass = shape_masses[y];
				float inner_radius = shape_inner_radius[y];

				btVector3 local_inertia(0, 0, 0);
				shape->calculateLocalInertia(mass, local_inertia);

				btTransform trans;
				trans.setIdentity();
				trans.setOrigin(btVector3(7.5f * x, 15.0f + 2.0f * y, 7.5f * z));
				btDefaultMotionState* motion_state = new btDefaultMotionState(trans);

				btRigidBody::btRigidBodyConstructionInfo cinfo(mass, motion_state, shape, local_inertia);
				btRigidBody* body = new btRigidBody(cinfo);
				body->setFriction(btScalar(0.5));
				body->setRestitution(btScalar(0.6));
				body->setCcdSweptSphereRadius(inner_radius);
				body->setCcdMotionThreshold(0.75f * inner_radius); // Using same distance as Jolt
				gDynamicsWorld->addRigidBody(body);
				gRigidBodies.push_back(body);
			}
}

void initPhysics(int inNumThreads, bool inCCD)
{
	// Collision configuration
	gCollisionConfiguration = new btDefaultCollisionConfiguration();

	// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	gOverlappingPairCache = new btDbvtBroadphase();

	if (inNumThreads > 1)
	{
		// Create task scheduler
		gTaskScheduler = btCreateDefaultTaskScheduler();
		gTaskScheduler->setNumThreads(inNumThreads);
		btSetTaskScheduler(gTaskScheduler);

		// Collision dispatcher
		gDispatcher = new btCollisionDispatcherMt(gCollisionConfiguration);

		// Create solver pool
		btConstraintSolver *solvers[BT_MAX_THREAD_COUNT];
		for (int i = 0; i < inNumThreads; ++i)
			solvers[i] = new btSequentialImpulseConstraintSolver();
		gSolverPool = new btConstraintSolverPoolMt(solvers, inNumThreads);

		// Create solver
		gSolver = new btSequentialImpulseConstraintSolverMt();

		// Create world
		gDynamicsWorld = new btDiscreteDynamicsWorldMt(gDispatcher, gOverlappingPairCache, gSolverPool, gSolver, gCollisionConfiguration);
	}
	else
	{
		// Collision dispatcher
		gDispatcher = new btCollisionDispatcher(gCollisionConfiguration);

		// Create solver
		gSolver = new btSequentialImpulseConstraintSolver();

		// Create world
		gDynamicsWorld = new btDiscreteDynamicsWorld(gDispatcher, gOverlappingPairCache, gSolver, gCollisionConfiguration);
	}

	// Configure the world
	gDynamicsWorld->setGravity(btVector3(0, -9.81f, 0));
	gDynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD | SOLVER_USE_WARMSTARTING;
	gDynamicsWorld->getSolverInfo().m_numIterations = 10; // Same amount of iterations as Jolt
	gDynamicsWorld->getDispatchInfo().m_useContinuous = inCCD;

	// Create test scene
	createTerrain();
	createDynamic();
}

void stepPhysics()
{
	gDynamicsWorld->stepSimulation(1.0f / 60.0f);
}

void cleanupPhysics()
{
	// Remove the rigidbodies from the dynamics world and delete them
	for (int i = 0; i < gRigidBodies.size(); ++i)
	{
		btRigidBody* body = gRigidBodies[i];
		gDynamicsWorld->removeRigidBody(body);
		delete body->getMotionState();
		delete body;
	}
	gRigidBodies.clear();

	// Delete shapes
	for (int j = 0; j < gCollisionShapes.size(); j++)
		delete gCollisionShapes[j];
	gCollisionShapes.clear();

	// Delete our vertex and index buffer
	delete [] gVertices;
	delete [] gIndices;

	// Delete dynamics world
	delete gDynamicsWorld;
	gDynamicsWorld = nullptr;

	// Delete solver
	delete gSolver;
	gSolver = nullptr;

	// Delete solver pool
	if (gSolverPool != nullptr)
	{
		delete gSolverPool;
		gSolverPool = nullptr;
	}
	
	// Delete broadphase
	delete gOverlappingPairCache;
	gOverlappingPairCache = nullptr;

	// Delete dispatcher
	delete gDispatcher;
	gDispatcher = nullptr;

	// Delete configuration
	delete gCollisionConfiguration;
	gCollisionConfiguration = nullptr;

	// Delete task scheduler
	if (gTaskScheduler != nullptr)
	{
		btSetTaskScheduler(nullptr);
		delete gTaskScheduler;
		gTaskScheduler = nullptr;
	}
}

/// Convert type to string
template<typename T>
string ConvertToString(const T &inValue)
{
    ostringstream oss;
    oss << inValue;
    return oss.str();
}

int main(int argc, char** argv)
{
	// Trace header
	cout << "Motion Quality, Thread Count, Steps / Second" << endl;

	for (int mq = 0; mq < 2; ++mq)
	{
		// Determine motion quality
		bool use_ccd = mq == 1;
		string motion_quality_str = mq == 0? "Discrete" : "CCD";

		for (int num_threads = 1; num_threads <= (int)thread::hardware_concurrency(); ++num_threads)
		{
			// Init physics
			initPhysics(num_threads, use_ccd);

		#ifdef RECORD_PER_FRAME
			// Open per frame timing output
			ofstream per_frame_file;
			per_frame_file.open(("bullet_per_frame_" + motion_quality_str + "_th" + ConvertToString(num_threads) + ".csv").c_str(), ofstream::out | ofstream::trunc);
			per_frame_file << "Frame, Time (ms)" << endl;
		#endif // RECORD_PER_FRAME

			constexpr int cMaxIterations = 500;

			chrono::nanoseconds total_duration(0);

			// Step the world for a fixed amount of iterations
			for (int iterations = 0; iterations < cMaxIterations; ++iterations)
			{
				// Start measuring
				chrono::high_resolution_clock::time_point clock_start = chrono::high_resolution_clock::now();
				
				// Do step
				stepPhysics();
				
				// Stop measuring
				chrono::high_resolution_clock::time_point clock_end = chrono::high_resolution_clock::now();
				chrono::nanoseconds duration = chrono::duration_cast<chrono::nanoseconds>(clock_end - clock_start);
				total_duration += duration;

			#ifdef RECORD_PER_FRAME
				// Record time taken this iteration
				per_frame_file << iterations << ", " << (1.0e-6 * duration.count()) << endl;
			#endif // RECORD_PER_FRAME
			}

			// Trace stat line
			cout << motion_quality_str << ", " << num_threads << ", " << double(cMaxIterations) / (1.0e-9 * total_duration.count()) << endl;

			cleanupPhysics();
		}
	}
}
