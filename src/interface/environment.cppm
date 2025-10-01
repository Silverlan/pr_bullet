// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <unordered_set>
#include <queue>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <sharedutils/util_hash.hpp>
#include "mathutil/transform.hpp"
#include "mathutil/color.h"
#include "sharedutils/util_shared_handle.hpp"
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodySolvers.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>

export module pragma.modules.bullet:environment;

export import :collision_object;
export import :overlap_filter_callback;
export import pragma.shared;

#define PHYS_WORLD_TYPE_DISCRETE_DYNAMICS 0
#define PHYS_WORLD_TYPE_DISCRETE_DYNAMICS_MT 1
#define PHYS_WORLD_SOFT_RIGID_DYNAMICS 2

#define PHYS_WORLD_TYPE PHYS_WORLD_TYPE_DISCRETE_DYNAMICS_MT

export namespace pragma::physics
{
	class BtDebugDrawer;

#if PHYS_WORLD_TYPE == PHYS_WORLD_SOFT_RIGID_DYNAMICS
	using btWorldType = btSoftRigidDynamicsWorld;
#elif PHYS_WORLD_TYPE == PHYS_WORLD_TYPE_DISCRETE_DYNAMICS
	using btWorldType = btDiscreteDynamicsWorld;
#elif PHYS_WORLD_TYPE == PHYS_WORLD_TYPE_DISCRETE_DYNAMICS_MT
	using btWorldType = btDiscreteDynamicsWorldMt;
#endif

	class BtEnvironment
		: public pragma::physics::IEnvironment
	{
	public:
		static const double WORLD_SCALE;
		static const double WORLD_SCALE_SQR; // = WORLD_SCALE^2, required for torque
		static const float CCD_MOTION_THRESHOLD;
		static const float CCD_SWEPT_SPHERE_RADIUS;
	public:
		static void SimulationCallback(btDynamicsWorld *world,btScalar timeStep);
		static umath::Transform CreateTransform(const btTransform &btTransform);
		static btTransform CreateBtTransform(const umath::Transform &btTransform);
		static Vector3 ToPragmaPosition(const btVector3 &pos);
		static btVector3 ToBtPosition(const Vector3 &pos);
		static Vector3 ToPragmaNormal(const btVector3 &n);
		static btVector3 ToBtNormal(const Vector3 &n);
		static double ToPragmaDistance(btScalar d);
		static btScalar ToBtDistance(double d);
		static Color ToPragmaColor(const btVector3 &col);

		BtEnvironment(NetworkState &state);
		virtual ~BtEnvironment() override;

		BtRigidBody &ToBtType(IRigidBody &body);

		virtual float GetWorldScale() const override;

		virtual void StartProfiling() override;
		virtual void EndProfiling() override;

		virtual util::TSharedHandle<IFixedConstraint> CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IBallSocketConstraint> CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB) override;
		virtual util::TSharedHandle<IHingeConstraint> CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis) override;
		virtual util::TSharedHandle<ISliderConstraint> CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IConeTwistConstraint> CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFConstraint> CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFSpringConstraint> CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;

		virtual util::TSharedHandle<IController> CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,float slopeLimitDeg=DEFAULT_CHARACTER_SLOPE_LIMIT,const umath::Transform &startTransform={}) override;
		virtual util::TSharedHandle<IController> CreateBoxController(const Vector3 &halfExtents,float stepHeight,float slopeLimitDeg=DEFAULT_CHARACTER_SLOPE_LIMIT,const umath::Transform &startTransform={}) override;
		virtual util::TSharedHandle<ICollisionObject> CreateCollisionObject(IShape &shape) override;
		virtual util::TSharedHandle<IRigidBody> CreateRigidBody(IShape &shape,bool dynamic=true) override;
		virtual util::TSharedHandle<ISoftBody> CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations) override;
		virtual util::TSharedHandle<IGhostObject> CreateGhostObject(IShape &shape) override;

		virtual std::shared_ptr<IConvexShape> CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateCylinderShape(float radius,float height,const IMaterial &mat) override;
		std::shared_ptr<ICompoundShape> CreateTorusShape(uint32_t subdivisions,double outerRadius,double innerRadius,const IMaterial &mat);
		virtual std::shared_ptr<IConvexShape> CreateSphereShape(float radius,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexHullShape> CreateConvexHullShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ITriangleShape> CreateTriangleShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ICompoundShape> CreateCompoundShape(std::vector<IShape*> &shapes) override;
		virtual std::shared_ptr<IShape> CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,double maxHeight,uint32_t upAxis,const IMaterial &mat) override;
		virtual std::shared_ptr<IMaterial> CreateMaterial(float staticFriction,float dynamicFriction,float restitution) override;
		virtual util::TSharedHandle<ICollisionObject> CreatePlane(const Vector3 &n,float d,const IMaterial &mat) override;
		virtual util::TSharedHandle<IVehicle> CreateVehicle(const VehicleCreateInfo &vhcDesc) override;

		virtual RemainingDeltaTime DoStepSimulation(float timeStep,int maxSubSteps=1,float fixedTimeStep=(1.f /60.f)) override;

		virtual Bool Overlap(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;
		virtual Bool RayCast(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;
		virtual Bool Sweep(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;

		virtual void RemoveConstraint(IConstraint &constraint) override;
		virtual void RemoveCollisionObject(ICollisionObject &obj) override;
		virtual void RemoveController(IController &controller) override;

		const btWorldType *GetWorld() const;
		btWorldType *GetWorld();
		btDefaultCollisionConfiguration *GetBtCollisionConfiguration();
		btCollisionDispatcher *GetBtCollisionDispatcher();
		btBroadphaseInterface *GetBtOverlappingPairCache();
		btSequentialImpulseConstraintSolver *GetBtConstraintSolver();
		btSoftBodyWorldInfo *GetBtSoftBodyWorldInfo();
		btSoftBodySolver *GetSoftBodySolver();
		const btSoftBodySolver *GetSoftBodySolver() const;
		
		// For internal or debugging purposes only!
		util::TSharedHandle<IFixedConstraint> AddFixedConstraint(std::unique_ptr<btFixedConstraint> c);
		util::TSharedHandle<IBallSocketConstraint> AddBallSocketConstraint(std::unique_ptr<btPoint2PointConstraint> c);
		util::TSharedHandle<IHingeConstraint> AddHingeConstraint(std::unique_ptr<btHingeConstraint> c);
		util::TSharedHandle<ISliderConstraint> AddSliderConstraint(std::unique_ptr<btSliderConstraint> c);
		util::TSharedHandle<IConeTwistConstraint> AddConeTwistConstraint(std::unique_ptr<btConeTwistConstraint> c);
		util::TSharedHandle<IDoFConstraint> AddDoFConstraint(std::unique_ptr<btGeneric6DofConstraint> c);
		util::TSharedHandle<IDoFSpringConstraint> AddDoFSpringConstraint(std::unique_ptr<btGeneric6DofSpring2Constraint> c);

		uint64_t GetCurrentSimulationStepIndex() const {return m_curSimStepIndex;}
		void PushEvent(const std::function<void()> &ev) {m_events.push(ev);}
	protected:
		virtual void UpdateSurfaceTypes() override;
		virtual void OnVisualDebuggerChanged(pragma::physics::IVisualDebugger *debugger) override;
		std::unique_ptr<btWorldType> m_btWorld = nullptr;
		std::unique_ptr<btDefaultCollisionConfiguration> m_btCollisionConfiguration = nullptr;
		std::unique_ptr<btCollisionDispatcher> m_btDispatcher = nullptr;
		std::unique_ptr<btBroadphaseInterface> m_btOverlappingPairCache = nullptr;
		std::unique_ptr<PhysOverlapFilterCallback> m_overlapFilterCallback;
		std::unique_ptr<btSequentialImpulseConstraintSolver> m_btSolver = nullptr;
		std::unique_ptr<btGhostPairCallback> m_btGhostPairCallback = nullptr;
#if PHYS_WORLD_TYPE == PHYS_WORLD_SOFT_RIGID_DYNAMICS
		std::unique_ptr<btSoftBodySolver> m_softBodySolver = nullptr;
#elif PHYS_WORLD_TYPE == PHYS_WORLD_TYPE_DISCRETE_DYNAMICS_MT
		std::unique_ptr<btConstraintSolverPoolMt> m_constraintSolverPool = nullptr;
#endif
		std::unique_ptr<btSoftBodyWorldInfo> m_softBodyWorldInfo;
        std::unique_ptr<BtDebugDrawer> m_btDebugDrawer;
		uint64_t m_curSimStepIndex = 0;
		std::queue<std::function<void()>> m_events;

		void AddAction(btActionInterface *action);

		void SimulationCallback(double timeStep);
	};
};
