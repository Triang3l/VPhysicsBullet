// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_ENVIRONMENT_H
#define PHYSICS_ENVIRONMENT_H

#include "physics_internal.h"
#include "vphysics/friction.h"
#include "vphysics/performance.h"
#include "tier1/utlrbtree.h"
#include "tier1/utlvector.h"

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	CPhysicsEnvironment();
	virtual ~CPhysicsEnvironment();

	// IPhysicsEnvironment methods.

	virtual void SetDebugOverlay(CreateInterfaceFn debugOverlayFactory);
	virtual IVPhysicsDebugOverlay *GetDebugOverlay();

	virtual void SetGravity(const Vector &gravityVector);
	virtual void GetGravity(Vector *pGravityVector) const;

	virtual void SetAirDensity(float density);
	virtual float GetAirDensity() const;

	virtual IPhysicsObject *CreatePolyObject(
			const CPhysCollide *pCollisionModel, int materialIndex,
			const Vector &position, const QAngle &angles, objectparams_t *pParams);
	virtual IPhysicsObject *CreatePolyObjectStatic(
			const CPhysCollide *pCollisionModel, int materialIndex,
			const Vector &position, const QAngle &angles, objectparams_t *pParams);
	virtual IPhysicsObject *CreateSphereObject(float radius, int materialIndex,
			const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic);
	virtual void DestroyObject(IPhysicsObject *pObject);

	/* DUMMY */ virtual IPhysicsFluidController *CreateFluidController(IPhysicsObject *pFluidObject,
			fluidparams_t *pParams);
	/* DUMMY */ virtual void DestroyFluidController(IPhysicsFluidController *pFluid);

	/* DUMMY */ virtual IPhysicsSpring *CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd,
			springparams_t *pParams);
	/* DUMMY */ virtual void DestroySpring(IPhysicsSpring *pSpring);

	/* DUMMY */ virtual IPhysicsConstraint *CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll);
	/* DUMMY */ virtual IPhysicsConstraint *CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge);
	/* DUMMY */ virtual IPhysicsConstraint *CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed);
	/* DUMMY */ virtual IPhysicsConstraint *CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding);
	/* DUMMY */ virtual IPhysicsConstraint *CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket);
	/* DUMMY */ virtual IPhysicsConstraint *CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley);
	/* DUMMY */ virtual IPhysicsConstraint *CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length);
	/* DUMMY */ virtual void DestroyConstraint(IPhysicsConstraint *pConstraint);

	/* DUMMY */ virtual IPhysicsConstraintGroup *CreateConstraintGroup(const constraint_groupparams_t &groupParams);
	/* DUMMY */ virtual void DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup);

	virtual IPhysicsShadowController *CreateShadowController(IPhysicsObject *pObject,
			bool allowTranslation, bool allowRotation);
	virtual void DestroyShadowController(IPhysicsShadowController *pController);
	virtual IPhysicsPlayerController *CreatePlayerController(IPhysicsObject *pObject);
	virtual void DestroyPlayerController(IPhysicsPlayerController *pController);
	virtual IPhysicsMotionController *CreateMotionController(IMotionEvent *pHandler);
	virtual void DestroyMotionController(IPhysicsMotionController *pController);
	/* DUMMY */ virtual IPhysicsVehicleController *CreateVehicleController(IPhysicsObject *pVehicleBodyObject,
			const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);
	/* DUMMY */ virtual void DestroyVehicleController(IPhysicsVehicleController *pController);

	virtual void SetCollisionSolver(IPhysicsCollisionSolver *pSolver);

	virtual void Simulate(float deltaTime);
	virtual bool IsInSimulation() const;
	virtual float GetSimulationTimestep() const;
	virtual void SetSimulationTimestep(float timestep);
	virtual float GetSimulationTime() const;
	virtual void ResetSimulationClock();
	virtual float GetNextFrameTime() const;

	virtual void SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents);
	virtual void SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents);
	/* DUMMY */ virtual void SetConstraintEventHandler(IPhysicsConstraintEvent *pConstraintEvents) {}

	/* DUMMY */ virtual void SetQuickDelete(bool bQuick) {}

	virtual int GetActiveObjectCount() const;
	virtual void GetActiveObjects(IPhysicsObject **pOutputObjectList) const;
	virtual const IPhysicsObject **GetObjectList(int *pOutputObjectCount) const;
	/* DUMMY */ virtual bool TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment) { return false; }
	virtual void CleanupDeleteList();
	virtual void EnableDeleteQueue(bool enable);

	/* DUMMY */ virtual bool Save(const physsaveparams_t &params) { return false; }
	/* DUMMY */ virtual void PreRestore(const physprerestoreparams_t &params) {}
	/* DUMMY */ virtual bool Restore(const physrestoreparams_t &params) { return false; }
	/* DUMMY */ virtual void PostRestore() {}

	virtual bool IsCollisionModelUsed(CPhysCollide *pCollide) const;

	virtual void TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);
	virtual void SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
			const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);

	virtual void GetPerformanceSettings(physics_performanceparams_t *pOutput) const;
	virtual void SetPerformanceSettings(const physics_performanceparams_t *pSettings);

	/* DUMMY */ virtual void ReadStats(physics_stats_t *pOutput);
	/* DUMMY */ virtual void ClearStats() {}

	/* DUMMY */ virtual unsigned int GetObjectSerializeSize(IPhysicsObject *pObject) const { return 0; }
	/* DUMMY */ virtual void SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize) {}
	/* DUMMY */ virtual IPhysicsObject *UnserializeObjectFromBuffer(
			void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions) { return nullptr; }

	/* DUMMY */ virtual void EnableConstraintNotify(bool bEnable) {}

	/* DUMMY */ virtual void DebugCheckContacts() {}

	// Internal methods.

	FORCEINLINE btCollisionDispatcher *GetCollisionDispatcher() const {
		return m_Dispatcher;
	}

	FORCEINLINE const btVector3 &GetBulletGravity() const {
		return m_Gravity;
	}

	void NotifyObjectRemoving(IPhysicsObject *object);

	void NotifyPlayerControllerAttached(IPhysicsPlayerController *controller);
	void NotifyPlayerControllerDetached(IPhysicsPlayerController *controller);

	FORCEINLINE btScalar GetTimeSinceLastPSI() const { return m_TimeSinceLastPSI; }

	void RecheckObjectCollisionFilter(btCollisionObject *object);
	void RemoveObjectCollisionPairs(btCollisionObject *object);

	// Friction snapshot pool - to avoid allocating them within frames.
	IPhysicsFrictionSnapshot *CreateFrictionSnapshot(IPhysicsObject *object);
	void DestroyFrictionSnapshot(IPhysicsFrictionSnapshot *snapshot);

	void NotifyTriggerRemoved(IPhysicsObject *trigger);

	FORCEINLINE btScalar GetMaxSpeed() const {
		return HL2BULLET(m_PerformanceSettings.maxVelocity);
	}
	FORCEINLINE btScalar GetMaxAngularSpeed() const {
		return DEG2RAD(m_PerformanceSettings.maxAngularVelocity);
	}

private:
	btDefaultCollisionConfiguration *m_CollisionConfiguration;
	btCollisionDispatcher *m_Dispatcher;
	btDbvtBroadphase *m_Broadphase;
	btSequentialImpulseConstraintSolver *m_Solver;
	btDiscreteDynamicsWorld *m_DynamicsWorld;

	class DebugDrawer : public btIDebugDraw {
	public:
		DebugDrawer() : m_DebugOverlay(nullptr) {}
		virtual void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color);
		virtual void drawContactPoint(const btVector3 &PointOnB, const btVector3 &normalOnB,
				btScalar distance, int lifeTime, const btVector3 &color);
		virtual void reportErrorWarning(const char *warningString);
		virtual void draw3dText(const btVector3 &location, const char *textString);
		virtual void setDebugMode(int debugMode);
		virtual int getDebugMode() const;
		FORCEINLINE IVPhysicsDebugOverlay *GetDebugOverlay() const { return m_DebugOverlay; }
		FORCEINLINE void SetDebugOverlay(IVPhysicsDebugOverlay *debugOverlay) { m_DebugOverlay = debugOverlay; }
	private:
		IVPhysicsDebugOverlay *m_DebugOverlay;
	};
	DebugDrawer m_DebugDrawer;

	btVector3 m_Gravity;

	float m_AirDensity;

	void AddObject(IPhysicsObject *object);
	void UpdateActiveObjects();
	void UpdateObjectInterpolation();
	CUtlVector<IPhysicsObject *> m_Objects; // Doesn't include objects in the deletion queue!
	CUtlVector<IPhysicsObject *> m_NonStaticObjects;
	CUtlVector<IPhysicsObject *> m_ActiveNonStaticObjects;
	IPhysicsObjectEvent *m_ObjectEvents;
	bool m_QueueDeleteObject;
	CUtlVector<IPhysicsObject *> m_DeadObjects;

	CUtlVector<IPhysicsPlayerController *> m_PlayerControllers;

	btScalar m_SimulationTimeStep, m_SimulationInvTimeStep;
	bool m_InSimulation;
	btScalar m_LastPSITime, m_TimeSinceLastPSI;
	static void PreTickCallback(btDynamicsWorld *world, btScalar timeStep);
	static void TickCallback(btDynamicsWorld *world, btScalar timeStep);
	class TickActionInterface : public btActionInterface {
		virtual void updateAction(btCollisionWorld *collisionWorld, btScalar deltaTimeStep);
		virtual void debugDraw(btIDebugDraw *debugDrawer) {}
	};
	TickActionInterface m_TickAction;

	IPhysicsCollisionSolver *m_CollisionSolver;
	struct OverlapFilterCallback : public btOverlapFilterCallback {
		OverlapFilterCallback(CPhysicsEnvironment *environment) : m_Environment(environment) {}
		virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const;
	private:
		CPhysicsEnvironment *m_Environment;
	};
	OverlapFilterCallback m_OverlapFilterCallback;

	IPhysicsCollisionEvent *m_CollisionEvents;

	CUtlVector<IPhysicsFrictionSnapshot *> m_FrictionSnapshots;
	int m_HighestActiveFrictionSnapshot;
	inline void UpdateHighestActiveFrictionSnapshot() {
		while (m_HighestActiveFrictionSnapshot >= 0) {
			if (m_FrictionSnapshots[m_HighestActiveFrictionSnapshot]->GetObject(0) != nullptr) {
				break;
			}
			--m_HighestActiveFrictionSnapshot;
		}
	}

	struct TriggerTouch_t {
		IPhysicsObject *m_Trigger;
		IPhysicsObject *m_Object;
		// Temporary flag indicating there's still a touch between this pair.
		// Touches with this flag being false after checking all contacts are removed.
		bool m_TouchingThisTick;

		TriggerTouch_t() {} // Required by CUtlRBTree.
		TriggerTouch_t(IPhysicsObject *trigger, IPhysicsObject *object) :
				m_Trigger(trigger), m_Object(object), m_TouchingThisTick(true) {}
	};
	static bool TriggerTouchLessFunc(const TriggerTouch_t &lhs, const TriggerTouch_t &rhs) {
		if (lhs.m_Trigger != rhs.m_Trigger) {
			return (lhs.m_Trigger < rhs.m_Trigger);
		}
		return (lhs.m_Object < rhs.m_Object);
	}
	CUtlRBTree<TriggerTouch_t> m_TriggerTouches;
	void CheckTriggerTouches();

	physics_performanceparams_t m_PerformanceSettings;
};

#endif
