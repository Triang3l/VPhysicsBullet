// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_environment.h"
#include "physics_collide.h"
#include "physics_constraint.h"
#include "physics_fluid.h"
#include "physics_friction.h"
#include "physics_motioncontroller.h"
#include "physics_object.h"
#include "physics_shadow.h"
#include "physics_spring.h"
#include "physics_vehicle.h"
#include "vphysics/stats.h"
#include "const.h"
#include "tier1/convar.h"

#ifdef WIN32
#pragma warning(push)
#pragma warning(disable : 4355) // 'this' : used in base member initializer list
#endif

CPhysicsEnvironment::CPhysicsEnvironment() :
		m_Gravity(0.0f, 0.0f, 0.0f),
		m_AirDensity(2.0f),
		m_ObjectEvents(nullptr),
		m_QueueDeleteObject(false),
		m_SimulationTimeStep(DEFAULT_TICK_INTERVAL),
		m_SimulationInvTimeStep(1.0f / btScalar(DEFAULT_TICK_INTERVAL)),
		m_InSimulation(false),
		m_LastPSITime(0.0f), m_TimeSinceLastPSI(0.0f),
		m_CollisionSolver(nullptr), m_OverlapFilterCallback(this),
		m_CollisionEvents(nullptr),
		m_HighestActiveFrictionSnapshot(-1),
		m_QuickDelete(false) {
	m_PerformanceSettings.Defaults();

	m_CollisionConfiguration = VPhysicsNew(btDefaultCollisionConfiguration);
	m_CollisionConfiguration->setConvexConvexMultipointIterations(); // Stability - very important.
	m_Dispatcher = VPhysicsNew(btCollisionDispatcher, m_CollisionConfiguration);
	m_Broadphase = VPhysicsNew(btDbvtBroadphase);
	m_Solver = VPhysicsNew(btSequentialImpulseConstraintSolver);
	m_DynamicsWorld = VPhysicsNew(btDiscreteDynamicsWorld, m_Dispatcher, m_Broadphase, m_Solver, m_CollisionConfiguration);
	m_DynamicsWorld->setWorldUserInfo(this);

	m_DynamicsWorld->setDebugDrawer(&m_DebugDrawer);

	// Gravity is applied by CPhysicsObjects, also objects assume zero Bullet forces.
	m_DynamicsWorld->setGravity(btVector3(0.0f, 0.0f, 0.0f));

	m_Broadphase->getOverlappingPairCache()->setOverlapFilterCallback(&m_OverlapFilterCallback);

	m_DynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = VPHYSICS_CONVEX_DISTANCE_MARGIN;
	btContactSolverInfo &solverInfo = m_DynamicsWorld->getSolverInfo();
	// Stability.
	solverInfo.m_erp2 = 0.25f; // Slightly above the default 0.2, which has visible penetration, but not too high bounce.
	solverInfo.m_splitImpulse = false;
	solverInfo.m_solverMode |= SOLVER_RANDMIZE_ORDER | SOLVER_USE_2_FRICTION_DIRECTIONS;

	m_TriggerTouches.SetLessFunc(TriggerTouchLessFunc);

	m_DynamicsWorld->setInternalTickCallback(PreTickCallback, this, true);
	m_DynamicsWorld->setInternalTickCallback(TickCallback, this, false);
	m_DynamicsWorld->addAction(&m_TickAction);
}

#ifdef WIN32
#pragma warning(pop)
#endif

CPhysicsEnvironment::~CPhysicsEnvironment() {
	int snapshotCount = m_FrictionSnapshots.Count();
	for (int snapshotIndex = 0; snapshotIndex < snapshotCount; ++snapshotIndex) {
		VPhysicsDelete(CPhysicsFrictionSnapshot, m_FrictionSnapshots[snapshotIndex]);
	}

	VPhysicsDelete(btDiscreteDynamicsWorld, m_DynamicsWorld);
	VPhysicsDelete(btSequentialImpulseConstraintSolver, m_Solver);
	VPhysicsDelete(btDbvtBroadphase, m_Broadphase);
	VPhysicsDelete(btCollisionDispatcher, m_Dispatcher);
	VPhysicsDelete(btDefaultCollisionConfiguration, m_CollisionConfiguration);
}

void CPhysicsEnvironment::Release() {
	CleanupDeleteList();

	int constraintCount = m_ConstraintObjects.Count();
	for (int constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex) {
		DeleteConstraint(m_ConstraintObjects[constraintIndex], false);
	}

	int objectCount = m_Objects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		static_cast<CPhysicsObject *>(m_Objects[objectIndex])->Release();
	}

	VPhysicsDelete(CPhysicsEnvironment, this);
}

/****************
 * Debug overlay
 ****************/

// A ConVar can't support more than 25 flags because it uses a float! As of February 2018, Bullet has 16.
static ConVar physics_bullet_debugmode("physics_bullet_debugmode", "0", FCVAR_CHEAT,
		"Bullet Physics debug drawer mode flags. Refer to LinearMath/btIDebugDraw::DebugDrawModes for bit meanings.",
		true, 0.0f, true, (float) (((btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE - 1) << 1) - 1));

void CPhysicsEnvironment::DebugDrawer::drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) {
	if (m_DebugOverlay == nullptr) {
		return;
	}
	Vector hlFrom, hlTo;
	ConvertPositionToHL(from, hlFrom);
	ConvertPositionToHL(to, hlTo);
	m_DebugOverlay->AddLineOverlay(hlFrom, hlTo,
			(int) (color.getX() * 255.0f), (int) (color.getY() * 255.0f), (int) (color.getZ() * 255.0f),
			false, 0.0f);
}

void CPhysicsEnvironment::DebugDrawer::drawContactPoint(const btVector3 &PointOnB, const btVector3 &normalOnB,
		btScalar distance, int lifeTime, const btVector3 &color) {
	// TODO: Check if the distance is too short to be useful.
	drawLine(PointOnB, PointOnB + normalOnB * 0.3f, color);
}

void CPhysicsEnvironment::DebugDrawer::reportErrorWarning(const char *warningString) {
	DevMsg("Bullet: %s\n", warningString);
}

void CPhysicsEnvironment::DebugDrawer::setDebugMode(int debugMode) {
	// physics_bullet_debugdrawmode is a cheat ConVar and must not be changed without sv_cheats.
	// This is never called by Bullet anyway.
}

int CPhysicsEnvironment::DebugDrawer::getDebugMode() const {
	return physics_bullet_debugmode.GetInt();
}

void CPhysicsEnvironment::DebugDrawer::draw3dText(const btVector3 &location, const char *textString) {
	if (m_DebugOverlay == nullptr) {
		return;
	}
	Vector hlLocation;
	ConvertPositionToHL(location, hlLocation);
	m_DebugOverlay->AddTextOverlay(hlLocation, 0.0f, "%s", textString);
}

void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
	IVPhysicsDebugOverlay *debugOverlay = reinterpret_cast<IVPhysicsDebugOverlay *>(
			debugOverlayFactory(VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, nullptr));
	m_DebugDrawer.SetDebugOverlay(debugOverlay);
}

IVPhysicsDebugOverlay *CPhysicsEnvironment::GetDebugOverlay() {
	return m_DebugDrawer.GetDebugOverlay();
}

/********************
 * Object management
 ********************/

void CPhysicsEnvironment::AddObject(IPhysicsObject *object) {
	CPhysicsObject *physicsObject = static_cast<CPhysicsObject *>(object);
	m_DynamicsWorld->addRigidBody(physicsObject->GetRigidBody());
	m_Objects.AddToTail(object);
	if (!object->IsStatic()) {
		m_NonStaticObjects.AddToTail(object);
		if (!physicsObject->WasAsleep()) {
			m_ActiveNonStaticObjects.AddToTail(object);
		}
	}
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObject(
		const CPhysCollide *pCollisionModel, int materialIndex,
		const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *object = VPhysicsNew(CPhysicsObject, this, pCollisionModel, materialIndex,
			position, angles, pParams, false);
	AddObject(object);
	return object;
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObjectStatic(
		const CPhysCollide *pCollisionModel, int materialIndex,
		const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *object = VPhysicsNew(CPhysicsObject, this, pCollisionModel, materialIndex,
			position, angles, pParams, true);
	AddObject(object);
	return object;
}

IPhysicsObject *CPhysicsEnvironment::CreateSphereObject(float radius, int materialIndex,
		const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic) {
	IPhysicsObject *object = VPhysicsNew(CPhysicsObject, this, static_cast<CPhysicsCollision *>(
			g_pPhysCollision)->CreateCachedSphereCollide(HL2BULLET(radius)),
			materialIndex, position, angles, pParams, isStatic);
	AddObject(object);
	return object;
}

void CPhysicsEnvironment::SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents) {
	m_ObjectEvents = pObjectEvents;
}

void CPhysicsEnvironment::UpdateActiveObjects() {
	for (int objectIndex = 0; objectIndex < m_ActiveNonStaticObjects.Count(); ++objectIndex) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(m_ActiveNonStaticObjects[objectIndex]);
		if (object->UpdateEventSleepState() != object->IsAsleep()) {
			Assert(object->IsAsleep());
			m_ActiveNonStaticObjects.FastRemove(objectIndex--);
			if (m_ObjectEvents != nullptr) {
				m_ObjectEvents->ObjectSleep(object);
			}
		}
	}
	int nonStaticObjectCount = m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < nonStaticObjectCount; ++objectIndex) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(m_NonStaticObjects[objectIndex]);
		if (object->UpdateEventSleepState() != object->IsAsleep()) {
			Assert(!object->IsAsleep());
			m_ActiveNonStaticObjects.AddToTail(object);
			if (m_ObjectEvents != nullptr) {
				m_ObjectEvents->ObjectWake(object);
			}
		}
	}
}

int CPhysicsEnvironment::GetActiveObjectCount() const {
	return m_ActiveNonStaticObjects.Count();
}

void CPhysicsEnvironment::GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
	memcpy(pOutputObjectList, m_ActiveNonStaticObjects.Base(),
			m_ActiveNonStaticObjects.Count() * sizeof(IPhysicsObject *));
}

const IPhysicsObject **CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount) const {
	if (pOutputObjectCount != nullptr) {
		*pOutputObjectCount = m_Objects.Count();
	}
	return const_cast<const IPhysicsObject **>(m_Objects.Base());
}

void CPhysicsEnvironment::UpdateNonStaticObjectsAfterPSI() {
	int objectCount = m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		static_cast<CPhysicsObject *>(m_NonStaticObjects[objectIndex])->UpdateAfterPSI();
	}
}

bool CPhysicsEnvironment::IsCollisionModelUsed(CPhysCollide *pCollide) const {
	return pCollide->GetObjectReferenceList() != nullptr;
}

void CPhysicsEnvironment::SetQuickDelete(bool bQuick) {
	m_QuickDelete = bQuick;
}

void CPhysicsEnvironment::EnableDeleteQueue(bool enable) {
	m_QueueDeleteObject = enable;
}

void CPhysicsEnvironment::DestroyObject(IPhysicsObject *pObject) {
	if (pObject == nullptr) {
		DevMsg("Deleted NULL vphysics object\n");
		return;
	}
	// TODO: Activate islands containing this object if not deleting quickly.
	m_Objects.FindAndFastRemove(pObject);
	if (IsInSimulation() || m_QueueDeleteObject) {
		pObject->SetCallbackFlags(pObject->GetCallbackFlags() | CALLBACK_MARKED_FOR_DELETE);
		m_DeadObjects.AddToTail(pObject);
	} else {
		static_cast<CPhysicsObject *>(pObject)->Release();
	}
}

void CPhysicsEnvironment::CleanupDeleteList() {
	int constraintCount = m_DeadConstraints.Count();
	for (int constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex) {
		DeleteConstraint(m_DeadConstraints[constraintIndex]);
	}
	m_DeadConstraints.RemoveAll();

	int objectCount = m_DeadObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		static_cast<CPhysicsObject *>(m_DeadObjects[objectIndex])->Release();
	}
	m_DeadObjects.RemoveAll();
}

void CPhysicsEnvironment::NotifyObjectRemoving(IPhysicsObject *object) {
	CPhysicsObject *physicsObject = static_cast<CPhysicsObject *>(object);

	if (physicsObject->IsAttachedToConstraintObjects()) {
		if (physicsObject->IsAttachedToConstraint(false)) {
			for (int constraintIndex = m_DynamicsWorld->getNumConstraints() - 1; constraintIndex >= 0; --constraintIndex) {
				btTypedConstraint *constraint = m_DynamicsWorld->getConstraint(constraintIndex);
				if (constraint->getRigidBodyA().getUserPointer() == object ||
						constraint->getRigidBodyB().getUserPointer() == object) {
					// Force remove it - if it's fine to destroy objects, it's fine to remove constraints too.
					m_DynamicsWorld->removeConstraint(constraint);
				}
			}
		}
		int constraintCount = m_ConstraintObjects.Count();
		for (int constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex) {
			IPhysicsConstraint *constraint = m_ConstraintObjects[constraintIndex];
			if (constraint->GetReferenceObject() == object || constraint->GetAttachedObject() == object) {
				static_cast<CPhysicsConstraint *>(constraint)->NotifyObjectRemoving();
			}
		}
		physicsObject->NotifyAllConstraintsRemoved();
	}

	int playerCount = m_PlayerControllers.Count();
	for (int playerIndex = 0; playerIndex < playerCount; ++playerIndex) {
		static_cast<CPhysicsPlayerController *>(
				m_PlayerControllers[playerIndex])->NotifyPotentialGroundRemoving(object);
	}

	if (object->IsTrigger()) {
		NotifyTriggerRemoved(object);
	}

	if (physicsObject->IsTouchingTriggers()) {
		unsigned short touchIndex = m_TriggerTouches.FirstInorder();
		while (touchIndex != m_TriggerTouches.InvalidIndex()) {
			unsigned short nextTouch = m_TriggerTouches.NextInorder(touchIndex);
			if (m_TriggerTouches[touchIndex].m_Object == object) {
				physicsObject->RemoveTriggerTouchReference();
				m_TriggerTouches.RemoveAt(touchIndex);
				if (!physicsObject->IsTouchingTriggers()) {
					break;
				}
			}
			touchIndex = nextTouch;
		}
		Assert(!physicsObject->IsTouchingTriggers());
	}

	for (int snapshotIndex = m_HighestActiveFrictionSnapshot; snapshotIndex >= 0; --snapshotIndex) {
		CPhysicsFrictionSnapshot *snapshot = static_cast<CPhysicsFrictionSnapshot *>(
				m_FrictionSnapshots[snapshotIndex]);
		if (snapshot->GetObject(0) == object) {
			snapshot->Reset(nullptr);
		}
	}
	UpdateHighestActiveFrictionSnapshot();

	if (!object->IsStatic()) {
		if (!physicsObject->WasAsleep()) {
			m_ActiveNonStaticObjects.FindAndFastRemove(object);
		}
		m_NonStaticObjects.FindAndFastRemove(object);
	}

	// Already removed from m_Objects by the method which requested removal.

	m_DynamicsWorld->removeRigidBody(physicsObject->GetRigidBody());
}

/****************
 * Global forces
 ****************/

void CPhysicsEnvironment::SetGravity(const Vector &gravityVector) {
	ConvertPositionToBullet(gravityVector, m_Gravity);
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	ConvertPositionToHL(m_Gravity, *pGravityVector);
}

void CPhysicsEnvironment::SetAirDensity(float density) {
	m_AirDensity = density;
}

float CPhysicsEnvironment::GetAirDensity() const {
	return m_AirDensity;
}

/**************
 * Constraints
 **************/

/* DUMMY */ IPhysicsSpring *CPhysicsEnvironment::CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd,
		springparams_t *pParams) {
	return VPhysicsNew(CPhysicsSpring, pObjectStart, pObjectEnd, pParams);
}

/* DUMMY */ void CPhysicsEnvironment::DestroySpring(IPhysicsSpring *pSpring) {
	VPhysicsDelete(CPhysicsSpring, pSpring);
}

/* DUMMY */ IPhysicsConstraint *CPhysicsEnvironment::CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Dummy, pReferenceObject, pAttachedObject);
	AddConstraint(constraint);
	return constraint;
}

IPhysicsConstraint *CPhysicsEnvironment::CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Hinge, pReferenceObject, pAttachedObject, hinge);
	AddConstraint(constraint);
	return constraint;
}

/* DUMMY */ IPhysicsConstraint *CPhysicsEnvironment::CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Dummy, pReferenceObject, pAttachedObject);
	AddConstraint(constraint);
	return constraint;
}

/* DUMMY */ IPhysicsConstraint *CPhysicsEnvironment::CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Dummy, pReferenceObject, pAttachedObject);
	AddConstraint(constraint);
	return constraint;
}

/* DUMMY */ IPhysicsConstraint *CPhysicsEnvironment::CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Dummy, pReferenceObject, pAttachedObject);
	AddConstraint(constraint);
	return constraint;
}

/* DUMMY */ IPhysicsConstraint *CPhysicsEnvironment::CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Dummy, pReferenceObject, pAttachedObject);
	AddConstraint(constraint);
	return constraint;
}

/* DUMMY */ IPhysicsConstraint *CPhysicsEnvironment::CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	IPhysicsConstraint *constraint = VPhysicsNew(CPhysicsConstraint_Dummy, pReferenceObject, pAttachedObject);
	AddConstraint(constraint);
	return constraint;
}

void CPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint *pConstraint) {
	if (pConstraint == nullptr) {
		return;
	}
	if (!IsInSimulation() || m_QuickDelete) {
		DeleteConstraint(pConstraint);
	} else {
		pConstraint->Deactivate();
		m_DeadConstraints.AddToTail(pConstraint);
	}
}

/* DUMMY */ IPhysicsConstraintGroup *CPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t &groupParams) {
	return VPhysicsNew(CPhysicsConstraintGroup);
}

/* DUMMY */ void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	VPhysicsDelete(CPhysicsConstraintGroup, pGroup);
}

void CPhysicsEnvironment::AddConstraint(IPhysicsConstraint *constraint) {
	m_ConstraintObjects.AddToTail(constraint);
	btTypedConstraint *bulletConstraint = static_cast<CPhysicsConstraint *>(
			constraint)->GetBulletConstraint();
	bool valid = (bulletConstraint != nullptr);
	if (valid) {
		m_DynamicsWorld->addConstraint(bulletConstraint);
	}

	IPhysicsObject *object = constraint->GetReferenceObject();
	if (object != nullptr) {
		static_cast<CPhysicsObject *>(object)->NotifyConstraintAdded(valid);
	}
	object = constraint->GetAttachedObject();
	if (object != nullptr) {
		static_cast<CPhysicsObject *>(object)->NotifyConstraintAdded(valid);
	}
}

void CPhysicsEnvironment::DeleteConstraint(IPhysicsConstraint *constraint, bool removeFromList) {
	CPhysicsConstraint *physicsConstraint = static_cast<CPhysicsConstraint *>(constraint);

	btTypedConstraint *bulletConstraint = physicsConstraint->GetBulletConstraint();
	bool valid = (bulletConstraint != nullptr);
	if (valid) {
		m_DynamicsWorld->removeConstraint(bulletConstraint);
	}

	if (removeFromList) {
		m_ConstraintObjects.FindAndFastRemove(constraint);
	}

	IPhysicsObject *object = constraint->GetReferenceObject();
	if (object != nullptr) {
		static_cast<CPhysicsObject *>(object)->NotifyConstraintRemoved(valid);
	}
	object = constraint->GetAttachedObject();
	if (object != nullptr) {
		static_cast<CPhysicsObject *>(object)->NotifyConstraintRemoved(valid);
	}

	physicsConstraint->Release();
}

/**************
 * Controllers
 **************/

/* DUMMY */ IPhysicsFluidController *CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject,
		fluidparams_t *pParams) {
	return VPhysicsNew(CPhysicsFluidController, pFluidObject, pParams);
}

/* DUMMY */ void CPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController *pFluid) {
	VPhysicsDelete(CPhysicsFluidController, pFluid);
}

IPhysicsShadowController *CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject,
		bool allowTranslation, bool allowRotation) {
	pObject->RemoveShadowController();
	return VPhysicsNew(CPhysicsShadowController, pObject, allowTranslation, allowRotation);
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController *pController) {
	VPhysicsDelete(CPhysicsShadowController, pController);
}

IPhysicsPlayerController *CPhysicsEnvironment::CreatePlayerController(IPhysicsObject *pObject) {
	static_cast<CPhysicsObject *>(pObject)->RemovePlayerController();
	return VPhysicsNew(CPhysicsPlayerController, pObject);
}

void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController *pController) {
	VPhysicsDelete(CPhysicsPlayerController, pController);
}

void CPhysicsEnvironment::NotifyPlayerControllerAttached(IPhysicsPlayerController *controller) {
	m_PlayerControllers.AddToTail(controller);
}

void CPhysicsEnvironment::NotifyPlayerControllerDetached(IPhysicsPlayerController *controller) {
	m_PlayerControllers.FindAndFastRemove(controller);
}

IPhysicsMotionController *CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	return VPhysicsNew(CPhysicsMotionController, pHandler);
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	VPhysicsDelete(CPhysicsMotionController, pController);
}

/* DUMMY */ IPhysicsVehicleController *CPhysicsEnvironment::CreateVehicleController(IPhysicsObject *pVehicleBodyObject,
		const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	return VPhysicsNew(CPhysicsVehicleController, params);
}

/* DUMMY */ void CPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController *pController) {
	VPhysicsDelete(CPhysicsVehicleController, pController);
}

/*******************
 * Simulation steps
 *******************/

void CPhysicsEnvironment::Simulate(float deltaTime) {
	if (deltaTime > 0.0f && deltaTime < 1.0f) { // Trap interrupts and clock changes.
		deltaTime = MIN(deltaTime, 0.1f);
		m_TimeSinceLastPSI += deltaTime;
		int psiCount = (int) (m_TimeSinceLastPSI * m_SimulationInvTimeStep);
		if (psiCount > 0) {
			btScalar oldTimeSinceLastPSI = m_TimeSinceLastPSI;
			// We're in a PSI, so in case something tries to interpolate transforms with
			// m_InSimulation being false, the PSI values will be used.
			m_TimeSinceLastPSI = 0.0f;
			for (int psi = 0; psi < psiCount; ++psi) {
				// Using fake variable timestep with fixed timestep and interpolating manually.
				m_DynamicsWorld->stepSimulation(m_SimulationTimeStep, 0, m_SimulationTimeStep);
				m_LastPSITime += m_SimulationTimeStep;
			}
			m_TimeSinceLastPSI = oldTimeSinceLastPSI - psiCount * m_SimulationTimeStep;
		}
		int objectCount = m_ActiveNonStaticObjects.Count();
		for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
			CPhysicsObject *object = static_cast<CPhysicsObject *>(m_ActiveNonStaticObjects[objectIndex]);
			object->InterpolateBetweenPSIs();
		}
	}
	if (!m_QueueDeleteObject) {
		CleanupDeleteList();
	}
	if (m_DebugDrawer.getDebugMode() != 0) {
		m_DynamicsWorld->debugDrawWorld();
	}
}

bool CPhysicsEnvironment::IsInSimulation() const {
	return m_InSimulation;
}

float CPhysicsEnvironment::GetSimulationTimestep() const {
	return m_SimulationTimeStep;
}

void CPhysicsEnvironment::SetSimulationTimestep(float timestep) {
	m_SimulationTimeStep = MAX(timestep, 0.001f);
	m_SimulationInvTimeStep = 1.0f / m_SimulationTimeStep;
}

float CPhysicsEnvironment::GetSimulationTime() const {
	return (float) (m_LastPSITime + m_TimeSinceLastPSI);
}

void CPhysicsEnvironment::ResetSimulationClock() {
	m_LastPSITime = 0.0f;
	m_TimeSinceLastPSI = 0.0f;
	m_Solver->reset();
	// Move interpolated transforms to the last PSI.
	int objectCount = m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		static_cast<CPhysicsObject *>(m_NonStaticObjects[objectIndex])->InterpolateBetweenPSIs();
	}
}

float CPhysicsEnvironment::GetNextFrameTime() const {
	return m_LastPSITime + m_SimulationTimeStep;
}

void CPhysicsEnvironment::PreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(world->getWorldUserInfo());

	if (!environment->m_QueueDeleteObject) {
		environment->CleanupDeleteList();
	}

	environment->m_InSimulation = true;

	IPhysicsObject * const *objects = environment->m_NonStaticObjects.Base();
	int objectCount = environment->m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(objects[objectIndex]);

		// Async force fields.
		object->SimulateMotionControllers(IPhysicsMotionController::HIGH_PRIORITY, timeStep);

		// Gravity.
		object->ApplyDamping(timeStep);
		object->ApplyForcesAndSpeedLimit(timeStep);
		object->ApplyGravity(timeStep);

		// Shadows.
		object->SimulateShadowAndPlayer(timeStep);

		// Unconstrained motion.
		object->ApplyDrag(timeStep);
		object->SimulateMotionControllers(IPhysicsMotionController::MEDIUM_PRIORITY, timeStep);

		// Vehicles.

		object->CheckAndClearBulletForces();
	}
}

void CPhysicsEnvironment::TickActionInterface::updateAction(
		btCollisionWorld *collisionWorld, btScalar deltaTimeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(
			static_cast<btDynamicsWorld *>(collisionWorld)->getWorldUserInfo());
	IPhysicsObject * const *objects = environment->m_NonStaticObjects.Base();
	int objectCount = environment->m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(objects[objectIndex]);
		object->SimulateMotionControllers(IPhysicsMotionController::LOW_PRIORITY, deltaTimeStep);
	}
}

void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(world->getWorldUserInfo());
	environment->CheckTriggerTouches();
	environment->UpdateActiveObjects();
	environment->UpdateNonStaticObjectsAfterPSI();
	environment->m_InSimulation = false;
}

/************
 * Collision
 ************/

void CPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver *pSolver) {
	m_CollisionSolver = pSolver;
	// Assuming this is only called when setting up, so not rechecking collision filter.
	// IVP VPhysics assumes this too.
}

bool CPhysicsEnvironment::NeedCollision(const btCollisionObject *collisionObject0,
		const btCollisionObject *collisionObject1) {
	if (collisionObject0 == nullptr || collisionObject1 == nullptr) {
		// In case client objects in needBroadphaseCollision are null.
		return false;
	}

	if (collisionObject0->isStaticObject() && collisionObject1->isStaticObject()) {
		return false;
	}

	IPhysicsObject *object0 = reinterpret_cast<IPhysicsObject *>(collisionObject0->getUserPointer());
	IPhysicsObject *object1 = reinterpret_cast<IPhysicsObject *>(collisionObject1->getUserPointer());
	if (object0 == nullptr || object1 == nullptr) {
		return false;
	}

	// Check if any object isn't expecting collisions at all.
	if (!object0->IsCollisionEnabled() || !object1->IsCollisionEnabled()) {
		return false;
	}

	if (m_CollisionSolver != nullptr) {
		unsigned int callbackFlags0 = object0->GetCallbackFlags();
		unsigned int callbackFlags1 = object1->GetCallbackFlags();
		if ((callbackFlags0 & CALLBACK_ENABLING_COLLISION) && (callbackFlags1 & CALLBACK_MARKED_FOR_DELETE)) {
			return false;
		}
		if ((callbackFlags1 & CALLBACK_ENABLING_COLLISION) && (callbackFlags0 & CALLBACK_MARKED_FOR_DELETE)) {
			return false;
		}
		if (!m_CollisionSolver->ShouldCollide(object0, object1,
				object0->GetGameData(), object1->GetGameData())) {
			return false;
		}
	}

	return true;
}

bool CPhysicsEnvironment::OverlapFilterCallback::needBroadphaseCollision(
		btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const {
	return m_Environment->NeedCollision(
			reinterpret_cast<const btCollisionObject *>(proxy0->m_clientObject),
			reinterpret_cast<const btCollisionObject *>(proxy1->m_clientObject));
}

void CPhysicsEnvironment::SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents) {
	m_CollisionEvents = pCollisionEvents;
}

void CPhysicsEnvironment::RecheckObjectCollisionFilter(btCollisionObject *object) {
	struct RecheckObjectCollisionFilterCallback : public btOverlapCallback {
		btCollisionObject *m_Object;
		btOverlapFilterCallback *m_Filter;
		RecheckObjectCollisionFilterCallback(btCollisionObject *object, btOverlapFilterCallback *filter) :
				m_Object(object), m_Filter(filter) {}
		virtual bool processOverlap(btBroadphasePair &pair) {
			if (reinterpret_cast<btCollisionObject *>(pair.m_pProxy0->m_clientObject) == m_Object ||
					reinterpret_cast<btCollisionObject *>(pair.m_pProxy1->m_clientObject) == m_Object) {
				return !m_Filter->needBroadphaseCollision(pair.m_pProxy0, pair.m_pProxy1);
			}
			return false;
		}
	};
	RecheckObjectCollisionFilterCallback recheckCallback(object, &m_OverlapFilterCallback);
	m_Broadphase->getOverlappingPairCache()->processAllOverlappingPairs(&recheckCallback, m_Dispatcher);
	// Narrowphase contact manifolds are cleared by overlapping pair destruction.
	// No need to add any pairs here, wait until the next PSI (this is usually called during game ticks).
}

void CPhysicsEnvironment::RemoveObjectCollisionPairs(btCollisionObject *object) {
	struct RemoveObjectCollisionPairsCallback : public btOverlapCallback {
		btCollisionObject *m_Object;
		RemoveObjectCollisionPairsCallback(btCollisionObject *object) : m_Object(object) {}
		virtual bool processOverlap(btBroadphasePair &pair) {
			return (reinterpret_cast<btCollisionObject *>(pair.m_pProxy0->m_clientObject) == m_Object ||
					reinterpret_cast<btCollisionObject *>(pair.m_pProxy1->m_clientObject) == m_Object);
		}
	};
	RemoveObjectCollisionPairsCallback removeCallback(object);
	m_Broadphase->getOverlappingPairCache()->processAllOverlappingPairs(&removeCallback, m_Dispatcher);
	// Narrowphase contact manifolds are cleared by overlapping pair destruction.
}

IPhysicsFrictionSnapshot *CPhysicsEnvironment::CreateFrictionSnapshot(IPhysicsObject *object) {
	int snapshotCount = m_FrictionSnapshots.Count(), snapshotIndex;
	for (snapshotIndex = 0; snapshotIndex < snapshotCount; ++snapshotIndex) {
		if (m_FrictionSnapshots[snapshotIndex]->GetObject(0) == nullptr) {
			break;
		}
	}
	if (snapshotIndex >= snapshotCount) {
		m_FrictionSnapshots.AddToTail(VPhysicsNew(CPhysicsFrictionSnapshot));
	}
	CPhysicsFrictionSnapshot *snapshot = static_cast<CPhysicsFrictionSnapshot *>(
			m_FrictionSnapshots[snapshotIndex]);
	snapshot->Reset(object);
	m_HighestActiveFrictionSnapshot = MAX(snapshotIndex, m_HighestActiveFrictionSnapshot);
	return snapshot;
}

void CPhysicsEnvironment::DestroyFrictionSnapshot(IPhysicsFrictionSnapshot *snapshot) {
	static_cast<CPhysicsFrictionSnapshot *>(snapshot)->Reset(nullptr);
	UpdateHighestActiveFrictionSnapshot();
}

void CPhysicsEnvironment::CheckTriggerTouches() {
	int numManifolds = m_Dispatcher->getNumManifolds();
	for (int manifoldIndex = 0; manifoldIndex < numManifolds; ++manifoldIndex) {
		const btPersistentManifold *manifold = m_Dispatcher->getManifoldByIndexInternal(manifoldIndex);
		int contactCount = manifold->getNumContacts();
		if (contactCount == 0) {
			continue;
		}

		IPhysicsObject *object0 = reinterpret_cast<IPhysicsObject *>(manifold->getBody0()->getUserPointer());
		IPhysicsObject *object1 = reinterpret_cast<IPhysicsObject *>(manifold->getBody1()->getUserPointer());
		if (object0 == nullptr || object1 == nullptr) {
			continue;
		}
		IPhysicsObject *trigger, *object;
		if (object0->IsTrigger()) {
			if (object1->IsTrigger()) {
				continue;
			}
			trigger = object0;
			object = object1;
		} else if (object1->IsTrigger()) {
			if (object0->IsTrigger()) {
				continue;
			}
			trigger = object1;
			object = object0;
		} else {
			continue;
		}
		if (object->IsStatic()) {
			continue;
		}

		TriggerTouch_t newTouch(trigger, object);
		unsigned short foundIndex = m_TriggerTouches.Find(newTouch);
		btScalar maxDistance = 0.0f;
		if (foundIndex != m_TriggerTouches.InvalidIndex()) {
			if (m_TriggerTouches[foundIndex].m_TouchingThisTick) {
				continue;
			}
			maxDistance = 0.1f;
		}

		for (int contactIndex = 0; contactIndex < contactCount; ++contactIndex) {
			if (manifold->getContactPoint(contactIndex).getDistance() < maxDistance) {
				if (foundIndex != m_TriggerTouches.InvalidIndex()) {
					m_TriggerTouches[foundIndex].m_TouchingThisTick = true;
				} else {
					m_TriggerTouches.Insert(newTouch);
					static_cast<CPhysicsObject *>(object)->AddTriggerTouchReference();
					if (m_CollisionEvents != nullptr) {
						m_CollisionEvents->ObjectEnterTrigger(trigger, object);
					}
				}
				break;
			}
		}
	}

	unsigned short index = m_TriggerTouches.FirstInorder();
	while (m_TriggerTouches.IsValidIndex(index)) {
		unsigned short next = m_TriggerTouches.NextInorder(index);
		TriggerTouch_t &touch = m_TriggerTouches[index];
		if (!touch.m_TouchingThisTick) {
			static_cast<CPhysicsObject *>(touch.m_Object)->RemoveTriggerTouchReference();
			if (m_CollisionEvents != nullptr) {
				m_CollisionEvents->ObjectLeaveTrigger(touch.m_Trigger, touch.m_Object);
			}
			m_TriggerTouches.RemoveAt(index);
		} else {
			touch.m_TouchingThisTick = false;
		}
		index = next;
	}
}

void CPhysicsEnvironment::NotifyTriggerRemoved(IPhysicsObject *trigger) {
	unsigned short index = m_TriggerTouches.FirstInorder();
	while (m_TriggerTouches.IsValidIndex(index)) {
		unsigned short next = m_TriggerTouches.NextInorder(index);
		TriggerTouch_t &touch = m_TriggerTouches[index];
		if (touch.m_Trigger > trigger) {
			break;
		}
		if (touch.m_Trigger == trigger) {
			static_cast<CPhysicsObject *>(touch.m_Object)->RemoveTriggerTouchReference();
			// TODO: Trigger the leave event?
			// Probably shouldn't be done because this usually happens when the trigger is removed.
			m_TriggerTouches.RemoveAt(index);
		}
		index = next;
	}
}

/******************
 * Traces (unused)
 ******************/

void CPhysicsEnvironment::TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	// Not implemented in IVP VPhysics, can be implemented using rayTest.
}

void CPhysicsEnvironment::SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
		const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	// Not implemented in IVP VPhysics.
	// For compound objects, possibly the closest hit of every child can be returned.
}

/**************
 * Performance
 **************/

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	*pOutput = m_PerformanceSettings;
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	m_PerformanceSettings = *pSettings;
}

/* DUMMY */ void CPhysicsEnvironment::ReadStats(physics_stats_t *pOutput) {
	if (pOutput != nullptr) {
		memset(pOutput, 0, sizeof(*pOutput));
	}
}
