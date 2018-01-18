// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_environment.h"
#include "physics_collision.h"
#include "physics_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsEnvironment::CPhysicsEnvironment() :
		m_AirDensity(2.0f),
		m_ObjectEvents(nullptr), m_CollisionEvents(nullptr) {
	m_PerformanceSettings.Defaults();

	m_CollisionConfiguration = new btDefaultCollisionConfiguration();
	m_Dispatcher = new btCollisionDispatcher(m_CollisionConfiguration);
	m_Broadphase = new btDbvtBroadphase();
	m_Solver = new btSequentialImpulseConstraintSolver();
	m_DynamicsWorld = new btDiscreteDynamicsWorld(m_Dispatcher, m_Broadphase, m_Solver, m_CollisionConfiguration);
	m_DynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = VPHYSICS_CONVEX_DISTANCE_MARGIN;

	m_TriggerTouches.SetLessFunc(TriggerTouchLessFunc);

	m_DynamicsWorld->setInternalTickCallback(PreTickCallback, this, true);
	m_DynamicsWorld->setInternalTickCallback(TickCallback, this, false);
}

CPhysicsEnvironment::~CPhysicsEnvironment() {
	delete m_DynamicsWorld;
	delete m_Solver;
	delete m_Broadphase;
	delete m_Dispatcher;
	delete m_CollisionConfiguration;
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
	IPhysicsObject *object = new CPhysicsObject(this, pCollisionModel, materialIndex,
			position, angles, pParams);
	AddObject(object);
	return object;
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObjectStatic(
		const CPhysCollide *pCollisionModel, int materialIndex,
		const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *object = new CPhysicsObject(this, pCollisionModel, materialIndex,
			position, angles, pParams, true);
	AddObject(object);
	return object;
}

IPhysicsObject *CPhysicsEnvironment::CreateSphereObject(float radius, int materialIndex,
		const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic) {
	btScalar bulletRadius = HL2BULLET(radius);
	const btScalar bulletRadiusThreshold = HL2BULLET(0.1f);
	CPhysCollide_Sphere *collide = nullptr, *freeCollide = nullptr;
	for (int cacheIndex = m_SphereCache.Count() - 1; cacheIndex >= 0; --cacheIndex) {
		CPhysCollide_Sphere *cacheCollide = static_cast<CPhysCollide_Sphere *>(m_SphereCache[cacheIndex]);
		if (btFabs(cacheCollide->GetSphereShape()->getRadius() - bulletRadius) < bulletRadiusThreshold) {
			collide = cacheCollide;
			break;
		}
		if (freeCollide == nullptr && cacheCollide->GetObjectReferenceList() == nullptr) {
			freeCollide = cacheCollide;
		}
	}
	if (collide == nullptr) {
		if (freeCollide != nullptr) {
			freeCollide->GetSphereShape()->setUnscaledRadius(bulletRadius);
			collide = freeCollide;
		} else {
			collide = g_pPhysCollision->CreateSphereCollide(bulletRadius);
			collide->SetOwner(CPhysCollide::OWNER_INTERNAL);
			m_SphereCache.AddToTail(collide);
		}
	}
	IPhysicsObject *object = new CPhysicsObject(this, collide, materialIndex,
			position, angles, pParams, isStatic);
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

void CPhysicsEnvironment::NotifyObjectRemoving(IPhysicsObject *object) {
	CPhysicsObject *physicsObject = static_cast<CPhysicsObject *>(object);

	if (object->IsTrigger()) {
		NotifyTriggerRemoved(object);
	}

	if (physicsObject->IsTouchingTriggers()) {
		unsigned short touchIndex = m_TriggerTouches.FirstInorder();
		while (touchIndex != m_TriggerTouches.InvalidIndex()) {
			unsigned short nextTouch = m_TriggerTouches.NextInorder(touchIndex);
			if (m_TriggerTouches[touchIndex].m_Object == object) {
				m_TriggerTouches.RemoveAt(touchIndex);
				physicsObject->RemoveTriggerTouchReference();
				if (!physicsObject->IsTouchingTriggers()) {
					break;
				}
			}
			touchIndex = nextTouch;
		}
		Assert(!physicsObject->IsTouchingTriggers());
	}

	if (!object->IsStatic()) {
		m_NonStaticObjects.FindAndFastRemove(object);
		if (!physicsObject->WasAsleep()) {
			m_ActiveNonStaticObjects.FindAndFastRemove(object);
		}
	}
}

/****************
 * Global forces
 ****************/

void CPhysicsEnvironment::SetGravity(const Vector &gravityVector) {
	btVector3 gravity;
	ConvertPositionToBullet(gravityVector, gravity);
	m_DynamicsWorld->setGravity(gravity);
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	ConvertPositionToHL(m_DynamicsWorld->getGravity(), *pGravityVector);
}

void CPhysicsEnvironment::SetAirDensity(float density) {
	m_AirDensity = density;
}

float CPhysicsEnvironment::GetAirDensity() const {
	return m_AirDensity;
}

/*******************
 * Collision events
 *******************/

void CPhysicsEnvironment::SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents) {
	m_CollisionEvents = pCollisionEvents;
}

void CPhysicsEnvironment::CheckTriggerTouches() {
	int numManifolds = m_Dispatcher->getNumManifolds();
	for (int manifoldIndex = 0; manifoldIndex < numManifolds; ++manifoldIndex) {
		const btPersistentManifold *manifold = m_Dispatcher->getManifoldByIndexInternal(manifoldIndex);

		void *body0Pointer = manifold->getBody0()->getUserPointer();
		void *body1Pointer = manifold->getBody1()->getUserPointer();
		if (body0Pointer == nullptr || body1Pointer == nullptr) {
			continue;
		}
		IPhysicsObject *object0 = reinterpret_cast<IPhysicsObject *>(body0Pointer);
		IPhysicsObject *object1 = reinterpret_cast<IPhysicsObject *>(body1Pointer);
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

		int numContacts = manifold->getNumContacts();
		for (int contactIndex = 0; contactIndex < numContacts; ++contactIndex) {
			if (manifold->getContactPoint(contactIndex).getDistance() < maxDistance) {
				if (foundIndex != m_TriggerTouches.InvalidIndex()) {
					m_TriggerTouches[foundIndex].m_TouchingThisTick = true;
				} else {
					m_TriggerTouches.Insert(newTouch);
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
			// TODO: Trigger the leave event?
			// Probably shouldn't be done because this usually happens when the trigger is removed.
			m_TriggerTouches.RemoveAt(index);
		}
		index = next;
	}
}

/***********************
 * Performance settings
 ***********************/

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	*pOutput = m_PerformanceSettings;
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	m_PerformanceSettings = *pSettings;
}

/*****************
 * Tick callbacks
 *****************/

void CPhysicsEnvironment::PreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(world->getWorldUserInfo());
	IPhysicsObject * const *objects = environment->m_NonStaticObjects.Base();
	int objectCount = environment->m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(objects[objectIndex]);
		object->ApplyDamping(timeStep);
		object->ApplyDrag(timeStep);
		object->ApplyForcesAndSpeedLimit();
	}
}

void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(world->getWorldUserInfo());
	environment->CheckTriggerTouches();
	environment->UpdateActiveObjects();
}
