// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_environment.h"
#include "physics_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsEnvironment::CPhysicsEnvironment() :
		m_CollisionEvents(nullptr) {
	m_PerformanceSettings.Defaults();

	BEGIN_BULLET_ALLOCATION();
	m_CollisionConfiguration = new btDefaultCollisionConfiguration();
	m_Dispatcher = new btCollisionDispatcher(m_CollisionConfiguration);
	m_Broadphase = new btDbvtBroadphase();
	m_Solver = new btSequentialImpulseConstraintSolver();
	m_DynamicsWorld = new btDiscreteDynamicsWorld(m_Dispatcher, m_Broadphase, m_Solver, m_CollisionConfiguration);
	END_BULLET_ALLOCATION();

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
		object->ApplyDamping((float) timeStep);
		object->ApplyForcesAndSpeedLimit();
	}
}

void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(world->getWorldUserInfo());
	environment->CheckTriggerTouches();
}
