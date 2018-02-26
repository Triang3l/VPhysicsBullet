// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_motioncontroller.h"
#include "physics_object.h"

CPhysicsMotionController::~CPhysicsMotionController() {
	ClearObjects();
}

void CPhysicsMotionController::SetEventHandler(IMotionEvent *handler) {
	m_Handler = handler;
}

void CPhysicsMotionController::AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached) {
	// TODO: Check if restoring sends nullptr here too like in IVP VPhysics v29?
	if (pObject == nullptr || pObject->IsStatic()) {
		return;
	}
	if (checkIfAlreadyAttached && m_Objects.IsValidIndex(m_Objects.Find(pObject))) {
		DevMsg("Attached object twice!!!\n");
		return;
	}
	m_Objects.AddToTail(pObject);
	static_cast<CPhysicsObject *>(pObject)->NotifyAttachedToMotionController(this);
}

void CPhysicsMotionController::DetachObjectInternal(IPhysicsObject *object, bool notify) {
	int objectIndex = m_Objects.Find(object);
	if (!m_Objects.IsValidIndex(objectIndex)) {
		DevMsg("Removed invalid object!!!\n");
		return;
	}
	if (notify) {
		static_cast<CPhysicsObject *>(object)->NotifyDetachedFromMotionController(this);
	}
	m_Objects.FastRemove(objectIndex);
}

void CPhysicsMotionController::DetachObject(IPhysicsObject *pObject) {
	DetachObjectInternal(pObject, true);
}

int CPhysicsMotionController::CountObjects() {
	return m_Objects.Count();
}

void CPhysicsMotionController::GetObjects(IPhysicsObject **pObjectList) {
	memcpy(pObjectList, m_Objects.Base(), m_Objects.Count() * sizeof(IPhysicsObject *));
}

void CPhysicsMotionController::ClearObjects() {
	int objectCount = m_Objects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		static_cast<CPhysicsObject *>(
				m_Objects[objectIndex])->NotifyDetachedFromMotionController(this);
	}
	m_Objects.RemoveAll();
}

void CPhysicsMotionController::WakeObjects() {
	int objectCount = m_Objects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		m_Objects[objectIndex]->Wake();
	}
}

void CPhysicsMotionController::SetPriority(priority_t priority) {
	m_Priority = priority;
}

void CPhysicsMotionController::Simulate(IPhysicsObject *object, btScalar timeStep) {
	if (m_Handler == nullptr) {
		return;
	}

	Vector linear;
	AngularImpulse angular;
	IMotionEvent::simresult_e result = m_Handler->Simulate(
			this, object, (float) timeStep, linear, angular);
	if (result == IMotionEvent::SIM_NOTHING) {
		return;
	}

	btVector3 bulletLinear, bulletAngular;
	ConvertForceImpulseToBullet(linear, bulletLinear);
	ConvertAngularImpulseToBullet(angular, bulletAngular);
	static_cast<CPhysicsObject *>(object)->ApplyEventMotion(
			result == IMotionEvent::SIM_GLOBAL_ACCELERATION ||
					result == IMotionEvent::SIM_GLOBAL_FORCE,
			result == IMotionEvent::SIM_GLOBAL_FORCE ||
					result == IMotionEvent::SIM_LOCAL_FORCE,
			bulletLinear * timeStep, bulletAngular * timeStep);
}
