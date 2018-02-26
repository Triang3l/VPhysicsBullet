// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_friction.h"
#include "physics_environment.h"
#include "physics_object.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot() {
	Reset(nullptr);
}

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot(IPhysicsObject *object) {
	Reset(object);
}

void CPhysicsFrictionSnapshot::Reset(IPhysicsObject *object) {
	m_Object = object;
	if (object == nullptr) {
		return;
	}
	m_Dispatcher = static_cast<CPhysicsEnvironment *>(static_cast<CPhysicsObject *>(
			object)->GetEnvironment())->GetCollisionDispatcher();
	m_ManifoldIndex = -1;
	NextFrictionData();
}

void CPhysicsFrictionSnapshot::NextFrictionData() {
	if (!IsValid()) {
		return;
	}

	// Try the next contact in the current manifold.
	if (m_ManifoldIndex >= 0) { // Initially the index is -1.
		if (++m_ContactIndex < m_Dispatcher->getManifoldByIndexInternal(m_ManifoldIndex)->getNumContacts()) {
			return;
		}
	}

	// Try to find the next manifold.
	const btCollisionObject *collisionObject = static_cast<CPhysicsObject *>(m_Object)->GetRigidBody();
	int manifoldCount = m_Dispatcher->getNumManifolds();
	for (++m_ManifoldIndex; m_ManifoldIndex < manifoldCount; ++m_ManifoldIndex) {
		const btPersistentManifold *manifold = m_Dispatcher->getManifoldByIndexInternal(m_ManifoldIndex);
		if (manifold->getNumContacts() == 0) {
			continue;
		}
		if (manifold->getBody0() == collisionObject) {
			m_ObjectIsB = false;
		} else if (manifold->getBody1() == collisionObject) {
			m_ObjectIsB = true;
		} else {
			continue;
		}
		m_ContactIndex = 0;
		return;
	}
}

bool CPhysicsFrictionSnapshot::IsValid() {
	return m_ManifoldIndex < m_Dispatcher->getNumManifolds();
}

IPhysicsObject *CPhysicsFrictionSnapshot::GetObject(int index) {
	if (index == 0) {
		return m_Object;
	}
	const btPersistentManifold *manifold = m_Dispatcher->getManifoldByIndexInternal(m_ManifoldIndex);
	const btCollisionObject *collisionObject = (m_ObjectIsB ? manifold->getBody0() : manifold->getBody1());
	return reinterpret_cast<IPhysicsObject *>(collisionObject->getUserPointer());
}

int CPhysicsFrictionSnapshot::GetMaterial(int index) {
	// TODO: Per-plane overrides.
	return GetObject(index)->GetMaterialIndex();
}

void CPhysicsFrictionSnapshot::GetContactPoint(Vector &out) {
	const btManifoldPoint &contact = GetCurrentContact();
	ConvertPositionToHL(m_ObjectIsB ? contact.getPositionWorldOnB() : contact.getPositionWorldOnA(), out);
}

void CPhysicsFrictionSnapshot::GetSurfaceNormal(Vector &out) {
	const btVector3 &normal = GetCurrentContact().m_normalWorldOnB;
	ConvertDirectionToHL(m_ObjectIsB ? normal : -normal, out);
}

float CPhysicsFrictionSnapshot::GetNormalForce() {
	return BULLET2HL(GetCurrentContact().m_appliedImpulse);
}

float CPhysicsFrictionSnapshot::GetFrictionCoefficient() {
	return GetCurrentContact().m_combinedFriction;
}
