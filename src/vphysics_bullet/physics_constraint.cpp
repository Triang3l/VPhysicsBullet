// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_constraint.h"
#include "physics_object.h"

// memdbgon must be the last include file in a .cpp file!!!
// #include "tier0/memdbgon.h"

CPhysicsConstraint::CPhysicsConstraint(IPhysicsObject *objectReference, IPhysicsObject *objectAttached) :
		m_Constraint(nullptr), m_GameData(nullptr) {
	if (objectReference == nullptr || objectAttached == nullptr ||
			objectReference == objectAttached || objectAttached->IsStatic()) {
		objectReference = nullptr;
		objectAttached = nullptr;
	}
	m_ObjectReference = objectReference;
	m_ObjectAttached = objectAttached;
}

void CPhysicsConstraint::Activate() {
	if (m_Constraint != nullptr) {
		m_Constraint->setEnabled(true);
	}
}

void CPhysicsConstraint::Deactivate() {
	if (m_Constraint != nullptr) {
		m_Constraint->setEnabled(false);
	}
}

void CPhysicsConstraint::SetGameData(void *gameData) {
	m_GameData = gameData;
}

void *CPhysicsConstraint::GetGameData() const {
	return m_GameData;
}

IPhysicsObject *CPhysicsConstraint::GetReferenceObject() const {
	return m_ObjectReference;
}

IPhysicsObject *CPhysicsConstraint::GetAttachedObject() const {
	return m_ObjectAttached;
}

void CPhysicsConstraint::InitializeBulletConstraint(const constraint_breakableparams_t &params) {
	m_Constraint->setUserConstraintPtr(static_cast<IPhysicsConstraint *>(this));
	m_Constraint->setEnabled(params.isActive);
}

void CPhysicsConstraint::MakeInvalid() {
	if (m_Constraint == nullptr) {
		return;
	}
	m_Constraint->~btTypedConstraint();
	btAlignedFree(m_Constraint);
	m_Constraint = nullptr;
	m_ObjectAttached = m_ObjectReference = nullptr;
}

/********
 * Hinge
 ********/

CPhysicsConstraint_Hinge::CPhysicsConstraint_Hinge(
		IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
		const constraint_hingeparams_t &params) :
		CPhysicsConstraint(objectReference, objectAttached) {
	if (!AreObjectsValid()) {
		return;
	}

	btRigidBody *rigidBodyA = static_cast<CPhysicsObject *>(m_ObjectAttached)->GetRigidBody();
	btRigidBody *rigidBodyB = static_cast<CPhysicsObject *>(m_ObjectReference)->GetRigidBody();
	const btTransform &transformA = rigidBodyA->getWorldTransform();
	const btTransform &transformB = rigidBodyB->getWorldTransform();

	btVector3 worldPosition, worldAxisDirection;
	ConvertPositionToBullet(params.worldPosition, worldPosition);
	ConvertDirectionToBullet(params.worldAxisDirection, worldAxisDirection);

	m_Constraint = new(btAlignedAlloc(sizeof(btHingeConstraint), 16))
			btHingeConstraint(*rigidBodyA, *rigidBodyB,
			transformA.invXform(worldPosition), transformB.invXform(worldPosition),
			worldAxisDirection * transformA.getBasis(), worldAxisDirection * transformB.getBasis());
	InitializeBulletConstraint(params.constraint);
}

void CPhysicsConstraint_Hinge::SetAngularMotor(float rotSpeed, float maxAngularImpulse) {
	static_cast<btHingeConstraint *>(m_Constraint)->enableAngularMotor(
			rotSpeed != 0.0f, DEG2RAD(rotSpeed), DEG2RAD(maxAngularImpulse));
}
