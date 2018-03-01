// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_constraint.h"
#include "physics_object.h"

CPhysicsConstraint::CPhysicsConstraint(IPhysicsObject *objectReference, IPhysicsObject *objectAttached) :
		m_ObjectReference(objectReference), m_ObjectAttached(objectAttached), m_GameData(nullptr) {}

void CPhysicsConstraint::Activate() {
	btTypedConstraint *constraint = GetBulletConstraint();
	if (constraint != nullptr) {
		constraint->setEnabled(true);
	}
}

void CPhysicsConstraint::Deactivate() {
	btTypedConstraint *constraint = GetBulletConstraint();
	if (constraint != nullptr) {
		constraint->setEnabled(false);
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
	btTypedConstraint *constraint = GetBulletConstraint();
	constraint->setUserConstraintPtr(static_cast<IPhysicsConstraint *>(this));
	constraint->setEnabled(params.isActive);
}

bool CPhysicsConstraint::AreObjectsValid() const {
	return m_ObjectReference != nullptr && m_ObjectAttached != nullptr &&
			m_ObjectReference != m_ObjectAttached && !m_ObjectAttached->IsStatic();
}

/********
 * Hinge
 ********/

CPhysicsConstraint_Hinge::CPhysicsConstraint_Hinge(
		IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
		const constraint_hingeparams_t &params) :
		CPhysicsConstraint(objectReference, objectAttached) {
	if (!AreObjectsValid()) {
		m_Constraint = nullptr;
		return;
	}

	btRigidBody *rigidBodyA = static_cast<CPhysicsObject *>(m_ObjectAttached)->GetRigidBody();
	btRigidBody *rigidBodyB = static_cast<CPhysicsObject *>(m_ObjectReference)->GetRigidBody();
	const btTransform &transformA = rigidBodyA->getWorldTransform();
	const btTransform &transformB = rigidBodyB->getWorldTransform();

	btVector3 worldPosition, worldAxisDirection;
	ConvertPositionToBullet(params.worldPosition, worldPosition);
	ConvertDirectionToBullet(params.worldAxisDirection, worldAxisDirection);

	m_Constraint = VPhysicsNew(btHingeConstraint, *rigidBodyA, *rigidBodyB,
			transformA.invXform(worldPosition), transformB.invXform(worldPosition),
			worldAxisDirection * transformA.getBasis(), worldAxisDirection * transformB.getBasis());
	InitializeBulletConstraint(params.constraint);
}

btTypedConstraint *CPhysicsConstraint_Hinge::GetBulletConstraint() const {
	return m_Constraint;
}

void CPhysicsConstraint_Hinge::SetAngularMotor(float rotSpeed, float maxAngularImpulse) {
	m_Constraint->enableAngularMotor(rotSpeed != 0.0f, DEG2RAD(rotSpeed), btFabs(DEG2RAD(maxAngularImpulse)));
}

void CPhysicsConstraint_Hinge::DeleteBulletConstraint() {
	VPhysicsDelete(btHingeConstraint, m_Constraint);
	m_Constraint = nullptr;
}

void CPhysicsConstraint_Hinge::Release() {
	VPhysicsDelete(CPhysicsConstraint_Hinge, this);
}

CPhysicsConstraint_Hinge::~CPhysicsConstraint_Hinge() {
	DeleteBulletConstraint();
}
