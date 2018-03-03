// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_constraint.h"
#include "physics_object.h"

// TODO: Breakability, mass ratio, etc.

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

/******************
 * Hinge
 * A attached to B
 ******************/

CPhysicsConstraint_Hinge::CPhysicsConstraint_Hinge(
		IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
		const constraint_hingeparams_t &params) :
		CPhysicsConstraint(objectReference, objectAttached),
		m_TargetAngularVelocity(DEG2RAD(params.hingeAxis.angularVelocity)),
		m_MaxAngularImpulse(btFabs(DEG2RAD(params.hingeAxis.torque))) {
	if (!AreObjectsValid()) {
		m_Constraint = nullptr;
		return;
	}

	CPhysicsObject *objectA = static_cast<CPhysicsObject *>(m_ObjectAttached);
	CPhysicsObject *objectB = static_cast<CPhysicsObject *>(m_ObjectReference);
	btRigidBody *rigidBodyA = objectA->GetRigidBody();
	btRigidBody *rigidBodyB = objectB->GetRigidBody();
	const btTransform &transformA = objectA->GetInterPSIWorldTransform();
	const btTransform &transformB = objectB->GetInterPSIWorldTransform();

	btVector3 worldPosition, worldAxisDirection;
	ConvertPositionToBullet(params.worldPosition, worldPosition);
	ConvertDirectionToBullet(params.worldAxisDirection, worldAxisDirection);

	// Revolving around Z in the frame.

	btVector3 frameZ = worldAxisDirection * transformA.getBasis();
	btVector3 frameX, frameY;
	btPlaneSpace1(frameZ, frameX, frameY);
	btTransform frameInA;
	frameInA.getBasis().setValue(frameX.getX(), frameY.getX(), frameZ.getX(),
			frameX.getY(), frameY.getY(), frameZ.getY(),
			frameX.getZ(), frameY.getZ(), frameZ.getZ());
	frameInA.setOrigin(transformA.invXform(worldPosition));

	frameZ = worldAxisDirection * transformB.getBasis();
	btPlaneSpace1(frameZ, frameX, frameY);
	btTransform frameInB;
	frameInB.getBasis().setValue(frameX.getX(), frameY.getX(), frameZ.getX(),
			frameX.getY(), frameY.getY(), frameZ.getY(),
			frameX.getZ(), frameY.getZ(), frameZ.getZ());
	frameInB.setOrigin(transformB.invXform(worldPosition));

	m_Constraint = VPhysicsNew(btHingeConstraint, *rigidBodyA, *rigidBodyB, frameInA, frameInB, false);
	InitializeBulletConstraint(params.constraint);

	m_MaxAngularImpulse *= objectA->GetEnvironment()->GetSimulationTimestep();
	m_Constraint->enableAngularMotor(m_MaxAngularImpulse != 0.0f,
			m_TargetAngularVelocity, m_MaxAngularImpulse);
}

btTypedConstraint *CPhysicsConstraint_Hinge::GetBulletConstraint() const {
	return m_Constraint;
}

void CPhysicsConstraint_Hinge::SetAngularMotor(float rotSpeed, float maxAngularImpulse) {
	if (m_Constraint != nullptr) {
		if (rotSpeed != 0.0f) {
			m_Constraint->enableAngularMotor(rotSpeed != 0.0f,
					DEG2RAD(rotSpeed), btFabs(DEG2RAD(maxAngularImpulse)));
		} else {
			m_Constraint->enableAngularMotor(m_MaxAngularImpulse != 0.0f,
					m_TargetAngularVelocity, m_MaxAngularImpulse);
		}
	}
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

/******************
 * Ball and socket
 * A attached to B
 ******************/

CPhysicsConstraint_Ballsocket::CPhysicsConstraint_Ballsocket(
		IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
		const constraint_ballsocketparams_t &params) :
		CPhysicsConstraint(objectReference, objectAttached) {
	if (!AreObjectsValid()) {
		m_Constraint = nullptr;
		return;
	}

	CPhysicsObject *objectA = static_cast<CPhysicsObject *>(m_ObjectAttached);
	CPhysicsObject *objectB = static_cast<CPhysicsObject *>(m_ObjectReference);
	const btTransform &transformA = objectA->GetInterPSIWorldTransform();
	const btTransform &transformB = objectB->GetInterPSIWorldTransform();

	btVector3 objectLocalPositionA, objectLocalPositionB;
	ConvertPositionToBullet(params.constraintPosition[1], objectLocalPositionA);
	ConvertPositionToBullet(params.constraintPosition[0], objectLocalPositionB);

	m_Constraint = VPhysicsNew(btPoint2PointConstraint, *objectA->GetRigidBody(), *objectB->GetRigidBody(),
			objectLocalPositionA - (transformA.getBasis() * objectA->GetBulletMassCenter()),
			objectLocalPositionB - (transformB.getBasis() * objectB->GetBulletMassCenter()));
	InitializeBulletConstraint(params.constraint);
}

btTypedConstraint *CPhysicsConstraint_Ballsocket::GetBulletConstraint() const {
	return m_Constraint;
}

void CPhysicsConstraint_Ballsocket::DeleteBulletConstraint() {
	VPhysicsDelete(btPoint2PointConstraint, m_Constraint);
	m_Constraint = nullptr;
}

void CPhysicsConstraint_Ballsocket::Release() {
	VPhysicsDelete(CPhysicsConstraint_Ballsocket, this);
}

CPhysicsConstraint_Ballsocket::~CPhysicsConstraint_Ballsocket() {
	DeleteBulletConstraint();
}
