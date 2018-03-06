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

void CPhysicsConstraint::InitializeBulletConstraint(const constraint_breakableparams_t *params) {
	btTypedConstraint *constraint = GetBulletConstraint();
	constraint->setUserConstraintPtr(static_cast<IPhysicsConstraint *>(this));
	if (params != nullptr) {
		constraint->setEnabled(params->isActive); // Active by default in both params and Bullet.
	}
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

	btRigidBody *rigidBodyA = static_cast<CPhysicsObject *>(m_ObjectAttached)->GetRigidBody();
	btRigidBody *rigidBodyB = static_cast<CPhysicsObject *>(m_ObjectReference)->GetRigidBody();
	const btTransform &transformA = rigidBodyA->getCenterOfMassTransform();
	const btTransform &transformB = rigidBodyB->getCenterOfMassTransform();

	btVector3 worldPosition, worldAxisDirection;
	ConvertPositionToBullet(params.worldPosition, worldPosition);
	ConvertDirectionToBullet(params.worldAxisDirection, worldAxisDirection);

	m_Constraint = VPhysicsNew(btHingeConstraint, *rigidBodyA, *rigidBodyB,
			transformA.invXform(worldPosition), transformB.invXform(worldPosition),
			worldAxisDirection * transformA.getBasis(), worldAxisDirection * transformB.getBasis());
	InitializeBulletConstraint(&params.constraint);

	m_MaxAngularImpulse *= static_cast<CPhysicsObject *>(
			m_ObjectAttached)->GetEnvironment()->GetSimulationTimestep();
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
	InitializeBulletConstraint(&params.constraint);
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

/*******************
 * Wheel suspension
 * B attached to A
 *******************/

// TODO: Modify the 6DoF spring since two damping coefficients are needed.
// Maybe also simplify it, get rid of motor stuff if it's not needed.

CPhysicsConstraint_Suspension::CPhysicsConstraint_Suspension(
		IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
		const Vector &wheelPositionInReference, const vehicle_suspensionparams_t &params) :
		CPhysicsConstraint(objectReference, objectAttached) {
	if (!AreObjectsValid()) {
		m_Constraint = nullptr;
		return;
	}

	CPhysicsObject *objectA = static_cast<CPhysicsObject *>(m_ObjectReference);
	CPhysicsObject *objectB = static_cast<CPhysicsObject *>(m_ObjectAttached);

	btTransform frameInA;
	frameInA.getBasis().setIdentity();
	ConvertPositionToBullet(wheelPositionInReference, frameInA.getOrigin());
	frameInA.getOrigin() -= objectA->GetBulletMassCenter();
	// Force the body's coordinate system for wheels, so frame in B is identity.
	// Assume that the wheel is rotated the same as the body when creating.

	m_Constraint = VPhysicsNew(btGeneric6DofSpring2Constraint,
			*objectA->GetRigidBody(), *objectB->GetRigidBody(),
			frameInA, btTransform::getIdentity(), RO_YZX /* Naming is in reverse */);
	InitializeBulletConstraint();

	btScalar bodyMass = objectA->GetMass();
	m_Constraint->setLinearLowerLimit(btVector3(0.0f, 1.0f, 0.0f));
	m_Constraint->setLinearUpperLimit(btVector3(0.0f, -1.0f, 0.0f));
	m_Constraint->enableSpring(1, true);
	m_Constraint->setStiffness(1, params.springConstant * bodyMass);
	m_Constraint->setDamping(1, params.springDampingCompression * bodyMass);
	m_Constraint->setAngularLowerLimit(btVector3(1.0f, 1.0f, 0.0f));
	m_Constraint->setAngularUpperLimit(btVector3(-1.0f, -1.0f, 0.0f));
}

btTypedConstraint *CPhysicsConstraint_Suspension::GetBulletConstraint() const {
	return m_Constraint;
}

void CPhysicsConstraint_Suspension::DeleteBulletConstraint() {
	VPhysicsDelete(btGeneric6DofSpring2Constraint, m_Constraint);
	m_Constraint = nullptr;
}

void CPhysicsConstraint_Suspension::Release() {
	VPhysicsDelete(CPhysicsConstraint_Suspension, this);
}

CPhysicsConstraint_Suspension::~CPhysicsConstraint_Suspension() {
	DeleteBulletConstraint();
}
