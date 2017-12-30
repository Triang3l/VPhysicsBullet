// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_object.h"
#include "physics_environment.h"
#include "tier0/dbg.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsObject::CPhysicsObject(IPhysicsEnvironment *environment,
		btCollisionShape *collisionShape, int materialIndex,
		const Vector &position, const QAngle &angles,
		objectparams_t *pParams, bool isStatic) :
		m_Environment(environment),
		m_Mass((!isStatic && !collisionShape->isNonMoving()) ? pParams->mass : 0.0f),
		m_Inertia(pParams->inertia, pParams->inertia, pParams->inertia),
		m_Damping(pParams->damping), m_RotDamping(pParams->rotdamping),
		m_GameData(nullptr), m_GameFlags(0), m_GameIndex(0),
		m_Callbacks(CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION |
				CALLBACK_FLUID_TOUCH | CALLBACK_GLOBAL_TOUCH |
				CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_DO_FLUID_SIMULATION) {
	btVector3 inertia;
	ConvertDirectionToBullet(m_Inertia, inertia);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo(
			m_Mass, &m_MotionState, collisionShape, inertia);
	BEGIN_BULLET_ALLOCATION();
	m_RigidBody = new btRigidBody(constructionInfo);
	END_BULLET_ALLOCATION();
	m_RigidBody->setUserPointer(this);
}

CPhysicsObject::~CPhysicsObject() {
	// Prevent callbacks to the game code and unlink from this object.
	m_Callbacks = 0;
	m_GameData = nullptr;

	// TODO: Delete or add to the deletion queue.
}

/*******************
 * Mass and inertia
 *******************/

bool CPhysicsObject::IsStatic() const {
	return m_RigidBody->isStaticObject();
}

void CPhysicsObject::SetMass(float mass) {
	Assert(mass > 0.0f);
	if (IsStatic()) {
		return;
	}
	m_Mass = mass;
	btVector3 bulletInertia;
	ConvertDirectionToBullet(m_Inertia, bulletInertia);
	m_RigidBody->setMassProps(mass, bulletInertia);
}

float CPhysicsObject::GetMass() const {
	return m_Mass;
}

float CPhysicsObject::GetInvMass() const {
	return m_RigidBody->getInvMass();
}

Vector CPhysicsObject::GetInertia() const {
	return m_Inertia;
}

Vector CPhysicsObject::GetInvInertia() const {
	Vector inertia;
	ConvertDirectionToHL(m_RigidBody->getInvInertiaDiagLocal(), inertia);
	return inertia;
}

void CPhysicsObject::SetInertia(const Vector &inertia) {
	m_Inertia = inertia;
	btVector3 bulletInertia;
	ConvertDirectionToBullet(inertia, bulletInertia);
	m_RigidBody->setMassProps(m_Mass, bulletInertia);
}

/*******************
 * Activation state
 *******************/

bool CPhysicsObject::IsAsleep() const {
	return !m_RigidBody->isActive();
}

void CPhysicsObject::Wake() {
	if (!IsStatic()) {
		// Forcing because it may be used for external forces without contacts.
		// Also waking up from DISABLE_SIMULATION, which is not possible with setActivationState.
		m_RigidBody->forceActivationState(ACTIVE_TAG);
		m_RigidBody->setDeactivationTime(0.0f);
	}
}

void CPhysicsObject::Sleep() {
	if (!IsStatic()) {
		m_RigidBody->setActivationState(DISABLE_SIMULATION);
	}
}

/**********************
 * Gravity and damping
 **********************/

bool CPhysicsObject::IsGravityEnabled() const {
	return !IsStatic() && !(m_RigidBody->getFlags() & BT_DISABLE_WORLD_GRAVITY);
}

void CPhysicsObject::EnableGravity(bool enable) {
	if (IsStatic()) {
		return;
	}

	int flags = m_RigidBody->getFlags();
	if (enable == !(flags & BT_DISABLE_WORLD_GRAVITY)) {
		return;
	}

	if (enable) {
		const CPhysicsEnvironment *environment = static_cast<CPhysicsEnvironment *>(m_Environment);
		m_RigidBody->setGravity(environment->GetDynamicsWorld()->getGravity());
		m_RigidBody->setFlags(flags & ~BT_DISABLE_WORLD_GRAVITY);
	} else {
		m_RigidBody->setGravity(btVector3(0.0f, 0.0f, 0.0f));
		m_RigidBody->setFlags(flags | BT_DISABLE_WORLD_GRAVITY);
	}
}

void CPhysicsObject::SetDamping(const float *speed, const float *rot) {
	if (speed != nullptr) {
		m_Damping = *speed;
	}
	if (rot != nullptr) {
		m_RotDamping = *rot;
	}
}

void CPhysicsObject::GetDamping(float *speed, float *rot) const {
	if (speed != nullptr) {
		*speed = m_Damping;
	}
	if (rot != nullptr) {
		*rot = m_RotDamping;
	}
}

void CPhysicsObject::ApplyDamping(float timeStep) {
	if (m_RigidBody->isStaticOrKinematicObject() || !IsGravityEnabled()) {
		return;
	}

	const btVector3 &linearVelocity = m_RigidBody->getLinearVelocity();
	const btVector3 &angularVelocity = m_RigidBody->getAngularVelocity();

	btScalar damping = m_Damping, rotDamping = m_RotDamping;
	if (linearVelocity.length2() < 0.01f && angularVelocity.length2() < 0.01f) {
		damping += 0.1f;
		rotDamping += 0.1f;
	}
	damping *= timeStep;
	rotDamping *= timeStep;

	if (damping < 0.25f) {
		damping = btScalar(1.0f) - damping;
	} else {
		damping = btExp(-damping);
	}
	m_RigidBody->setLinearVelocity(linearVelocity * damping);

	if (rotDamping < 0.4f) {
		rotDamping = btScalar(1.0f) - rotDamping;
	} else {
		rotDamping = btExp(-rotDamping);
	}
	m_RigidBody->setAngularVelocity(angularVelocity * rotDamping);
}

/************
 * Game data
 ************/

void CPhysicsObject::SetGameData(void *pGameData) {
	m_GameData = pGameData;
}

void *CPhysicsObject::GetGameData() const {
	return m_GameData;
}

void CPhysicsObject::SetGameFlags(unsigned short userFlags) {
	m_GameFlags = userFlags;
}

unsigned short CPhysicsObject::GetGameFlags() const {
	return m_GameFlags;
}

void CPhysicsObject::SetGameIndex(unsigned short gameIndex) {
	m_GameIndex = gameIndex;
}

unsigned short CPhysicsObject::GetGameIndex() const {
	return m_GameIndex;
}

void CPhysicsObject::SetCallbackFlags(unsigned short callbackflags) {
	m_Callbacks = callbackflags;
}

unsigned short CPhysicsObject::GetCallbackFlags() const {
	return m_Callbacks;
}
