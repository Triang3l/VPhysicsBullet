// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_object.h"
#include "tier0/dbg.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsObject::CPhysicsObject(btCollisionShape *collisionShape, int materialIndex,
		const Vector &position, const QAngle &angles,
		objectparams_t *pParams, bool isStatic) :
		m_Mass((!isStatic && !collisionShape->isNonMoving()) ? pParams->mass : 0.0f),
		m_Inertia(pParams->inertia, pParams->inertia, pParams->inertia),
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
	if (!m_RigidBody->isStaticObject()) {
		// Forcing because it may be used for external forces without contacts.
		// Also waking up from DISABLE_SIMULATION, which is not possible with setActivationState.
		m_RigidBody->forceActivationState(ACTIVE_TAG);
		m_RigidBody->setDeactivationTime(0.0f);
	}
}

void CPhysicsObject::Sleep() {
	if (!m_RigidBody->isStaticObject()) {
		m_RigidBody->setActivationState(DISABLE_SIMULATION);
	}
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
