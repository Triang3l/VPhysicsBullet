// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_object.h"
#include "physics_collision.h"
#include "physics_environment.h"
#include "bspflags.h"
#include "tier0/dbg.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsObject::CPhysicsObject(IPhysicsEnvironment *environment,
		btCollisionShape *collisionShape, int materialIndex,
		const Vector &position, const QAngle &angles,
		objectparams_t *pParams, bool isStatic) :
		m_Environment(environment),
		m_CollideObjectNext(this), m_CollideObjectPrevious(this),
		m_MassCenterOverride(0.0f, 0.0f, 0.0f), m_MassCenterOverrideShape(nullptr),
		m_Mass((!isStatic && !collisionShape->isNonMoving()) ? pParams->mass : 0.0f),
		m_Inertia(pParams->inertia, pParams->inertia, pParams->inertia),
		m_Damping(pParams->damping), m_RotDamping(pParams->rotdamping),
		m_GameData(nullptr), m_GameFlags(0), m_GameIndex(0),
		m_Callbacks(CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION |
				CALLBACK_FLUID_TOUCH | CALLBACK_GLOBAL_TOUCH |
				CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_DO_FLUID_SIMULATION),
		m_ContentsMask(CONTENTS_SOLID) {
	VectorAbs(m_Inertia, m_Inertia);
	btVector3 inertia;
	ConvertDirectionToBullet(m_Inertia, inertia);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo(
			m_Mass, nullptr, collisionShape, inertia.absolute());

	const Vector *massCenterOverride = pParams->massCenterOverride;
	if (massCenterOverride != nullptr && *massCenterOverride != vec3_origin) {
		ConvertPositionToBullet(*massCenterOverride, m_MassCenterOverride);
		BEGIN_BULLET_ALLOCATION();
		m_MassCenterOverrideShape = new btCompoundShape(false, 1);
		m_MassCenterOverrideShape->addChildShape(btTransform(btMatrix3x3::getIdentity(),
				g_pPhysCollision->CollideGetBulletMassCenter(collisionShape) - m_MassCenterOverride),
				collisionShape);
		END_BULLET_ALLOCATION();
		constructionInfo.m_collisionShape = m_MassCenterOverrideShape;
	}

	matrix3x4_t startMatrix;
	AngleMatrix(angles, position, startMatrix);
	ConvertMatrixToBullet(startMatrix, constructionInfo.m_startWorldTransform);
	btTransform &startWorldTransform = constructionInfo.m_startWorldTransform;
	startWorldTransform.getOrigin() += startWorldTransform.getBasis() *
			g_pPhysCollision->CollideGetBulletMassCenter(collisionShape);

	BEGIN_BULLET_ALLOCATION();
	m_RigidBody = new btRigidBody(constructionInfo);
	END_BULLET_ALLOCATION();
	m_RigidBody->setUserPointer(this);

	AddReferenceToCollide();
}

CPhysicsObject::~CPhysicsObject() {
	// Prevent callbacks to the game code and unlink from this object.
	m_Callbacks = 0;
	m_GameData = nullptr;

	// TODO: Delete or add to the deletion queue.
}

btCollisionShape *CPhysicsObject::GetCollisionShape() const {
	if (m_MassCenterOverrideShape != nullptr) {
		return m_MassCenterOverrideShape->getChildShape(0);
	}
	return m_RigidBody->getCollisionShape();
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
	m_RigidBody->setMassProps(mass, bulletInertia.absolute());
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
	VectorAbs(inertia, inertia);
	return inertia;
}

void CPhysicsObject::SetInertia(const Vector &inertia) {
	m_Inertia = inertia;
	VectorAbs(m_Inertia, m_Inertia);
	btVector3 bulletInertia;
	ConvertDirectionToBullet(inertia, bulletInertia);
	m_RigidBody->setMassProps(m_Mass, bulletInertia.absolute());
}

bool CPhysicsObject::IsMotionEnabled() const {
	return !m_RigidBody->getAngularFactor().isZero();
}

bool CPhysicsObject::IsMoveable() const {
	return !IsStatic() && IsMotionEnabled();
}

void CPhysicsObject::EnableMotion(bool enable) {
	if (IsMotionEnabled() == enable) {
		return;
	}

	btVector3 zero(0.0f, 0.0f, 0.0f);

	// IVP clears velocity even if unpinning.
	m_RigidBody->clearForces();
	m_RigidBody->setLinearVelocity(zero);
	m_RigidBody->setAngularVelocity(zero);

	if (enable) {
		btVector3 one(1.0f, 1.0f, 1.0f);
		m_RigidBody->setLinearFactor(one);
		m_RigidBody->setAngularFactor(one);
	} else {
		m_RigidBody->setLinearFactor(zero);
		m_RigidBody->setAngularFactor(zero);
	}
}

/*******************
 * Activation state
 *******************/

bool CPhysicsObject::IsAsleep() const {
	return !m_RigidBody->isActive();
}

void CPhysicsObject::Wake() {
	if (!IsStatic() && m_RigidBody->getActivationState() != DISABLE_DEACTIVATION) {
		// Forcing because it may be used for external forces without contacts.
		// Also waking up from DISABLE_SIMULATION, which is not possible with setActivationState.
		m_RigidBody->forceActivationState(ACTIVE_TAG);
		m_RigidBody->setDeactivationTime(0.0f);
	}
}

void CPhysicsObject::Sleep() {
	if (!IsStatic() && m_RigidBody->getActivationState() != DISABLE_DEACTIVATION) {
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

unsigned int CPhysicsObject::GetContents() const {
	return m_ContentsMask;
}

void CPhysicsObject::SetContents(unsigned int contents) {
	m_ContentsMask = contents;
}

/**********************
 * Position and forces
 **********************/

const btVector3 &CPhysicsObject::GetBulletMassCenter() const {
	if (m_MassCenterOverrideShape != nullptr) {
		return m_MassCenterOverride;
	}
	return g_pPhysCollision->CollideGetBulletMassCenter(m_RigidBody->getCollisionShape());
}

Vector CPhysicsObject::GetMassCenterLocalSpace() const {
	Vector massCenter;
	ConvertPositionToHL(GetBulletMassCenter(), massCenter);
	return massCenter;
}

void CPhysicsObject::NotifyMassCenterChanged(const btVector3 &oldMassCenter) {
	btVector3 offset = g_pPhysCollision->CollideGetBulletMassCenter(GetCollisionShape()) - oldMassCenter;
	if (m_MassCenterOverrideShape != nullptr) {
		btTransform childTransform = m_MassCenterOverrideShape->getChildTransform(0);
		childTransform.getOrigin() -= offset;
		m_MassCenterOverrideShape->updateChildTransform(0, childTransform);
	} else {
		btTransform worldTransform = m_RigidBody->getWorldTransform();
		worldTransform.getOrigin() -= worldTransform.getBasis() * offset;
		m_RigidBody->setWorldTransform(worldTransform);
	}
}

void CPhysicsObject::GetPosition(Vector *worldPosition, QAngle *angles) const {
	const btTransform &transform = m_RigidBody->getWorldTransform();
	const btMatrix3x3 &basis = transform.getBasis();
	if (worldPosition != nullptr) {
		ConvertPositionToHL(transform.getOrigin() - basis * GetBulletMassCenter(), *worldPosition);
	}
	if (angles != nullptr) {
		ConvertRotationToHL(basis, *angles);
	}
}

void CPhysicsObject::GetPositionMatrix(matrix3x4_t *positionMatrix) const {
	const btTransform &transform = m_RigidBody->getWorldTransform();
	const btMatrix3x3 &basis = transform.getBasis();
	btVector3 origin = transform.getOrigin() - basis * GetBulletMassCenter();
	ConvertMatrixToHL(basis, origin, *positionMatrix);
}

void CPhysicsObject::LocalToWorld(Vector *worldPosition, const Vector &localPosition) const {
	matrix3x4_t matrix;
	GetPositionMatrix(&matrix);
	// Copy in case src == dest.
	VectorTransform(Vector(localPosition), matrix, *worldPosition);
}

void CPhysicsObject::WorldToLocal(Vector *localPosition, const Vector &worldPosition) const {
	matrix3x4_t matrix;
	GetPositionMatrix(&matrix);
	// Copy in case src == dest.
	VectorITransform(Vector(worldPosition), matrix, *localPosition);
}

void CPhysicsObject::LocalToWorldVector(Vector *worldVector, const Vector &localVector) const {
	matrix3x4_t matrix;
	GetPositionMatrix(&matrix);
	// Copy in case src == dest.
	VectorRotate(Vector(localVector), matrix, *worldVector);
}

void CPhysicsObject::WorldToLocalVector(Vector *localVector, const Vector &worldVector) const {
	matrix3x4_t matrix;
	GetPositionMatrix(&matrix);
	// Copy in case src == dest.
	VectorIRotate(Vector(worldVector), matrix, *localVector);
}

void CPhysicsObject::SetVelocity(const Vector *velocity, const AngularImpulse *angularVelocity) {
	if (!IsMoveable()) {
		return;
	}
	Wake();
	btVector3 bulletForce = m_RigidBody->getTotalForce();
	btVector3 bulletTorque = m_RigidBody->getTotalTorque();
	m_RigidBody->clearForces();
	btVector3 zero(0.0f, 0.0f, 0.0f);
	if (velocity != nullptr) {
		ConvertPositionToBullet(*velocity, bulletForce);
		m_RigidBody->setLinearVelocity(zero);
	}
	if (angularVelocity != nullptr) {
		AngularImpulse torque;
		LocalToWorldVector(&torque, *angularVelocity);
		ConvertAngularImpulseToBullet(torque, bulletTorque);
		m_RigidBody->setAngularVelocity(zero);
	}
	m_RigidBody->applyCentralForce(bulletForce);
	m_RigidBody->applyTorque(bulletTorque);
}

void CPhysicsObject::ApplyForceCenter(const Vector &forceVector) {
	if (!IsMoveable()) {
		return;
	}

	btVector3 bulletForce;
	ConvertForceImpulseToBullet(forceVector, bulletForce);

	btScalar maxSpeed = static_cast<CPhysicsEnvironment *>(m_Environment)->GetMaxSpeed();
	btClamp(bulletForce[0], -maxSpeed, maxSpeed);
	btClamp(bulletForce[1], -maxSpeed, maxSpeed);
	btClamp(bulletForce[2], -maxSpeed, maxSpeed);

	m_RigidBody->applyCentralForce(bulletForce);
	Wake();
}

void CPhysicsObject::ApplyForceOffset(const Vector &forceVector, const Vector &worldPosition) {
	if (!IsMoveable()) {
		return;
	}

	btVector3 bulletForce;
	ConvertForceImpulseToBullet(forceVector, bulletForce);

	Vector localPosition;
	WorldToLocal(&localPosition, worldPosition);
	btVector3 bulletRelPos;
	ConvertPositionToBullet(localPosition, bulletRelPos);
	btVector3 bulletTorque = bulletRelPos.cross(bulletForce * m_RigidBody->getLinearFactor());

	const CPhysicsEnvironment *environment = static_cast<CPhysicsEnvironment *>(m_Environment);

	btScalar maxSpeed = environment->GetMaxSpeed();
	btClamp(bulletForce[0], -maxSpeed, maxSpeed);
	btClamp(bulletForce[1], -maxSpeed, maxSpeed);
	btClamp(bulletForce[2], -maxSpeed, maxSpeed);

	btScalar maxAngularSpeed = environment->GetMaxAngularSpeed();
	btClamp(bulletTorque[0], -maxAngularSpeed, maxAngularSpeed);
	btClamp(bulletTorque[1], -maxAngularSpeed, maxAngularSpeed);
	btClamp(bulletTorque[2], -maxAngularSpeed, maxAngularSpeed);

	m_RigidBody->applyCentralForce(bulletForce);
	m_RigidBody->applyTorque(bulletTorque);
	Wake();
}

// In world space.
void CPhysicsObject::ApplyTorqueCenter(const AngularImpulse &torque) {
	if (!IsMoveable()) {
		return;
	}

	btVector3 bulletTorque;
	ConvertAngularImpulseToBullet(torque, bulletTorque);

	btScalar maxAngularSpeed = static_cast<CPhysicsEnvironment *>(m_Environment)->GetMaxAngularSpeed();
	btClamp(bulletTorque[0], -maxAngularSpeed, maxAngularSpeed);
	btClamp(bulletTorque[1], -maxAngularSpeed, maxAngularSpeed);
	btClamp(bulletTorque[2], -maxAngularSpeed, maxAngularSpeed);

	m_RigidBody->applyTorque(bulletTorque);
	Wake();
}

/***************************************
 * Collide object reference linked list
 ***************************************/

void CPhysicsObject::AddReferenceToCollide() {
	btCollisionShape *shape = GetCollisionShape();
	void *nextPointer = shape->getUserPointer();
	shape->setUserPointer(static_cast<IPhysicsObject *>(this));
	if (nextPointer != nullptr) {
		m_CollideObjectNext = static_cast<CPhysicsObject *>(
				reinterpret_cast<IPhysicsObject *>(nextPointer));
		m_CollideObjectPrevious = m_CollideObjectNext->m_CollideObjectPrevious;
		m_CollideObjectNext->m_CollideObjectPrevious = this;
		m_CollideObjectPrevious->m_CollideObjectNext = this;
	} else {
		m_CollideObjectNext = m_CollideObjectPrevious = this;
	}
}

void CPhysicsObject::RemoveReferenceFromCollide() {
	btCollisionShape *shape = GetCollisionShape();
	void *nextPointer = shape->getUserPointer();
	if (nextPointer != nullptr && reinterpret_cast<IPhysicsObject *>(nextPointer) == this) {
		shape->setUserPointer(m_CollideObjectNext != this ?
				static_cast<IPhysicsObject *>(m_CollideObjectNext) : nullptr);
	}
	m_CollideObjectNext->m_CollideObjectPrevious = m_CollideObjectPrevious;
	m_CollideObjectPrevious->m_CollideObjectNext = m_CollideObjectNext;
}
