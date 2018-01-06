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
		CPhysCollide *collide, int materialIndex,
		const Vector &position, const QAngle &angles,
		objectparams_t *pParams, bool isStatic) :
		m_Environment(environment),
		m_CollideObjectNext(this), m_CollideObjectPrevious(this),
		m_MassCenterOverride(0.0f, 0.0f, 0.0f),
		m_Mass((!isStatic && !collide->GetShape()->isNonMoving()) ? pParams->mass : 0.0f),
		m_HingeAxis(-1),
		m_Damping(pParams->damping), m_RotDamping(pParams->rotdamping),
		m_GameData(pParams->pGameData), m_GameFlags(0), m_GameIndex(0),
		m_Callbacks(CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION |
				CALLBACK_FLUID_TOUCH | CALLBACK_GLOBAL_TOUCH |
				CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_DO_FLUID_SIMULATION),
		m_ContentsMask(CONTENTS_SOLID),
		m_LinearVelocityChange(0.0f, 0.0f, 0.0f),
		m_LocalAngularVelocityChange(0.0f, 0.0f, 0.0f) {
	btRigidBody::btRigidBodyConstructionInfo constructionInfo(
			m_Mass, nullptr, collide->GetShape(), collide->GetInertia());

	btVector3 massCenter = collide->GetMassCenter();
	const Vector *massCenterOverride = pParams->massCenterOverride;
	if (massCenterOverride != nullptr && *massCenterOverride != vec3_origin) {
		ConvertPositionToBullet(*massCenterOverride, m_MassCenterOverride);
		BEGIN_BULLET_ALLOCATION();
		btCompoundShape *massCenterOverrideShape = new btCompoundShape(false, 1);
		btVector3 massCenterOffset = m_MassCenterOverride - massCenter;
		massCenterOverrideShape->addChildShape(btTransform(btMatrix3x3::getIdentity(),
				-massCenterOffset), collide->GetShape());
		END_BULLET_ALLOCATION();
		constructionInfo.m_collisionShape = massCenterOverrideShape;
		massCenter = m_MassCenterOverride;
		constructionInfo.m_localInertia = CPhysicsCollision::OffsetInertia(
				constructionInfo.m_localInertia, massCenterOffset).absolute();
	}
	constructionInfo.m_localInertia *= pParams->inertia * m_Mass;
	if (pParams->rotInertiaLimit > 0.0f) {
		btScalar minInertia = constructionInfo.m_localInertia.length() * pParams->rotInertiaLimit;
		constructionInfo.m_localInertia.setMax(btVector3(minInertia, minInertia, minInertia));
	}
	ConvertDirectionToHL(constructionInfo.m_localInertia, m_Inertia);
	VectorAbs(m_Inertia, m_Inertia);

	matrix3x4_t startMatrix;
	AngleMatrix(angles, position, startMatrix);
	btTransform &startWorldTransform = constructionInfo.m_startWorldTransform;
	ConvertMatrixToBullet(startMatrix, startWorldTransform);
	startWorldTransform.getOrigin() += startWorldTransform.getBasis() * massCenter;

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

	// TODO: Do deletion actions such as unlinking from controllers.

	static_cast<CPhysicsEnvironment *>(m_Environment)->NotifyObjectRemoving(this);

	btCollisionShape *shape = m_RigidBody->getCollisionShape();
	if (shape->getUserPointer() == nullptr) {
		// Delete the mass override shape.
		m_RigidBody->setCollisionShape(static_cast<btCompoundShape *>(shape)->getChildShape(0));
		delete shape;
	}
	RemoveReferenceFromCollide();

	delete m_RigidBody;
}

CPhysCollide *CPhysicsObject::GetCollide() {
	btCollisionShape *shape = m_RigidBody->getCollisionShape();
	if (shape->getUserPointer() == nullptr) {
		// Overriding mass center.
		shape = static_cast<btCompoundShape *>(shape)->getChildShape(0);
	}
	return reinterpret_cast<CPhysCollide *>(shape->getUserPointer());
}

const CPhysCollide *CPhysicsObject::GetCollide() const {
	const btCollisionShape *shape = m_RigidBody->getCollisionShape();
	if (shape->getUserPointer() == nullptr) {
		// Overriding mass center.
		shape = static_cast<const btCompoundShape *>(shape)->getChildShape(0);
	}
	return reinterpret_cast<const CPhysCollide *>(shape->getUserPointer());
}

float CPhysicsObject::GetSphereRadius() const {
	const CPhysCollide *collide = GetCollide();
	if (CPhysCollide_Sphere::IsSphere(collide)) {
		return BULLET2HL(static_cast<const CPhysCollide_Sphere *>(collide)->GetSphereShape()->getRadius());
	}
	return 0.0f;
}

void CPhysicsObject::NotifyTransferred(IPhysicsEnvironment *newEnvironment) {
	m_Environment = newEnvironment;
	Assert(!IsTouchingTriggers());
}

/*******************
 * Mass and inertia
 *******************/

bool CPhysicsObject::IsStatic() const {
	return m_RigidBody->isStaticObject();
}

void CPhysicsObject::UpdateMassProps(bool inertiaChanged) {
	btVector3 bulletInertia;
	ConvertDirectionToBullet(m_Inertia, bulletInertia);
	bulletInertia = bulletInertia.absolute();
	if (m_HingeAxis >= 0) {
		bulletInertia[m_HingeAxis] = 50000.0f;
	}
	m_RigidBody->setMassProps(m_Mass, bulletInertia);
	if (inertiaChanged) {
		m_RigidBody->updateInertiaTensor();
	}
}

void CPhysicsObject::SetMass(float mass) {
	Assert(mass > 0.0f);
	if (IsStatic()) {
		return;
	}
	m_Mass = mass;
	UpdateMassProps(false);
}

float CPhysicsObject::GetMass() const {
	return m_Mass;
}

float CPhysicsObject::GetInvMass() const {
	return m_RigidBody->getInvMass();
}

Vector CPhysicsObject::GetInertia() const {
	Vector inertia = m_Inertia;
	if (m_HingeAxis >= 0) {
		inertia[ConvertCoordinateAxisToHL(m_HingeAxis)] = 50000.0f;
	}
	return m_Inertia;
}

Vector CPhysicsObject::GetInvInertia() const {
	Vector inertia;
	ConvertDirectionToHL(m_RigidBody->getInvInertiaDiagLocal(), inertia);
	VectorAbs(inertia, inertia);
	return inertia;
}

void CPhysicsObject::SetInertia(const Vector &inertia) {
	if (!IsStatic()) {
		return;
	}
	VectorAbs(inertia, m_Inertia);
	UpdateMassProps(true);
}

bool CPhysicsObject::IsHinged() const {
	return m_HingeAxis >= 0;
}

void CPhysicsObject::BecomeHinged(int localAxis) {
	Assert(localAxis >= 0 && localAxis <= 2);
	if (IsStatic()) {
		return;
	}
	int bulletAxis = ConvertCoordinateAxisToBullet(localAxis);
	if (m_HingeAxis == bulletAxis) {
		return;
	}
	m_HingeAxis = bulletAxis;
	UpdateMassProps(true);
}

void CPhysicsObject::RemoveHinged() {
	if (!IsHinged()) {
		return;
	}
	m_HingeAxis = -1;
	UpdateMassProps(true);
}

bool CPhysicsObject::IsMotionEnabled() const {
	return !m_RigidBody->getLinearFactor().isZero();
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
	m_LinearVelocityChange.setZero();
	m_LocalAngularVelocityChange.setZero();

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
	if (m_RigidBody->getCollisionShape()->getUserPointer() == nullptr) {
		return m_MassCenterOverride;
	}
	return GetCollide()->GetMassCenter();
}

Vector CPhysicsObject::GetMassCenterLocalSpace() const {
	Vector massCenter;
	ConvertPositionToHL(GetBulletMassCenter(), massCenter);
	return massCenter;
}

void CPhysicsObject::NotifyMassCenterChanged(const btVector3 &oldMassCenter) {
	const btVector3 &newMassCenter = GetCollide()->GetMassCenter();
	btCollisionShape *shape = m_RigidBody->getCollisionShape();
	if (shape->getUserPointer() == nullptr) {
		btCompoundShape *compoundShape = static_cast<btCompoundShape *>(shape);
		btTransform childTransform = compoundShape->getChildTransform(0);
		childTransform.setOrigin(newMassCenter - m_MassCenterOverride);
		compoundShape->updateChildTransform(0, childTransform);
	} else {
		// Updates the same properties as setCenterOfMassTransform.
		btVector3 offset = newMassCenter - oldMassCenter;
		btTransform worldTransform = m_RigidBody->getWorldTransform();
		btVector3 worldOffset = worldTransform.getBasis() * offset;
		worldTransform.getOrigin() += worldOffset;
		m_RigidBody->setWorldTransform(worldTransform);
		btTransform interpolationWorldTransform = m_RigidBody->getInterpolationWorldTransform();
		interpolationWorldTransform.getOrigin() += worldOffset;
		m_RigidBody->setInterpolationWorldTransform(interpolationWorldTransform);
		// TODO: Nothing is done to inertia. But SetCollideMassCenter isn't called for live collides apparently.
	}
}

void CPhysicsObject::SetPosition(const Vector &worldPosition, const QAngle &angles, bool isTeleport) {
	// TODO: Update the shadow.
	matrix3x4_t matrix;
	AngleMatrix(angles, worldPosition, matrix);
	btTransform transform;
	ConvertMatrixToBullet(matrix, transform);
	transform.getOrigin() += transform.getBasis() * GetBulletMassCenter();
	m_RigidBody->proceedToTransform(transform);
}

void CPhysicsObject::SetPositionMatrix(const matrix3x4_t &matrix, bool isTeleport) {
	// TODO: Update the shadow.
	btTransform transform;
	ConvertMatrixToBullet(matrix, transform);
	transform.getOrigin() += transform.getBasis() * GetBulletMassCenter();
	m_RigidBody->proceedToTransform(transform);
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

void CPhysicsObject::ApplyForcesAndSpeedLimit() {
	if (IsMoveable() && !IsAsleep()) {
		const CPhysicsEnvironment *environment = static_cast<const CPhysicsEnvironment *>(m_Environment);

		btVector3 linearVelocity = m_RigidBody->getLinearVelocity();
		linearVelocity += m_LinearVelocityChange * m_RigidBody->getLinearFactor();
		btScalar maxSpeed = environment->GetMaxSpeed();
		btClamp(linearVelocity[0], -maxSpeed, maxSpeed);
		btClamp(linearVelocity[1], -maxSpeed, maxSpeed);
		btClamp(linearVelocity[2], -maxSpeed, maxSpeed);
		m_RigidBody->setLinearVelocity(linearVelocity);

		btVector3 angularVelocity = m_RigidBody->getAngularVelocity();
		angularVelocity += (m_RigidBody->getWorldTransform().getBasis() * m_LocalAngularVelocityChange) *
				m_RigidBody->getAngularFactor();
		btScalar maxAngularSpeed = environment->GetMaxAngularSpeed();
		btClamp(angularVelocity[0], -maxAngularSpeed, maxAngularSpeed);
		btClamp(angularVelocity[1], -maxAngularSpeed, maxAngularSpeed);
		btClamp(angularVelocity[2], -maxAngularSpeed, maxAngularSpeed);
		m_RigidBody->setAngularVelocity(angularVelocity);
	}

	m_LinearVelocityChange.setZero();
	m_LocalAngularVelocityChange.setZero();
}

void CPhysicsObject::SetVelocity(const Vector *velocity, const AngularImpulse *angularVelocity) {
	if (!IsMoveable()) {
		return;
	}
	Wake();
	btVector3 zero(0.0f, 0.0f, 0.0f);
	if (velocity != nullptr) {
		ConvertPositionToBullet(*velocity, m_LinearVelocityChange);
		m_RigidBody->setLinearVelocity(zero);
	}
	if (angularVelocity != nullptr) {
		ConvertAngularImpulseToBullet(*angularVelocity, m_LocalAngularVelocityChange);
		m_RigidBody->setAngularVelocity(zero);
	}
}

void CPhysicsObject::SetVelocityInstantaneous(const Vector *velocity, const AngularImpulse *angularVelocity) {
	if (!IsMoveable()) {
		return;
	}

	Wake();

	const CPhysicsEnvironment *environment = static_cast<const CPhysicsEnvironment *>(m_Environment);

	if (velocity != nullptr) {
		btVector3 bulletVelocity;
		ConvertPositionToBullet(*velocity, bulletVelocity);
		btScalar maxSpeed = environment->GetMaxSpeed();
		btClamp(bulletVelocity[0], -maxSpeed, maxSpeed);
		btClamp(bulletVelocity[1], -maxSpeed, maxSpeed);
		btClamp(bulletVelocity[2], -maxSpeed, maxSpeed);
		m_RigidBody->setLinearVelocity(bulletVelocity);
		m_LinearVelocityChange.setZero();
	}

	if (angularVelocity != nullptr) {
		btVector3 bulletAngularVelocity;
		ConvertAngularImpulseToBullet(*angularVelocity, bulletAngularVelocity);
		bulletAngularVelocity = m_RigidBody->getWorldTransform().getBasis() * bulletAngularVelocity;
		btScalar maxAngularSpeed = environment->GetMaxAngularSpeed();
		btClamp(bulletAngularVelocity[0], -maxAngularSpeed, maxAngularSpeed);
		btClamp(bulletAngularVelocity[1], -maxAngularSpeed, maxAngularSpeed);
		btClamp(bulletAngularVelocity[2], -maxAngularSpeed, maxAngularSpeed);
		m_RigidBody->setAngularVelocity(bulletAngularVelocity);
	}
}

void CPhysicsObject::GetVelocity(Vector *velocity, AngularImpulse *angularVelocity) const {
	if (velocity != nullptr) {
		ConvertPositionToHL(m_RigidBody->getLinearVelocity() + m_LinearVelocityChange, *velocity);
	}
	if (angularVelocity != nullptr) {
		AngularImpulse worldAngularVelocity;
		ConvertAngularImpulseToHL(m_RigidBody->getAngularVelocity() +
				(m_RigidBody->getWorldTransform().getBasis() * m_LocalAngularVelocityChange),
				worldAngularVelocity);
		WorldToLocalVector(angularVelocity, worldAngularVelocity);
	}
}

void CPhysicsObject::GetVelocityAtPoint(const Vector &worldPosition, Vector *pVelocity) const {
	const btTransform &worldTransform = m_RigidBody->getWorldTransform();

	btVector3 angularVelocity = m_RigidBody->getAngularVelocity() +
			(worldTransform.getBasis() * m_LocalAngularVelocityChange);

	btVector3 bulletWorldPosition;
	ConvertPositionToBullet(worldPosition, bulletWorldPosition);
	btVector3 relativePosition = bulletWorldPosition - (worldTransform.getOrigin() -
			worldTransform.getBasis() * GetBulletMassCenter());

	btVector3 speed = m_RigidBody->getLinearVelocity() +
			angularVelocity.cross(relativePosition) + m_LinearVelocityChange;
	ConvertPositionToHL(speed, *pVelocity);
}

void CPhysicsObject::AddVelocity(const Vector *velocity, const AngularImpulse *angularVelocity) {
	if (!IsMoveable()) {
		return;
	}
	Wake();
	if (velocity != nullptr) {
		btVector3 bulletVelocity;
		ConvertPositionToBullet(*velocity, bulletVelocity);
		m_LinearVelocityChange += bulletVelocity;
	}
	if (angularVelocity != nullptr) {
		btVector3 bulletAngularVelocity;
		ConvertAngularImpulseToBullet(*angularVelocity, bulletAngularVelocity);
		m_LocalAngularVelocityChange += bulletAngularVelocity;
	}
}

float CPhysicsObject::GetEnergy() const {
	const btVector3 &angularVelocity = m_RigidBody->getAngularVelocity();
	// 1/2mv^2 + 1/2Iw^2
	return ConvertEnergyToHL(0.5f * (
			btScalar(GetMass()) * m_RigidBody->getLinearVelocity().length2() +
			(m_RigidBody->getInvInertiaTensorWorld() * angularVelocity).dot(angularVelocity)));
}

void CPhysicsObject::ApplyForceCenter(const Vector &forceVector) {
	if (!IsMoveable()) {
		return;
	}
	btVector3 bulletForce;
	ConvertForceImpulseToBullet(forceVector, bulletForce);
	m_LinearVelocityChange += bulletForce * m_RigidBody->getInvMass();
	Wake();
}

void CPhysicsObject::ApplyForceOffset(const Vector &forceVector, const Vector &worldPosition) {
	if (!IsMoveable()) {
		return;
	}

	btVector3 bulletWorldForce;
	ConvertForceImpulseToBullet(forceVector, bulletWorldForce);
	m_LinearVelocityChange += bulletWorldForce * m_RigidBody->getInvMass();

	Vector localForce;
	WorldToLocalVector(&localForce, forceVector);
	btVector3 bulletLocalForce;
	ConvertForceImpulseToBullet(localForce, bulletLocalForce);
	Vector localPosition;
	WorldToLocal(&localPosition, worldPosition);
	btVector3 bulletLocalPosition;
	ConvertPositionToBullet(localPosition, bulletLocalPosition);
	bulletLocalPosition -= GetBulletMassCenter();
	m_LocalAngularVelocityChange += bulletLocalPosition.cross(bulletLocalForce) *
			m_RigidBody->getInvInertiaDiagLocal();

	Wake();
}

void CPhysicsObject::ApplyTorqueCenter(const AngularImpulse &torque) {
	if (!IsMoveable()) {
		return;
	}
	AngularImpulse localTorque;
	WorldToLocalVector(&localTorque, torque);
	btVector3 bulletLocalTorque;
	ConvertAngularImpulseToBullet(localTorque, bulletLocalTorque);
	m_LocalAngularVelocityChange += bulletLocalTorque *
			m_RigidBody->getInvInertiaDiagLocal();
	Wake();
}

/***********
 * Triggers
 ***********/

bool CPhysicsObject::IsTrigger() const {
	return (m_RigidBody->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE) != 0;
}

void CPhysicsObject::BecomeTrigger() {
	if (IsTrigger()) {
		return;
	}
	EnableDrag(false);
	EnableGravity(false);
	m_RigidBody->setCollisionFlags(m_RigidBody->getCollisionFlags() |
			btCollisionObject::CF_NO_CONTACT_RESPONSE);
}

void CPhysicsObject::RemoveTrigger() {
	if (!IsTrigger()) {
		return;
	}
	m_RigidBody->setCollisionFlags(m_RigidBody->getCollisionFlags() &
			~btCollisionObject::CF_NO_CONTACT_RESPONSE);
	static_cast<CPhysicsEnvironment *>(m_Environment)->NotifyTriggerRemoved(this);
}

/***************************************
 * Collide object reference linked list
 ***************************************/

void CPhysicsObject::AddReferenceToCollide() {
	IPhysicsObject *next = GetCollide()->AddObjectReference(this);
	if (next != nullptr) {
		m_CollideObjectNext = static_cast<CPhysicsObject *>(next);
		m_CollideObjectPrevious = m_CollideObjectNext->m_CollideObjectPrevious;
		m_CollideObjectNext->m_CollideObjectPrevious = this;
		m_CollideObjectPrevious->m_CollideObjectNext = this;
	} else {
		m_CollideObjectNext = m_CollideObjectPrevious = this;
	}
}

void CPhysicsObject::RemoveReferenceFromCollide() {
	GetCollide()->RemoveObjectReference(this);
	m_CollideObjectNext->m_CollideObjectPrevious = m_CollideObjectPrevious;
	m_CollideObjectPrevious->m_CollideObjectNext = m_CollideObjectNext;
}
