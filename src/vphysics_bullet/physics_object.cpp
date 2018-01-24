// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_object.h"
#include "physics_collision.h"
#include "physics_environment.h"
#include "physics_material.h"
#include "physics_motioncontroller.h"
#include "bspflags.h"
#include "tier0/dbg.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsObject::CPhysicsObject(IPhysicsEnvironment *environment,
		CPhysCollide *collide, int materialIndex,
		const Vector &position, const QAngle &angles,
		objectparams_t *params, bool isStatic) :
		m_Environment(environment),
		m_CollideObjectNext(this), m_CollideObjectPrevious(this),
		m_MassCenterOverride(0.0f, 0.0f, 0.0f),
		m_Mass((!isStatic && !collide->GetShape()->isNonMoving()) ? params->mass : 0.0f),
		m_HingeAxis(-1),
		m_MotionEnabled(true),
		m_MotionDisabledByShadows(0), m_AngularMotionDisabledByShadows(0),
		m_Damping(params->damping), m_RotDamping(params->rotdamping),
		m_GameData(params->pGameData), m_GameFlags(0), m_GameIndex(0),
		m_Callbacks(CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION |
				CALLBACK_FLUID_TOUCH | CALLBACK_GLOBAL_TOUCH |
				CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_DO_FLUID_SIMULATION),
		m_MaterialIndex(materialIndex), m_ContentsMask(CONTENTS_SOLID),
		m_WasAsleep(true),
		m_LinearVelocityChange(0.0f, 0.0f, 0.0f),
		m_LocalAngularVelocityChange(0.0f, 0.0f, 0.0f),
		m_TouchingTriggers(0) {
	if (params->pName != nullptr) {
		V_strncpy(m_Name, params->pName, sizeof(m_Name));
	} else {
		m_Name[0] = '\0';
	}

	btRigidBody::btRigidBodyConstructionInfo constructionInfo(
			m_Mass, nullptr, collide->GetShape(), collide->GetInertia());

	btVector3 massCenter = collide->GetMassCenter();
	const Vector *massCenterOverride = params->massCenterOverride;
	if (massCenterOverride != nullptr && *massCenterOverride != vec3_origin) {
		ConvertPositionToBullet(*massCenterOverride, m_MassCenterOverride);
		btCompoundShape *massCenterOverrideShape = new btCompoundShape(false, 1);
		btVector3 massCenterOffset = m_MassCenterOverride - massCenter;
		massCenterOverrideShape->addChildShape(btTransform(btMatrix3x3::getIdentity(),
				-massCenterOffset), collide->GetShape());
		constructionInfo.m_collisionShape = massCenterOverrideShape;
		massCenter = m_MassCenterOverride;
		constructionInfo.m_localInertia = CPhysicsCollision::OffsetInertia(
				constructionInfo.m_localInertia, massCenterOffset);
	}
	constructionInfo.m_localInertia *= params->inertia * m_Mass;
	if (params->rotInertiaLimit > 0.0f) {
		btScalar minInertia = constructionInfo.m_localInertia.length() * params->rotInertiaLimit;
		constructionInfo.m_localInertia.setMax(btVector3(minInertia, minInertia, minInertia));
	}
	ConvertInertiaToHL(constructionInfo.m_localInertia, m_Inertia);

	float friction, elasticity;
	g_pPhysSurfaceProps->GetPhysicsProperties(materialIndex, nullptr, nullptr, &friction, &elasticity);
	constructionInfo.m_friction = friction;
	constructionInfo.m_restitution = elasticity;

	matrix3x4_t startMatrix;
	AngleMatrix(angles, position, startMatrix);
	btTransform &startWorldTransform = constructionInfo.m_startWorldTransform;
	ConvertMatrixToBullet(startMatrix, startWorldTransform);
	startWorldTransform.getOrigin() += startWorldTransform.getBasis() * massCenter;

	m_RigidBody = new btRigidBody(constructionInfo);
	m_RigidBody->setUserPointer(this);
	m_RigidBody->setSleepingThresholds(0.1f, 0.2f);

	if (!IsStatic()) {
		m_GravityEnabled = true;
		m_DragCoefficient = m_AngularDragCoefficient = params->dragCoefficient;
		ComputeDragBases();
		m_DragEnabled = (m_DragCoefficient != 0.0f);
	} else {
		const btVector3 zero(0.0f, 0.0f, 0.0f);
		// To remove additional IsStatic check in CanReceiveForce/Torque.
		m_RigidBody->setLinearFactor(zero);
		m_RigidBody->setAngularFactor(zero);
		m_GravityEnabled = false;
		m_DragCoefficient = m_AngularDragCoefficient = 0.0f;
		m_DragBasis.setZero();
		m_AngularDragBasis.setZero();
		m_DragEnabled = false;
	}

	AddReferenceToCollide();

	Sleep();
}

CPhysicsObject::~CPhysicsObject() {
	// Prevent callbacks to the game code and unlink from this object.
	m_Callbacks = 0;
	m_GameData = nullptr;

	DetachFromMotionControllers();

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
		return BULLET2HL(static_cast<const CPhysCollide_Sphere *>(collide)->GetRadius());
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

void CPhysicsObject::UpdateMassProps() {
	btVector3 bulletInertia;
	ConvertInertiaToBullet(m_Inertia, bulletInertia);
	if (m_HingeAxis >= 0) {
		bulletInertia[m_HingeAxis] = 50000.0f;
	}
	m_RigidBody->setMassProps(m_Mass, bulletInertia);
	m_RigidBody->updateInertiaTensor();
}

void CPhysicsObject::SetMass(float mass) {
	Assert(mass > 0.0f);
	if (IsStatic()) {
		return;
	}
	m_Inertia *= mass / m_Mass;
	m_Mass = mass;
	UpdateMassProps();
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
	ConvertInertiaToHL(m_RigidBody->getInvInertiaDiagLocal(), inertia);
	return inertia;
}

void CPhysicsObject::SetInertia(const Vector &inertia) {
	if (!IsStatic()) {
		return;
	}
	m_Inertia = inertia;
	UpdateMassProps();
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
	UpdateMassProps();
}

void CPhysicsObject::RemoveHinged() {
	if (!IsHinged()) {
		return;
	}
	m_HingeAxis = -1;
	UpdateMassProps();
}

bool CPhysicsObject::IsMotionEnabled() const {
	return m_MotionEnabled;
}

bool CPhysicsObject::IsMoveable() const {
	// By definition of the method in the interface, this is how it must be implemented.
	// The game handles the shadow controller case in certain places separately.
	return !IsStatic() && IsMotionEnabled();
}

void CPhysicsObject::EnableMotion(bool enable) {
	if (IsMotionEnabled() == enable) {
		return;
	}

	m_MotionEnabled = enable;

	if (IsStatic()) {
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
		if (!CanReceiveForce() && m_MotionDisabledByShadows <= 0) {
			m_RigidBody->setLinearFactor(one);
		}
		if (!CanReceiveTorque() && m_AngularMotionDisabledByShadows <= 0) {
			m_RigidBody->setAngularFactor(one);
		}
	} else {
		if (CanReceiveForce()) {
			m_RigidBody->setLinearFactor(zero);
		}
		if (CanReceiveTorque()) {
			m_RigidBody->setAngularFactor(zero);
		}
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
	return m_GravityEnabled;
}

void CPhysicsObject::EnableGravity(bool enable) {
	if (IsStatic()) {
		return;
	}
	m_GravityEnabled = enable;
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

void CPhysicsObject::ApplyDamping(btScalar timeStep) {
	if (!IsMoveable() || IsAsleep() || !IsGravityEnabled()) {
		return;
	}

	const btVector3 &linearVelocity = m_RigidBody->getLinearVelocity();
	const btVector3 &angularVelocity = m_RigidBody->getAngularVelocity();

	btScalar damping = m_Damping, rotDamping = m_RotDamping;
	if (linearVelocity.length2() < (0.01f * 0.01f) &&
			angularVelocity.length2() < (0.005f * 0.005f)) {
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

void CPhysicsObject::ApplyGravity(btScalar timeStep) {
	if (!CanReceiveForce() || IsAsleep() || !IsGravityEnabled()) {
		return;
	}
	m_RigidBody->setLinearVelocity(m_RigidBody->getLinearVelocity() +
			static_cast<const CPhysicsEnvironment *>(m_Environment)->GetBulletGravity() * timeStep);
}

/*******
 * Drag
 *******/

btScalar CPhysicsObject::AngularDragIntegral(btScalar l, btScalar w, btScalar h) {
	btScalar w2 = w * w, l2 = l * l, h2 = h * h;
	return (1.0f / 3.0f) * w2 * l * l2 + 0.5f * w2 * w2 * l + l * w2 * h2;
}

void CPhysicsObject::ComputeDragBases() {
	const CPhysCollide *collide = GetCollide();
	btVector3 aabbMin, aabbMax;
	collide->GetShape()->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
	btVector3 extents = aabbMax - aabbMin;
	const btVector3 &areas = collide->GetOrthographicAreas();
	m_DragBasis.setValue(
			extents.getY() * extents.getZ(),
			extents.getX() * extents.getZ(),
			extents.getX() * extents.getY());
	m_DragBasis *= areas;
	extents *= 0.5f;
	m_AngularDragBasis.setValue(
			AngularDragIntegral(extents.getX(), extents.getY(), extents.getZ()) +
					AngularDragIntegral(extents.getX(), extents.getZ(), extents.getY()),
			AngularDragIntegral(extents.getY(), extents.getX(), extents.getZ()) +
					AngularDragIntegral(extents.getY(), extents.getZ(), extents.getX()),
			AngularDragIntegral(extents.getZ(), extents.getX(), extents.getY()) +
					AngularDragIntegral(extents.getZ(), extents.getY(), extents.getX()));
	m_AngularDragBasis *= areas;
}

bool CPhysicsObject::IsDragEnabled() const {
	return m_DragEnabled;
}

void CPhysicsObject::EnableDrag(bool enable) {
	if (IsStatic()) {
		return;
	}
	m_DragEnabled = enable;
}

void CPhysicsObject::SetDragCoefficient(float *pDrag, float *pAngularDrag) {
	if (pDrag != nullptr) {
		m_DragCoefficient = *pDrag;
	}
	if (pAngularDrag != nullptr) {
		m_AngularDragCoefficient = *pAngularDrag;
	}
}

btScalar CPhysicsObject::CalculateLinearDrag(const btVector3 &velocity) const {
	btVector3 drag = ((velocity * m_RigidBody->getWorldTransform().getBasis()) *
			m_DragBasis).absolute();
	return m_DragCoefficient * m_RigidBody->getInvMass() * (drag.getX() + drag.getY() + drag.getZ());
}

float CPhysicsObject::CalculateLinearDrag(const Vector &unitDirection) const {
	btVector3 bulletUnitDirection;
	ConvertDirectionToBullet(unitDirection, bulletUnitDirection);
	return CalculateLinearDrag(bulletUnitDirection);
}

btScalar CPhysicsObject::CalculateAngularDrag(const btVector3 &objectSpaceRotationAxis) const {
	btVector3 drag = (objectSpaceRotationAxis * m_AngularDragBasis *
			m_RigidBody->getInvInertiaDiagLocal()).absolute();
	return m_AngularDragCoefficient * (drag.getX() + drag.getY() + drag.getZ());
}

float CPhysicsObject::CalculateAngularDrag(const Vector &objectSpaceRotationAxis) const {
	btVector3 bulletAxis;
	ConvertDirectionToBullet(objectSpaceRotationAxis, bulletAxis);
	return DEG2RAD(CalculateAngularDrag(bulletAxis));
}

void CPhysicsObject::ApplyDrag(btScalar timeStep) {
	if (!IsMoveable() || IsAsleep() || !IsDragEnabled()) {
		return;
	}

	btScalar dragForceScale = m_Environment->GetAirDensity() * timeStep;

	const btVector3 &linearVelocity = m_RigidBody->getLinearVelocity();
	btScalar dragForce = -0.5f * CalculateLinearDrag(linearVelocity) * dragForceScale;
	if (dragForce < 0.0f) {
		btSetMax(dragForce, btScalar(-1.0f));
		m_RigidBody->setLinearVelocity(linearVelocity + (linearVelocity * dragForce));
	}

	const btVector3 &angularVelocity = m_RigidBody->getAngularVelocity();
	float angularDragForce = -CalculateAngularDrag(angularVelocity *
			m_RigidBody->getWorldTransform().getBasis()) * dragForceScale;
	if (angularDragForce < 0.0f) {
		btSetMax(angularDragForce, btScalar(-1.0f));
		m_RigidBody->setAngularVelocity(angularVelocity + (angularVelocity * angularDragForce));
	}
}

void CPhysicsObject::NotifyOrthographicAreasChanged() {
	if (!IsStatic()) {
		ComputeDragBases();
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

unsigned int CPhysicsObject::GetContents() const {
	return m_ContentsMask;
}

void CPhysicsObject::SetContents(unsigned int contents) {
	m_ContentsMask = contents;
}

const char *CPhysicsObject::GetName() const {
	return m_Name;
}

/*************
 * Collisions
 *************/

int CPhysicsObject::GetMaterialIndex() const {
	return m_MaterialIndex;
}

void CPhysicsObject::SetMaterialIndex(int materialIndex) {
	m_MaterialIndex = materialIndex;
	float friction, elasticity;
	g_pPhysSurfaceProps->GetPhysicsProperties(materialIndex, nullptr, nullptr, &friction, &elasticity);
	m_RigidBody->setFriction(friction);
	m_RigidBody->setRestitution(elasticity);
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
	const btTransform &transform = m_RigidBody->getInterpolationWorldTransform();
	const btMatrix3x3 &basis = transform.getBasis();
	if (worldPosition != nullptr) {
		ConvertPositionToHL(transform.getOrigin() - (basis * GetBulletMassCenter()), *worldPosition);
	}
	if (angles != nullptr) {
		ConvertRotationToHL(basis, *angles);
	}
}

void CPhysicsObject::GetPositionMatrix(matrix3x4_t *positionMatrix) const {
	const btTransform &transform = m_RigidBody->getInterpolationWorldTransform();
	const btMatrix3x3 &basis = transform.getBasis();
	btVector3 origin = transform.getOrigin() - (basis * GetBulletMassCenter());
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
	if (!IsAsleep()) {
		const CPhysicsEnvironment *environment = static_cast<const CPhysicsEnvironment *>(m_Environment);
		if (CanReceiveForce()) {
			btVector3 linearVelocity = m_RigidBody->getLinearVelocity();
			linearVelocity += m_LinearVelocityChange;
			btScalar maxSpeed = environment->GetMaxSpeed();
			btClamp(linearVelocity[0], -maxSpeed, maxSpeed);
			btClamp(linearVelocity[1], -maxSpeed, maxSpeed);
			btClamp(linearVelocity[2], -maxSpeed, maxSpeed);
			m_RigidBody->setLinearVelocity(linearVelocity);
		}
		if (CanReceiveTorque()) {
			btVector3 angularVelocity = m_RigidBody->getAngularVelocity();
			angularVelocity += (m_RigidBody->getWorldTransform().getBasis() * m_LocalAngularVelocityChange);
			btScalar maxAngularSpeed = environment->GetMaxAngularSpeed();
			btClamp(angularVelocity[0], -maxAngularSpeed, maxAngularSpeed);
			btClamp(angularVelocity[1], -maxAngularSpeed, maxAngularSpeed);
			btClamp(angularVelocity[2], -maxAngularSpeed, maxAngularSpeed);
			m_RigidBody->setAngularVelocity(angularVelocity);
		}
	}
	m_LinearVelocityChange.setZero();
	m_LocalAngularVelocityChange.setZero();
}

void CPhysicsObject::SetVelocity(const Vector *velocity, const AngularImpulse *angularVelocity) {
	btVector3 zero(0.0f, 0.0f, 0.0f);
	bool wake = false;
	if (velocity != nullptr && CanReceiveForce()) {
		ConvertPositionToBullet(*velocity, m_LinearVelocityChange);
		m_RigidBody->setLinearVelocity(zero);
		wake = (wake || !m_LinearVelocityChange.isZero());
	}
	if (angularVelocity != nullptr && CanReceiveTorque()) {
		ConvertAngularImpulseToBullet(*angularVelocity, m_LocalAngularVelocityChange);
		m_RigidBody->setAngularVelocity(zero);
		wake = (wake || !m_LocalAngularVelocityChange.isZero());
	}
	if (wake) {
		Wake();
	}
}

void CPhysicsObject::SetVelocityInstantaneous(const Vector *velocity, const AngularImpulse *angularVelocity) {
	const CPhysicsEnvironment *environment = static_cast<const CPhysicsEnvironment *>(m_Environment);
	bool wake = false;
	if (velocity != nullptr && CanReceiveForce()) {
		btVector3 bulletVelocity;
		ConvertPositionToBullet(*velocity, bulletVelocity);
		btScalar maxSpeed = environment->GetMaxSpeed();
		btClamp(bulletVelocity[0], -maxSpeed, maxSpeed);
		btClamp(bulletVelocity[1], -maxSpeed, maxSpeed);
		btClamp(bulletVelocity[2], -maxSpeed, maxSpeed);
		m_RigidBody->setLinearVelocity(bulletVelocity);
		m_LinearVelocityChange.setZero();
		wake = (wake || !bulletVelocity.isZero());
	}
	if (angularVelocity != nullptr && CanReceiveTorque()) {
		btVector3 bulletAngularVelocity;
		ConvertAngularImpulseToBullet(*angularVelocity, bulletAngularVelocity);
		bulletAngularVelocity = m_RigidBody->getWorldTransform().getBasis() * bulletAngularVelocity;
		btScalar maxAngularSpeed = environment->GetMaxAngularSpeed();
		btClamp(bulletAngularVelocity[0], -maxAngularSpeed, maxAngularSpeed);
		btClamp(bulletAngularVelocity[1], -maxAngularSpeed, maxAngularSpeed);
		btClamp(bulletAngularVelocity[2], -maxAngularSpeed, maxAngularSpeed);
		m_RigidBody->setAngularVelocity(bulletAngularVelocity);
		wake = (wake || !bulletAngularVelocity.isZero());
	}
	if (wake) {
		Wake();
	}
}

void CPhysicsObject::GetVelocity(Vector *velocity, AngularImpulse *angularVelocity) const {
	if (velocity != nullptr) {
		ConvertPositionToHL(m_RigidBody->getLinearVelocity() + m_LinearVelocityChange, *velocity);
	}
	if (angularVelocity != nullptr) {
		AngularImpulse worldAngularVelocity;
		ConvertAngularImpulseToHL((m_RigidBody->getAngularVelocity() * 
				m_RigidBody->getWorldTransform().getBasis()) +
				m_LocalAngularVelocityChange, *angularVelocity);
	}
}

void CPhysicsObject::GetVelocityAtPoint(const Vector &worldPosition, Vector *pVelocity) const {
	const btTransform &worldTransform = m_RigidBody->getWorldTransform();
	btVector3 angularVelocity = m_RigidBody->getAngularVelocity() +
			(worldTransform.getBasis() * m_LocalAngularVelocityChange);
	// Position is relative to the center of mass (the rigid body's position).
	btVector3 bulletWorldPosition;
	ConvertPositionToBullet(worldPosition, bulletWorldPosition);
	btVector3 speed = m_RigidBody->getLinearVelocity() + angularVelocity.cross(
			bulletWorldPosition - worldTransform.getOrigin()) + m_LinearVelocityChange;
	ConvertPositionToHL(speed, *pVelocity);
}

void CPhysicsObject::AddVelocity(const Vector *velocity, const AngularImpulse *angularVelocity) {
	bool wake = false;
	if (velocity != nullptr && CanReceiveForce()) {
		btVector3 bulletVelocity;
		ConvertPositionToBullet(*velocity, bulletVelocity);
		m_LinearVelocityChange += bulletVelocity;
		wake = true;
	}
	if (angularVelocity != nullptr && CanReceiveTorque()) {
		btVector3 bulletAngularVelocity;
		ConvertAngularImpulseToBullet(*angularVelocity, bulletAngularVelocity);
		m_LocalAngularVelocityChange += bulletAngularVelocity;
		wake = true;
	}
	if (wake) {
		Wake();
	}
}

float CPhysicsObject::GetEnergy() const {
	btVector3 angularVelocity = m_RigidBody->getAngularVelocity() *
			m_RigidBody->getWorldTransform().getBasis();
	btVector3 inertia;
	ConvertInertiaToBullet(GetInertia(), inertia);
	// 1/2mv^2 + 1/2Iw^2
	return ConvertEnergyToHL(0.5f * (
			btScalar(GetMass()) * m_RigidBody->getLinearVelocity().length2() +
			(angularVelocity * inertia).dot(angularVelocity)));
}

void CPhysicsObject::ApplyForceCenter(const Vector &forceVector) {
	if (!CanReceiveForce()) {
		return;
	}
	btVector3 bulletForce;
	ConvertForceImpulseToBullet(forceVector, bulletForce);
	m_LinearVelocityChange += bulletForce * m_RigidBody->getInvMass();
	Wake();
}

void CPhysicsObject::ApplyForceOffset(const Vector &forceVector, const Vector &worldPosition) {
	bool applyForce = CanReceiveForce(), applyTorque = CanReceiveTorque();
	if (!applyForce && !applyTorque) {
		return;
	}
	btVector3 bulletWorldForce;
	ConvertForceImpulseToBullet(forceVector, bulletWorldForce);
	if (applyForce) {
		m_LinearVelocityChange += bulletWorldForce * m_RigidBody->getInvMass();
	}
	if (applyTorque) {
		btVector3 bulletWorldPosition;
		ConvertPositionToBullet(worldPosition, bulletWorldPosition);
		const btTransform &worldTransform = m_RigidBody->getWorldTransform();
		m_LocalAngularVelocityChange += ((bulletWorldPosition - worldTransform.getOrigin()).cross(
				bulletWorldForce) * worldTransform.getBasis()) * m_RigidBody->getInvInertiaDiagLocal();
	}
	Wake();
}

void CPhysicsObject::ApplyTorqueCenter(const AngularImpulse &torque) {
	if (!CanReceiveTorque()) {
		return;
	}
	btVector3 bulletWorldTorque;
	ConvertAngularImpulseToBullet(torque, bulletWorldTorque);
	m_LocalAngularVelocityChange += (bulletWorldTorque *
			m_RigidBody->getWorldTransform().getBasis()) * m_RigidBody->getInvInertiaDiagLocal();
	Wake();
}

void CPhysicsObject::CalculateForceOffset(const Vector &forceVector, const Vector &worldPosition,
		Vector *centerForce, AngularImpulse *centerTorque) const {
	if (centerForce != nullptr) {
		*centerForce = forceVector;
	}
	if (centerTorque != nullptr) {
		btVector3 bulletWorldForce, bulletWorldPosition;
		ConvertPositionToBullet(forceVector, bulletWorldForce);
		ConvertPositionToBullet(worldPosition, bulletWorldPosition);
		// Center torque is in mass center-relative space (for motion controllers, not ApplyTorqueCenter).
		const btTransform &worldTransform = m_RigidBody->getWorldTransform();
		btVector3 bulletCenterTorque = (bulletWorldPosition - worldTransform.getOrigin()).cross(
				bulletWorldForce) * worldTransform.getBasis();
		ConvertAngularImpulseToHL(bulletCenterTorque, *centerTorque);
	}
}

void CPhysicsObject::CalculateVelocityOffset(const Vector &forceVector, const Vector &worldPosition,
		Vector *centerVelocity, AngularImpulse *centerAngularVelocity) const {
	if (centerVelocity != nullptr) {
		*centerVelocity = forceVector * (float) m_RigidBody->getInvMass();
	}
	if (centerAngularVelocity != nullptr) {
		btVector3 bulletWorldForce, bulletWorldPosition;
		ConvertPositionToBullet(forceVector, bulletWorldForce);
		ConvertPositionToBullet(worldPosition, bulletWorldPosition);
		// Center angular velocity is in mass center-relative space.
		const btTransform &worldTransform = m_RigidBody->getWorldTransform();
		btVector3 bulletCenterAngularVelocity = ((bulletWorldPosition - worldTransform.getOrigin()).cross(
				bulletWorldForce) * worldTransform.getBasis()) * m_RigidBody->getInvInertiaDiagLocal();
		ConvertAngularImpulseToHL(bulletCenterAngularVelocity, *centerAngularVelocity);
	}
}

void CPhysicsObject::NotifyAttachedToMotionController(IPhysicsMotionController *controller) {
	m_MotionControllers.AddToTail(controller);
}

void CPhysicsObject::NotifyDetachedFromMotionController(IPhysicsMotionController *controller) {
	m_MotionControllers.FindAndFastRemove(controller);
}

void CPhysicsObject::DetachFromMotionControllers() {
	int controllerCount = m_MotionControllers.Count();
	for (int controllerIndex = 0; controllerIndex < controllerCount; ++controllerIndex) {
		static_cast<CPhysicsMotionController *>(
				m_MotionControllers[controllerIndex])->DetachObjectInternal(this, false);
	}
	m_MotionControllers.RemoveAll();
}

void CPhysicsObject::ApplyEventMotion(bool isWorld, bool isForce,
		const btVector3 &linear, const btVector3 &angular) {
	const btMatrix3x3 &worldTransform = m_RigidBody->getWorldTransform().getBasis();
	bool wake = false;
	if (CanReceiveForce() && !linear.isZero()) {
		btVector3 worldLinearAcceleration = linear;
		if (!isWorld) {
			worldLinearAcceleration = worldTransform * worldLinearAcceleration;
		}
		if (isForce) {
			worldLinearAcceleration *= m_RigidBody->getInvMass();
		}
		m_RigidBody->setLinearVelocity(m_RigidBody->getLinearVelocity() +
				worldLinearAcceleration);
		wake = true;
	}
	if (CanReceiveTorque() && !angular.isZero()) {
		btVector3 localAngularAcceleration = angular;
		if (isWorld) {
			localAngularAcceleration = localAngularAcceleration * worldTransform;
		}
		if (isForce) {
			localAngularAcceleration *= m_RigidBody->getInvInertiaDiagLocal();
		}
		m_RigidBody->setAngularVelocity(m_RigidBody->getAngularVelocity() +
				(worldTransform * localAngularAcceleration));
		wake = true;
	}
	if (wake) {
		Wake();
	}
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
