// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_OBJECT_H
#define PHYSICS_OBJECT_H

#include "physics_internal.h"

class CPhysicsObject : public IPhysicsObject {
public:
	CPhysicsObject(IPhysicsEnvironment *environment,
			btCollisionShape *collisionShape, int materialIndex,
			const Vector &position, const QAngle &angles,
			objectparams_t *pParams, bool isStatic);
	virtual ~CPhysicsObject();

	// IPhysicsObject methods.

	virtual bool IsStatic() const;
	virtual bool IsAsleep() const;
	virtual bool IsGravityEnabled() const;
	virtual bool IsMotionEnabled() const;
	virtual bool IsMoveable() const;

	virtual void EnableGravity(bool enable);
	virtual void EnableMotion(bool enable);

	virtual void SetGameData(void *pGameData);
	virtual void *GetGameData() const;
	virtual void SetGameFlags(unsigned short userFlags);
	virtual unsigned short GetGameFlags() const;
	virtual void SetGameIndex(unsigned short gameIndex);
	virtual unsigned short GetGameIndex() const;

	virtual void SetCallbackFlags(unsigned short callbackflags);
	virtual unsigned short GetCallbackFlags() const;

	virtual void Wake();
	virtual void Sleep();

	virtual void SetMass(float mass);
	virtual float GetMass() const;
	virtual float GetInvMass() const;
	virtual Vector GetInertia() const;
	virtual Vector GetInvInertia() const;
	virtual void SetInertia(const Vector &inertia);

	virtual void SetDamping(const float *speed, const float *rot);
	virtual void GetDamping(float *speed, float *rot) const;

	virtual unsigned int GetContents() const;
	virtual void SetContents(unsigned int contents);

	virtual Vector GetMassCenterLocalSpace() const;

	virtual void SetPosition(const Vector &worldPosition, const QAngle &angles, bool isTeleport);
	virtual void SetPositionMatrix(const matrix3x4_t &matrix, bool isTeleport);
	virtual void GetPosition(Vector *worldPosition, QAngle *angles) const;
	virtual void GetPositionMatrix(matrix3x4_t *positionMatrix) const;

	virtual void SetVelocity(const Vector *velocity, const AngularImpulse *angularVelocity);
	virtual void GetVelocity(Vector *velocity, AngularImpulse *angularVelocity) const;
	virtual void AddVelocity(const Vector *velocity, const AngularImpulse *angularVelocity);

	virtual void LocalToWorld(Vector *worldPosition, const Vector &localPosition) const;
	virtual void WorldToLocal(Vector *localPosition, const Vector &worldPosition) const;
	virtual void LocalToWorldVector(Vector *worldVector, const Vector &localVector) const;
	virtual void WorldToLocalVector(Vector *localVector, const Vector &worldVector) const;

	virtual void ApplyForceCenter(const Vector &forceVector);
	virtual void ApplyForceOffset(const Vector &forceVector, const Vector &worldPosition);
	virtual void ApplyTorqueCenter(const AngularImpulse &torque);

	// Internal methods.

	// Bullet doesn't allow damping factors over 1, so it has to be done manually.
	// Also applies damping in a way more similar to how IVP VPhysics does it.
	void ApplyDamping(float timeStep);

	// Bullet integrates forces and torques over time, in IVP async pushes are applied fully.
	void ApplyForcesAndSpeedLimit();

	FORCEINLINE CPhysicsObject *GetNextCollideObject() const {
		return m_CollideObjectNext;
	}

	void NotifyMassCenterChanged(const btVector3 &oldMassCenter);

private:
	IPhysicsEnvironment *m_Environment;

	btRigidBody *m_RigidBody;

	btCollisionShape *GetCollisionShape() const;
	CPhysicsObject *m_CollideObjectNext, *m_CollideObjectPrevious;
	void AddReferenceToCollide();
	void RemoveReferenceFromCollide();

	btVector3 m_MassCenterOverride;
	btCompoundShape *m_MassCenterOverrideShape;
	const btVector3 &GetBulletMassCenter() const;

	float m_Mass;
	Vector m_Inertia;

	float m_Damping, m_RotDamping;

	void *m_GameData;
	unsigned short m_GameFlags;
	unsigned short m_GameIndex;

	unsigned short m_Callbacks;

	unsigned int m_ContentsMask;

	btVector3 m_LinearVelocityChange;
	btVector3 m_LocalAngularVelocityChange;
};

#endif
