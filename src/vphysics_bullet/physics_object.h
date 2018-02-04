// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_OBJECT_H
#define PHYSICS_OBJECT_H

#include "physics_internal.h"
#include "tier1/utlvector.h"

class CPhysicsObject : public IPhysicsObject {
public:
	CPhysicsObject(IPhysicsEnvironment *environment,
			CPhysCollide *collide, int materialIndex,
			const Vector &position, const QAngle &angles,
			objectparams_t *params, bool isStatic);
	virtual ~CPhysicsObject();

	// IPhysicsObject methods.

	virtual bool IsStatic() const;
	virtual bool IsAsleep() const;
	virtual bool IsTrigger() const;
	virtual bool IsHinged() const;
	virtual bool IsGravityEnabled() const;
	virtual bool IsDragEnabled() const;
	virtual bool IsMotionEnabled() const;
	virtual bool IsMoveable() const;

	virtual void EnableGravity(bool enable);
	virtual void EnableDrag(bool enable);
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

	virtual void SetDragCoefficient(float *pDrag, float *pAngularDrag);

	virtual int GetMaterialIndex() const;
	virtual void SetMaterialIndex(int materialIndex);

	virtual unsigned int GetContents() const;
	virtual void SetContents(unsigned int contents);

	virtual float GetSphereRadius() const;
	virtual float GetEnergy() const;
	virtual Vector GetMassCenterLocalSpace() const;

	virtual void SetPosition(const Vector &worldPosition, const QAngle &angles, bool isTeleport);
	virtual void SetPositionMatrix(const matrix3x4_t &matrix, bool isTeleport);
	virtual void GetPosition(Vector *worldPosition, QAngle *angles) const;
	virtual void GetPositionMatrix(matrix3x4_t *positionMatrix) const;

	virtual void SetVelocity(const Vector *velocity, const AngularImpulse *angularVelocity);
	virtual void SetVelocityInstantaneous(const Vector *velocity, const AngularImpulse *angularVelocity);
	virtual void GetVelocity(Vector *velocity, AngularImpulse *angularVelocity) const;
	virtual void AddVelocity(const Vector *velocity, const AngularImpulse *angularVelocity);
	virtual void GetVelocityAtPoint(const Vector &worldPosition, Vector *pVelocity) const;

	virtual void LocalToWorld(Vector *worldPosition, const Vector &localPosition) const;
	virtual void WorldToLocal(Vector *localPosition, const Vector &worldPosition) const;
	virtual void LocalToWorldVector(Vector *worldVector, const Vector &localVector) const;
	virtual void WorldToLocalVector(Vector *localVector, const Vector &worldVector) const;

	virtual void ApplyForceCenter(const Vector &forceVector);
	virtual void ApplyForceOffset(const Vector &forceVector, const Vector &worldPosition);
	virtual void ApplyTorqueCenter(const AngularImpulse &torque);

	virtual void CalculateForceOffset(const Vector &forceVector, const Vector &worldPosition,
			Vector *centerForce, AngularImpulse *centerTorque) const;
	virtual void CalculateVelocityOffset(const Vector &forceVector, const Vector &worldPosition,
			Vector *centerVelocity, AngularImpulse *centerAngularVelocity) const;

	virtual float CalculateLinearDrag(const Vector &unitDirection) const;
	virtual float CalculateAngularDrag(const Vector &objectSpaceRotationAxis) const;

	virtual void SetShadow(float maxSpeed, float maxAngularSpeed,
			bool allowPhysicsMovement, bool allowPhysicsRotation);
	virtual int GetShadowPosition(Vector *position, QAngle *angles) const;
	virtual IPhysicsShadowController *GetShadowController() const;
	virtual void RemoveShadowController();

	virtual const CPhysCollide *GetCollide() const;

	virtual const char *GetName() const;

	virtual void BecomeTrigger();
	virtual void RemoveTrigger();

	virtual void BecomeHinged(int localAxis);
	virtual void RemoveHinged();

	// Internal methods.

	FORCEINLINE btRigidBody *GetRigidBody() const { return m_RigidBody; }

	inline bool WasAsleep() const { return m_WasAsleep; }
	inline bool UpdateEventSleepState() {
		bool wasAsleep = m_WasAsleep;
		m_WasAsleep = IsAsleep();
		return wasAsleep;
	}

	// Bullet doesn't allow damping factors over 1, so it has to be done manually.
	// Also applies damping in a way more similar to how IVP VPhysics does it.
	void ApplyDamping(btScalar timeStep);
	void ApplyGravity(btScalar timeStep);

	btScalar CalculateLinearDrag(const btVector3 &velocity) const;
	btScalar CalculateAngularDrag(const btVector3 &objectSpaceRotationAxis) const;
	void ApplyDrag(btScalar timeStep);
	void NotifyOrthographicAreasChanged();

	void UpdateMaterial();

	// Bullet integrates forces and torques over time, in IVP async pushes are applied fully.
	void ApplyForcesAndSpeedLimit(btScalar timeStep);

	// No force should EVER be applied with applyForce/applyCentralForce/applyTorque,
	// as we take over force application. Bullet may only apply gravity, but we set it to 0.
	void CheckAndClearBulletForces();

	void NotifyAttachedToMotionController(IPhysicsMotionController *controller);
	void NotifyDetachedFromMotionController(IPhysicsMotionController *controller);
	void SimulateMotionControllers(
			IPhysicsMotionController::priority_t priority, btScalar timeStep);
	void ApplyEventMotion(bool isWorld, bool isForce,
			const btVector3 &linear, const btVector3 &angular);

	void NotifyAttachedToShadowController(IPhysicsShadowController *shadow);
	void StepUp(btScalar height); // May be called outside PSIs.
	btScalar ComputeBulletShadowControl(struct ShadowControlBulletParameters_t &params,
			btScalar secondsToArrival, btScalar timeStep);
	void SimulateShadowAndPlayerController(btScalar timeStep);

	FORCEINLINE CPhysicsObject *GetNextCollideObject() const {
		return m_CollideObjectNext;
	}

	FORCEINLINE void AddTriggerTouchReference() {
		++m_TouchingTriggers;
	}
	inline void RemoveTriggerTouchReference() {
		--m_TouchingTriggers;
		Assert(m_TouchingTriggers >= 0);
		m_TouchingTriggers = MAX(m_TouchingTriggers, 0);
	}
	FORCEINLINE bool IsTouchingTriggers() {
		return m_TouchingTriggers > 0;
	}

	void UpdateInterpolation(); // Called in the end of each PSI.
	void InterpolateWorldTransform();

	void NotifyTransferred(IPhysicsEnvironment *newEnvironment);

private:
	/***********************************
	 * Properties and persistent values
	 ***********************************/

	IPhysicsEnvironment *m_Environment;

	btRigidBody *m_RigidBody;

	CPhysCollide *GetCollide();
	CPhysicsObject *m_CollideObjectNext, *m_CollideObjectPrevious;
	void AddReferenceToCollide();
	void RemoveReferenceFromCollide();

	btVector3 m_MassCenterOverride;
	const btVector3 &GetBulletMassCenter() const;

	float m_Mass;
	Vector m_Inertia;
	int m_HingeHLAxis;
	void UpdateMassProps();

	bool m_GravityEnabled;
	bool m_ShadowTempGravityDisable;
	float m_Damping, m_RotDamping;

	int m_MaterialIndex, m_RealMaterialIndex;
	unsigned int m_ContentsMask;

	btScalar m_DragCoefficient, m_AngularDragCoefficient;
	btVector3 m_DragBasis, m_AngularDragBasis;
	bool m_DragEnabled;
	static btScalar AngularDragIntegral(btScalar l, btScalar w, btScalar h);
	void ComputeDragBases();

	CUtlVector<IPhysicsMotionController *> m_MotionControllers;
	void DetachFromMotionControllers();

	IPhysicsShadowController *m_Shadow;
	IPhysicsPlayerController *m_Player;

	void *m_GameData;
	unsigned short m_GameFlags;
	unsigned short m_GameIndex;
	char m_Name[128];

	unsigned short m_Callbacks;

	/******************
	 * Transient state
	 ******************/

	// Was the object active in the previous PSI - used to trigger sleep events.
	bool m_WasAsleep;

	btVector3 m_LinearVelocityChange, m_LocalAngularVelocityChange;

	int m_TouchingTriggers;

	// Interpolation between PSIs for non-static objects.
	btTransform m_InterpolationWorldTransform;
	btVector3 m_InterpolationLinearVelocity, m_InterpolationAngularVelocity;
};

#endif
