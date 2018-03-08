// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H

#include "physics_internal.h"
#include "vphysics/constraints.h"
#include "vphysics/vehicles.h"

class CPhysicsConstraint : public IPhysicsConstraint {
public:
	CPhysicsConstraint(IPhysicsObject *objectReference, IPhysicsObject *objectAttached);

	// IPhysicsConstraint methods.

	virtual void Activate();
	virtual void Deactivate();
	virtual void SetGameData(void *gameData);
	virtual void *GetGameData() const;
	virtual IPhysicsObject *GetReferenceObject() const;
	virtual IPhysicsObject *GetAttachedObject() const;
	/* DUMMY */ virtual void SetLinearMotor(float speed, float maxLinearImpulse) {}
	virtual void SetAngularMotor(float rotSpeed, float maxAngularImpulse) {}
	/* DUMMY */ virtual void UpdateRagdollTransforms(
			const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached) {}
	/* DUMMY */ virtual bool GetConstraintTransform(
			matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const {
		if (pConstraintToReference != nullptr) {
			SetIdentityMatrix(*pConstraintToReference);
		}
		if (pConstraintToAttached != nullptr) {
			SetIdentityMatrix(*pConstraintToAttached);
		}
		return true;
	}
	/* DUMMY */ virtual bool GetConstraintParams(constraint_breakableparams_t *pParams) const {
		if (pParams != nullptr) {
			pParams->Defaults();
		}
		return true;
	}
	/* DUMMY */ virtual void OutputDebugInfo() {}

	// Internal methods.

	virtual btTypedConstraint *GetBulletConstraint() const = 0;

	// Safe to call when the constraint is already invalid.
	FORCEINLINE void NotifyObjectRemoving() {
		DeleteBulletConstraint();
		m_ObjectReference = m_ObjectAttached = nullptr;
	}

	virtual void Release() = 0;

protected:
	void InitializeBulletConstraint(const constraint_breakableparams_t *params = nullptr);

	IPhysicsObject *m_ObjectReference, *m_ObjectAttached;
	virtual bool AreObjectsValid() const;

	virtual void DeleteBulletConstraint() = 0; // May be called when the constraint is null.

private:
	void *m_GameData;
};

/* DUMMY */ class CPhysicsConstraint_Dummy : public CPhysicsConstraint {
public:
	/* DUMMY */ CPhysicsConstraint_Dummy(IPhysicsObject *objectReference, IPhysicsObject *objectAttached) :
			CPhysicsConstraint(objectReference, objectAttached) {}
	/* DUMMY */ virtual btTypedConstraint *GetBulletConstraint() const { return nullptr; }
	/* DUMMY */ virtual void Release() { VPhysicsDelete(CPhysicsConstraint_Dummy, this); }
protected:
	/* DUMMY */ virtual void DeleteBulletConstraint() {}
};

class CPhysicsConstraint_Hinge : public CPhysicsConstraint {
public:
	CPhysicsConstraint_Hinge(
			IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
			const constraint_hingeparams_t &params);
	virtual ~CPhysicsConstraint_Hinge();
	virtual void SetAngularMotor(float rotSpeed, float maxAngularImpulse);
	virtual btTypedConstraint *GetBulletConstraint() const;
	FORCEINLINE btHingeConstraint *GetBulletHingeConstraint() const {
		return m_Constraint;
	}
	virtual void Release();
protected:
	virtual void DeleteBulletConstraint();
private:
	btHingeConstraint *m_Constraint;
	btScalar m_TargetAngularVelocity;
	btScalar m_MaxAngularImpulse;
};

class CPhysicsConstraint_Ballsocket : public CPhysicsConstraint {
public:
	CPhysicsConstraint_Ballsocket(
			IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
			const constraint_ballsocketparams_t &params);
	virtual ~CPhysicsConstraint_Ballsocket();
	virtual btTypedConstraint *GetBulletConstraint() const;
	FORCEINLINE btPoint2PointConstraint *GetBulletPoint2PointConstraint() const {
		return m_Constraint;
	}
	virtual void Release();
protected:
	virtual void DeleteBulletConstraint();
private:
	btPoint2PointConstraint *m_Constraint;
};

// Y axis suspension for car wheels.
class CPhysicsConstraint_Suspension : public CPhysicsConstraint {
public:
	class SpringConstraint : public btGeneric6DofSpring2Constraint {
	public:
		SpringConstraint(btRigidBody &rbA, btRigidBody &rbB,
				const btTransform &frameInA, const btTransform &frameInB, RotateOrder rotOrder);
		virtual void getInfo2(btConstraintInfo2 *info);
		FORCEINLINE void SetYDamping(btScalar extension, btScalar compression) {
			m_YDampingExtension = extension;
			m_YDampingCompression = compression;
		}
	private:
		btScalar m_YDampingExtension, m_YDampingCompression;
	};

	CPhysicsConstraint_Suspension(
			IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
			const Vector &wheelPositionInReference, const vehicle_suspensionparams_t &params);
	virtual ~CPhysicsConstraint_Suspension();
	virtual btTypedConstraint *GetBulletConstraint() const;
	FORCEINLINE SpringConstraint *GetBulletSpring2Constraint() const {
		return m_Constraint;
	}
	virtual void Release();

protected:
	virtual void DeleteBulletConstraint();

private:
	SpringConstraint *m_Constraint;
};

class CPhysicsConstraintGroup : public IPhysicsConstraintGroup {
public:
	/* DUMMY */ virtual void Activate() {}
	/* DUMMY */ virtual bool IsInErrorState() { return false; }
	/* DUMMY */ virtual void ClearErrorState() {}
	/* DUMMY */ virtual void GetErrorParams(constraint_groupparams_t *pParams) {
		if (pParams != nullptr) {
			pParams->Defaults();
		}
	}
	/* DUMMY */ virtual void SetErrorParams(const constraint_groupparams_t &params) {}
	/* DUMMY */ virtual void SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1) {}
};

#endif
