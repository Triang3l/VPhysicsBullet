// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H

#include "physics_internal.h"
#include "vphysics/constraints.h"

class CPhysicsConstraint : public IPhysicsConstraint {
public:
	BT_DECLARE_ALIGNED_ALLOCATOR()

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

	FORCEINLINE btTypedConstraint *GetBulletConstraint() const { return m_Constraint; }

	void MakeInvalid();

protected:
	btTypedConstraint *m_Constraint;
	void InitializeBulletConstraint(const constraint_breakableparams_t &params);

	IPhysicsObject *m_ObjectReference, *m_ObjectAttached;
	FORCEINLINE bool AreObjectsValid() const { return m_ObjectAttached != nullptr; }

private:
	void *m_GameData;
};

class CPhysicsConstraint_Hinge : public CPhysicsConstraint {
public:
	CPhysicsConstraint_Hinge(
			IPhysicsObject *objectReference, IPhysicsObject *objectAttached,
			const constraint_hingeparams_t &params);
	virtual void SetAngularMotor(float rotSpeed, float maxAngularImpulse);
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
