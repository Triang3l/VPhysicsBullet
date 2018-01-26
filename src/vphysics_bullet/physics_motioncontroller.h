// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_MOTIONCONTROLLER_H
#define PHYSICS_MOTIONCONTROLLER_H

#include "physics_internal.h"
#include "tier1/utlvector.h"

class CPhysicsMotionController : public IPhysicsMotionController {
public:
	CPhysicsMotionController(IMotionEvent *handler) :
			m_Handler(handler), m_Priority(MEDIUM_PRIORITY) {}
	virtual ~CPhysicsMotionController();

	// IPhysicsMotionController methods.

	virtual void SetEventHandler(IMotionEvent *handler);
	virtual void AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached);
	virtual void DetachObject(IPhysicsObject *pObject);
	virtual int CountObjects();
	virtual void GetObjects(IPhysicsObject **pObjectList);
	virtual void ClearObjects();
	virtual void WakeObjects();
	virtual void SetPriority(priority_t priority);

	// Internal methods.

	FORCEINLINE priority_t GetPriority() const { return m_Priority; }

	void Simulate(IPhysicsObject *object, btScalar timeStep);

	void DetachObjectInternal(IPhysicsObject *object, bool notify);

private:
	IMotionEvent *m_Handler;
	priority_t m_Priority;
	CUtlVector<IPhysicsObject *> m_Objects;
};

#endif
