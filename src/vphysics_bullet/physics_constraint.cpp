// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_constraint.h"

void CPhysicsConstraint::SetGameData(void *gameData) {
	m_GameData = gameData;
}

void *CPhysicsConstraint::GetGameData() const {
	return m_GameData;
}

IPhysicsObject *CPhysicsConstraint::GetReferenceObject() const {
	return m_ObjectReference;
}

IPhysicsObject *CPhysicsConstraint::GetAttachedObject() const {
	return m_ObjectAttached;
}
