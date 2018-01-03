// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_INTERNAL_H
#define PHYSICS_INTERNAL_H

#include "vphysics_interface.h"
#include <btBulletDynamicsCommon.h>
#include "mathlib/mathlib.h"
#include "tier0/memalloc.h"

#ifdef _DEBUG
#define BEGIN_BULLET_ALLOCATION() g_pMemAlloc->PushAllocDbgInfo("Bullet", 0)
#define END_BULLET_ALLOCATION() g_pMemAlloc->PopAllocDbgInfo()
#else
#define BEGIN_BULLET_ALLOCATION() 0
#define END_BULLET_ALLOCATION() 0
#endif

#define HL2BULLET_FACTOR METERS_PER_INCH
#define BULLET2HL_FACTOR (1.0f / HL2BULLET_FACTOR)
#define HL2BULLET(x) ((btScalar) ((x) * HL2BULLET_FACTOR))
#define BULLET2HL(x) ((float) ((x) * BULLET2HL_FACTOR))

inline void ConvertPositionToBullet(const Vector &in, btVector3 &out) {
	out.setX(HL2BULLET(in.x));
	out.setY(HL2BULLET(in.z));
	out.setZ(-HL2BULLET(in.y));
}

inline void ConvertPositionToHL(const btVector3 &in, Vector &out) {
	out.x = BULLET2HL(in.getX());
	out.y = -BULLET2HL(in.getZ());
	out.z = BULLET2HL(in.getY());
}

#define ConvertForceImpulseToBullet ConvertPositionToBullet
#define ConvertForceImpulseToHL ConvertPositionToHL

inline void ConvertDirectionToBullet(const Vector &in, btVector3 &out) {
	out.setX(in.x);
	out.setY(in.z);
	out.setZ(-in.y);
}

inline void ConvertDirectionToHL(const btVector3 &in, Vector &out) {
	out.x = in.getX();
	out.y = -in.getZ();
	out.z = in.getY();
}

inline void ConvertAngularImpulseToBullet(const AngularImpulse &in, btVector3 &out) {
	out.setX(DEG2RAD(in.x));
	out.setY(DEG2RAD(in.z));
	out.setZ(-DEG2RAD(in.y));
}

void ConvertMatrixToBullet(const matrix3x4_t &matrix, btTransform &transform);
void ConvertMatrixToHL(const btMatrix3x3 &basis, const btVector3 &origin, matrix3x4_t &matrix);
inline void ConvertMatrixToHL(const btTransform &transform, matrix3x4_t &matrix) {
	ConvertMatrixToHL(transform.getBasis(), transform.getOrigin(), matrix);
}

void ConvertRotationToHL(const btMatrix3x3 &basis, QAngle &angles);

inline int ConvertAxisIndexToBullet(int axisIndex) {
	if (axisIndex == 1) {
		return 2;
	}
	if (axisIndex == 2) {
		return 1;
	}
	return axisIndex;
}

#endif
