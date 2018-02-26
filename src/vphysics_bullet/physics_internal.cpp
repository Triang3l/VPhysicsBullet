// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_internal.h"

void ConvertMatrixToBullet(const matrix3x4_t &matrix, btTransform &transform) {
	transform.getBasis().setValue(
			matrix[0][0], matrix[0][2], -matrix[0][1],
			matrix[2][0], matrix[2][2], -matrix[2][1],
			-matrix[1][0], -matrix[1][2], matrix[1][1]);
	transform.getOrigin().setValue(
			HL2BULLET(matrix[0][3]), HL2BULLET(matrix[2][3]), -HL2BULLET(matrix[1][3]));
}

void ConvertMatrixToHL(const btMatrix3x3 &basis, const btVector3 &origin, matrix3x4_t &matrix) {
	matrix[0][0] = basis[0][0];
	matrix[0][1] = -basis[0][2];
	matrix[0][2] = basis[0][1];
	matrix[0][3] = BULLET2HL(origin[0]);

	matrix[1][0] = -basis[2][0];
	matrix[1][1] = basis[2][2];
	matrix[1][2] = -basis[2][1];
	matrix[1][3] = -BULLET2HL(origin[2]);

	matrix[2][0] = basis[1][0];
	matrix[2][1] = -basis[1][2];
	matrix[2][2] = basis[1][1];
	matrix[2][3] = BULLET2HL(origin[1]);
}

void ConvertRotationToBullet(const QAngle &angles, btMatrix3x3 &basis) {
	Vector forward, right, up;
	AngleVectors(angles, &forward, &right, &up);
	// The third column is left, but the conversion negates it.
	basis.setValue(
			forward[0], up[0], right[0],
			forward[2], up[2], right[2],
			-forward[1], -up[1], -right[1]);
}

void ConvertRotationToHL(const btMatrix3x3 &basis, QAngle &angles) {
	// Like in ConvertMatrixToHL, but with left instead of right.
	float forward[3] = { basis[0][0], -basis[2][0], basis[1][0] };
	float left[3] = { -basis[0][2], basis[2][2], -basis[1][2] };
	float up2 = basis[1][1];
	float xyDist = sqrtf(forward[0] * forward[0] + forward[1] * forward[1]);
	if (xyDist > 0.001f) { // enough here to get angles?
		// (yaw)	y = ATAN( forward.y, forward.x );		-- in our space, forward is the X axis
		angles[1] = RAD2DEG(atan2f(forward[1], forward[0]));
		// (pitch)	x = ATAN( -forward.z, sqrt(forward.x*forward.x+forward.y*forward.y) );
		angles[0] = RAD2DEG(atan2f(-forward[2], xyDist));
		// (roll)	z = ATAN( left.z, up.z );
		angles[2] = RAD2DEG(atan2f(left[2], up2));
	} else { // forward is mostly Z, gimbal lock-
		// (yaw)	y = ATAN( -left.x, left.y );			-- forward is mostly z, so use right for yaw
		angles[1] = RAD2DEG(atan2f(-left[0], left[1]));
		// (pitch)	x = ATAN( -forward.z, sqrt(forward.x*forward.x+forward.y*forward.y) );
		angles[0] = RAD2DEG(atan2f(-forward[2], xyDist));
		// Assume no roll in this case as one degree of freedom has been lost (i.e. yaw == roll)
		angles[2] = 0.0f;
	}
}
