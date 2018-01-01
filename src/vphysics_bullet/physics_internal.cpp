// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_internal.h"

void ConvertMatrixToBullet(const matrix3x4_t &matrix, btTransform &transform) {
	transform.getBasis().setValue(
			matrix[0][0], matrix[2][0], -matrix[1][0],
			matrix[0][2], matrix[2][2], -matrix[1][2],
			-matrix[0][1], -matrix[2][1], matrix[1][1]);
	transform.getOrigin().setValue(
			HL2BULLET(matrix[0][3]), HL2BULLET(matrix[2][3]), -HL2BULLET(matrix[1][3]));
}

void ConvertMatrixToHL(const btTransform &transform, matrix3x4_t &matrix) {
	const btMatrix3x3 &basis = transform.getBasis();
	const btVector3 &origin = transform.getOrigin();

	matrix[0][0] = basis[0][0];
	matrix[0][1] = -basis[2][0];
	matrix[0][2] = basis[1][0];
	matrix[0][3] = origin[0];

	matrix[1][0] = -basis[0][2];
	matrix[1][1] = basis[2][2];
	matrix[1][2] = -basis[1][2];
	matrix[1][3] = -origin[2];

	matrix[2][0] = basis[0][1];
	matrix[2][1] = -basis[2][1];
	matrix[2][2] = basis[1][1];
	matrix[2][3] = origin[1];
}
