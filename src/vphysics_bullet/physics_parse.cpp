// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_parse.h"
#include "filesystem_helpers.h"

const char *ParseKeyvalue(const char *buffer, char *key, char *value) {
	buffer = ParseFileInternal(buffer, key, nullptr, nullptr, PHYSICS_MAX_KEYVALUE);
	V_strlower(key);
	if (V_strcmp(key, "}") == 0) {
		// No value on a close brace.
		value[0] = '\0';
		return buffer;
	}
	buffer = ParseFileInternal(buffer, value, nullptr, nullptr, PHYSICS_MAX_KEYVALUE);
	V_strlower(value);
	return buffer;
}
