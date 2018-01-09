// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_PARSE_H
#define PHYSICS_PARSE_H

#include "physics_internal.h"
#include "tier1/strtools.h"

#define PHYSICS_MAX_KEYVALUE 1024

const char *ParseKeyvalue(const char *buffer, char *key, char *value);

#endif
