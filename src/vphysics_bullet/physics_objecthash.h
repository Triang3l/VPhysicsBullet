// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_OBJECTHASH_H
#define PHYSICS_OBJECTHASH_H

#include "physics_internal.h"
#include "vphysics/object_hash.h"

class CPhysicsObjectPairHash : public IPhysicsObjectPairHash {
public:
	/* DUMMY */ virtual void AddObjectPair(void *pObject0, void *pObject1) {}
	/* DUMMY */ virtual void RemoveObjectPair(void *pObject0, void *pObject1) {}
	/* DUMMY */ virtual bool IsObjectPairInHash(void *pObject0, void *pObject1) { return false; }
	/* DUMMY */ virtual void RemoveAllPairsForObject(void *pObject0) {}
	/* DUMMY */ virtual bool IsObjectInHash(void *pObject0) { return false; }
	/* DUMMY */ virtual int GetPairCountForObject(void *pObject0) { return 0; }
	/* DUMMY */ virtual int GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList) { return 0; }
};

#endif
