// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_OBJECTHASH_H
#define PHYSICS_OBJECTHASH_H

#include "physics_internal.h"
#include "vphysics/object_hash.h"

// Based heavily on btHashedSimplePairCache from Bullet - MIT licensed.

class CPhysicsObjectPairHash : public IPhysicsObjectPairHash {
public:
	CPhysicsObjectPairHash();

	virtual void AddObjectPair(void *pObject0, void *pObject1);
	/* DUMMY */ virtual void RemoveObjectPair(void *pObject0, void *pObject1) {}
	virtual bool IsObjectPairInHash(void *pObject0, void *pObject1);
	/* DUMMY */ virtual void RemoveAllPairsForObject(void *pObject0) {}
	virtual bool IsObjectInHash(void *pObject0);
	/* DUMMY */ virtual int GetPairCountForObject(void *pObject0) { return 0; }
	/* DUMMY */ virtual int GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList) { return 0; }

private:
	// Using addition so swapping isn't required.
	static FORCEINLINE unsigned int GetHash(IPhysicsObject *objectA, IPhysicsObject *objectB) {
		return btHashPtr(reinterpret_cast<void *>(
				reinterpret_cast<size_t>(objectA) + reinterpret_cast<size_t>(objectB))).getHash();
	}
	FORCEINLINE unsigned int GetHashTableIndex(IPhysicsObject *objectA, IPhysicsObject *objectB) const {
		return GetHash(objectA, objectB) & ((unsigned int) m_PairArray.capacity() - 1);
	}

	struct ObjectPair {
		IPhysicsObject *m_ObjectA, *m_ObjectB;
		int m_PreviousInA, m_PreviousInB;
		union {
			int m_NextInA;
			int m_NextFree;
		};
		int m_NextInB;

		FORCEINLINE unsigned int GetHash() const {
			return CPhysicsObjectPairHash::GetHash(m_ObjectA, m_ObjectB);
		}
		FORCEINLINE bool Equals(const IPhysicsObject *objectA, const IPhysicsObject *objectB) const {
			return (objectA == m_ObjectA && objectB == m_ObjectB) ||
					(objectA == m_ObjectB && objectB == m_ObjectA);
		}
	};

	btAlignedObjectArray<ObjectPair> m_PairArray;
	int m_FirstFreePair; // Using a free list to make indices persistent.

	btAlignedObjectArray<int> m_HashTable;
	btAlignedObjectArray<int> m_Next;

	btHashMap<btHashPtr, int> m_FirstPairsForObjects;

	void GrowTables();

	inline int InternalFindPair(IPhysicsObject *objectA, IPhysicsObject *objectB, unsigned int hashTableIndex) const {
		int index = m_HashTable[hashTableIndex];
		while (index >= 0 && !m_PairArray[index].Equals(objectA, objectB)) {
			index = m_Next[index];
		}
		return index;
	}
};

#endif
