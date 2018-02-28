// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_OBJECTHASH_H
#define PHYSICS_OBJECTHASH_H

#include "physics_internal.h"
#include "vphysics/object_hash.h"

// Objects are not necessarily physics objects.

class CPhysicsObjectPairHash : public IPhysicsObjectPairHash {
public:
	CPhysicsObjectPairHash();

	virtual void AddObjectPair(void *pObject0, void *pObject1);
	virtual void RemoveObjectPair(void *pObject0, void *pObject1);
	virtual bool IsObjectPairInHash(void *pObject0, void *pObject1);
	virtual void RemoveAllPairsForObject(void *pObject0);
	virtual bool IsObjectInHash(void *pObject0);
	virtual int GetPairCountForObject(void *pObject0);
	virtual int GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList);

private:
	// Using addition so swapping isn't required.
	FORCEINLINE unsigned int GetHash(void *object0, void *object1) const {
		return btHashPtr(reinterpret_cast<void *>(reinterpret_cast<size_t>(object0) +
				reinterpret_cast<size_t>(object1))).getHash() &
				((unsigned int) m_PairArray.capacity() - 1);
	}

	struct ObjectPair {
		void *m_Objects[2]; // Free if the first object is nullptr.
		// Linked list of pairs for each object. m_Next[0] also links the free list.
		int m_Previous[2], m_Next[2];
		FORCEINLINE bool Equals(void *object0, void *object1) const {
			return (object0 == m_Objects[0] && object1 == m_Objects[1]) ||
					(object0 == m_Objects[1] && object1 == m_Objects[0]);
		}
	};

	btAlignedObjectArray<ObjectPair> m_PairArray;
	int m_FirstFreePair; // Using a free list to make indices persistent.

	btAlignedObjectArray<int> m_HashTable;
	btAlignedObjectArray<int> m_Next;

	btHashMap<btHashPtr, int> m_FirstPairsForObjects;

	void GrowTables();

	inline int InternalFindPair(void *object0, void *object1, unsigned int hashTableIndex) const {
		int pairIndex = m_HashTable[hashTableIndex];
		while (pairIndex >= 0 && !m_PairArray[pairIndex].Equals(object0, object1)) {
			pairIndex = m_Next[pairIndex];
		}
		return pairIndex;
	}
};

#endif
