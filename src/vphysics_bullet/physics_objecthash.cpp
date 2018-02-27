// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_objecthash.h"

// Based heavily on btHashedSimplePairCache from Bullet - MIT licensed.

CPhysicsObjectPairHash::CPhysicsObjectPairHash() : m_FirstFreePair(-1) {
	m_PairArray.reserve(2);
	GrowTables();
}

void CPhysicsObjectPairHash::GrowTables() {
	int currentHashTableSize = m_HashTable.size();
	int newCapacity = m_PairArray.capacity();
	if (currentHashTableSize >= newCapacity) {
		return;
	}

	m_HashTable.resizeNoInitialize(newCapacity);
	memset(&m_HashTable[0], 0xff, newCapacity * sizeof(m_HashTable[0]));
	m_Next.resizeNoInitialize(newCapacity);
	memset(&m_Next[0], 0xff, newCapacity * sizeof(m_Next[0]));

	unsigned int hashMask = (unsigned int) newCapacity - 1;
	for (int pairIndex = 0; pairIndex < currentHashTableSize; ++pairIndex) {
		unsigned int hashValue = m_PairArray[pairIndex].GetHash() & hashMask;
		m_Next[pairIndex] = m_HashTable[hashValue];
		m_HashTable[hashValue] = pairIndex;
	}
}

void CPhysicsObjectPairHash::AddObjectPair(void *pObject0, void *pObject1) {
	if (pObject0 == nullptr || pObject1 == nullptr) {
		return;
	}
	IPhysicsObject *objectA = reinterpret_cast<IPhysicsObject *>(pObject0);
	IPhysicsObject *objectB = reinterpret_cast<IPhysicsObject *>(pObject1);
	unsigned int hashTableIndex = GetHashTableIndex(objectA, objectB);

	int pairIndex = InternalFindPair(objectA, objectB, hashTableIndex);
	if (pairIndex >= 0) {
		return;
	}

	if (m_FirstFreePair >= 0) {
		pairIndex = m_FirstFreePair;
		m_FirstFreePair = m_PairArray[pairIndex].m_NextFree;
	} else {
		pairIndex = m_PairArray.size();
		int oldCapacity = m_PairArray.capacity();
		m_PairArray.expandNonInitializing();
		int newCapacity = m_PairArray.capacity();
		if (oldCapacity < newCapacity) {
			GrowTables();
			hashTableIndex = GetHashTableIndex(objectA, objectB);
		}
	}

	ObjectPair &pair = m_PairArray[pairIndex];
	pair.m_ObjectA = objectA;
	pair.m_ObjectB = objectB;

	pair.m_PreviousInA = -1;
	pair.m_PreviousInB = -1;

	// Updating the linked list for object A.
	int *firstPair = m_FirstPairsForObjects.find(objectA);
	if (firstPair != nullptr) {
		ObjectPair &nextPair = m_PairArray[*firstPair];
		if (objectA == nextPair.m_ObjectA) {
			nextPair.m_PreviousInA = pairIndex;
		}
		if (objectB == nextPair.m_ObjectB) {
			nextPair.m_PreviousInB = pairIndex;
		}
		pair.m_NextInA = *firstPair;
		*firstPair = pairIndex;
	} else {
		pair.m_NextInA = -1;
		m_FirstPairsForObjects.insert(objectA, pairIndex);
	}

	// Updating the linked list for object B.
	if (objectA != objectB) {
		firstPair = m_FirstPairsForObjects.find(objectA);
		if (firstPair != nullptr) {
			ObjectPair &nextPair = m_PairArray[*firstPair];
			if (objectA == nextPair.m_ObjectA) {
				nextPair.m_PreviousInA = pairIndex;
			}
			if (objectB == nextPair.m_ObjectB) {
				nextPair.m_PreviousInB = pairIndex;
			}
			pair.m_NextInB = *firstPair;
			*firstPair = pairIndex;
		} else {
			pair.m_NextInB = -1;
			m_FirstPairsForObjects.insert(objectB, pairIndex);
		}
	} else {
		pair.m_NextInB = pair.m_NextInA;
	}

	m_Next[pairIndex] = m_HashTable[hashTableIndex];
	m_HashTable[hashTableIndex] = pairIndex;
}

bool CPhysicsObjectPairHash::IsObjectPairInHash(void *pObject0, void *pObject1) {
	IPhysicsObject *objectA = reinterpret_cast<IPhysicsObject *>(pObject0);
	IPhysicsObject *objectB = reinterpret_cast<IPhysicsObject *>(pObject1);
	return InternalFindPair(objectA, objectB, GetHashTableIndex(objectA, objectB)) >= 0;
}

bool CPhysicsObjectPairHash::IsObjectInHash(void *pObject0) {
	return m_FirstPairsForObjects.find(pObject0) != nullptr;
}
