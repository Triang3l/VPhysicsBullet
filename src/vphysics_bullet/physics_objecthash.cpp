// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_objecthash.h"

CPhysicsObjectPairHash::CPhysicsObjectPairHash() : m_FirstFreePair(-1) {
	m_PairArray.reserve(2);
	GrowTables();
}

void CPhysicsObjectPairHash::GrowTables() {
	int newCapacity = m_PairArray.capacity();
	if (m_HashTable.size() >= newCapacity) {
		return;
	}

	m_HashTable.resizeNoInitialize(newCapacity);
	memset(&m_HashTable[0], 0xff, newCapacity * sizeof(m_HashTable[0]));
	m_Next.resizeNoInitialize(newCapacity);
	memset(&m_Next[0], 0xff, newCapacity * sizeof(m_Next[0]));

	int pairCount = m_PairArray.size();
	for (int pairIndex = 0; pairIndex < pairCount; ++pairIndex) {
		const ObjectPair &pair = m_PairArray[pairIndex];
		if (pair.m_Objects[0] == nullptr) {
			continue; // Free pair.
		}
		unsigned int hash = GetHash(pair.m_Objects[0], pair.m_Objects[1]);
		m_Next[pairIndex] = m_HashTable[hash];
		m_HashTable[hash] = pairIndex;
	}
}

void CPhysicsObjectPairHash::AddObjectPair(void *pObject0, void *pObject1) {
	if (pObject0 == nullptr || pObject1 == nullptr) {
		return;
	}

	unsigned int hash = GetHash(pObject0, pObject1);
	int pairIndex = InternalFindPair(pObject0, pObject1, hash);
	if (pairIndex >= 0) {
		return;
	}

	if (m_FirstFreePair >= 0) {
		pairIndex = m_FirstFreePair;
		m_FirstFreePair = m_PairArray[pairIndex].m_Next[0];
	} else {
		pairIndex = m_PairArray.size();
		int oldCapacity = m_PairArray.capacity();
		m_PairArray.expandNonInitializing();
		int newCapacity = m_PairArray.capacity();
		if (oldCapacity < newCapacity) {
			m_PairArray[pairIndex].m_Objects[0] = nullptr; // Ignore during expansion.
			GrowTables();
			hash = GetHash(pObject0, pObject1);
		}
	}

	ObjectPair &pair = m_PairArray[pairIndex];
	pair.m_Objects[0] = pObject0;
	pair.m_Objects[1] = pObject1;

	pair.m_Previous[0] = -1;
	pair.m_Previous[1] = -1;

	int *firstPair = m_FirstPairsForObjects.find(pObject0);
	if (firstPair != nullptr) {
		ObjectPair &nextPair = m_PairArray[*firstPair];
		if (nextPair.m_Objects[0] == pObject0) {
			nextPair.m_Previous[0] = pairIndex;
		}
		if (nextPair.m_Objects[1] == pObject0) {
			nextPair.m_Previous[1] = pairIndex;
		}
		pair.m_Next[0] = *firstPair;
		*firstPair = pairIndex;
	} else {
		pair.m_Next[0] = -1;
		m_FirstPairsForObjects.insert(pObject0, pairIndex);
	}

	if (pObject0 != pObject1) {
		firstPair = m_FirstPairsForObjects.find(pObject1);
		if (firstPair != nullptr) {
			ObjectPair &nextPair = m_PairArray[*firstPair];
			if (nextPair.m_Objects[0] == pObject1) {
				nextPair.m_Previous[0] = pairIndex;
			}
			if (nextPair.m_Objects[1] == pObject1) {
				nextPair.m_Previous[1] = pairIndex;
			}
			pair.m_Next[1] = *firstPair;
			*firstPair = pairIndex;
		} else {
			pair.m_Next[1] = -1;
			m_FirstPairsForObjects.insert(pObject1, pairIndex);
		}
	} else {
		pair.m_Next[1] = pair.m_Next[0];
	}

	m_Next[pairIndex] = m_HashTable[hash];
	m_HashTable[hash] = pairIndex;
}

void CPhysicsObjectPairHash::RemoveObjectPair(void *pObject0, void *pObject1) {
	if (pObject0 == nullptr || pObject1 == nullptr) {
		return;
	}

	unsigned int hash = GetHash(pObject0, pObject1);
	int previousHashPairIndex = -1, pairIndex = m_HashTable[hash];
	while (pairIndex >= 0 && !m_PairArray[pairIndex].Equals(pObject0, pObject1)) {
		previousHashPairIndex = pairIndex;
		pairIndex = m_Next[pairIndex];
	}
	if (pairIndex < 0) {
		return;
	}
	if (previousHashPairIndex >= 0) {
		m_Next[previousHashPairIndex] = m_Next[pairIndex];
	} else {
		m_HashTable[hash] = m_Next[pairIndex];
	}

	ObjectPair &pair = m_PairArray[pairIndex];

	if (pair.m_Next[0] >= 0) {
		ObjectPair &nextPair = m_PairArray[pair.m_Next[0]];
		if (nextPair.m_Objects[0] == pair.m_Objects[0]) {
			nextPair.m_Previous[0] = pair.m_Previous[0];
		}
		if (nextPair.m_Objects[1] == pair.m_Objects[0]) {
			nextPair.m_Previous[1] = pair.m_Previous[0];
		}
	}
	if (pair.m_Objects[0] != pair.m_Objects[1] && pair.m_Next[1] >= 0) {
		ObjectPair &nextPair = m_PairArray[pair.m_Next[1]];
		if (nextPair.m_Objects[0] == pair.m_Objects[1]) {
			nextPair.m_Previous[0] = pair.m_Previous[1];
		}
		if (nextPair.m_Objects[1] == pair.m_Objects[1]) {
			nextPair.m_Previous[1] = pair.m_Previous[1];
		}
	}

	if (pair.m_Previous[0] >= 0) {
		ObjectPair &previousPair = m_PairArray[pair.m_Previous[0]];
		if (previousPair.m_Objects[0] == pair.m_Objects[0]) {
			previousPair.m_Next[0] = pair.m_Next[0];
		}
		if (previousPair.m_Objects[1] == pair.m_Objects[0]) {
			previousPair.m_Next[1] = pair.m_Next[0];
		}
	} else {
		if (pair.m_Next[0] >= 0) {
			m_FirstPairsForObjects.insert(pair.m_Objects[0], pair.m_Next[0]);
		} else {
			m_FirstPairsForObjects.remove(pair.m_Objects[0]);
		}
	}
	if (pair.m_Objects[0] != pair.m_Objects[1]) {
		if (pair.m_Previous[1] >= 0) {
			ObjectPair &previousPair = m_PairArray[pair.m_Previous[1]];
			if (previousPair.m_Objects[0] == pair.m_Objects[1]) {
				previousPair.m_Next[0] = pair.m_Next[1];
			}
			if (previousPair.m_Objects[1] == pair.m_Objects[1]) {
				previousPair.m_Next[1] = pair.m_Next[1];
			}
		} else {
			if (pair.m_Next[1] >= 0) {
				m_FirstPairsForObjects.insert(pair.m_Objects[1], pair.m_Next[1]);
			} else {
				m_FirstPairsForObjects.remove(pair.m_Objects[1]);
			}
		}
	}

	pair.m_Objects[0] = nullptr;
	pair.m_Next[0] = m_FirstFreePair;
	m_FirstFreePair = pairIndex;
}

bool CPhysicsObjectPairHash::IsObjectPairInHash(void *pObject0, void *pObject1) {
	if (pObject0 == nullptr || pObject1 == nullptr) {
		return false;
	}
	return InternalFindPair(pObject0, pObject1, GetHash(pObject0, pObject1)) >= 0;
}

void CPhysicsObjectPairHash::RemoveAllPairsForObject(void *pObject0) {
	if (pObject0 == nullptr) {
		return;
	}

	int *firstPair = m_FirstPairsForObjects.find(pObject0);
	if (firstPair == nullptr) {
		return;
	}
	int pairIndex = *firstPair;
	while (pairIndex >= 0) {
		ObjectPair &pair = m_PairArray[pairIndex];

		if (pair.m_Objects[0] != pair.m_Objects[1]) {
			int otherObjectIndex = (int) (pair.m_Objects[0] == pObject0);
			void *otherObject = pair.m_Objects[otherObjectIndex];
			int otherNextPairIndex = pair.m_Next[otherObjectIndex];
			int otherPreviousPairIndex = pair.m_Previous[otherObjectIndex];

			if (otherNextPairIndex >= 0) {
				ObjectPair &nextPair = m_PairArray[otherNextPairIndex];
				if (nextPair.m_Objects[0] == otherObject) {
					nextPair.m_Previous[0] = otherPreviousPairIndex;
				}
				if (nextPair.m_Objects[1] == otherObject) {
					nextPair.m_Previous[1] = otherPreviousPairIndex;
				}
			}

			if (otherPreviousPairIndex >= 0) {
				ObjectPair &previousPair = m_PairArray[otherPreviousPairIndex];
				if (previousPair.m_Objects[0] == otherObject) {
					previousPair.m_Next[0] = otherNextPairIndex;
				}
				if (previousPair.m_Objects[1] == otherObject) {
					previousPair.m_Next[1] = otherNextPairIndex;
				}
			} else {
				if (otherNextPairIndex >= 0) {
					m_FirstPairsForObjects.insert(otherObject, otherNextPairIndex);
				} else {
					m_FirstPairsForObjects.remove(otherObject);
				}
			}
		}

		unsigned int hash = GetHash(pair.m_Objects[0], pair.m_Objects[1]);
		int previousHashPairIndex = -1, hashPairIndex = m_HashTable[hash];
		while (hashPairIndex != pairIndex && hashPairIndex >= 0 /* Safety */) {
			previousHashPairIndex = hashPairIndex;
			hashPairIndex = m_Next[hashPairIndex];
		}
		Assert(hashPairIndex == pairIndex);
		if (previousHashPairIndex >= 0) {
			m_Next[previousHashPairIndex] = m_Next[hashPairIndex];
		} else {
			m_HashTable[hash] = m_Next[hashPairIndex];
		}

		pair.m_Objects[0] = nullptr;
		pair.m_Next[0] = m_FirstFreePair;
		m_FirstFreePair = pairIndex;
	}

	m_FirstPairsForObjects.remove(pObject0);
}

bool CPhysicsObjectPairHash::IsObjectInHash(void *pObject0) {
	if (pObject0 == nullptr) {
		return false;
	}
	return m_FirstPairsForObjects.find(pObject0) != nullptr;
}

int CPhysicsObjectPairHash::GetPairCountForObject(void *pObject0) {
	if (pObject0 == nullptr) {
		return 0;
	}
	int *firstPair = m_FirstPairsForObjects.find(pObject0);
	if (firstPair == nullptr) {
		return 0;
	}
	int pairIndex = *firstPair;
	int pairCount = 0;
	while (pairIndex >= 0) {
		++pairCount;
		const ObjectPair &pair = m_PairArray[pairIndex];
		pairIndex = pair.m_Next[pair.m_Objects[0] != pObject0];
	}
	return pairCount;
}

int CPhysicsObjectPairHash::GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList) {
	if (pObject0 == nullptr) {
		return 0;
	}
	int *firstPair = m_FirstPairsForObjects.find(pObject0);
	if (firstPair == nullptr) {
		return 0;
	}
	int pairIndex = *firstPair;
	int pairCount = 0;
	while (pairIndex >= 0 && pairCount < nMaxCount) {
		const ObjectPair &pair = m_PairArray[pairIndex];
		int pairObjectIndex = (pair.m_Objects[0] != pObject0);
		ppObjectList[pairCount++] = pair.m_Objects[pairObjectIndex ^ 1];
		pairIndex = pair.m_Next[pairObjectIndex];
	}
	return pairCount;
}
