// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_internal.h"
#include "physics_collide.h"
#include "physics_environment.h"
#include "physics_objecthash.h"
#include "vphysics/collision_set.h"
#include "tier1/tier1.h"
#include "tier1/utlvector.h"
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

// memdbgon must be the last include file in a .cpp file!!!
// #include "tier0/memdbgon.h"

static void *VPhysicsBulletAlloc(size_t size) {
	return MemAlloc_Alloc(size);
}

static void *VPhysicsBulletAlignedAlloc(size_t size, int alignment) {
	return MemAlloc_AllocAligned(size, alignment);
}

static void VPhysicsBulletFree(void *memblock) {
	MemAlloc_FreeAligned(memblock);
}

#ifdef POSIX
__attribute__((constructor))
#endif
void VPhysicsInit() {
	MathLib_Init(2.2f, 2.2f, 0.0f, 2, false, true, true, false);

	btAlignedAllocSetCustom(VPhysicsBulletAlloc, VPhysicsBulletFree);
	btAlignedAllocSetCustomAligned(VPhysicsBulletAlignedAlloc, VPhysicsBulletFree);

	gContactBreakingThreshold = 0.5f * VPHYSICS_CONVEX_DISTANCE_MARGIN;

	gDeactivationTime = 4.0f; // To match IVP more closely.
}

#ifdef WIN32
BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved) {
	if (fdwReason == DLL_PROCESS_ATTACH) {
		VPhysicsInit();
	}
	return TRUE;
}
#endif

class CPhysicsCollisionSet : public IPhysicsCollisionSet {
public:
	CPhysicsCollisionSet();

	virtual void EnableCollisions(int index0, int index1);
	virtual void DisableCollisions(int index0, int index1);
	virtual bool ShouldCollide(int index0, int index1);

private:
	uint32 m_Set[32];
};

CPhysicsCollisionSet::CPhysicsCollisionSet() {
	memset(m_Set, 0, sizeof(m_Set));
}

void CPhysicsCollisionSet::EnableCollisions(int index0, int index1) {
	m_Set[index0] |= ((uint32) 1) << index1;
	m_Set[index1] |= ((uint32) 1) << index0;
}

void CPhysicsCollisionSet::DisableCollisions(int index0, int index1) {
	m_Set[index0] &= ~(((uint32) 1) << index1);
	m_Set[index1] &= ~(((uint32) 1) << index0);
}

bool CPhysicsCollisionSet::ShouldCollide(int index0, int index1) {
	return (m_Set[index0] & (((uint32) 1) << index1)) != 0;
}

class CPhysicsInterface : public CTier1AppSystem<IPhysics> {
public:
	virtual void *QueryInterface(const char *pInterfaceName);

	virtual IPhysicsEnvironment *CreateEnvironment();
	virtual void DestroyEnvironment(IPhysicsEnvironment *pEnvironment);
	virtual IPhysicsEnvironment *GetActiveEnvironmentByIndex(int index);
	/* DUMMY */ virtual IPhysicsObjectPairHash *CreateObjectPairHash() { return new CPhysicsObjectPairHash; }
	/* DUMMY */ virtual void DestroyObjectPairHash(IPhysicsObjectPairHash *pHash) { delete static_cast<CPhysicsObjectPairHash *>(pHash); }
	virtual IPhysicsCollisionSet *FindOrCreateCollisionSet(unsigned int id, int maxElementCount);
	virtual IPhysicsCollisionSet *FindCollisionSet(unsigned int id);
	virtual void DestroyAllCollisionSets();

private:
	CUtlVector<IPhysicsEnvironment *> m_Environments;
	btHashMap<btHashInt, IPhysicsCollisionSet *> m_CollisionSets;
};

static CPhysicsInterface s_MainDLLInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsInterface, IPhysics,
		VPHYSICS_INTERFACE_VERSION, s_MainDLLInterface);

void *CPhysicsInterface::QueryInterface(const char *pInterfaceName) {
	return Sys_GetFactoryThis()(pInterfaceName, nullptr);
}

IPhysicsEnvironment *CPhysicsInterface::CreateEnvironment() {
	IPhysicsEnvironment *environment = new CPhysicsEnvironment;
	m_Environments.AddToTail(environment);
	return environment;
}

void CPhysicsInterface::DestroyEnvironment(IPhysicsEnvironment *pEnvironment) {
	m_Environments.FindAndRemove(pEnvironment);
	delete pEnvironment;
}

IPhysicsEnvironment *CPhysicsInterface::GetActiveEnvironmentByIndex(int index) {
	if (index < 0 || index >= m_Environments.Count()) {
		return nullptr;
	}
	return m_Environments[index];
}

IPhysicsCollisionSet *CPhysicsInterface::FindOrCreateCollisionSet(unsigned int id, int maxElementCount) {
	IPhysicsCollisionSet * const *setPointer = m_CollisionSets.find((btHashInt) id);
	if (setPointer == nullptr) {
		m_CollisionSets.insert((btHashInt) id, new CPhysicsCollisionSet);
		setPointer = m_CollisionSets.find((btHashInt) id);
	}
	return *setPointer;
}

IPhysicsCollisionSet *CPhysicsInterface::FindCollisionSet(unsigned int id) {
	IPhysicsCollisionSet * const *setPointer = m_CollisionSets.find((btHashInt) id);
	return (setPointer != nullptr ? *setPointer : nullptr);
}

void CPhysicsInterface::DestroyAllCollisionSets() {
	int setCount = m_CollisionSets.size();
	for (int setIndex = 0; setIndex < setCount; ++setIndex) {
		delete *m_CollisionSets.getAtIndex(setIndex);
	}
	m_CollisionSets.clear();
}
