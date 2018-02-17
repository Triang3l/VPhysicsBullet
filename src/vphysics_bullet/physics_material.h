// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_MATERIAL_H
#define PHYSICS_MATERIAL_H

#include "physics_internal.h"
#include "tier1/utllinkedlist.h"
#include "tier1/UtlSortVector.h"
#include "tier1/utlsymbol.h"
#include "tier1/utlvector.h"

enum {
	MATERIAL_INDEX_SHADOW = 0xf000
};

class CPhysicsSurfaceProps : public IPhysicsSurfaceProps {
public:
	CPhysicsSurfaceProps();

	// IPhysicsSurfaceProps methods.

	virtual int ParseSurfaceData(const char *pFilename, const char *pTextfile);
	virtual int SurfacePropCount() const;
	virtual int GetSurfaceIndex(const char *pSurfacePropName) const;
	virtual void GetPhysicsProperties(int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity) const;
	virtual surfacedata_t *GetSurfaceData(int surfaceDataIndex);
	virtual const char *GetString(unsigned short stringTableIndex) const;
	virtual const char *GetPropName(int surfaceDataIndex) const;
	virtual void SetWorldMaterialIndexTable(int *pMapArray, int mapSize);
	virtual void GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t *pParamsOut) const;

	// Internal methods.

	const surfacedata_t *GetSurfaceData(int surfaceDataIndex) const;
	int RemapCollideMaterialIndex(int materialIndex) const;

private:
	void CopyPhysicsProperties(const surfacedata_t &from, surfacedata_t &to);

	int GetReservedSurfaceIndex(const char *surfacePropName) const;
	surfacedata_t *GetReservedSurfaceData(int surfaceDataIndex);
	const surfacedata_t *GetReservedSurfaceData(int surfaceDataIndex) const;
	const char *GetReservedSurfaceName(int surfaceDataIndex) const;

	CUtlSymbolTable m_Strings;

	struct Surface_t {
		CUtlSymbol m_Name;
		surfacedata_t m_Data;
	};
	CUtlFixedLinkedList<Surface_t> m_Surfaces;
	struct SurfaceDictEntry_t {
		CUtlSymbol m_Symbol;
		int m_Index;
		inline SurfaceDictEntry_t(CUtlSymbol symbol, int index = 0) :
				m_Symbol(symbol), m_Index(index) {}
	};
	class SurfaceDictLessFunc {
	public:
		bool Less(const SurfaceDictEntry_t &src1, const SurfaceDictEntry_t &src2, void *ctx) {
			return (UtlSymId_t) src1.m_Symbol < (UtlSymId_t) src2.m_Symbol;
		}
	};
	CUtlSortVector<SurfaceDictEntry_t, SurfaceDictLessFunc> m_SurfaceDict;

	CUtlVector<CUtlSymbol> m_FileList;

	surfacedata_t m_EmptySurfaceData;
	surfacedata_t m_ShadowSurfaceData;

	int m_CollideMaterialMap[128];
};

extern CPhysicsSurfaceProps *g_pPhysSurfaceProps;

#endif
