// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_material.h"
#include "physics_parse.h"

// memdbgon must be the last include file in a .cpp file!!!
// #include "tier0/memdbgon.h"

static CPhysicsSurfaceProps s_PhysSurfaceProps;
CPhysicsSurfaceProps *g_pPhysSurfaceProps = &s_PhysSurfaceProps;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsSurfaceProps, IPhysicsSurfaceProps,
		VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, s_PhysSurfaceProps);

CPhysicsSurfaceProps::CPhysicsSurfaceProps() {
	memset(&m_EmptySurfaceData, 0, sizeof(m_EmptySurfaceData));
	memset(&m_ShadowSurfaceData, 0, sizeof(m_ShadowSurfaceData));
	m_ShadowSurfaceData.physics.friction = 0.8f;

	// By default, every index maps to itself.
	for (int materialIndex = 0; materialIndex < Q_ARRAYSIZE(m_CollideMaterialMap); ++materialIndex) {
		m_CollideMaterialMap[materialIndex] = materialIndex;
	}
}

void CPhysicsSurfaceProps::CopyPhysicsProperties(const surfacedata_t &from, surfacedata_t &to) {
	to.physics = from.physics;
	to.audio = from.audio;
	to.sounds = from.sounds;
	to.game = from.game;
}

int CPhysicsSurfaceProps::ParseSurfaceData(const char *pFilename, const char *pTextfile) {
	CUtlSymbol fileNameID = m_Strings.AddString(pFilename);
	if (m_FileList.Find(fileNameID) != m_FileList.InvalidIndex()) {
		return 0;
	}
	m_FileList.AddToTail(fileNameID);

	int defaultSurfaceIndex = GetSurfaceIndex("default");
	const surfacedata_t *defaultSurfaceData = nullptr;
	if (defaultSurfaceIndex >= 0) {
		defaultSurfaceData = &m_Surfaces[defaultSurfaceIndex].m_Data;
	}

	const char *text = pTextfile;
	do {
		char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
		text = CVPhysicsKeyParser::ParseKeyvalue(text, key, value);
		if (V_strcmp(value, "{") != 0) {
			continue;
		}

		Surface_t surface;
		surface.m_Name = m_Strings.AddString(key);
		memset(&surface.m_Data, 0, sizeof(surface.m_Data));
		if (defaultSurfaceData != nullptr) {
			CopyPhysicsProperties(*defaultSurfaceData, surface.m_Data);
		} else {
			surface.m_Data.game.maxSpeedFactor = 1.0f;
			surface.m_Data.game.jumpFactor = 1.0f;
		}

		do {
			text = CVPhysicsKeyParser::ParseKeyvalue(text, key, value);
			if (V_strcmp(key, "}") == 0) {
				const char *surfaceName = m_Strings.String(surface.m_Name);
				if (GetSurfaceIndex(surfaceName) >= 0) {
					// Already in the database, don't add again.
					break;
				}
				int surfaceIndex = m_Surfaces.AddToTail(surface);
				m_SurfaceDict.Insert(SurfaceDictEntry_t(surface.m_Name, surfaceIndex));
				if (defaultSurfaceIndex < 0 && V_stricmp(surfaceName, "default") == 0) {
					defaultSurfaceIndex = surfaceIndex;
					defaultSurfaceData = &m_Surfaces[defaultSurfaceIndex].m_Data;
				}
				break;
			}
			if (V_stricmp(key, "base") == 0) {
				int baseSurfaceIndex = GetSurfaceIndex(value);
				if (baseSurfaceIndex >= 0) {
					CopyPhysicsProperties(m_Surfaces[baseSurfaceIndex].m_Data, surface.m_Data);
				}
			} else if (V_stricmp(key, "dampening") == 0) {
				surface.m_Data.physics.dampening = atof(value);
			} else if (V_stricmp(key, "thickness") == 0) {
				surface.m_Data.physics.thickness = atof(value);
			} else if (V_stricmp(key, "density") == 0) {
				surface.m_Data.physics.density = atof(value);
			} else if (V_stricmp(key, "elasticity") == 0) {
				surface.m_Data.physics.elasticity = atof(value);
			} else if (V_stricmp(key, "friction") == 0) {
				surface.m_Data.physics.friction = atof(value);
			} else if (V_stricmp(key, "maxspeedfactor") == 0) {
				surface.m_Data.game.maxSpeedFactor = atof(value);
			} else if (V_stricmp(key, "jumpfactor") == 0) {
				surface.m_Data.game.jumpFactor = atof(value);
			} else if (V_stricmp(key, "climbable") == 0) {
				surface.m_Data.game.climbable = atoi(value);
			} else if (V_stricmp(key, "audioReflectivity") == 0) {
				surface.m_Data.audio.reflectivity = atof(value);
			} else if (V_stricmp(key, "audioHardnessFactor") == 0) {
				surface.m_Data.audio.hardnessFactor = atof(value);
			} else if (V_stricmp(key, "audioHardMinVelocity") == 0) {
				surface.m_Data.audio.hardVelocityThreshold = atof(value);
			} else if (V_stricmp(key, "audioRoughnessFactor") == 0) {
				surface.m_Data.audio.roughnessFactor = atof(value);
			} else if (V_stricmp(key, "scrapeRoughThreshold") == 0) {
				surface.m_Data.audio.roughThreshold = atof(value);
			} else if (V_stricmp(key, "impactHardThreshold") == 0) {
				surface.m_Data.audio.hardThreshold = atof(value);
			} else if (V_stricmp(key, "stepleft") == 0) {
				surface.m_Data.sounds.stepleft = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "stepright") == 0) {
				surface.m_Data.sounds.stepright = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "impactsoft") == 0) {
				surface.m_Data.sounds.impactSoft = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "impacthard") == 0) {
				surface.m_Data.sounds.impactHard = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "scrapesmooth") == 0) {
				surface.m_Data.sounds.scrapeSmooth = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "scraperough") == 0) {
				surface.m_Data.sounds.scrapeRough = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "bulletimpact") == 0) {
				surface.m_Data.sounds.bulletImpact = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "break") == 0) {
				surface.m_Data.sounds.breakSound = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "strain") == 0) {
				surface.m_Data.sounds.strainSound = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "rolling") == 0) {
				surface.m_Data.sounds.rolling = m_Strings.AddString(value) + 1;
			} else if (V_stricmp(key, "gamematerial") == 0) {
				if (V_strlen(value) == 1 && !V_isdigit(value[0])) {
					surface.m_Data.game.material = toupper(value[0]);
				} else {
					surface.m_Data.game.material = atoi(value);
				}
			} else {
				AssertMsg2(false, "Bad surfaceprop key %s (%s)", key, value);
			}
		} while (text != nullptr);
	} while (text != nullptr);

	return m_Surfaces.Count();
}

int CPhysicsSurfaceProps::SurfacePropCount() const {
	return m_Surfaces.Count();
}

int CPhysicsSurfaceProps::GetReservedSurfaceIndex(const char *surfacePropName) const {
	if (V_stricmp(surfacePropName, "$MATERIAL_INDEX_SHADOW") == 0) {
		return MATERIAL_INDEX_SHADOW;
	}
	return -1;
}

surfacedata_t *CPhysicsSurfaceProps::GetReservedSurfaceData(int surfaceDataIndex) {
	switch (surfaceDataIndex) {
	case MATERIAL_INDEX_SHADOW:
		return &m_ShadowSurfaceData;
	}
	return nullptr;
}

const surfacedata_t *CPhysicsSurfaceProps::GetReservedSurfaceData(int surfaceDataIndex) const {
	switch (surfaceDataIndex) {
	case MATERIAL_INDEX_SHADOW:
		return &m_ShadowSurfaceData;
	}
	return nullptr;
}

const char *CPhysicsSurfaceProps::GetReservedSurfaceName(int surfaceDataIndex) const {
	switch (surfaceDataIndex) {
	case MATERIAL_INDEX_SHADOW:
		return "$MATERIAL_INDEX_SHADOW";
	}
	return nullptr;
}

int CPhysicsSurfaceProps::GetSurfaceIndex(const char *pSurfacePropName) const {
	int index;

	if (pSurfacePropName[0] == '$') {
		index = GetReservedSurfaceIndex(pSurfacePropName);
		if (index >= 0) {
			return index;
		}
	}

	CUtlSymbol id = m_Strings.Find(pSurfacePropName);
	if (id.IsValid()) {
		SurfaceDictEntry_t key(id);
		int dictIndex = m_SurfaceDict.Find(key);
		if (dictIndex != m_SurfaceDict.InvalidIndex()) {
			return m_SurfaceDict[dictIndex].m_Index;
		}
	}

	return -1;
}

void CPhysicsSurfaceProps::GetPhysicsProperties(int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity) const {
	const surfacephysicsparams_t &physicsParams = GetSurfaceData(surfaceDataIndex)->physics;
	if (density != nullptr) {
		*density = physicsParams.density;
	}
	if (thickness != nullptr) {
		*thickness = physicsParams.thickness;
	}
	if (elasticity != nullptr) {
		*elasticity = physicsParams.elasticity;
	}
	if (friction != nullptr) {
		*friction = physicsParams.friction;
	}
}

surfacedata_t *CPhysicsSurfaceProps::GetSurfaceData(int surfaceDataIndex) {
	if (surfaceDataIndex >= 0 && surfaceDataIndex < m_Surfaces.Count()) {
		return &m_Surfaces[surfaceDataIndex].m_Data;
	}
	surfacedata_t *reservedSurfaceData = GetReservedSurfaceData(surfaceDataIndex);
	if (reservedSurfaceData != nullptr) {
		return reservedSurfaceData;
	}
	surfaceDataIndex = GetSurfaceIndex("default");
	if (surfaceDataIndex >= 0) {
		return &m_Surfaces[surfaceDataIndex].m_Data;
	}
	return &m_EmptySurfaceData;
}

const surfacedata_t *CPhysicsSurfaceProps::GetSurfaceData(int surfaceDataIndex) const {
	if (surfaceDataIndex >= 0 && surfaceDataIndex < m_Surfaces.Count()) {
		return &m_Surfaces[surfaceDataIndex].m_Data;
	}
	const surfacedata_t *reservedSurfaceData = GetReservedSurfaceData(surfaceDataIndex);
	if (reservedSurfaceData != nullptr) {
		return reservedSurfaceData;
	}
	surfaceDataIndex = GetSurfaceIndex("default");
	if (surfaceDataIndex >= 0) {
		return &m_Surfaces[surfaceDataIndex].m_Data;
	}
	return &m_EmptySurfaceData;
}

const char *CPhysicsSurfaceProps::GetString(unsigned short stringTableIndex) const {
	return m_Strings.String(CUtlSymbol(stringTableIndex - 1));
}

const char *CPhysicsSurfaceProps::GetPropName(int surfaceDataIndex) const {
	const char *surfaceName = GetReservedSurfaceName(surfaceDataIndex);
	if (surfaceName == nullptr && surfaceDataIndex >= 0 && surfaceDataIndex < m_Surfaces.Count()) {
		surfaceName = m_Strings.String(m_Surfaces[surfaceDataIndex].m_Name);
	}
	return surfaceName;
}

void CPhysicsSurfaceProps::GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t *pParamsOut) const {
	if (pParamsOut != nullptr) {
		*pParamsOut = GetSurfaceData(surfaceDataIndex)->physics;
	}
}

void CPhysicsSurfaceProps::SetWorldMaterialIndexTable(int *pMapArray, int mapSize) {
	memcpy(m_CollideMaterialMap, pMapArray, MIN(mapSize, Q_ARRAYSIZE(m_CollideMaterialMap)) * sizeof(int));
}

int CPhysicsSurfaceProps::RemapCollideMaterialIndex(int materialIndex) const {
	if (materialIndex >= Q_ARRAYSIZE(m_CollideMaterialMap)) {
		return materialIndex;
	}
	return m_CollideMaterialMap[materialIndex];
}
