// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_PARSE_H
#define PHYSICS_PARSE_H

#include "physics_internal.h"
#include "vcollide_parse.h"
#include "vphysics/constraints.h"
#include "vphysics/vehicles.h"
#include "tier1/strtools.h"

#define VPHYSICS_MAX_KEYVALUE 1024

class CVPhysicsKeyParser : public IVPhysicsKeyParser {
public:
	CVPhysicsKeyParser(const char *pKeyData);

	virtual const char *GetCurrentBlockName();
	virtual bool Finished();
	virtual void ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler);
	virtual void ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler);
	virtual void ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler);
	virtual void ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler);
	virtual void ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler);
	virtual void ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler);
	virtual void SkipBlock();

	static const char *ParseKeyvalue(const char *buffer, char *key, char *value);
	static void ReadVector(const char *pString, Vector &out);
	static void ReadVector4D(const char *pString, Vector4D &out);

private:
	void NextBlock();

	void ParseVehicleAxle(vehicle_axleparams_t &axle);
	void ParseVehicleWheel(vehicle_wheelparams_t &wheel);
	void ParseVehicleSuspension(vehicle_suspensionparams_t &suspension);
	void ParseVehicleBody(vehicle_bodyparams_t &body);
	void ParseVehicleEngine(vehicle_engineparams_t &engine);
	void ParseVehicleEngineBoost(vehicle_engineparams_t &engine);
	void ParseVehicleSteering(vehicle_steeringparams_t &steering);

	const char *m_Text;

	char m_BlockName[VPHYSICS_MAX_KEYVALUE];
};

#endif
