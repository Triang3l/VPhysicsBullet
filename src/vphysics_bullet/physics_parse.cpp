// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_parse.h"
#include "physics_material.h"
#include "filesystem_helpers.h"

const char *CVPhysicsKeyParser::ParseKeyvalue(const char *buffer, char *key, char *value) {
	buffer = ParseFileInternal(buffer, key, nullptr, nullptr, VPHYSICS_MAX_KEYVALUE);
	V_strlower(key);
	if (V_strcmp(key, "}") == 0) {
		// No value on a close brace.
		value[0] = '\0';
		return buffer;
	}
	buffer = ParseFileInternal(buffer, value, nullptr, nullptr, VPHYSICS_MAX_KEYVALUE);
	V_strlower(value);
	return buffer;
}

void CVPhysicsKeyParser::ReadVector(const char *pString, Vector &out) {
	out.Zero();
	sscanf(pString, "%f %f %f", &out.x, &out.y, &out.z);
}

void CVPhysicsKeyParser::ReadVector4D(const char *pString, Vector4D &out) {
	out.Init();
	sscanf(pString, "%f %f %f %f", &out.x, &out.y, &out.z, &out.w);
}

CVPhysicsKeyParser::CVPhysicsKeyParser(const char *pKeyData) : m_Text(pKeyData) {
	NextBlock();
}

const char *CVPhysicsKeyParser::GetCurrentBlockName() {
	if (m_Text != nullptr) {
		return m_BlockName;
	}
	return nullptr;
}

bool CVPhysicsKeyParser::Finished() {
	return (m_Text == nullptr);
}

void CVPhysicsKeyParser::NextBlock() {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (V_strcmp(value, "{") == 0) {
			V_strcpy(m_BlockName, key);
			return;
		}
	}
}

void CVPhysicsKeyParser::SkipBlock() {
	ParseCustom(nullptr, nullptr);
}

void CVPhysicsKeyParser::ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	if (unknownKeyHandler != nullptr) {
		unknownKeyHandler->SetDefaults(pSolid);
	} else {
		memset(pSolid, 0, sizeof(*pSolid));
	}
	// Disable these until the ragdoll is created.
	pSolid->params.enableCollisions = false;

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			NextBlock();
			return;
		}
		if (V_stricmp(key, "index") == 0) {
			pSolid->index = atoi(value);
		} else if (V_stricmp(key, "name") == 0) {
			V_strncpy(pSolid->name, value, sizeof(pSolid->name));
		} else if (V_stricmp(key, "parent") == 0) {
			V_strncpy(pSolid->parent, value, sizeof(pSolid->parent));
		} else if (V_stricmp(key, "surfaceprop") == 0) {
			V_strncpy(pSolid->surfaceprop, value, sizeof(pSolid->surfaceprop));
		} else if (V_stricmp(key, "mass") == 0) {
			pSolid->params.mass = atof(value);
		} else if (V_stricmp(key, "massCenterOverride") == 0) {
			ReadVector(value, pSolid->massCenterOverride);
			pSolid->params.massCenterOverride = &pSolid->massCenterOverride;
		} else if (V_stricmp(key, "inertia") == 0) {
			pSolid->params.inertia = atof(value);
		} else if (V_stricmp(key, "damping") == 0) {
			pSolid->params.damping = atof(value);
		} else if (V_stricmp(key, "rotdamping") == 0) {
			pSolid->params.rotdamping = atof(value);
		} else if (V_stricmp(key, "volume") == 0) {
			pSolid->params.volume = atof(value);
		} else if (V_stricmp(key, "drag") == 0) {
			pSolid->params.dragCoefficient = atof(value);
		} else if (V_stricmp(key, "rollingdrag") == 0) {
			// Rolling drag is not implemented in v29, removed from object parameters in v31.
		} else if (unknownKeyHandler != nullptr) {
			unknownKeyHandler->ParseKeyValue(pSolid, key, value);
		}
	}
}

void CVPhysicsKeyParser::ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	if (unknownKeyHandler == nullptr) {
		memset(pFluid, 0, sizeof(*pFluid));
	}
	pFluid->index = -1;
	if (unknownKeyHandler != nullptr) {
		unknownKeyHandler->SetDefaults(pFluid);
	}

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			NextBlock();
			return;
		}
		if (V_stricmp(key, "index") == 0) {
			pFluid->index = atoi(value);
		} else if (V_stricmp(key, "damping") == 0) {
			pFluid->params.damping = atof(value);
		} else if (V_stricmp(key, "surfaceplane") == 0) {
			ReadVector4D(value, pFluid->params.surfacePlane);
		} else if (V_stricmp(key, "currentvelocity") == 0) {
			ReadVector(value, pFluid->params.currentVelocity);
		} else if (V_stricmp(key, "contents") == 0) {
			pFluid->params.contents = atoi(value);
		} else if (V_stricmp(key, "surfaceprop") == 0) {
			V_strncpy(pFluid->surfaceprop, value, sizeof(pFluid->surfaceprop));
		} else if (unknownKeyHandler != nullptr) {
			unknownKeyHandler->ParseKeyValue(pFluid, key, value);
		}
	}
}

void CVPhysicsKeyParser::ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	if (unknownKeyHandler == nullptr) {
		memset(pConstraint, 0, sizeof(*pConstraint));
	}
	pConstraint->childIndex = -1;
	pConstraint->parentIndex = -1;
	if (unknownKeyHandler != nullptr) {
		unknownKeyHandler->SetDefaults(pConstraint);
	}

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			NextBlock();
			return;
		}
		if (V_stricmp(key, "parent") == 0) {
			pConstraint->parentIndex = atoi(value);
		} else if (V_stricmp(key, "child") == 0) {
			pConstraint->childIndex = atoi(value);
		} else if (V_stricmp(key, "xmin") == 0) {
			pConstraint->axes[0].minRotation = atof(value);
		} else if (V_stricmp(key, "xmax") == 0) {
			pConstraint->axes[0].maxRotation = atof(value);
		} else if (V_stricmp(key, "xfriction") == 0) {
			pConstraint->axes[0].angularVelocity = 0.0f;
			pConstraint->axes[0].torque = atof(value);
		} else if (V_stricmp(key, "ymin") == 0) {
			pConstraint->axes[1].minRotation = atof(value);
		} else if (V_stricmp(key, "ymax") == 0) {
			pConstraint->axes[1].maxRotation = atof(value);
		} else if (V_stricmp(key, "yfriction") == 0) {
			pConstraint->axes[1].angularVelocity = 0.0f;
			pConstraint->axes[1].torque = atof(value);
		} else if (V_stricmp(key, "zmin") == 0) {
			pConstraint->axes[2].minRotation = atof(value);
		} else if (V_stricmp(key, "zmax") == 0) {
			pConstraint->axes[2].maxRotation = atof(value);
		} else if (V_stricmp(key, "zfriction") == 0) {
			pConstraint->axes[2].angularVelocity = 0.0f;
			pConstraint->axes[2].torque = atof(value);
		} else if (unknownKeyHandler != nullptr) {
			unknownKeyHandler->ParseKeyValue(pConstraint, key, value);
		}
	}
}

void CVPhysicsKeyParser::ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			NextBlock();
			return;
		}
		int propIndex = g_pPhysSurfaceProps->GetSurfaceIndex(key);
		int tableIndex = atoi(value);
		if (tableIndex < 128) {
			table[tableIndex] = propIndex;
		}
	}
}

void CVPhysicsKeyParser::ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	int indent = 0;
	if (unknownKeyHandler != nullptr) {
		unknownKeyHandler->SetDefaults(pCustom);
	}

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (m_Text == nullptr) {
			return;
		}
		if (key[0] == '{') {
			++indent;
		} else if (value[0] == '{') {
			// They've got a named block here.
			// Increase our indent, and let them parse the key.
			++indent;
			if (unknownKeyHandler != nullptr) {
				unknownKeyHandler->ParseKeyValue(pCustom, key, value);
			}
		} else if (key[0] == '}') {
			--indent;
			if (indent < 0) {
				NextBlock();
				return;
			}
		} else if (unknownKeyHandler != nullptr) {
			unknownKeyHandler->ParseKeyValue(pCustom, key, value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	if (unknownKeyHandler != nullptr) {
		unknownKeyHandler->SetDefaults(pVehicle);
	} else {
		memset(pVehicle, 0, sizeof(*pVehicle));
	}

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			NextBlock();
			return;
		}
		if (value[0] == '{') {
			if (V_stricmp(key, "axle") == 0) {
				if (pVehicle->axleCount < ARRAYSIZE(pVehicle->axles)) {
					ParseVehicleAxle(pVehicle->axles[pVehicle->axleCount++]);
				} else {
					SkipBlock();
				}
			} else if (V_stricmp(key, "body") == 0) {
				ParseVehicleBody(pVehicle->body);
			} else if (V_stricmp(key, "engine") == 0) {
				ParseVehicleEngine(pVehicle->engine);
			} else if (V_stricmp(key, "steering") == 0) {
				ParseVehicleSteering(pVehicle->steering);
			} else {
				SkipBlock();
			}
		} else if (V_stricmp(key, "wheelsperaxle") == 0) {
			pVehicle->wheelsPerAxle = atoi(value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleAxle(vehicle_axleparams_t &axle) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (value[0] == '{') {
			if (V_stricmp(key, "wheel") == 0) {
				ParseVehicleWheel(axle.wheels);
			} else if (V_stricmp(key, "suspension") == 0) {
				ParseVehicleSuspension(axle.suspension);
			} else {
				SkipBlock();
			}
		} else if (V_stricmp(key, "offset") == 0) {
			ReadVector(value, axle.offset);
		} else if (V_stricmp(key, "wheeloffset") == 0) {
			ReadVector(value, axle.wheelOffset);
		} else if (V_stricmp(key, "torquefactor") == 0) {
			axle.torqueFactor = atof(value);
		} else if (V_stricmp(key, "brakefactor") == 0) {
			axle.brakeFactor = atof(value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleWheel(vehicle_wheelparams_t &wheel) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (V_stricmp(key, "radius") == 0) {
			wheel.radius = atof(value);
		} else if (V_stricmp(key, "mass") == 0) {
			wheel.mass = atof(value);
		} else if (V_stricmp(key, "inertia") == 0) {
			wheel.inertia = atof(value);
		} else if (V_stricmp(key, "damping") == 0) {
			wheel.damping = atof(value);
		} else if (V_stricmp(key, "rotdamping") == 0) {
			wheel.rotdamping = atof(value);
		} else if (V_stricmp(key, "frictionscale") == 0) {
			wheel.frictionScale = atof(value);
		} else if (V_stricmp(key, "material") == 0) {
			wheel.materialIndex = g_pPhysSurfaceProps->GetSurfaceIndex(value);
		} else if (V_stricmp(key, "skidmaterial") == 0) {
			wheel.skidMaterialIndex = g_pPhysSurfaceProps->GetSurfaceIndex(value);
		} else if (V_stricmp(key, "brakematerial") == 0) {
			wheel.brakeMaterialIndex = g_pPhysSurfaceProps->GetSurfaceIndex(value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleSuspension(vehicle_suspensionparams_t &suspension) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (V_stricmp(key, "springconstant") == 0) {
			suspension.springConstant = atof(value);
		} else if (V_stricmp(key, "springdamping") == 0) {
			suspension.springDamping = atof(value);
		} else if (V_stricmp(key, "stabilizerconstant") == 0) {
			suspension.stabilizerConstant = atof(value);
		} else if (V_stricmp(key, "springdampingcompression") == 0) {
			suspension.springDampingCompression = atof(value);
		} else if (V_stricmp(key, "maxbodyforce") == 0) {
			suspension.maxBodyForce = atof(value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleBody(vehicle_bodyparams_t &body) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (V_stricmp(key, "massCenterOverride") == 0) {
			ReadVector(value, body.massCenterOverride);
		} else if (V_stricmp(key, "addgravity") == 0) {
			body.addGravity = atof(value);
		} else if (V_stricmp(key, "maxAngularVelocity") == 0) {
			body.maxAngularVelocity = atof(value);
		} else if (V_stricmp(key, "massOverride") == 0) {
			body.massOverride = atof(value);
		} else if (V_stricmp(key, "tiltforce") == 0) {
			body.tiltForce = atof(value);
		} else if (V_stricmp(key, "tiltforceheight") == 0) {
			body.tiltForceHeight = atof(value);
		} else if (V_stricmp(key, "countertorquefactor") == 0) {
			body.counterTorqueFactor = atof(value);
		} else if (V_stricmp(key, "keepuprighttorque") == 0) {
			body.keepUprightTorque = atof(value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleEngine(vehicle_engineparams_t &engine) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (value[0] == '{') {
			if (V_stricmp(key, "boost") == 0) {
				ParseVehicleEngineBoost(engine);
			} else {
				SkipBlock();
			}
		} else if (V_stricmp(key, "gear") == 0) {
			if (engine.gearCount < ARRAYSIZE(engine.gearRatio)) {
				engine.gearRatio[engine.gearCount++] = atof(value);
			}
		} else if (V_stricmp(key, "horsepower") == 0) {
			engine.horsepower = atof(value);
		} else if (V_stricmp(key, "maxSpeed") == 0) {
			engine.maxSpeed = atof(value);
		} else if (V_stricmp(key, "maxReverseSpeed") == 0) {
			engine.maxRevSpeed = atof(value);
		} else if (V_stricmp(key, "axleratio") == 0) {
			engine.axleRatio = atof(value);
		} else if (V_stricmp(key, "maxRPM") == 0) {
			engine.maxRPM = atof(value);
		} else if (V_stricmp(key, "throttleTime") == 0) {
			engine.throttleTime = atof(value);
		} else if (V_stricmp(key, "AutoTransmission") == 0) {
			engine.isAutoTransmission = (atoi(value) != 0);
		} else if (V_stricmp(key, "shiftUpRPM") == 0) {
			engine.shiftUpRPM = atof(value);
		} else if (V_stricmp(key, "shiftDownRPM") == 0) {
			engine.shiftDownRPM = atof(value);
		} else if (V_stricmp(key, "autobrakeSpeedGain") == 0) {
			engine.autobrakeSpeedGain = atof(value);
		} else if (V_stricmp(key, "autobrakeSpeedFactor") == 0) {
			engine.autobrakeSpeedFactor = atof(value);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleEngineBoost(vehicle_engineparams_t &engine) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (V_stricmp(key, "force") == 0) {
			engine.boostForce = atof(value);
		} else if (V_stricmp(key, "duration") == 0) {
			engine.boostDuration = atof(value);
		} else if (V_stricmp(key, "delay") == 0) {
			engine.boostDelay = atof(value);
		} else if (V_stricmp(key, "maxspeed") == 0) {
			engine.boostMaxSpeed = atof(value);
		} else if (V_stricmp(key, "torqueboost") == 0) {
			engine.torqueBoost = (atoi(value) != 0);
		}
	}
}

void CVPhysicsKeyParser::ParseVehicleSteering(vehicle_steeringparams_t &steering) {
	char key[VPHYSICS_MAX_KEYVALUE], value[VPHYSICS_MAX_KEYVALUE];
	key[0] = '\0';

	while (m_Text != nullptr) {
		m_Text = ParseKeyvalue(m_Text, key, value);
		if (key[0] == '}') {
			return;
		}
		if (V_stricmp(key, "degreesSlow") == 0) {
			steering.degreesSlow = atof(value);
		} else if (V_stricmp(key, "degreesFast") == 0) {
			steering.degreesFast = atof(value);
		} else if (V_stricmp(key, "degreesBoost") == 0) {
			steering.degreesBoost = atof(value);
		} else if (V_stricmp(key, "fastcarspeed") == 0) {
			steering.speedFast = atof(value);
		} else if (V_stricmp(key, "slowcarspeed") == 0) {
			steering.speedSlow = atof(value);
		} else if (V_stricmp(key, "slowsteeringrate") == 0) {
			steering.steeringRateSlow = atof(value);
		} else if (V_stricmp(key, "faststeeringrate") == 0) {
			steering.steeringRateFast = atof(value);
		} else if (V_stricmp(key, "steeringRestRateSlow") == 0) {
			steering.steeringRestRateSlow = atof(value);
		} else if (V_stricmp(key, "steeringRestRateFast") == 0) {
			steering.steeringRestRateFast = atof(value);
		} else if (V_stricmp(key, "throttleSteeringRestRateFactor") == 0) {
			steering.throttleSteeringRestRateFactor = atof(value);
		} else if (V_stricmp(key, "boostSteeringRestRateFactor") == 0) {
			steering.boostSteeringRestRateFactor = atof(value);
		} else if (V_stricmp(key, "boostSteeringRateFactor") == 0) {
			steering.boostSteeringRateFactor = atof(value);
		} else if (V_stricmp(key, "steeringExponent") == 0) {
			steering.steeringExponent = atof(value);
		} else if (V_stricmp(key, "turnThrottleReduceSlow") == 0) {
			steering.turnThrottleReduceSlow = atof(value);
		} else if (V_stricmp(key, "turnThrottleReduceFast") == 0) {
			steering.turnThrottleReduceFast = atof(value);
		} else if (V_stricmp(key, "brakeSteeringRateFactor") == 0) {
			steering.brakeSteeringRateFactor = atof(value);
		} else if (V_stricmp(key, "powerSlideAccel") == 0) {
			steering.powerSlideAccel = atof(value);
		} else if (V_stricmp(key, "skidallowed") == 0) {
			steering.isSkidAllowed = (atoi(value) != 0);
		} else if (V_stricmp(key, "dustcloud") == 0) {
			steering.dustCloud = (atoi(value) != 0);
		}
	}
}
