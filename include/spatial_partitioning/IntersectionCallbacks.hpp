// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "Aabb.hpp"
#include "EntityTypes.hpp"
#include "RayInfo.hpp"

namespace spp
{
SPP_TEMPLATE_DECL
class BroadphaseBase;

SPP_TEMPLATE_DECL
class AabbCallback
{
public:
	AabbCallback() = default;
	~AabbCallback() = default;

	bool IsRelevant(AabbCentered aabb) const;
	bool IsRelevant(Aabb aabb) const;

	void ExecuteCallback(EntityType entity);

	bool ExecuteIfRelevant(AabbCentered aabb, EntityType entity);
	bool ExecuteIfRelevant(Aabb aabb, EntityType entity);

	void (*callback)(AabbCallback *, EntityType entity) = nullptr;

	Aabb aabb;
	MaskType mask;

	BroadphaseBase<SPP_TEMPLATE_ARGS> *broadphase = nullptr;

	size_t nodesTestedCount = 0;
	size_t testedCount = 0;
};

struct RayPartialResult {
	/*
	 * value ranges between 0.0 and 1.0.
	 * 0.0 - ray intersection is at start of ray
	 * 1.0 - ray intersection is at current end of ray
	 * 0.5 - ray intersection is at current mid-point of ray
	 *
	 * end point becomes:
	 *  end <- start + (end-start) * dist
	 */
	float dist;

	/*
	 * True if there is and intersection
	 */
	bool intersection = false;
};

SPP_TEMPLATE_DECL
class RayCallback : public RayInfo
{
public:
	RayCallback() = default;
	~RayCallback() = default;

	bool IsRelevant(AabbCentered aabb, float &near, float &far) const;
	bool IsRelevant(Aabb aabb, float &near, float &far) const;
	bool IsRelevant(AabbCentered aabb) const;
	bool IsRelevant(Aabb aabb) const;

	RayPartialResult ExecuteCallback(EntityType entity);

	RayPartialResult ExecuteIfRelevant(AabbCentered aabb, EntityType entity,
									   float &near, float &far);
	RayPartialResult ExecuteIfRelevant(Aabb aabb, EntityType entity,
									   float &near, float &far);
	RayPartialResult ExecuteIfRelevant(AabbCentered aabb, EntityType entity);
	RayPartialResult ExecuteIfRelevant(Aabb aabb, EntityType entity);

	RayPartialResult (*callback)(RayCallback *, EntityType entity) = nullptr;

	// end and dir can change during execution
	MaskType mask;
	float cutFactor;
	bool initedVars = false;

	void InitVariables();

	BroadphaseBase<SPP_TEMPLATE_ARGS> *broadphase = nullptr;

	size_t nodesTestedCount = 0;
	size_t testedCount = 0;
	size_t hitCount = 0;
};

SPP_TEMPLATE_DECL
class RayCallbackFirstHit : public RayCallback<SPP_TEMPLATE_ARGS>
{
public:
	RayCallbackFirstHit() = default;
	~RayCallbackFirstHit() = default;

	glm::vec3 hitNormal;
	glm::vec3 hitPoint;
	EntityType hitEntity;
	bool hasHit = false;
};

SPP_EXTERN_VARIANTS(AabbCallback)
SPP_EXTERN_VARIANTS(RayCallback)
SPP_EXTERN_VARIANTS(RayCallbackFirstHit)

} // namespace spp
