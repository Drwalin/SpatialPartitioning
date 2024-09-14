// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include "AABB.hpp"

namespace spp
{
using EntityType = uint64_t;
using MaskType = uint32_t;
#define EMPTY_ENTITY ((EntityType)0)

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
	bool intersection;
};

struct RayCallback {
	RayCallback() = default;
	~RayCallback() = default;

	RayPartialResult (*callback)(RayCallback *, EntityType entity) = nullptr;

	// end and dir can change during execution
	glm::vec3 start;
	glm::vec3 end;
	
	glm::vec3 dir;
	glm::vec3 dirNormalized;
	glm::vec3 invDir;
	float length;

	MaskType mask;
	
	size_t testedCount = 0;
	size_t hitCount = 0;
};

struct RayCallbackFirstHit : public RayCallback {
	RayCallbackFirstHit() = default;
	~RayCallbackFirstHit() = default;

	glm::vec3 normal;
	glm::vec3 hitPoint;
	EntityType entity;
	bool hasHit;
};

struct IntersectionCallback {
	IntersectionCallback() = default;
	virtual ~IntersectionCallback() = default;

	void (*callback)(IntersectionCallback *, EntityType entity) = nullptr;

	AABB aabb;
	MaskType mask;
	
	size_t testedCount = 0;
};

class BroadphaseBase
{
public:
	BroadphaseBase() = default;
	virtual ~BroadphaseBase() = default;

public:
	virtual void Clear() = 0;
	virtual size_t GetMemoryUsage() const = 0;
	
	virtual void Add(EntityType entity, AABB aabb,
					 MaskType mask) = 0;
	virtual void Update(EntityType entity, AABB aabb) = 0;
	virtual void Remove(EntityType entity) = 0;
	virtual void SetMask(EntityType entity, MaskType mask) = 0;

	// returns number of tested entities
	virtual void IntersectAABB(IntersectionCallback &callback) const = 0;
	virtual void IntersectRay(RayCallback &callback) const = 0;
};
} // namespace spp
