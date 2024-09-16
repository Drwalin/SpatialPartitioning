// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "Aabb.hpp"
#include "EntityTypes.hpp"

namespace spp
{
struct IntersectionCallback {
	IntersectionCallback() = default;
	virtual ~IntersectionCallback() = default;

	void (*callback)(IntersectionCallback *, EntityType entity) = nullptr;

	Aabb aabb;
	MaskType mask;
	
	class BroadphaseBase *broadphase = nullptr;
	
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

struct RayCallback {
	RayCallback() = default;
	~RayCallback() = default;

	RayPartialResult (*callback)(RayCallback *, EntityType entity) = nullptr;

	// end and dir can change during execution
	glm::vec3 start;
	glm::vec3 end;
	MaskType mask;
	
	glm::vec3 dir;
	glm::vec3 dirNormalized;
	glm::vec3 invDir;
	float length;
	float invLength;
	
	class BroadphaseBase *broadphase = nullptr;
	
	size_t nodesTestedCount = 0;
	size_t testedCount = 0;
	size_t hitCount = 0;
};

struct RayCallbackFirstHit : public RayCallback {
	RayCallbackFirstHit() = default;
	~RayCallbackFirstHit() = default;

	glm::vec3 hitNormal;
	glm::vec3 hitPoint;
	EntityType hitEntity;
	float hitDistance;
	bool hasHit = false;
};
}
