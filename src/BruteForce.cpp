// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BruteForce.hpp"

namespace spp
{
void BruteForce::Clear() { entities.clear(); }

size_t BruteForce::GetMemoryUsage() const
{
	return entities.bucket_count() * sizeof(void *) +
		   entities.size() *
			   (sizeof(void *) * 2lu + sizeof(Data) + sizeof(EntityType));
}

void BruteForce::Add(EntityType entity, AABB aabb, MaskType mask)
{
	entities[entity] = Data{entity, aabb, mask};
}

void BruteForce::Update(EntityType entity, AABB aabb)
{
	entities[entity].aabb = aabb;
}

void BruteForce::Remove(EntityType entity) { entities.erase(entity); }

void BruteForce::SetMask(EntityType entity, MaskType mask)
{
	entities[entity].mask = mask;
}

void BruteForce::IntersectAABB(IntersectionCallback &callback) const
{
	if (callback.callback == nullptr) {
		return;
	}
	
	for (const auto &it : entities) {
		if (it.second.mask & callback.mask) {
			if (it.second.aabb && callback.aabb) {
				callback.callback(&callback, it.first);
				++callback.testedCount;
			}
		}
	}
}

void BruteForce::IntersectRay(RayCallback &callback) const
{
	if (callback.callback == nullptr) {
		return;
	}

	callback.dir = callback.end - callback.start;
	callback.length = glm::length(callback.dir);
	callback.dirNormalized = callback.dir / callback.length;
	callback.invDir = glm::vec3(1.f, 1.f, 1.f) / callback.dirNormalized;

	for (const auto &it : entities) {
		if (it.second.mask & callback.mask) {
			float n, f;
			if (it.second.aabb.FastRayTest(
					callback.start, callback.dirNormalized, callback.invDir,
					callback.length, n, f)) {
				auto res = callback.callback(&callback, it.first);
				if (res.intersection) {
					if (callback.length + 0.00000001f < 1.0f) {
						callback.length *= res.dist;
						callback.dir *= res.dist;
						callback.end = callback.start + callback.dir;
					}
					++callback.hitCount;
				}
				++callback.testedCount;
			}
		}
	}
}
} // namespace spp
