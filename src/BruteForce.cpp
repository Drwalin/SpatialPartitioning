// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BruteForce.hpp"

namespace spp
{
BruteForce::BruteForce() {}
BruteForce::~BruteForce() {}

void BruteForce::Clear() { entitiesData.clear(); }

size_t BruteForce::GetMemoryUsage() const
{
	return entitiesData.bucket_count() * sizeof(void *) +
		   entitiesData.size() *
			   (sizeof(void *) * 2lu + sizeof(Data) + sizeof(EntityType));
}

void BruteForce::ShrinkToFit() {}

void BruteForce::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	entitiesData[entity] = Data{aabb, entity, mask};
}

void BruteForce::Update(EntityType entity, Aabb aabb)
{
	entitiesData[entity].aabb = aabb;
}

void BruteForce::Remove(EntityType entity) { entitiesData.erase(entity); }

void BruteForce::SetMask(EntityType entity, MaskType mask)
{
	entitiesData[entity].mask = mask;
}

void BruteForce::Rebuild() {}

void BruteForce::IntersectAabb(IntersectionCallback &callback)
{
	if (callback.callback == nullptr) {
		return;
	}

	for (const auto &it : entitiesData) {
		if (it.second.mask & callback.mask) {
			if (it.second.aabb && callback.aabb) {
				callback.callback(&callback, it.first);
				++callback.testedCount;
			}
			++callback.nodesTestedCount;
		}
	}
}

void BruteForce::IntersectRay(RayCallback &callback)
{
	if (callback.callback == nullptr) {
		return;
	}

	callback.dir = callback.end - callback.start;
	callback.length = glm::length(callback.dir);
	callback.dirNormalized = callback.dir / callback.length;
	callback.invDir = glm::vec3(1.f, 1.f, 1.f) / callback.dirNormalized;

	for (const auto &it : entitiesData) {
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
				++callback.nodesTestedCount;
			}
		}
	}
}
} // namespace spp
