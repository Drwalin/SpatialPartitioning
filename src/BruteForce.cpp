// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BruteForce.hpp"

namespace spp
{
BruteForce::BruteForce() {}
BruteForce::~BruteForce() {}

const char *BruteForce::GetName() const { return "BruteForce"; }

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

Aabb BruteForce::GetAabb(EntityType entity) const
{
	auto it = entitiesData.find(entity);
	if (it != entitiesData.end()) {
		return it->second.aabb;
	}
	return {};
}

MaskType BruteForce::GetMask(EntityType entity) const
{
	auto it = entitiesData.find(entity);
	if (it != entitiesData.end()) {
		return it->second.mask;
	}
	return 0;
}

void BruteForce::Rebuild() {}

void BruteForce::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;

	for (const auto &it : entitiesData) {
		if (it.second.mask & cb.mask) {
			if (it.second.aabb && cb.aabb) {
				cb.callback(&cb, it.first);
				++cb.testedCount;
			}
			++cb.nodesTestedCount;
		}
	}
}

void BruteForce::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	cb.dir = cb.end - cb.start;
	cb.length = glm::length(cb.dir);
	cb.dirNormalized = glm::normalize(cb.dir);
	cb.invDir = glm::vec3(1.f, 1.f, 1.f) / cb.dirNormalized;

	for (const auto &it : entitiesData) {
		if (it.second.mask & cb.mask) {
			float n, f;
			if (it.second.aabb.FastRayTest(cb.start, cb.dirNormalized,
										   cb.invDir, cb.length, n, f)) {
				auto res = cb.callback(&cb, it.first);
				if (res.intersection) {
					if (res.dist + 0.00000001f < 1.0f) {
						if (res.dist < 0.0f)
							res.dist = 0.0f;
						else
							res.dist += 0.00000001f;
						cb.length *= res.dist;
						cb.dir *= res.dist;
						cb.end = cb.start + cb.dir;
					}
					++cb.hitCount;
				}
				++cb.testedCount;
			}
			++cb.nodesTestedCount;
		}
	}
}
} // namespace spp
