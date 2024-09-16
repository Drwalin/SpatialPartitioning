// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/LooseOctree.hpp"

namespace spp
{
LooseOctree::LooseOctree(spp::Aabb aabb, int32_t depth, float cellExtensionFactor) : aabb(aabb), depth(depth), cellExtensionFactor(cellExtensionFactor)
{
}
LooseOctree::~LooseOctree() {}

const char *LooseOctree::GetName() const { return "LooseOctree"; }

void LooseOctree::Clear() { entitiesData.clear(); }

size_t LooseOctree::GetMemoryUsage() const
{
	return entitiesOffsets.bucket_count() * sizeof(void *) +
		   entitiesOffsets.size() *
			   (sizeof(void *) * 2lu + sizeof(uint32_t) + sizeof(EntityType)) +
		   nodes.capacity() * sizeof(NodeData) +
		   entitiesData.capacity() * sizeof(Data) +
		   emptyNodesOffsets.capacity() * sizeof(int32_t);
}

void LooseOctree::ShrinkToFit() {}

void LooseOctree::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	entitiesData[entity] = Data{aabb, entity, mask};
	_Internal_Add(entity, aabb);
	
	
	
	
}

void LooseOctree::Update(EntityType entity, Aabb aabb)
{
	// TODO: optimise movement of objects
	_Internal_RemoveNodes(entity);
	entitiesData[entity].aabb = aabb;
	_Internal_Add(entity, aabb);
}

void LooseOctree::Remove(EntityType entity) {
	_Internal_RemoveNodes(entity);
	entitiesData.erase(entity);
	
	
	
	
}

void LooseOctree::SetMask(EntityType entity, MaskType mask)
{
	entitiesData[entity].mask = mask;
}

Aabb LooseOctree::GetAabb(EntityType entity) const
{
	auto it = entitiesData.find(entity);
	if (it != entitiesData.end()) {
		return it->second.aabb;
	}
	return {};
}

MaskType LooseOctree::GetMask(EntityType entity) const
{
	auto it = entitiesData.find(entity);
	if (it != entitiesData.end()) {
		return it->second.mask;
	}
	return 0;
}

void LooseOctree::Rebuild() {}

void LooseOctree::IntersectAabb(IntersectionCallback &cb)
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

void LooseOctree::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	cb.dir = cb.end - cb.start;
	cb.length = glm::length(cb.dir);
	cb.invLength = 1.0f / cb.length;
	cb.dirNormalized = cb.dir * cb.invLength;
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
						cb.invLength /= res.dist;
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
