// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../thirdparty/glm/glm/ext/vector_int3.hpp"

#include "../include/spatial_partitioning/LooseOctree.hpp"

namespace spp
{
LooseOctree::LooseOctree(glm::vec3 centerOffset, glm::vec3 sizeScale,
		int32_t depth, float cellExtensionFactor)
	: centerOffset(centerOffset), sizeScale(sizeScale), depth(depth), cellExtensionFactor(cellExtensionFactor),
	halfTotalSize(1<<depth)
{
	Clear();
}
LooseOctree::~LooseOctree() {}

const char *LooseOctree::GetName() const { return "LooseOctree"; }

void LooseOctree::Clear()
{
	data.Clear();
	nodes.Clear();
	rootNode = nodes.Add({});
}

size_t LooseOctree::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.GetMemoryUsage();
}

void LooseOctree::ShrinkToFit() {
	data.ShrinkToFit();
	nodes.ShrinkToFit();
}

void LooseOctree::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	int32_t offset = data.Add(entity, {aabb, entity, mask});
	
	// TODO: add to octree
	
	
	
	
	
}

void LooseOctree::Update(EntityType entity, Aabb aabb)
{
	// TODO: update
}

void LooseOctree::Remove(EntityType entity) {
	// TODO: remove
	
	
	
}

void LooseOctree::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = data.GetOffset(entity);
	data[offset].mask = mask;
	
	// TODO: update masks of tree
}

Aabb LooseOctree::GetAabb(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	return data[offset].aabb;
}

MaskType LooseOctree::GetMask(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	return data[offset].mask;
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

glm::vec3 LooseOctree::GetSizeOnDepth(int32_t depth) const
{
	const int32_t v = 1 << (this->depth - depth);
	return glm::vec3(v, v, v) * cellExtensionFactor;
}

int32_t LooseOctree::GetChildIdFromCenter(glm::vec3 p) const
{
	int32_t id = 0;
	for (int i=0; i<3; ++i) {
		id += p[i] > 0 ? 1<<i : 0;
	}
	return id;
}

glm::vec3 LooseOctree::GetPosOffsetOfNodeAtDepth(int32_t depth,
												 int32_t childId) const
{
	const int32_t b = 1 << (this->depth - depth);
	const glm::vec3 hs = glm::vec3(b, b, b) * 0.5f;
	const glm::vec3 extensinoOffset = hs * (cellExtensionFactor - 1.0f);

	const glm::vec3 bs = {childId & 1 ? 1.0f : 0.0f, childId & 2 ? 1.0f : 0.0f,
						  childId & 4 ? 1.0f : 0.0f};

	return extensinoOffset + bs * hs;
}

LooseOctree::NodePath LooseOctree::CalculatePathTo(Aabb aabb)
{
	aabb.min -= centerOffset;
	const glm::vec3 size = aabb.GetSizes() * sizeScale;
	aabb.max = aabb.min + size;
	const glm::ivec3 icenter = aabb.GetCenter();
	const float maxSizeValue = glm::max(glm::max(size.x, size.y), size.z);
	
	NodePath path;
	
	const int32_t maxDepth = this->depth - std::ilogb(maxSizeValue);
	path.depth = 0;
	for (int32_t &i=path.depth; i<depth; ++i) {
// 		int32_t g = 
		
		
	}
	
	
	
	
	
	
	
	
	
	
	
}
} // namespace spp
