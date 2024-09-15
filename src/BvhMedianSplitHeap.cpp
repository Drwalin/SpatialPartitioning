// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>
#include <algorithm>

#include "../include/spatial_partitioning/BvhMedianSplitHeap.hpp"

namespace spp
{
BvhMedianSplitHeap::BvhMedianSplitHeap() {}
BvhMedianSplitHeap::~BvhMedianSplitHeap() {}

void BvhMedianSplitHeap::SetAabbUpdatePolicy(AabbUpdatePolicy policy)
{
	updatePolicy = policy;
}

BvhMedianSplitHeap::AabbUpdatePolicy
BvhMedianSplitHeap::GetAabbUpdatePolicy() const
{
	return updatePolicy;
}

void BvhMedianSplitHeap::Clear()
{
	entitiesData.clear();
	nodesHeapAabb.clear();
	entitiesOffsets.clear();
	rebuildTree = false;
	entitiesCount = 0;
	entitiesPowerOfTwoCount = 0;
}

size_t BvhMedianSplitHeap::GetMemoryUsage() const
{
	return entitiesOffsets.bucket_count() * sizeof(void *) +
		   entitiesOffsets.size() *
			   (sizeof(void *) * 2lu + sizeof(uint32_t) + sizeof(EntityType)) +
		   nodesHeapAabb.capacity() * sizeof(Aabb) +
		   entitiesData.capacity() * sizeof(Data);
}

void BvhMedianSplitHeap::ShrinkToFit()
{
	nodesHeapAabb.shrink_to_fit();
	entitiesData.shrink_to_fit();
}

void BvhMedianSplitHeap::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	entitiesOffsets[entity] = entitiesData.size();
	entitiesData.push_back({aabb, entity, mask});
	rebuildTree = true;
	++entitiesCount;
}

void BvhMedianSplitHeap::Update(EntityType entity, Aabb aabb)
{
	uint32_t offset = entitiesOffsets[entity];
	entitiesData[offset].aabb = aabb;
	if (updatePolicy == ON_UPDATE_EXTEND_AABB) {
		UpdateAabb(offset);
	} else {
		rebuildTree = true;
	}
}

void BvhMedianSplitHeap::Remove(EntityType entity)
{
	--entitiesCount;

	if (entitiesCount == 0) {
		Clear();
		return;
	}

	uint32_t offset = entitiesOffsets[entity];
	entitiesOffsets.erase(entity);
	entitiesData[offset].entity = EMPTY_ENTITY;

	PruneEmptyEntitiesAtEnd();

	if (updatePolicy == ON_UPDATE_EXTEND_AABB) {
		UpdateAabb(offset);
	} else {
		if (entitiesCount != entitiesData.size()) {
			rebuildTree = true;
		}
	}
}

void BvhMedianSplitHeap::SetMask(EntityType entity, MaskType mask)
{
	uint32_t offset = entitiesOffsets[entity];

	entitiesData[offset].mask = mask;

	if (offset + 1 < entitiesData.size()) {
		if (entitiesData[offset + 1].entity != EMPTY_ENTITY) {
			mask |= entitiesData[offset + 1].mask;
		}
	}

	for (uint32_t n = (offset + entitiesPowerOfTwoCount) >> 1; n > 0; n >>= 1) {
		nodesHeapAabb[n].mask = mask;
		if (n + 1 < nodesHeapAabb.size()) {
			mask |= nodesHeapAabb[n + 1].mask;
		}
	}
}

void BvhMedianSplitHeap::IntersectAabb(IntersectionCallback &callback)
{
	if (callback.callback == nullptr) {
		return;
	}

	if (rebuildTree) {
		Rebuild();
	}

	_Internal_IntersectAabb(callback, 1);
}

void BvhMedianSplitHeap::_Internal_IntersectAabb(IntersectionCallback &cb,
												 const int32_t nodeId)
{
	int32_t n = nodeId << 1;
	if (n >= entitiesPowerOfTwoCount) {
		int32_t o = n - entitiesPowerOfTwoCount;
		for (int i = o; i <= o + 1 && i < entitiesData.size(); ++i) {
			if (entitiesData[i].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (entitiesData[i].aabb && cb.aabb) {
					++cb.testedCount;
					cb.callback(&cb, entitiesData[i].entity);
				}
			}
		}
	} else {
		for (int i = n; i <= n + 1; ++i) {
			if (nodesHeapAabb[nodeId].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (nodesHeapAabb[nodeId].aabb && cb.aabb) {
					_Internal_IntersectAabb(cb, i);
				} else {
				}
			}
		}
	}
}

void BvhMedianSplitHeap::IntersectRay(RayCallback &callback)
{
	if (callback.callback == nullptr) {
		return;
	}

	if (rebuildTree) {
		Rebuild();
	}

	callback.dir = callback.end - callback.start;
	callback.length = glm::length(callback.dir);
	callback.dirNormalized = callback.dir / callback.length;
	callback.invDir = glm::vec3(1.f, 1.f, 1.f) / callback.dirNormalized;

	int32_t n = 2;

	while (n < entitiesPowerOfTwoCount &&
		   n < ((entitiesPowerOfTwoCount + entitiesData.size()) >> 1)) {
		if (n >= nodesHeapAabb.size()) {
			break;
		}

		float _n, _f;
		// TODO choose closest to ray origin first instead of by index
		if ((nodesHeapAabb[n].mask & callback.mask) &&
			(nodesHeapAabb[n].aabb.FastRayTest(
				callback.start, callback.dirNormalized, callback.invDir,
				callback.length, _n, _f))) {
			++callback.nodesTestedCount;
			n <<= 1;
		} else {
			if (n & 1) {
				n >>= 1;
				while (n & 1) {
					n >>= 1;
				}
				n |= 1;
			} else {
				++n;
			}
		}

		if (n >= entitiesPowerOfTwoCount) {
			int32_t o = n - entitiesPowerOfTwoCount;
			for (int i = 0; i < 2 && o + i < entitiesData.size(); ++i, ++o) {
				float _n, _f;
				if ((entitiesData[o].mask & callback.mask) &&
					(entitiesData[o].aabb.FastRayTest(
						callback.start, callback.dirNormalized, callback.invDir,
						callback.length, _n, _f))) {
					auto res =
						callback.callback(&callback, entitiesData[o].entity);
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
			++n;
			while (n & 1) {
				n >>= 1;
			}
			n |= 1;
		}
	}
}

void BvhMedianSplitHeap::_Internal_IntersectRay(RayCallback &cb,
												const int32_t nodeId)
{
}

void BvhMedianSplitHeap::Rebuild()
{
	rebuildTree = false;
	{
		uint32_t v = entitiesCount;
		v--;
		v |= v >> 1;
		v |= v >> 2;
		v |= v >> 4;
		v |= v >> 8;
		v |= v >> 16;
		v++;
		entitiesPowerOfTwoCount = v;
	}

	nodesHeapAabb.resize(entitiesPowerOfTwoCount);
	for (auto &n : nodesHeapAabb) {
		n.mask = 0;
	}

	entitiesOffsets.reserve(((entitiesCount + 7) * 3) / 2);

	{ // prune empty entities data
		PruneEmptyEntitiesAtEnd();
		for (int32_t i = 0; i + 1 < entitiesData.size(); ++i) {
			if (entitiesData[i].entity == EMPTY_ENTITY) {
				entitiesData[i] = entitiesData.back();
				entitiesData.back().entity = EMPTY_ENTITY;
				PruneEmptyEntitiesAtEnd();
			}
		}
	}

	RebuildNode(1);

	{ // set entitiesOffsets
		for (int32_t i = 0; i < entitiesData.size(); ++i) {
			entitiesOffsets[entitiesData[i].entity] = i;
		}
	}
}

void BvhMedianSplitHeap::RebuildNode(int32_t nodeId)
{
	int32_t offset = nodeId;
	int32_t count = 1;
	while (offset < entitiesPowerOfTwoCount) {
		offset <<= 1;
		count <<= 1;
	}
	offset -= entitiesPowerOfTwoCount;
	if (offset >= entitiesData.size()) {
		return;
	}
	count = std::min<int32_t>(count, entitiesData.size() - offset);

	if (count == 0) {
		return;
	}

	Aabb totalAabb = entitiesData[offset].aabb;
	MaskType mask = entitiesData[offset].mask;
	for (int32_t i = offset + 1; i < offset + count; ++i) {
		totalAabb = totalAabb + entitiesData[i].aabb;
		mask |= entitiesData[i].mask;
	}
	nodesHeapAabb[nodeId] = {totalAabb, mask};

	if (count <= 2) {
		return;
	}

	int axis = 0;
	glm::vec3 ext = totalAabb.GetSizes();
	for (int i = 1; i < 3; ++i) {
		if (ext[axis] < ext[i]) {
			axis = i;
		}
	}

	struct SortFunctions {
		inline static bool SortX(const Data &l, const Data &r)
		{
			return l.aabb.min.x < r.aabb.min.x;
		}
		inline static bool SortY(const Data &l, const Data &r)
		{
			return l.aabb.min.y < r.aabb.min.y;
		}
		inline static bool SortZ(const Data &l, const Data &r)
		{
			return l.aabb.min.z < r.aabb.min.z;
		}
	};
	using CompareTypeFunc = bool (*)(const Data &, const Data &);
	const static CompareTypeFunc sortFuncs[] = {
		SortFunctions::SortX, SortFunctions::SortY, SortFunctions::SortZ};

	std::sort(entitiesData.data() + offset,
			  entitiesData.data() + offset + count, sortFuncs[axis]);

	nodeId <<= 1;
	RebuildNode(nodeId);
	RebuildNode(nodeId + 1);
}

void BvhMedianSplitHeap::PruneEmptyEntitiesAtEnd()
{
	for (int32_t i = entitiesData.size() - 1; i >= 0; --i) {
		if (entitiesData[i].entity != EMPTY_ENTITY) {
			entitiesData.resize(i + 1);
			return;
		}
	}
}

void BvhMedianSplitHeap::UpdateAabb(int32_t offset)
{
	MaskType mask = 0;
	Aabb aabb = {{0, 0, 0}, {-1, -1, -1}};
	offset = offset & (~(int32_t)1);
	for (int32_t i = offset; i <= offset + 1 && i < entitiesData.size(); ++i) {
		if (entitiesData[i].entity != EMPTY_ENTITY && entitiesData[i].mask) {
			mask |= entitiesData[i].mask;
			if (mask) {
				aabb = aabb + entitiesData[i].aabb;
			} else {
				aabb = entitiesData[i].aabb;
			}
		}
	}

	for (uint32_t n = (offset + entitiesPowerOfTwoCount) >> 1; n > 0; n >>= 1) {
		nodesHeapAabb[n].aabb = aabb;
		nodesHeapAabb[n].mask = mask;
		n ^= 1;
		if (n < nodesHeapAabb.size() && n > 0) {
			if (nodesHeapAabb[n].mask) {
				if (mask) {
					aabb = aabb + nodesHeapAabb[n].aabb;
				} else {
					aabb = nodesHeapAabb[n].aabb;
				}
				mask |= nodesHeapAabb[n].mask;
			}
		}
	}
}
} // namespace spp
