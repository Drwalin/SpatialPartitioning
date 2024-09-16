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
		offset = (offset | 1) ^ 1;
		if (offset < entitiesData.size()) {
			UpdateAabb(offset);
		}
	} else {
		if (entitiesCount != entitiesData.size()) {
			rebuildTree = true;
		} else {
			offset = (offset | 1) ^ 1;
			if (offset < entitiesData.size()) {
				UpdateAabb(offset);
			}
		}
	}
}

void BvhMedianSplitHeap::SetMask(EntityType entity, MaskType mask)
{
	uint32_t offset = entitiesOffsets[entity];
	entitiesData[offset].mask = mask;
	if ((offset ^ 1) < entitiesData.size()) {
		if (entitiesData[offset ^ 1].entity != EMPTY_ENTITY) {
			mask |= entitiesData[offset ^ 1].mask;
		}
	}

	for (uint32_t n = (offset + entitiesPowerOfTwoCount) >> 1; n > 0; n >>= 1) {
		nodesHeapAabb[n].mask = mask;
		if ((n ^ 1) < nodesHeapAabb.size()) {
			mask |= nodesHeapAabb[n ^ 1].mask;
		}
	}
}

Aabb BvhMedianSplitHeap::GetAabb(EntityType entity) const
{
	auto it = entitiesOffsets.find(entity);
	if (it != entitiesOffsets.end()) {
		return entitiesData[it->second].aabb;
	}
	return {};
}

MaskType BvhMedianSplitHeap::GetMask(EntityType entity) const
{
	auto it = entitiesOffsets.find(entity);
	if (it != entitiesOffsets.end()) {
		return entitiesData[it->second].mask;
	}
	return 0;
}

void BvhMedianSplitHeap::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	if (rebuildTree) {
		Rebuild();
	}

	cb.broadphase = this;

	_Internal_IntersectAabb(cb, 1);
}

void BvhMedianSplitHeap::_Internal_IntersectAabb(IntersectionCallback &cb,
												 const int32_t nodeId)
{
	const int32_t n = nodeId << 1;
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
		for (int i = 0; i <= 1; ++i) {
			if (nodesHeapAabb[n + i].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (nodesHeapAabb[n + i].aabb && cb.aabb) {
					_Internal_IntersectAabb(cb, n + i);
				}
			}
		}
	}
}

void BvhMedianSplitHeap::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	if (rebuildTree) {
		Rebuild();
	}

	cb.broadphase = this;
	cb.dir = cb.end - cb.start;
	cb.length = glm::length(cb.dir);
	cb.invLength = 1.0f * cb.length;
	cb.dirNormalized = cb.dir * cb.invLength;
	cb.invDir = glm::vec3(1.f, 1.f, 1.f) / cb.dirNormalized;

	if (false) {
		for (const auto &it : entitiesData) {
			if (it.mask & cb.mask) {
				float n, f;
				if (it.aabb.FastRayTest(cb.start, cb.dirNormalized, cb.invDir,
										cb.length, n, f)) {
					auto res = cb.callback(&cb, it.entity);
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
					++cb.nodesTestedCount;
				}
			}
		}
		return;
	}

	_Internal_IntersectRay(cb, 1);
}

void BvhMedianSplitHeap::_Internal_IntersectRay(RayCallback &cb,
												const int32_t nodeId)
{
	const int32_t n = nodeId << 1;
	if (n >= entitiesPowerOfTwoCount) {
		int32_t o = n - entitiesPowerOfTwoCount;
		for (int i = 0; i <= 1 && o + i < entitiesData.size(); ++i) {
			if (entitiesData[o + i].mask & cb.mask) {
				++cb.nodesTestedCount;
				float _n, _f;
				if (entitiesData[o + i].aabb.FastRayTest(
						cb.start, cb.dirNormalized, cb.invDir, cb.length, _n,
						_f)) {
					++cb.testedCount;
					auto res = cb.callback(&cb, entitiesData[o + i].entity);
					if (res.intersection) {
						if (res.dist + 0.00000001f < 1.0f) {
							if (res.dist < 0.0f)
								res.dist = 0.0f;
							cb.length *= res.dist;
							cb.invLength /= res.dist;
							cb.dir *= res.dist;
							cb.end = cb.start + cb.dir;
						}
						++cb.hitCount;
					}
				}
			}
		}
	} else {
		float __n[2], __f[2];
		int __has = 0;
		for (int i = 0; i <= 1; ++i) {
			if (nodesHeapAabb[n + i].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (nodesHeapAabb[n + i].aabb.FastRayTest(
						cb.start, cb.dirNormalized, cb.invDir, cb.length,
						__n[i], __f[i])) {
					if (__n[i] < 0.0f)
						__n[i] = 0.0f;
					__has += i + 1;
					_Internal_IntersectRay(cb, n + i);
					;
				}
			}
			return;
			switch (__has) {
			case 1:
				_Internal_IntersectRay(cb, n);
				break;
			case 2:
				_Internal_IntersectRay(cb, n + 1);
				break;
			case 3:
				if (__n[1] < __n[0]) {
					_Internal_IntersectRay(cb, n + 1);
					if (__n[0] < cb.length) {
						_Internal_IntersectRay(cb, n);
					}
				} else {
					_Internal_IntersectRay(cb, n);
					if (__n[1] < cb.length) {
						_Internal_IntersectRay(cb, n + 1);
					}
				}
				break;
			}
		}
	}
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
				std::swap(entitiesData[i], entitiesData.back());
				PruneEmptyEntitiesAtEnd();
			}
		}
	}

	RebuildNode(1);

	// set entitiesOffsets
	for (int32_t i = 0; i < entitiesData.size(); ++i) {
		entitiesOffsets[entitiesData[i].entity] = i;
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
	int32_t orgCount = count;
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

	int32_t mid = orgCount >> 1;
	if (mid < count) {
		auto beg = entitiesData.data() + offset;
		std::nth_element(beg, beg + mid, beg + count, sortFuncs[axis]);
	}

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
	for (int i = 0; i <= 1; ++i, offset ^= 1) {
		if (offset < entitiesData.size()) {
			if (entitiesData[offset].entity != EMPTY_ENTITY &&
				entitiesData[offset].mask) {
				if (mask) {
					aabb = aabb + entitiesData[offset].aabb;
				} else {
					aabb = entitiesData[offset].aabb;
				}
				mask |= entitiesData[offset].mask;
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
