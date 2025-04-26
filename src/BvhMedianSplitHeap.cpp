// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>
#include <bit>
#include <algorithm>

#include "../include/spatial_partitioning/BvhMedianSplitHeap.hpp"

namespace spp
{
BvhMedianSplitHeap::BvhMedianSplitHeap(EntityType denseEntityRange)
	: entitiesOffsets(denseEntityRange), iterator(*this)
{
}
BvhMedianSplitHeap::~BvhMedianSplitHeap() {}

const char *BvhMedianSplitHeap::GetName() const { return "BvhMedianSplitHeap"; }

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
	entitiesOffsets.Clear();
	rebuildTree = false;
	entitiesCount = 0;
	entitiesPowerOfTwoCount = 0;
}

size_t BvhMedianSplitHeap::GetMemoryUsage() const
{
	return entitiesOffsets.GetMemoryUsage() +
		   nodesHeapAabb.capacity() * sizeof(NodeData) +
		   entitiesData.capacity() * sizeof(Data);
}

void BvhMedianSplitHeap::ShrinkToFit()
{
	nodesHeapAabb.shrink_to_fit();
	entitiesData.shrink_to_fit();
}

void BvhMedianSplitHeap::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	if (entitiesOffsets.find(entity) != nullptr) {
		assert(!"Entity already exists");
		return;
	}
	entitiesOffsets.Set(entity, entitiesData.size());
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
	auto it = entitiesOffsets.find(entity);
	if (it == nullptr) {
		return;
	}

	assert(*it != -1);

	uint32_t offset = *it;
	entitiesOffsets.Remove(entity);
	entitiesData[offset].entity = EMPTY_ENTITY;
	entitiesData[offset].mask = 0;

	--entitiesCount;

	if (entitiesCount == 0) {
		Clear();
		return;
	}

	PruneEmptyEntitiesAtEnd();

	UpdateAabb(offset);
}

void BvhMedianSplitHeap::SetMask(EntityType entity, MaskType mask)
{
	auto it = entitiesOffsets.find(entity);
	if (it == nullptr) {
		return;
	}

	uint32_t offset = *it;

	if (entitiesData[offset].mask == mask) {
		return;
	}

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

int32_t BvhMedianSplitHeap::GetCount() const { return entitiesCount; }

bool BvhMedianSplitHeap::Exists(EntityType entity) const
{
	return entitiesOffsets.Has(entity);
}

Aabb BvhMedianSplitHeap::GetAabb(EntityType entity) const
{
	auto it = entitiesOffsets.find(entity);
	if (it != nullptr) {
		return entitiesData[*it].aabb;
	}
	return {};
}

MaskType BvhMedianSplitHeap::GetMask(EntityType entity) const
{
	auto it = entitiesOffsets.find(entity);
	if (it != nullptr) {
		return entitiesData[*it].mask;
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
		for (int i = 0; i <= 1 && o < entitiesData.size(); ++i, ++o) {
			if (entitiesData[o].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (entitiesData[o].aabb && cb.aabb) {
					++cb.testedCount;
					cb.callback(&cb, entitiesData[o].entity);
				}
			}
		}
	} else {
		for (int i = 0; i <= 1 && n + i < nodesHeapAabb.size(); ++i) {
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
	cb.InitVariables();

	_Internal_IntersectRay(cb, 1);
}

void BvhMedianSplitHeap::_Internal_IntersectRay(RayCallback &cb,
												const int32_t nodeId)
{
	const int32_t n = nodeId << 1;
	if (n >= entitiesPowerOfTwoCount) {
		int32_t o = n - entitiesPowerOfTwoCount;
		for (int i = 0; i <= 1 && o < entitiesData.size(); ++i, ++o) {
			if ((entitiesData[o].mask & cb.mask) &&
				entitiesData[o].entity != EMPTY_ENTITY) {
				auto &ed = entitiesData[o];
				cb.ExecuteIfRelevant(ed.aabb, ed.entity);
			}
		}
	} else {
		float __n[2], __f[2];
		int __has = 0;
		for (int i = 0; i <= 1 && n + i < nodesHeapAabb.size(); ++i) {
			if (nodesHeapAabb[n + i].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (cb.IsRelevant(nodesHeapAabb[n + i].aabb, __n[i], __f[i])) {
					__has += i + 1;
				}
			}
		}
		switch (__has) {
		case 0:
			break;
		case 1:
			_Internal_IntersectRay(cb, n);
			break;
		case 2:
			_Internal_IntersectRay(cb, n + 1);
			break;
		case 3:
			if (__n[1] < __n[0]) {
				_Internal_IntersectRay(cb, n + 1);
				if (__n[0] <= cb.cutFactor) {
					_Internal_IntersectRay(cb, n);
				}
			} else {
				_Internal_IntersectRay(cb, n);
				if (__n[1] <= cb.cutFactor) {
					_Internal_IntersectRay(cb, n + 1);
				}
			}
			break;
		}
	}
}

void BvhMedianSplitHeap::Rebuild()
{
	rebuildTree = false;
	entitiesPowerOfTwoCount = std::bit_ceil((uint32_t)entitiesCount);

	nodesHeapAabb.resize(entitiesPowerOfTwoCount / 2 + (entitiesCount + 1) / 2);
	for (auto &n : nodesHeapAabb) {
		n.mask = 0;
	}

	entitiesOffsets.Reserve(entitiesCount);

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
}

void BvhMedianSplitHeap::RebuildNode(int32_t nodeId)
{
	int32_t tcount = 0;
	nodeId = RebuildNodePartial(nodeId, &tcount);
	if (nodeId > 1 && nodeId < nodesHeapAabb.size()) {
		RebuildNode(nodeId);
		RebuildNode(nodeId + 1);
	}
}

int32_t BvhMedianSplitHeap::RebuildNodePartial(int32_t nodeId, int32_t *tcount)
{
	*tcount = 0;
	int32_t offset = nodeId;
	int32_t count = 1;
	while (offset < entitiesPowerOfTwoCount) {
		offset <<= 1;
		count <<= 1;
	}
	offset -= entitiesPowerOfTwoCount;
	if (offset >= entitiesData.size()) {
		return -1;
	}
	int32_t orgCount = count;
	count = std::min<int32_t>(count, ((int32_t)entitiesData.size()) - offset);

	if (count < 0) {
		return -1;
	}

	*tcount = count;

	Aabb totalAabb = entitiesData[offset].aabb;
	MaskType mask = entitiesData[offset].mask;
	for (int32_t i = offset; i < offset + count; ++i) {
		totalAabb = totalAabb + entitiesData[i].aabb;
		mask |= entitiesData[i].mask;
	}
	nodesHeapAabb[nodeId] = {totalAabb.Expanded(BIG_EPSILON), mask};

	if (count <= 2) {
		for (int32_t i = offset; i < offset + count; ++i) {
			entitiesOffsets.Set(entitiesData[i].entity, i);
		}
		return -1;
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
			return l.aabb.GetCenter().x < r.aabb.GetCenter().x;
		}
		inline static bool SortY(const Data &l, const Data &r)
		{
			return l.aabb.GetCenter().y < r.aabb.GetCenter().y;
		}
		inline static bool SortZ(const Data &l, const Data &r)
		{
			return l.aabb.GetCenter().z < r.aabb.GetCenter().z;
		}
	};
	using CompareTypeFunc = bool (*)(const Data &, const Data &);
	const static CompareTypeFunc sortFuncs[] = {
		SortFunctions::SortX, SortFunctions::SortY, SortFunctions::SortZ};

	int32_t mid = (orgCount >> 1);
	if (mid < count) {
		auto beg = entitiesData.data() + offset;
		std::nth_element(beg, beg + mid, beg + count, sortFuncs[axis]);
	}

	return nodeId << 1;
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
	Aabb aabb = {{0, 0, 0}, {0, 0, 0}};
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

	for (uint32_t n = (offset + entitiesPowerOfTwoCount) >> 1; n > 1; n >>= 1) {
		nodesHeapAabb[n].aabb = aabb.Expanded(BIG_EPSILON);
		nodesHeapAabb[n].mask = mask;
		n ^= 1;
		if (n < nodesHeapAabb.size()) {
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
	if (1 < nodesHeapAabb.size()) {
		nodesHeapAabb[1].aabb = aabb.Expanded(BIG_EPSILON);
		nodesHeapAabb[1].mask = mask;
	}
}

bool BvhMedianSplitHeap::RebuildStep(RebuildProgress &progress)
{
	if (progress.done) {
		return true;
	}

	switch (progress.stage) {
	case 0:
		rebuildTree = false;
		entitiesPowerOfTwoCount = std::bit_ceil((uint32_t)entitiesCount);
		nodesHeapAabb.resize(entitiesPowerOfTwoCount);
		progress.stage = 1;
		progress.it = 0;
		break;

	case 1:
		for (int32_t i = 0; i < 1024 && progress.it < nodesHeapAabb.size();
			 ++i, ++progress.it) {
			nodesHeapAabb[progress.it].mask = 0;
		}
		if (progress.it >= nodesHeapAabb.size()) {
			progress.stage = 2;
		}
		break;

	case 2:
		entitiesOffsets.Reserve(entitiesCount);
		progress.stage = 3;
		break;

	case 3:
		PruneEmptyEntitiesAtEnd();
		progress.it = 0;
		progress.stage = 4;
		break;

	case 4:
		for (int32_t i = 0; i < 4096 && progress.it < entitiesData.size();
			 ++i, ++progress.it) {
			if (entitiesData[i].entity == EMPTY_ENTITY) {
				std::swap(entitiesData[i], entitiesData.back());
				PruneEmptyEntitiesAtEnd();
			}
		}

		if (progress.it >= entitiesData.size()) {
			progress.size = 1;
			progress.stack[0] = 1;
			progress.stage = 5;
		}
		break;

	case 5: {
		int32_t sum = 0;
		while (sum < 300 && progress.size > 0) {
			++sum;
			int32_t tcount = 0;
			progress.size--;
			int32_t id = progress.stack[progress.size];
			int32_t more = RebuildNodePartial(id, &tcount);
			if (more > 0) {
				progress.stack[progress.size] = more + 1;
				progress.stack[progress.size + 1] = more;
				progress.size += 2;
			} else if (progress.size == 0) {
				progress.stage = 6;
				progress.it = 0;
			}
			sum += tcount;
		}
		if (progress.size <= 0) {
			progress.stage = 6;
		}
		break;
	}

	case 6:
		progress.done = true;
		progress.stage = 8;
		break;
	}
	return progress.done;
}

BroadphaseBaseIterator *BvhMedianSplitHeap::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

BvhMedianSplitHeap::Iterator::Iterator(BvhMedianSplitHeap &bp)
{
	data = &bp.entitiesData;
	it = -1;
	Next();
}

BvhMedianSplitHeap::Iterator::~Iterator() {}

bool BvhMedianSplitHeap::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);

	return FetchData();
}

bool BvhMedianSplitHeap::Iterator::FetchData()
{
	if (Valid()) {
		entity = (*data)[it].entity;
		aabb = (*data)[it].aabb;
		mask = (*data)[it].mask;
		return true;
	}
	return false;
}

bool BvhMedianSplitHeap::Iterator::Valid() { return it < data->size(); }
} // namespace spp
