// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>

#include <bit>
#include <algorithm>

#include "../include/spatial_partitioning/BvhMedianSplitHeap.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::
	BvhMedianSplitHeap(EntityType denseEntityRange)
	: entitiesOffsets(denseEntityRange), iterator(*this)
{
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::
	BvhMedianSplitHeap(EntitiesOffsetsMapType &&entityIdsMap)
	: entitiesOffsets(std::move(entityIdsMap)), iterator(*this)
{
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
										  SegmentType)>::~BvhMedianSplitHeap()
{
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
const char *BvhMedianSplitHeap<
	SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::GetName() const
{
	switch (SKIP_LOW_LAYERS) {
	case 0:
		return "BvhMedianSplitHeap";
	case 1:
		return "BvhMedianSplitHeap1";
	case 2:
		return "BvhMedianSplitHeap2";
	default:
		return "BvhMedianSplitHeapN";
	}
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::SetAabbUpdatePolicy(AabbUpdatePolicy policy)
{
	updatePolicy = policy;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
										  SegmentType)>::AabbUpdatePolicy
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::GetAabbUpdatePolicy() const
{
	return updatePolicy;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
											   SegmentType)>::Clear()
{
	ClearWithoutOffsets();
	entitiesOffsets.Clear();
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<
	SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::ClearWithoutOffsets()
{
	entitiesData.clear();
	nodesHeapAabb.clear();
	rebuildTree = false;
	entitiesCount = 0;
	entitiesPowerOfTwoCount = 0;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
size_t
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
										  SegmentType)>::GetMemoryUsage() const
{
	return (entitiesOffsets.owning ? entitiesOffsets.GetMemoryUsage()
								   : (size_t)0) +
		   nodesHeapAabb.capacity() * sizeof(NodeData) +
		   entitiesData.capacity() * sizeof(Data);
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
											   SegmentType)>::ShrinkToFit()
{
	nodesHeapAabb.shrink_to_fit();
	entitiesData.shrink_to_fit();
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::Add(EntityType entity, Aabb aabb,
										MaskType mask)
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::Update(EntityType entity, Aabb aabb)
{
	uint32_t offset = entitiesOffsets[entity];
	entitiesData[offset].aabb = aabb;
	if (updatePolicy == ON_UPDATE_EXTEND_AABB && rebuildTree == false) {
		UpdateAabb(offset);
	} else {
		rebuildTree = true;
	}
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::Remove(EntityType entity)
{
	auto it = entitiesOffsets.find(entity);
	if (it == nullptr) {
		return;
	}

	uint32_t offset = EntitiesOffsetsMapType::get_offset_from_it(it);

	assert(offset != -1);

	entitiesOffsets.Remove(entity);
	entitiesData[offset].entity = EMPTY_ENTITY;
	entitiesData[offset].mask = 0;

	--entitiesCount;

	if (entitiesCount == 0) {
		Clear();
		return;
	}

	PruneEmptyEntitiesAtEnd();

	if (rebuildTree == false) {
		UpdateAabb(offset);
	}
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::SetMask(EntityType entity, MaskType mask)
{
	auto it = entitiesOffsets.find(entity);
	if (it == nullptr) {
		return;
	}

	uint32_t offset = EntitiesOffsetsMapType::get_offset_from_it(it);

	if (entitiesData[offset].mask == mask) {
		return;
	}

	entitiesData[offset].mask = mask;
	if ((offset ^ 1) < entitiesData.size()) {
		if (entitiesData[offset ^ 1].entity != EMPTY_ENTITY) {
			mask |= entitiesData[offset ^ 1].mask;
		}
	}

	uint32_t n = (offset + entitiesPowerOfTwoCount) >> (1 + SKIP_LOW_LAYERS);

	for (; n > 0; n >>= 1) {
		nodesHeapAabb[n].mask = mask;
		if ((n ^ 1) < nodesHeapAabb.size() && (n ^ 1) > 0) {
			mask |= nodesHeapAabb[n ^ 1].mask;
		}
	}
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
EntityType BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::GetEntityByOffset(int32_t offset) const
{
	if (entitiesData.size() <= offset) {
		return EMPTY_ENTITY;
	}
	return entitiesData[offset].entity;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
int32_t BvhMedianSplitHeap<
	SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::GetCount() const
{
	return entitiesCount;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
bool BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::Exists(EntityType entity) const
{
	return entitiesOffsets.Has(entity);
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
Aabb BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::GetAabb(EntityType entity) const
{
	auto it = entitiesOffsets.find(entity);
	if (it != nullptr) {
		uint32_t offset = EntitiesOffsetsMapType::get_offset_from_it(it);
		return entitiesData[offset].aabb;
	}
	return {};
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
MaskType BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::GetMask(EntityType entity) const
{
	auto it = entitiesOffsets.find(entity);
	if (it != nullptr) {
		uint32_t offset = EntitiesOffsetsMapType::get_offset_from_it(it);
		return entitiesData[offset].mask;
	}
	return 0;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::IntersectAabb(AabbCallback &cb)
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::
	_Internal_IntersectAabb(AabbCallback &cb, const int32_t nodeId)
{
	const int32_t n = nodeId << 1;

	if (n >= entitiesPowerOfTwoCount) {
		int32_t o = n - entitiesPowerOfTwoCount;
		for (int i = 0; i <= 1 && o < entitiesData.size(); ++i, ++o) {
			if ((entitiesData[o].mask & cb.mask) &&
				entitiesData[o].entity != EMPTY_ENTITY) {
				cb.ExecuteIfRelevant(entitiesData[o].aabb,
									 entitiesData[o].entity);
			}
		}
	} else if (SKIP_LOW_LAYERS && n >= nodesHeapAabb.size()) {
		assert(SKIP_LOW_LAYERS);
		const int32_t start = (n << SKIP_LOW_LAYERS) - entitiesPowerOfTwoCount;
		const int32_t end_ = start + (2 << SKIP_LOW_LAYERS);
		const int32_t end = std::min<int32_t>(end_, entitiesData.size());
		assert(start >= 0);
		for (int32_t i = start; i < end; ++i) {
			if ((entitiesData[i].mask & cb.mask) &&
				entitiesData[i].entity != EMPTY_ENTITY) {
				cb.ExecuteIfRelevant(entitiesData[i].aabb,
									 entitiesData[i].entity);
			}
		}
	} else {
		for (int i = 0; i <= 1 && n + i < nodesHeapAabb.size(); ++i) {
			if (nodesHeapAabb[n + i].mask & cb.mask) {
				++cb.nodesTestedCount;
				if (cb.IsRelevant(nodesHeapAabb[n + i].aabb)) {
					_Internal_IntersectAabb(cb, n + i);
				}
			}
		}
	}
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::IntersectRay(RayCallback &cb)
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::_Internal_IntersectRay(RayCallback &cb,
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
	} else if (SKIP_LOW_LAYERS && n >= nodesHeapAabb.size()) {
		assert(SKIP_LOW_LAYERS);
		const int32_t start = (n << SKIP_LOW_LAYERS) - entitiesPowerOfTwoCount;
		const int32_t end_ = start + (2 << SKIP_LOW_LAYERS);
		const int32_t end = std::min<int32_t>(end_, entitiesData.size());
		assert(start >= 0);
		for (int32_t i = start; i < end; ++i) {
			if ((entitiesData[i].mask & cb.mask) &&
				entitiesData[i].entity != EMPTY_ENTITY) {
				auto &ed = entitiesData[i];
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
											   SegmentType)>::Rebuild()
{
	rebuildTree = false;
	entitiesPowerOfTwoCount = std::bit_ceil((uint32_t)entitiesCount);

	if (SKIP_LOW_LAYERS) {
		nodesHeapAabb.resize(entitiesPowerOfTwoCount >> SKIP_LOW_LAYERS);
	} else {
		nodesHeapAabb.resize(entitiesPowerOfTwoCount / 2 +
							 (entitiesCount + 1) / 2);
	}
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::RebuildNode(int32_t nodeId)
{
	int32_t tcount = 0;
	nodeId = RebuildNodePartial(nodeId, &tcount);
	if (nodeId > 1 && nodeId < nodesHeapAabb.size()) {
		RebuildNode(nodeId);
		if (nodeId + 1 < nodesHeapAabb.size()) {
			RebuildNode(nodeId + 1);
		}
	}
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
int32_t BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::RebuildNodePartial(int32_t nodeId,
													   int32_t *tcount)
{
	*tcount = 0;
	int32_t offset = nodeId;
	int32_t count = 1;
	while (offset < entitiesPowerOfTwoCount) {
		offset <<= 1;
		count <<= 1;
	}
	const int32_t orgCount = count;
	offset -= entitiesPowerOfTwoCount;
	if (offset >= entitiesData.size()) {
		return -1;
	}
	count = std::min<int32_t>(count, ((int32_t)entitiesData.size()) - offset);

	if (count <= 0) {
		return -1;
	}

	*tcount = count;

	Aabb totalAabb = entitiesData[offset].aabb;
	MaskType mask = entitiesData[offset].mask;
	for (int32_t i = offset + 1; i < offset + count; ++i) {
		totalAabb = totalAabb + entitiesData[i].aabb;
		mask |= entitiesData[i].mask;
	}

	if (nodeId < nodesHeapAabb.size()) {
		nodesHeapAabb[nodeId] = {totalAabb.Expanded(BIG_EPSILON), mask};
	}

	if (count <= (2 << SKIP_LOW_LAYERS)) {
		for (int32_t i = offset; i < offset + count; ++i) {
			entitiesOffsets.Set(entitiesData[i].entity, i);
		}
		if (orgCount <= 2) {
			return -1;
		} else {
			return nodeId << 1;
		}
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::PruneEmptyEntitiesAtEnd()
{
	for (int32_t i = entitiesData.size() - 1; i >= 0; --i) {
		if (entitiesData[i].entity != EMPTY_ENTITY) {
			entitiesData.resize(i + 1);
			return;
		}
	}
	entitiesData.clear();
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
void BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::UpdateAabb(int32_t offset)
{
	MaskType mask = 0;
	Aabb aabb = {{0, 0, 0}, {0, 0, 0}};
	for (int32_t i = 0; i < (2 << SKIP_LOW_LAYERS); ++i) {
		const int32_t o = offset ^ i;
		if (o < entitiesData.size()) {
			const auto &ed = entitiesData[o];
			if ((ed.entity != EMPTY_ENTITY) && (ed.mask != 0)) {
				if (mask != 0) {
					aabb = aabb + ed.aabb;
				} else {
					aabb = ed.aabb;
				}
				mask |= ed.mask;
			}
		}
	}

	aabb = aabb.Expanded(BIG_EPSILON);

	for (uint32_t n =
			 (offset + entitiesPowerOfTwoCount) >> (1 + SKIP_LOW_LAYERS);
		 n > 0; n >>= 1) {
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
bool BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::RebuildStep(RebuildProgress &progress)
{
	if (progress.done) {
		return true;
	}

	switch (progress.stage) {
	case 0:
		rebuildTree = false;
		entitiesPowerOfTwoCount = std::bit_ceil((uint32_t)entitiesCount);
		if (SKIP_LOW_LAYERS) {
			nodesHeapAabb.resize(entitiesPowerOfTwoCount >> SKIP_LOW_LAYERS);
		} else {
			nodesHeapAabb.resize(entitiesPowerOfTwoCount / 2 +
								 (entitiesCount + 1) / 2 + 7);
		}
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

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *BvhMedianSplitHeap<
	SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(
	SKIP_LOW_LAYERS, SegmentType)>::Iterator::Iterator(BvhMedianSplitHeap &bp)
{
	data = &bp.entitiesData;
	it = -1;
	Next();
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
										  SegmentType)>::Iterator::~Iterator()
{
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
bool BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
											   SegmentType)>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);

	return FetchData();
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
bool BvhMedianSplitHeap<
	SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS, SegmentType)>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = (*data)[it].entity;
		this->aabb = (*data)[it].aabb;
		this->mask = (*data)[it].mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS, typename SegmentType)
bool BvhMedianSplitHeap<SPP_TEMPLATE_ARGS_MORE(SKIP_LOW_LAYERS,
											   SegmentType)>::Iterator::Valid()
{
	return it < data->size();
}

SPP_DEFINE_VARIANTS_MORE(BvhMedianSplitHeap, 0, void)
SPP_DEFINE_VARIANTS_MORE(BvhMedianSplitHeap, 1, void)

SPP_DEFINE_VARIANTS_MORE(BvhMedianSplitHeap, 0, int32_t)
SPP_DEFINE_VARIANTS_MORE(BvhMedianSplitHeap, 1, int32_t)

} // namespace spp
