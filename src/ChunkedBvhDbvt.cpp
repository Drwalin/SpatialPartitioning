// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>

#include "../glm/glm/vector_relational.hpp"
#include "../glm/glm/common.hpp"

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::ChunkedBvhDbvt(
	EntityType denseEntityRange)
	:
	entitiesOffsets(denseEntityRange), iterator(*this),
		outerObjects(MapType(&entitiesOffsets, -1))
{
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::~ChunkedBvhDbvt() {}

SPP_TEMPLATE_DECL_NO_AABB
const char *ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetName() const
{
	return "ChunkedBvhDbvt";
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Clear()
{
	chunks.clear();
	chunksBvh.Clear();
	entitiesOffsets.Clear();
	entitiesCount = 0;
}

SPP_TEMPLATE_DECL_NO_AABB
size_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetMemoryUsage() const
{
	size_t size =
		chunksBvh.GetMemoryUsage() + chunks.GetMemoryUsage() + entitiesOffsets;
	for (const auto &c : chunks) {
		size += c.GetMemoryUsage();
	}
	return size;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::ShrinkToFit()
{
	chunks.shrink_to_fit();
	chunksBvh.ShrinkToFit();
	entitiesOffsets.ShrinkToFit();
	std::vector<int32_t> toRemove;
	for (auto &c : chunks) {
		if (c.second.GetCount() == 0) {
			toRemove.push_back(c.first);
		} else {
			c.ShrinkToFit();
		}
	}
	chunks.erase(toRemove.begin(), toRemove.end());
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Add(EntityType entity,
													Aabb aabb, MaskType mask)
{
	if (entitiesOffsets.find(entity) != nullptr) {
		assert(!"Entity already exists");
		return;
	}
	
	int32_t chunkId = GetChunkIdFromAabb(aabb);
	
	assert(chunkId == 0);
	
	if (chunkId == -1) {
		outerObjects.Add(entity, aabb, mask);
	} else {
		Chunk &chunk = chunks[chunkId];
		if (chunk.inited == false) {
			chunk.Init(chunkSize, aabb);
		}
		chunk.Add(entity, aabb, mask);
	}
	++entitiesCount;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Update(EntityType entity,
													   Aabb aabb)
{
	if (entitiesOffsets.find(entity) != nullptr) {
		assert(!"Entity already exists");
		return;
	}
	
	int32_t offset = 0;
	int32_t oldChunkId = GetChunkIdOfEntity(entity, offset);
	int32_t newChunkId = GetChunkIdFromAabb(aabb);
	
	if (oldChunkId>>1 == newChunkId>>1) {
		if (oldChunkId == -1) {
			outerObjects.Update(entity, aabb);
		} else {
			chunks[oldChunkId].Update(entity, aabb, oldChunkId);
		}
	} else {
		MaskType mask = 0;
		
		if (oldChunkId == -1) {
			mask = outerObjects.GetMask(entity);
			outerObjects.Remove(entity);
		} else {
			Chunk &oldChunk = chunks[oldChunkId];
			mask = oldChunk.GetMask(entity);
			oldChunk.Remove(entity, oldChunkId);
		}
		
		if (newChunkId == -1) {
			outerObjects.Add(entity);
		} else {
			chunks[newChunkId].Add(entity, aabb, mask);
		}
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Remove(EntityType entity)
{
	int32_t segment, offset;
	Chunk *chunk = GetChunkOfEntity(entity, segment, offset);
	
	if (chunk) {
		chunk->Remove(entity, segment);
	} else {
		outerObjects.Remove(entity);
	}
	
	--entitiesCount;
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkIdFromAabb(Aabb aabb) const
{
	glm::vec3 center = aabb.GetCenter() / chunkSize;
	glm::vec3 halfSize = aabb.GetSizes() * 0.5f;

	float maxHalfSize = glm::max(halfSize.x, glm::max(halfSize.y, halfSize.z));

	if (maxHalfSize > maxChunkedEntitySize) {
		return -1;
	}

	if (glm::any(glm::lessThanEqual(center, glm::vec3(-limitChunkOffset)))) {
		return -1;
	}

	if (glm::any(glm::greaterThanEqual(center, glm::vec3(limitChunkOffset)))) {
		return -1;
	}

	glm::ivec3 hs = center + 512.f;
	int32_t id =
		((hs.x & 0x3FF) << 1) | ((hs.y & 0x3FF) << 11) | ((hs.z & 0x3FF) << 21);

	return id;
}


SPP_TEMPLATE_DECL_NO_AABB
int32_t
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkIdOfEntity(
	EntityType entity, int32_t &offset)
{
	offset = 0;
	
	auto it = entitiesOffsets.find(entity);
	if (it == entitiesOffsets.end()) {
		return 0;
	} else {
		offset = it->second.offset;
		return it->second.segment;
	}
}
	

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkOfEntity(
	EntityType entity, int32_t &segment, int32_t &offset)
{
	segment = offset = 0;
	
	auto it = entitiesOffsets.find(entity);
	if (it == entitiesOffsets.end()) {
		return nullptr;
	} else {
		segment = it->second.segment;
		offset = it->second.offset;
		if (segment == -1 || segment == 0) {
			return nullptr;
		}
		int32_t s = segment & ~(int32_t)1;
		auto it2 = chunks.find(s);
		if (it2 == chunks.end()) {
			assert(!"Should not happen: entity exists and is assigned to a chunk that does not exist.");
			return nullptr;
		}
		return &(it2->second);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::SetMask(EntityType entity,
														MaskType mask)
{
	int32_t seg, off;
	Chunk *chunk = GetChunkOfEntity(entity, seg, off);
	if (seg == 0) {
		assert(seg != 0);
		return;
	}
	if (chunk == nullptr) {
		outerObjects.SetMask(entity, mask);
		return;
	} else {
		chunk->bvh[seg&1].SetMask(entity, mask);
		return;
	}
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetCount() const
{
	return entitiesCount;
}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Exists(EntityType entity) const
{
	return entitiesOffsets.Has(entity);
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetAabb(EntityType entity) const
{
	int32_t seg, off;
	Chunk *chunk = GetChunkOfEntity(entity, seg, off);
	if (seg == 0) {
		assert(seg != 0);
		return {};
	}
	if (chunk == nullptr) {
		return outerObjects.GetAabb(entity);
	} else {
		return chunk->GetAabb(entity);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
MaskType
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetMask(EntityType entity) const
{
	int32_t seg, off;
	Chunk *chunk = GetChunkOfEntity(entity, seg, off);
	if (seg == 0) {
		assert(seg != 0);
		return {};
	}
	if (chunk == nullptr) {
		return outerObjects.GetMask(entity);
	} else {
		return chunk->bvh[seg&1].GetMask(entity);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::IntersectAabb(AabbCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}
	
	cb.broadphase = this;
	
	outerObjects.IntersectAabb(cb);
	
	const AabbCallback orgCb = cb;
	
	class CB2 : public AabbCallback
	{
	public:
		AabbCallback *orgCb;
		ChunkedBvhDbvt *dbvt;
	} cb2(cb);
	
	cb2.callback = +[](AabbCallback *_cb2, int32_t chunkId)
	{
		CB2 *cb2 = (CB2)_cb2;
		
		
		
		cb2->orgCb->
	};
	cb2.broadphase = this;
	cb2.dbvt = this;
	
	

	_Internal_IntersectAabb(cb, 1);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::_Internal_IntersectAabb(
	AabbCallback &cb, const int32_t nodeId)
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

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::IntersectRay(RayCallback &cb)
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

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::_Internal_IntersectRay(
	RayCallback &cb, const int32_t nodeId)
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

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Rebuild()
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

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RebuildNode(int32_t nodeId)
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

SPP_TEMPLATE_DECL_NO_AABB
int32_t
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RebuildNodePartial(int32_t nodeId,
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

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::PruneEmptyEntitiesAtEnd()
{
	for (int32_t i = entitiesData.size() - 1; i >= 0; --i) {
		if (entitiesData[i].entity != EMPTY_ENTITY) {
			entitiesData.resize(i + 1);
			return;
		}
	}
	entitiesData.clear();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::UpdateAabb(int32_t offset)
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

SPP_TEMPLATE_DECL_NO_AABB
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::Iterator(
	ChunkedBvhDbvt &bp)
{
	data = &bp.entitiesData;
	it = -1;
	Next();
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);

	return FetchData();
}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = (*data)[it].entity;
		this->aabb = (*data)[it].aabb;
		this->mask = (*data)[it].mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::Valid()
{
	return it < data->size();
}

SPP_DEFINE_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
