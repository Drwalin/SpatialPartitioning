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
	: entitiesOffsets(denseEntityRange), chunksBvh(0),
	  outerObjects(EntitiesOffsetsMapType_Reference(&entitiesOffsets, -1)),
	  iterator(*this)
{
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::~ChunkedBvhDbvt() {}

SPP_TEMPLATE_DECL_NO_AABB
const char *ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetName() const
{
	static char _b[1024];
	snprintf(_b, 1023, "ChunkedBvhDbvt [%i, %li]", outerObjects.GetCount(),
			chunks.size());
	return _b;
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
	size_t size = chunksBvh.GetMemoryUsage() + chunks.GetMemoryUsage() +
				  entitiesOffsets.GetMemoryUsage();
	for (const auto &c : chunks) {
		size += c.second.GetMemoryUsage();
	}
	return size;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::ShrinkToFit()
{
	chunksBvh.ShrinkToFit();
	entitiesOffsets.ShrinkToFit();
	std::vector<int32_t> toRemove;
	for (auto &c : chunks) {
		if (c.second.GetCount() == 0) {
			toRemove.push_back(c.first);
		} else {
			c.second.ShrinkToFit();
		}
	}

	for (auto &c : toRemove) {
		assert(chunksBvh.Exists(c));
		chunksBvh.Remove(c);
		chunks.erase(c);
	}
	chunksBvh.Rebuild();
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

	assert(chunkId != 0);

	if (chunkId == -1) {
		outerObjects.Add(entity, aabb, mask);
	} else {
		Chunk *chunk = GetOrInitChunk(chunkId, aabb);
		chunk->Add(entity, aabb, mask);
	}
	++entitiesCount;
}

void static BREAKPOINT() {}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Update(EntityType entity,
													   Aabb aabb)
{
	static int dupa = 0;
	if (entity == 13) {
		++dupa;
		if (dupa == 14) {
			BREAKPOINT();
		}
	}

	if (entitiesOffsets.find(entity) == nullptr) {
		assert(!"Entity does not exists");
		return;
	}

	int32_t offset = 0;
	int32_t oldChunkId = GetChunkIdOfEntity(entity, offset);
	int32_t newChunkId = GetChunkIdFromAabb(aabb);

	if (oldChunkId == newChunkId) {
		if (oldChunkId == -1) {
			outerObjects.Update(entity, aabb);
		} else {
			auto it = chunks.find(oldChunkId);
			assert(it != chunks.end() &&
				   "Updating entity within chunk that does not exist.");
			it->second.Update(entity, aabb);
		}
	} else {
		MaskType mask = 0;

		if (oldChunkId == -1) {
			mask = outerObjects.GetMask(entity);
			outerObjects.Remove(entity);
		} else {
			auto it = chunks.find(oldChunkId);
			assert(it != chunks.end() &&
				   "While updating entity trying to remove it from it's old "
				   "chunk but that chunk does not exist.");
			Chunk *oldChunk = &(it->second);
			mask = oldChunk->GetMask(entity, offset);
			oldChunk->Remove(entity);
			if (oldChunk->GetCount() == 0) {
				chunksBvh.Remove(oldChunkId);
				chunks.erase(it);
			}
		}

		if (newChunkId == -1) {
			outerObjects.Add(entity, aabb, mask);
		} else {
			GetOrInitChunk(newChunkId, aabb)->Add(entity, aabb, mask);
		}
		
		++roundRobinCounter;
		if ((roundRobinCounter & 255) == 0) {
			ShrinkToFitIncremental();
		}
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::ShrinkToFitIncremental()
{
	size_t c = chunks.bucket_count();
	if (c == 0) {
		[[unlikely]];
		return;
	}
	size_t bucket = mt() % c;
	size_t e = chunks.bucket_size(bucket);
	if (e == 0) {
		[[unlikely]];
		return;
	}
	size_t g = mt() % e;
	auto it = chunks.begin(bucket);
	for (size_t i=0; i<g; ++i) {
		++it;
	}
	it->second.Rebuild();
	it->second.ShrinkToFit();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Remove(EntityType entity)
{
	int32_t offset;
	Chunk *chunk = GetChunkOfEntity(entity, offset);

	if (chunk) {
		[[likely]];
		chunk->Remove(entity);
		if (chunk->GetCount() == 0) {
			[[unlikely]];
			uint32_t id = chunk->chunkId;
			chunksBvh.Remove(id);
			chunks.erase(id);
		}
	} else {
		outerObjects.Remove(entity);
	}

	--entitiesCount;
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkIdFromAabb(Aabb aabb) const
{
	glm::vec3 min = glm::floor(aabb.GetCenter() / chunkSize);
	glm::vec3 size = aabb.GetSizes();

	float maxSize = glm::max(size.x, glm::max(size.y, size.z));

	if (maxSize > maxChunkedEntitySize) {
		return -1;
	}

	if (glm::any(glm::lessThanEqual(min, glm::vec3(-limitChunkOffset)))) {
		return -1;
	}

	if (glm::any(glm::greaterThanEqual(min, glm::vec3(limitChunkOffset)))) {
		return -1;
	}

	glm::ivec3 hs = min + 512.0f;
	int32_t id =
		(hs.x & 0x3FF) | ((hs.y & 0x3FF) << 10) | ((hs.z & 0x3FF) << 20);

	return id;
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkIdOfEntity(EntityType entity,
															  int32_t &offset)
{
	offset = 0;

	auto it = entitiesOffsets.find(entity);
	if (it == nullptr) {
		return 0;
	} else {
		offset = it->offset;
		return it->segment;
	}
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkOfEntity(EntityType entity,
															int32_t &offset)
{
	const Chunk *chunk = ((const ChunkedBvhDbvt *)this)
							 ->GetChunkOfEntity(entity, offset);
	return (Chunk *)chunk;
}

SPP_TEMPLATE_DECL_NO_AABB
const ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkOfEntity(
	EntityType entity, int32_t &offset) const
{
	offset = 0;

	auto it = entitiesOffsets.find(entity);
	if (it == nullptr) {
		return nullptr;
	} else {
		int32_t segment = it->segment;
		offset = it->offset;
		if (segment == -1 || segment == 0) {
			return nullptr;
		}
		auto it2 = chunks.find(segment);
		if (it2 == chunks.end()) {
			assert(!"Should not happen: entity exists and is assigned to a "
					"chunk that does not exist.");
			return nullptr;
		}
		return &(it2->second);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetOrInitChunk(int32_t chunkId,
														  Aabb aabb)
{
	auto it = chunks.find(chunkId);
	if (it == chunks.end()) {
		// auto e = chunks.insert_or_assign(chunkId, std::move(Chunk(this,
		//                                  chunkId, chunkSize, aabb)));
		// auto e = chunks.emplace(chunkId,
		//                         Chunk(this, chunkId, chunkSize, aabb));
		Chunk *chunk = &(chunks[chunkId]);
		chunk->Init(this, chunkId, chunkSize, aabb);
		chunksBvh.Add(chunkId, chunk->globalAabb, ~0);
		return chunk;
	}
	return &(it->second);
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetChunkById(uint32_t chunkId)
{
	auto it = chunks.find(chunkId);
	if (it != chunks.end()) {
		return &(it->second);
	}
	return &(it->second);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::SetMask(EntityType entity,
														MaskType mask)
{
	int32_t off;
	Chunk *chunk = GetChunkOfEntity(entity, off);
	if (chunk == nullptr) {
		outerObjects.SetMask(entity, mask);
		return;
	} else {
		chunk->bvh->SetMask(entity, mask);
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
	int32_t off;
	Chunk const *chunk = GetChunkOfEntity(entity, off);
	if (chunk == nullptr) {
		return outerObjects.GetAabb(entity);
	} else {
		return chunk->GetAabb(entity, off);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
MaskType
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::GetMask(EntityType entity) const
{
	int32_t off;
	Chunk const *chunk = GetChunkOfEntity(entity, off);
	if (chunk == nullptr) {
		return outerObjects.GetMask(entity);
	} else {
		return chunk->GetMask(entity, off);
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

	cb.broadphase = this;

	typename AabbCallbacks::InterChunkCb interChunkCb;
	interChunkCb.InitFrom(cb, this);

	chunksBvh.IntersectAabb(interChunkCb);

	cb.testedCount += interChunkCb.testedCount;
	cb.testedCount += interChunkCb.intraCb.testedCount;

	cb.nodesTestedCount += interChunkCb.nodesTestedCount;
	cb.nodesTestedCount += interChunkCb.intraCb.nodesTestedCount;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;

	outerObjects.IntersectRay(cb);

	cb.broadphase = this;

	typename RayCallbacks::InterChunkCb interChunkCb;
	interChunkCb.InitFrom(cb, this);
	interChunkCb.callback = RayCallbacks::InterChunkCb::CallbackImpl;
	interChunkCb.dbvt = this;
	interChunkCb.orgCb = &cb;

	chunksBvh.IntersectRay(interChunkCb);

	cb.testedCount += interChunkCb.testedCount;
	cb.testedCount += interChunkCb.intraCb.testedCount;

	cb.nodesTestedCount += interChunkCb.nodesTestedCount;
	cb.nodesTestedCount += interChunkCb.intraCb.nodesTestedCount;

	cb.hitCount += interChunkCb.hitCount;
	cb.hitCount += interChunkCb.intraCb.hitCount;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Rebuild()
{
	chunksBvh.ShrinkToFit();
	entitiesOffsets.ShrinkToFit();
	std::vector<int32_t> toRemove;
	for (auto &c : chunks) {
		if (c.second.GetCount() == 0) {
			toRemove.push_back(c.first);
		} else {
			c.second.Rebuild();
			c.second.ShrinkToFit();
		}
	}

	for (auto &c : toRemove) {
		assert(chunksBvh.Exists(c));
		chunksBvh.Remove(c);
		chunks.erase(c);
	}
	chunksBvh.Rebuild();
	
	// TODO: Implement round robin rebuild / optimize / shrink to fit / delete
	//         empty
}

SPP_TEMPLATE_DECL_NO_AABB
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RestartIterator()
{
	iterator = Iterator{*this};
	return &iterator;
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::Iterator(
	ChunkedBvhDbvt &bp)
	: it(bp.entitiesOffsets.begin()), end(bp.entitiesOffsets.end())
{
	this->bp = &bp;
	FetchData();
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::Next()
{
	it.Next();
	return FetchData();
}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = it.first;
		this->aabb = bp->GetAabb(this->entity);
		this->mask = bp->GetMask(this->entity);
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL_NO_AABB
bool ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Iterator::Valid()
{
	return it != end;
}

}

#include "ChunkedBvhDbvtCallbacks.cpp"
#include "ChunkedBvhDbvtChunk.cpp"

namespace spp
{

SPP_DEFINE_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
