// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cassert>

#include "../glm/glm/ext/vector_uint3.hpp"

#include "../include/spatial_partitioning/ChunkedLooseOctree.hpp"

namespace spp
{
namespace experimental
{
SPP_TEMPLATE_DECL
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::ChunkedLooseOctree(int32_t chunkSize,
														  int32_t worldSize,
														  float loosness)
	: bigObjects(0), iterator(*this)
{
	this->loosness = loosness;
	this->loosnessInv = 1.0f / loosness;
	this->chunkSize = chunkSize;
	this->worldSize = worldSize;
	this->chunkSizeHalf = chunkSize / 2;
	this->worldSizeHalf = worldSize / 2;
	this->chunkCheckOffset = (chunkSizeHalf * (loosness - 1.0f));
	this->rootNode = nodes.Add({.halfSize = worldSize});
}

SPP_TEMPLATE_DECL
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::~ChunkedLooseOctree() {}

SPP_TEMPLATE_DECL
const char *ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetName() const
{
	return "ChunkedLooseOctree";
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Clear()
{
	bigObjects.Clear();
	data.Clear();
	nodes.Clear();
	chunks.Clear();
	rootNode = nodes.Add({.halfSize = worldSizeHalf});
}

SPP_TEMPLATE_DECL
size_t ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.GetMemoryUsage() +
		   chunks.GetMemoryUsage() + bigObjects.GetMemoryUsage() + 8;
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::ShrinkToFit()
{
	data.ShrinkToFit();
	nodes.ShrinkToFit();
	chunks.ShrinkToFit();
	bigObjects.ShrinkToFit();
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Add(EntityType entity, Aabb aabb,
												MaskType mask)
{
	if (FitsInChunk(aabb)) {
		int32_t offset = data.Add(entity, {aabb, entity, mask});
		int32_t chunkId = GetCreateChunkId(aabb.GetCenter());
		data[offset].chunkId = chunkId;
		AddToChunk(chunkId, offset);
	} else {
		int32_t offset = data.Add(entity, {aabb, entity, mask});
		data[offset].big = true;
		bigObjects.Add(entity, aabb, mask);
	}
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Update(EntityType entity, Aabb aabb)
{
	int32_t offset = data.GetOffset(entity);
	Data &d = data[offset];
	if (d.big) {
		d.aabb = aabb;
		if (FitsInChunk(aabb)) {
			bigObjects.Remove(entity);
			int32_t chunkId = GetCreateChunkId(aabb.GetCenter());
			d.chunkId = chunkId;
			d.big = false;
			AddToChunk(chunkId, offset);
		} else {
			bigObjects.Update(entity, aabb);
		}
	} else {
		glm::ivec4 a = GetNodePosSize(d.aabb);
		glm::ivec4 b = GetNodePosSize(aabb);
		d.aabb = aabb;
		if (a != b) {
			int32_t nodeId = d.nodeId;
			UnlinkFromChunk(offset);
			int32_t chunkId = GetCreateChunkId(aabb.GetCenter());
			data[offset].chunkId = chunkId;
			AddToChunk(chunkId, offset);
			CleanIfEmptyNodes(nodeId);
		}
	}
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Remove(EntityType entity)
{
	int32_t offset = data.GetOffset(entity);
	Data &d = data[offset];
	if (d.big) {
		bigObjects.Remove(entity);
	} else {
		RemoveFromChunk(offset);
	}
	data.RemoveByKey(entity);
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::SetMask(EntityType entity,
													MaskType mask)
{
	data[data.GetOffset(entity)].mask = mask;
}

SPP_TEMPLATE_DECL
int32_t ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetCount() const
{
	return data.Size();
}

SPP_TEMPLATE_DECL
bool ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Exists(EntityType entity) const
{
	return data.GetOffset(entity) > 0;
}

SPP_TEMPLATE_DECL
Aabb ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetAabb(EntityType entity) const
{
	return data[data.GetOffset(entity)].aabb;
}

SPP_TEMPLATE_DECL
MaskType ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetMask(EntityType entity) const
{
	return data[data.GetOffset(entity)].mask;
}

SPP_TEMPLATE_DECL
AabbCentered ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::NodeData::GetAabb() const
{
	return AabbCentered{glm::vec3(cx, cy, cz),
						glm::vec3(halfSize, halfSize, halfSize)};
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::IntersectAabb(AabbCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;

	bigObjects.IntersectAabb(cb);
	_Internal_IntersectAabb(cb, rootNode);
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::_Internal_IntersectAabb(
	AabbCallback &cb, const int32_t nodeId)
{
	NodeData &n = nodes[nodeId];
	++cb.nodesTestedCount;

	if (cb.IsRelevant(n.GetAabb()) == false) {
		return;
	}

	int32_t of = n.firstEntity;
	while (of > 0) {
		if (data[of].mask & cb.mask) {
			++cb.nodesTestedCount;
			if (cb.IsRelevant(data[of].aabb)) {
				++cb.testedCount;
				cb.callback(&cb, data[of].entity);
			}
		}
		of = data[of].nextDataId;
	}

	for (int i = 0; i < 8; ++i) {
		if (n.childrenIdLinear[i]) {
			_Internal_IntersectAabb(cb, n.childrenIdLinear[i]);
		}
	}
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	cb.InitVariables();

	_Internal_IntersectRay(cb, 1);
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::_Internal_IntersectRay(
	RayCallback &cb, const int32_t nodeId)
{
	NodeData &n = nodes[nodeId];
	++cb.nodesTestedCount;

	float near, far;
	if (cb.IsRelevant(n.GetAabb(), near, far) == false) {
		return;
	}

	int32_t of = n.firstEntity;
	while (of > 0) {
		if (data[of].mask & cb.mask) {
			cb.ExecuteIfRelevant(data[of].aabb, data[of].entity);
		}
		of = data[of].nextDataId;
	}

	for (int i = 0; i < 8; ++i) {
		if (n.childrenIdLinear[i]) {
			_Internal_IntersectRay(cb, n.childrenIdLinear[i]);
		}
	}
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Rebuild() { bigObjects.Rebuild(); }

SPP_TEMPLATE_DECL
glm::ivec3
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetMaxChunk(glm::vec3 point) const
{
	return (point + chunkCheckOffset) / (float)chunkSize;
}

SPP_TEMPLATE_DECL
glm::ivec3
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetMinChunk(glm::vec3 point) const
{
	return (point - chunkCheckOffset) / (float)chunkSize;
}

SPP_TEMPLATE_DECL
glm::ivec4
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetNodePosSize(Aabb aabb) const
{
	glm::vec3 s = aabb.max - aabb.min;
	float sm = glm::max(s.x, glm::max(s.y, s.z)) * loosnessInv;
	int32_t orgSize = sm * 2.0f;
	int32_t size = 1 << std::bit_width((uint32_t)orgSize);
	glm::ivec3 center = aabb.GetCenter();
	center *= size;
	center /= (chunkSize / size);
	return {center, size};
}

SPP_TEMPLATE_DECL
bool ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::FitsInChunk(Aabb aabb) const
{
	const glm::ivec3 a = GetMaxChunk(aabb.min);
	const glm::ivec3 b = GetMinChunk(aabb.max);
	const glm::ivec3 c = b - a;
	const glm::uvec3 d = c;
	return ((d.x | d.y | d.z) & 0x80000000u) == 0;
}

SPP_TEMPLATE_DECL
int32_t ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetCreateChunkId(glm::vec3 pos)
{
	assert(false);
	return 0;
}

SPP_TEMPLATE_DECL
int32_t ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetChunkId(glm::vec3 pos) const
{
	assert(false);
	return 0;
}

SPP_TEMPLATE_DECL
int32_t
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetChunkId(EntityType entity) const
{
	assert(false);
	return 0;
}

SPP_TEMPLATE_DECL
int32_t ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetCreateNodeId(Aabb aabb)
{
	assert(false);
	return 0;
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::AddToChunk(int32_t chunkId,
													   int32_t entityOffset)
{
	Data &d = data[entityOffset];
	chunkId = d.chunkId;

	d.nodeId = GetCreateNodeId(d.aabb);

	d.nextDataId = nodes[d.nodeId].firstEntity;
	nodes[d.nodeId].firstEntity = entityOffset;
	if (d.nextDataId) {
		data[d.nextDataId].prevDataId = entityOffset;
	}
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::RemoveFromChunk(
	int32_t entityOffset)
{
	int32_t nodeId = data[entityOffset].nodeId;
	UnlinkFromChunk(entityOffset);
	CleanIfEmptyNodes(nodeId);
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::UnlinkFromChunk(
	int32_t entityOffset)
{
	Data &d = data[entityOffset];
	int32_t nodeId = d.nodeId;
	if (d.prevDataId) {
		data[d.prevDataId].nextDataId = d.nextDataId;
	} else {
		nodes[nodeId].firstEntity = d.nextDataId;
	}
	if (d.nextDataId) {
		data[d.nextDataId].prevDataId = d.nextDataId;
	}
	d.nodeId = 0;
	d.chunkId = 0;
	d.prevDataId = 0;
	d.nextDataId = 0;
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::CleanIfEmptyNodes(int32_t nodeId)
{
	NodeData &n = nodes[nodeId];
	if (n.firstEntity) {
		return;
	}
	for (int i = 0; i < 4; ++i) {
		if (n.bigInts[i]) {
			return;
		}
	}

	if (n.chunkId && n.halfSize == chunkSizeHalf) {
		if (chunks[n.chunkId].userData.get() == nullptr) {
			chunks.Remove(n.chunkId);
			n.chunkId = 0;
		} else {
			return;
		}
	}

	int32_t parentId = n.parentId;
	if (parentId) {
		NodeData &pn = nodes[parentId];
		glm::ivec3 p = {n.cx, n.cy, n.cz};
		p += n.halfSize;
		p -= glm::ivec3{pn.cx, pn.cy, pn.cz};
		p /= (n.halfSize << 1);
		assert(p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x <= 1 && p.y <= 1 &&
			   p.z <= 1);
		pn.childrenId[p.x][p.y][p.z] = 0;
		nodes.Remove(nodeId);
		CleanIfEmptyNodes(parentId);
	}
}

SPP_TEMPLATE_DECL
std::shared_ptr<void>
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetChunkData(int32_t chunkId)
{
	return chunks[chunkId].userData;
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::SetChunkData(
	int32_t chunkId, std::shared_ptr<void> userData)
{
	chunks[chunkId].userData = userData;
	if (userData.get() == nullptr) {
		CleanIfEmptyNodes(chunks[chunkId].smallNodeId);
	}
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetChunks(
	Aabb aabb, std::vector<int32_t> chunks)
{
	assert(false);
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::GetChunks(
	bool (*isIn)(Aabb test, void *testData), void *testData,
	std::vector<int32_t> chunksData)
{
	assert(false);
}

SPP_TEMPLATE_DECL
void ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::ForEachChunkData(
	bool (*isIn)(Aabb test, void *testData), void *testData,
	void (*func)(ChunkedLooseOctree &self, int32_t chunk, void *userData),
	void *userData)
{
	assert(false);
}

SPP_TEMPLATE_DECL
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Iterator::Iterator(
	ChunkedLooseOctree &bp)
{
	data = &bp.data._Data()._Data();
	it = 0;
	Next();
}

SPP_TEMPLATE_DECL
ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL
bool ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

SPP_TEMPLATE_DECL
bool ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = (*data)[it].entity;
		this->aabb = (*data)[it].aabb;
		this->mask = (*data)[it].mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL
bool ChunkedLooseOctree<SPP_TEMPLATE_ARGS>::Iterator::Valid()
{
	return it < data->size();
}

SPP_DEFINE_VARIANTS(ChunkedLooseOctree)

} // namespace experimental
} // namespace spp
