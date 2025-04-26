// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cassert>

#include "../../thirdparty/glm/glm/ext/vector_uint3.hpp"

#include "../include/spatial_partitioning/ChunkedLooseOctree.hpp"

namespace spp
{
namespace experimental
{
ChunkedLooseOctree::ChunkedLooseOctree(int32_t chunkSize, int32_t worldSize,
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

ChunkedLooseOctree::~ChunkedLooseOctree() {}

const char *ChunkedLooseOctree::GetName() const { return "ChunkedLooseOctree"; }

void ChunkedLooseOctree::Clear()
{
	bigObjects.Clear();
	data.Clear();
	nodes.Clear();
	chunks.Clear();
	rootNode = nodes.Add({.halfSize = worldSizeHalf});
}

size_t ChunkedLooseOctree::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.GetMemoryUsage() +
		   chunks.GetMemoryUsage() + bigObjects.GetMemoryUsage() + 8;
}

void ChunkedLooseOctree::ShrinkToFit()
{
	data.ShrinkToFit();
	nodes.ShrinkToFit();
	chunks.ShrinkToFit();
	bigObjects.ShrinkToFit();
}

void ChunkedLooseOctree::Add(EntityType entity, Aabb aabb, MaskType mask)
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

void ChunkedLooseOctree::Update(EntityType entity, Aabb aabb)
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

void ChunkedLooseOctree::Remove(EntityType entity)
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

void ChunkedLooseOctree::SetMask(EntityType entity, MaskType mask)
{
	data[data.GetOffset(entity)].mask = mask;
}

int32_t ChunkedLooseOctree::GetCount() const { return data.Size(); }

bool ChunkedLooseOctree::Exists(EntityType entity) const
{
	return data.GetOffset(entity) > 0;
}

Aabb ChunkedLooseOctree::GetAabb(EntityType entity) const
{
	return data[data.GetOffset(entity)].aabb;
}

MaskType ChunkedLooseOctree::GetMask(EntityType entity) const
{
	return data[data.GetOffset(entity)].mask;
}

AabbCentered ChunkedLooseOctree::NodeData::GetAabb() const
{
	return AabbCentered{glm::vec3(cx, cy, cz),
						glm::vec3(halfSize, halfSize, halfSize)};
}

void ChunkedLooseOctree::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;

	bigObjects.IntersectAabb(cb);
	_Internal_IntersectAabb(cb, rootNode);
}

void ChunkedLooseOctree::_Internal_IntersectAabb(IntersectionCallback &cb,
												 const int32_t nodeId)
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

void ChunkedLooseOctree::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	cb.InitVariables();

	_Internal_IntersectRay(cb, 1);
}

void ChunkedLooseOctree::_Internal_IntersectRay(RayCallback &cb,
												const int32_t nodeId)
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

void ChunkedLooseOctree::Rebuild() { bigObjects.Rebuild(); }

glm::ivec3 ChunkedLooseOctree::GetMaxChunk(glm::vec3 point) const
{
	return (point + chunkCheckOffset) / (float)chunkSize;
}

glm::ivec3 ChunkedLooseOctree::GetMinChunk(glm::vec3 point) const
{
	return (point - chunkCheckOffset) / (float)chunkSize;
}

glm::ivec4 ChunkedLooseOctree::GetNodePosSize(Aabb aabb) const
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

bool ChunkedLooseOctree::FitsInChunk(Aabb aabb) const
{
	const glm::ivec3 a = GetMaxChunk(aabb.min);
	const glm::ivec3 b = GetMinChunk(aabb.max);
	const glm::ivec3 c = b - a;
	const glm::uvec3 d = c;
	return ((d.x | d.y | d.z) & 0x80000000u) == 0;
}

int32_t ChunkedLooseOctree::GetCreateChunkId(glm::vec3 pos)
{
	assert(false);
	assert(true); // TODO
	return 0;
}
int32_t ChunkedLooseOctree::GetChunkId(glm::vec3 pos) const
{
	assert(false);
	assert(true); // TODO
	return 0;
}
int32_t ChunkedLooseOctree::GetChunkId(EntityType entity) const
{
	assert(false);
	assert(true); // TODO
	return 0;
}
int32_t ChunkedLooseOctree::GetCreateNodeId(Aabb aabb)
{
	assert(false);
	assert(true); // TODO
	return 0;
}

void ChunkedLooseOctree::AddToChunk(int32_t chunkId, int32_t entityOffset)
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

void ChunkedLooseOctree::RemoveFromChunk(int32_t entityOffset)
{
	int32_t nodeId = data[entityOffset].nodeId;
	UnlinkFromChunk(entityOffset);
	CleanIfEmptyNodes(nodeId);
}

void ChunkedLooseOctree::UnlinkFromChunk(int32_t entityOffset)
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

void ChunkedLooseOctree::CleanIfEmptyNodes(int32_t nodeId)
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

std::shared_ptr<void> ChunkedLooseOctree::GetChunkData(int32_t chunkId)
{
	return chunks[chunkId].userData;
}

void ChunkedLooseOctree::SetChunkData(int32_t chunkId,
									  std::shared_ptr<void> userData)
{
	chunks[chunkId].userData = userData;
	if (userData.get() == nullptr) {
		CleanIfEmptyNodes(chunks[chunkId].smallNodeId);
	}
}

void ChunkedLooseOctree::GetChunks(Aabb aabb, std::vector<int32_t> chunks)
{
	assert(false);
	assert(true); // TODO
}

void ChunkedLooseOctree::GetChunks(bool (*isIn)(Aabb test, void *testData),
								   void *testData,
								   std::vector<int32_t> chunksData)
{
	assert(false);
	assert(true); // TODO
}

void ChunkedLooseOctree::ForEachChunkData(
	bool (*isIn)(Aabb test, void *testData), void *testData,
	void (*func)(ChunkedLooseOctree &self, int32_t chunk, void *userData),
	void *userData)
{
	assert(false);
	assert(true); // TODO
}

BroadphaseBaseIterator *ChunkedLooseOctree::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

ChunkedLooseOctree::Iterator::Iterator(ChunkedLooseOctree &bp)
{
	data = &bp.data._Data()._Data();
	it = 0;
	Next();
}

ChunkedLooseOctree::Iterator::~Iterator() {}

bool ChunkedLooseOctree::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

bool ChunkedLooseOctree::Iterator::FetchData()
{
	if (Valid()) {
		entity = (*data)[it].entity;
		aabb = (*data)[it].aabb;
		mask = (*data)[it].mask;
		return true;
	}
	return false;
}

bool ChunkedLooseOctree::Iterator::Valid() { return it < data->size(); }
} // namespace experimental
} // namespace spp
