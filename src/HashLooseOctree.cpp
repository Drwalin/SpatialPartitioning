// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#ifdef __SPP_USE_HASH_LOOSE_OCTREE__

#include <cstdio>
#include <cstring>

#include <bit>

#include "../../thirdparty/glm/glm/ext/vector_int3.hpp"

#include "../include/spatial_partitioning/HashLooseOctree.hpp"

namespace spp
{
HashLooseOctree::HashLooseOctree(float resolution, int32_t levels,
								 float loosenessFactor)
	: nodes(12289, Key::Hash(this)), loosenessFactor(loosenessFactor),
	  invLoosenessFactor(1.0f / loosenessFactor), resolution(resolution),
	  invResolution(1.0f / resolution), levels(levels)
{
	Clear();
}
HashLooseOctree::~HashLooseOctree() {}

const char *HashLooseOctree::GetName() const { return "HashLooseOctree"; }

void HashLooseOctree::Clear()
{
	data.Clear();
	nodes.clear();
}

size_t HashLooseOctree::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.bucket_count() * sizeof(void *) +
		   nodes.size() *
			   (sizeof(void *) * 2lu + sizeof(Key) + sizeof(NodeData));
}

void HashLooseOctree::ShrinkToFit() { data.ShrinkToFit(); }

int32_t HashLooseOctree::CalcHashMinLevel(Aabb aabb)
{
	aabb.min *= invResolution;
	aabb.max *= invResolution;
	const glm::vec3 sizes = aabb.GetSizes();
	const float _size = glm::max(sizes.x, glm::max(sizes.y, sizes.z));
	const int32_t size = _size * invLoosenessFactor;
	int32_t minLevel = std::bit_width<uint32_t>(size);

	if (minLevel > levels) {
		return levels + 1;
	}

	return minLevel;
}

#define ROT64(V, R) ((V << R) | (V >> (64 - R)))

uint64_t HashLooseOctree::Hash(const glm::vec3 pos, int32_t level)
{
	if (level > levels) {
		return 3141592653589793238lu;
	}

	const glm::ivec3 p = pos * invResolution;
	return Hash(p, level);
}

uint64_t HashLooseOctree::Hash(const glm::ivec3 pos, int32_t level)
{
	if (level > levels) {
		return 3141592653589793238lu;
	}

	const glm::ivec3 a = pos >> level;
	return (14695981039346656037lu ^ ((uint64_t)((uint32_t)a.x)) ^
			ROT64((uint64_t)((uint32_t)a.x), 21) ^
			ROT64((uint64_t)((uint32_t)a.x), 42)) *
		   1099511628211lu;
}

void HashLooseOctree::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	const int32_t offset = data.Add(entity, {aabb, entity, mask});

	if (offset < 0) {
		Data &d = data[data.GetOffset(entity)];
		if (d.mask != mask) {
			SetMask(entity, mask);
		}
		if (d.aabb != aabb) {
			Update(entity, aabb);
		}
		return;
	}

	int32_t level = CalcHashMinLevel(aabb);
	const glm::ivec3 pos = aabb.GetCenter() * invResolution;

	{
		NodeData &nd = nodes[Key(this, pos, level)];
		nd.directChildrenCount++;

		data[offset].next = nd.firstChild;
		data[offset].prev = -1;
		if (nd.firstChild >= 0) {
			data[nd.firstChild].prev = offset;
		}
		nd.firstChild = offset;
	}

	for (++level; level <= levels; ++level) {
		glm::ivec3 p = glm::ivec3(pos) >> (level - 1);
		const int32_t childId = (p.x & 1) | ((p.y & 1) << 1) | ((p.y & 1) << 2);
		NodeData &nd = nodes[Key(this, pos, level)];
		nd.childrenInNodesCounts[childId]++;
	}
}

void HashLooseOctree::Update(EntityType entity, Aabb aabb)
{
	const int32_t offset = data.GetOffset(entity);
	const Aabb oldAabb = data[offset].aabb;
	data[offset].aabb = aabb;

	const int32_t oldLevel = CalcHashMinLevel(oldAabb);
	int32_t level = CalcHashMinLevel(aabb);

	if (level != oldLevel) {
		MaskType mask = data[offset].mask;
		Remove(entity);
		Add(entity, aabb, mask);
		return;
	}

	const glm::ivec3 oldPos = oldAabb.GetCenter() * invResolution;
	const glm::ivec3 pos = aabb.GetCenter() * invResolution;

	if (oldPos == pos) {
		return;
	}

	{
		NodeData &oldNd = nodes[Key(this, oldPos, level)];
		NodeData &nd = nodes[Key(this, pos, level)];

		oldNd.directChildrenCount--;
		if (data[offset].prev >= 0) {
			data[data[offset].prev].next = data[offset].next;
		} else {
			oldNd.firstChild = data[offset].next;
		}
		if (data[offset].next >= 0) {
			data[data[offset].next].prev = data[offset].prev;
		}

		nd.directChildrenCount++;
		data[offset].next = nd.firstChild;
		data[offset].prev = -1;
		if (nd.firstChild >= 0) {
			data[nd.firstChild].prev = offset;
		}
		nd.firstChild = offset;
	}

	for (++level; level <= levels; ++level) {
		if (Key(this, oldPos, level) == Key(this, pos, level)) {
			break;
		}

		NodeData &oldNd = nodes[Key(this, oldPos, level)];
		NodeData &nd = nodes[Key(this, pos, level)];

		glm::ivec3 op = glm::ivec3(oldPos) >> (level - 1);
		const int32_t oldChildId =
			(op.x & 1) | ((op.y & 1) << 1) | ((op.y & 1) << 2);
		oldNd.childrenInNodesCounts[oldChildId]--;

		glm::ivec3 p = glm::ivec3(pos) >> (level - 1);
		const int32_t childId = (p.x & 1) | ((p.y & 1) << 1) | ((p.y & 1) << 2);
		nd.childrenInNodesCounts[childId]++;
	}
}

void HashLooseOctree::Remove(EntityType entity)
{
	const int32_t offset = data.GetOffset(entity);
	const Aabb aabb = data[offset].aabb;
	data[offset] = {};
	int32_t level = CalcHashMinLevel(aabb);

	const glm::ivec3 pos = aabb.GetCenter() * invResolution;

	{
		NodeData &nd = nodes[Key(this, pos, level)];

		nd.directChildrenCount--;
		if (data[offset].prev >= 0) {
			data[data[offset].prev].next = data[offset].next;
		} else {
			nd.firstChild = data[offset].next;
		}
		if (data[offset].next >= 0) {
			data[data[offset].next].prev = data[offset].prev;
		}

		if (nd.firstChild == -1 || nd.HasIndirectChildren() == 0) {
			nodes.erase(Key(this, pos, level));
		}
	}

	for (++level; level <= levels; ++level) {
		NodeData &nd = nodes[Key(this, pos, level)];
		glm::ivec3 op = glm::ivec3(pos) >> (level - 1);
		const int32_t oldChildId =
			(op.x & 1) | ((op.y & 1) << 1) | ((op.y & 1) << 2);
		nd.childrenInNodesCounts[oldChildId]--;
		if (nd.firstChild == -1 || nd.HasIndirectChildren() == 0) {
			nodes.erase(Key(this, pos, level));
		}
	}

	data.RemoveByKey(entity);
}

void HashLooseOctree::SetMask(EntityType entity, MaskType mask)
{
	const int32_t offset = data.GetOffset(entity);
	data[offset].mask = mask;
}

Aabb HashLooseOctree::GetAabb(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	if (offset > 0) {
		return data[offset].aabb;
	}
	return {};
}

MaskType HashLooseOctree::GetMask(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	if (offset > 0) {
		return data[offset].mask;
	}
	return 0;
}

/*
Aabb HashLooseOctree::CalcAabbOfNode(glm::ivec3 pos, int32_t level) {
}
*/

void HashLooseOctree::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}
	cb.broadphase = this;
	
	const int32_t baseLevel = CalcHashMinLevel(cb.aabb);
	const glm::ivec3 ipos = cb.aabb.GetCenter() * invResolution;

	_Internal_IntersectAabb(cb, Key(this, ipos, baseLevel));
}

void HashLooseOctree::_Internal_IntersectAabb(IntersectionCallback &cb,
											  const int32_t node)
{
	if (node <= 0) {
		return;
	} else if (node <= OFFSET) {
		if (nodes[node].mask & cb.mask) {
			++cb.nodesTestedCount;
			for (int i = 0; i < 2; ++i) {
				if (nodes[node].aabb[i] && cb.aabb) {
					_Internal_IntersectAabb(cb, nodes[node].children[i]);
				}
			}
		}
	} else if (data[node - OFFSET].mask & cb.mask) {
		++cb.nodesTestedCount;
		if (data[node - OFFSET].aabb && cb.aabb) {
			++cb.testedCount;
			cb.callback(&cb, data[node - OFFSET].entity);
		}
	}
}

/*
void HashLooseOctree::IntersectRay(RayCallback &cb)
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

	_Internal_IntersectRay(cb, rootNode);
}

void HashLooseOctree::_Internal_IntersectRay(RayCallback &cb,
											 const int32_t node)
{
	if (node <= 0) {
		return;
	} else if (node < OFFSET) {
		if (nodes[node].mask & cb.mask) {
			float __n[2], __f[2];
			int __has = 0;
			for (int i = 0; i < 2; ++i) {
				++cb.nodesTestedCount;
				if (nodes[node].aabb[i].FastRayTest(cb.start, cb.dirNormalized,
													cb.invDir, cb.length,
													__n[i], __f[i])) {
					if (__n[i] < 0.0f)
						__n[i] = 0.0f;
					__has += i + 1;
				}
			}
			switch (__has) {
			case 1:
				_Internal_IntersectRay(cb, nodes[node].children[0]);
				break;
			case 2:
				_Internal_IntersectRay(cb, nodes[node].children[1]);
				break;
			case 3:
				if (__n[1] < __n[0]) {
					_Internal_IntersectRay(cb, nodes[node].children[1]);
					if (__n[0] < cb.length) {
						_Internal_IntersectRay(cb, nodes[node].children[0]);
					}
				} else {
					_Internal_IntersectRay(cb, nodes[node].children[0]);
					if (__n[1] < cb.length) {
						_Internal_IntersectRay(cb, nodes[node].children[1]);
					}
				}
				break;
			}
		}
	} else {
		const int32_t offset = node - OFFSET;
		if (data[offset].mask & cb.mask) {
			++cb.nodesTestedCount;
			float _n, _f;
			if (data[offset].aabb.FastRayTest(cb.start, cb.dirNormalized,
											  cb.invDir, cb.length, _n, _f)) {
				++cb.testedCount;
				auto res = cb.callback(&cb, data[offset].entity);
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
}

void HashLooseOctree::Rebuild() { FastRebalance(); }

void HashLooseOctree::FastRebalance()
{
	fastRebalance = false;

	RebalanceNodesRecursively(rootNode, -1);
}

void HashLooseOctree::RebalanceUpToRoot(int32_t node, int32_t rebalancingDepth)
{
	while (node > 0 && node != rootNode) {
		RebalanceNodesRecursively(node, rebalancingDepth);
		node = nodes[node].parent;
	}
}

void HashLooseOctree::RebalanceNodesRecursively(int32_t node, int32_t depth)
{
	if (depth == 0) {
		return;
	}

	if (node <= 0) { // leaf
		assert(!"should not happen");
		return;
	} else if (node > OFFSET) { // leaf
		return;
	}

	DoBestNodeRotation(node);

	for (int i = 0; i < 2; ++i) {
		RebalanceNodesRecursively(nodes[node].children[i], depth - 1);
	}
}

void HashLooseOctree::DoBestNodeRotation(int32_t node)
{
	if (node <= 0) { // leaf
		assert(!"should not happen");
		return;
	} else if (node > OFFSET) { // leaf
		return;
	}

	const static int32_t rotations[7][2] = {
		{0b0100, 0b1000}, {0b0100, 0b1010}, {0b0100, 0b1011}, {0b0110, 0b1000},
		{0b0111, 0b1000}, {0b0111, 0b1010}, {0b0111, 0b1011}};
	float minRotationValue;
	GetRotationIntersectionVolume(node, rotations[0][0], rotations[0][1],
								  &minRotationValue);
	int32_t bestRotationId = 0;
	for (int32_t i = 1; i < 7; ++i) {
		float v = 100000.0f;
		if (GetRotationIntersectionVolume(node, rotations[i][0],
										  rotations[i][1], &v)) {
			if (v < minRotationValue) {
				bestRotationId = i;
			}
		}
	}

	if (bestRotationId > 0) {
		DoRotation(node, rotations[bestRotationId][0],
				   rotations[bestRotationId][1]);
	}
}

bool HashLooseOctree::GetRotationIntersectionVolume(int32_t parentNode,
													int32_t lId, int32_t rId,
													float *resultValue) const
{
	if ((lId | rId) == 0b1100 || lId == 0 || rId == 0) {
		const NodeData &n = nodes[parentNode];
		if (n.aabb[0] && n.aabb[1]) {
			*resultValue = n.aabb[0].GetVolume() + n.aabb[1].GetVolume();
		} else {
			*resultValue = -1.0f;
		}
		return true;
	}

	int32_t leftId, leftParentId, leftOffset;
	if (GetNodeOffsetsAndInfo(parentNode, lId, &leftId, &leftParentId,
							  &leftOffset) == false) {
		return false;
	}
	int32_t rightId, rightParentId, rightOffset;
	if (GetNodeOffsetsAndInfo(parentNode, lId, &rightId, &rightParentId,
							  &rightOffset) == false) {
		return false;
	}

	const NodeData *root = &nodes[parentNode];
	const NodeData *leftParent = &nodes[leftParentId];
	const NodeData *rightParent = &nodes[rightParentId];

	Aabb leftAabb = leftParent->aabb[leftOffset];
	Aabb rightAabb = rightParent->aabb[rightOffset];

	if (leftParent == root) {
		Aabb otherAabb = rightParent->aabb[rightOffset ^ 1];
		rightAabb = rightAabb + otherAabb;
	} else if (rightParent == root) {
		Aabb otherAabb = leftParent->aabb[leftOffset ^ 1];
		rightAabb = leftAabb + otherAabb;
	}

	if (leftAabb && rightAabb) {
		*resultValue = leftAabb.GetVolume() + rightAabb.GetVolume();
	} else {
		*resultValue = -1.0f;
	}

	return true;
}

void HashLooseOctree::DoRotation(int32_t parentNode, int32_t lId, int32_t rId)
{
	if ((lId | rId) == 0b1100 || lId == 0 || rId == 0) {
		return;
	}

	int32_t leftId, leftParentId, leftOffset;
	if (GetNodeOffsetsAndInfo(parentNode, lId, &leftId, &leftParentId,
							  &leftOffset) == false) {
		assert(!"Should not happen");
		return;
	}
	int32_t rightId, rightParentId, rightOffset;
	if (GetNodeOffsetsAndInfo(parentNode, lId, &rightId, &rightParentId,
							  &rightOffset) == false) {
		assert(!"Should not happen");
		return;
	}

	NodeData *const root = &nodes[parentNode];
	NodeData *const leftParent = &nodes[leftParentId];
	NodeData *const rightParent = &nodes[rightParentId];

	std::swap(leftParent->children[leftOffset],
			  rightParent->children[rightOffset]);
	std::swap(leftParent->aabb[leftOffset], rightParent->aabb[rightOffset]);
	SetParent(leftId, rightParentId);
	SetParent(rightId, leftParentId);

	if (leftParent == root) {
		leftParent->aabb[leftOffset ^ 1] =
			GetDirectAabb(leftParent->children[leftOffset ^ 1]);
		rightParent->mask = GetIndirectMask(rightParentId);
		leftParent->mask = GetIndirectMask(leftParentId);
	} else if (rightParent == root) {
		rightParent->aabb[rightOffset ^ 1] =
			GetDirectAabb(rightParent->children[rightOffset ^ 1]);
		leftParent->mask = GetIndirectMask(leftParentId);
		rightParent->mask = GetIndirectMask(rightParentId);
	} else {
		leftParent->mask = GetIndirectMask(leftParentId);
		rightParent->mask = GetIndirectMask(rightParentId);
	}
}

void HashLooseOctree::SetParent(int32_t node, int32_t parent)
{
	if (node <= 0) {
		assert(!"Should not happen");
	} else if (node < OFFSET) {
		nodes[node].parent = parent;
	} else {
		data[node - OFFSET].parent = parent;
	}
}

bool HashLooseOctree::GetNodeOffsetsAndInfo(int32_t rootNodeId, int32_t id,
											int32_t *nodeId,
											int32_t *parentNodeId,
											int32_t *childIdOfParent) const
{
	NodeData const *root = &nodes[rootNodeId];
	NodeData const *child = nullptr;

	if (id & 0b0100) {
		if (id == 0b0100) {
			*parentNodeId = rootNodeId;
			*nodeId = root->children[0];
			*childIdOfParent = 0;
			return true;
		}
		*parentNodeId = root->children[0];
		if (*parentNodeId <= 0 || *parentNodeId > OFFSET) {
			return false;
		}
		child = &nodes[*parentNodeId];
	} else if (id & 0b1000) {
		if (id == 0b1000) {
			*parentNodeId = rootNodeId;
			*nodeId = root->children[1];
			*childIdOfParent = 1;
			return true;
		}
		*parentNodeId = root->children[1];
		if (*parentNodeId <= 0 || *parentNodeId > OFFSET) {
			return false;
		}
		child = &nodes[*parentNodeId];
	} else {
		*nodeId = rootNodeId;
		*parentNodeId = 0;
		*childIdOfParent = 0;
		return true;
	}

	*nodeId = child->children[id & 1];
	*childIdOfParent = id & 1;
	return true;
}

int32_t HashLooseOctree::GetIndirectMask(int32_t node) const
{
	if (node <= 0) {
		return 0;
	} else if (node < OFFSET) {
		int32_t mask = 0;
		for (int i = 0; i < 2; ++i) {
			mask |= GetDirectMask(nodes[node].children[i]);
		}
		return mask;
	} else {
		return data[node - OFFSET].mask;
	}
}

int32_t HashLooseOctree::GetDirectMask(int32_t node) const
{
	if (node <= 0) {
		return 0;
	} else if (node > OFFSET) {
		return data[node - OFFSET].mask;
	} else {
		return nodes[node].mask;
	}
}

Aabb HashLooseOctree::GetIndirectAabb(int32_t node) const
{
	if (node <= 0) {
		assert(!"cannot happen");
		return {};
	} else if (node > OFFSET) {
		AabbCentered r = data[node - OFFSET].aabb;
		r.halfSize += glm::vec3{1, 1, 1};
		return r;
	} else {
		Aabb l = GetDirectAabb(nodes[node].children[0]);
		Aabb r = GetDirectAabb(nodes[node].children[1]);
		if (nodes[node].children[0] > 0) {
			if (nodes[node].children[1] > 0) {
				return l + r;
			} else {
				return l;
			}
		} else if (nodes[node].children[1] > 0) {
			return r;
		}
	}
	return {};
}

Aabb HashLooseOctree::GetDirectAabb(int32_t node) const
{
	if (node <= 0) {
		assert(!"cannot happen");
		return {};
	} else if (node > OFFSET) {
		AabbCentered r = data[node - OFFSET].aabb;
		r.halfSize += glm::vec3{1, 1, 1};
		return r;
	} else {
		if (nodes[node].children[0] > 0) {
			if (nodes[node].children[1] > 0) {
				return nodes[node].aabb[0] + nodes[node].aabb[1];
			} else {
				return nodes[node].aabb[0];
			}
		} else if (nodes[node].children[1] > 0) {
			return nodes[node].aabb[1];
		}
	}
	return {};
}

void HashLooseOctree::UpdateAabb(const int32_t nodeId)
{
	if (nodeId > OFFSET) {
		UpdateAabb(data[nodeId - OFFSET].parent);
		return;
	}
	for (int i = 0; i < 2; ++i) {
		nodes[nodeId].aabb[i] = GetDirectAabb(nodes[nodeId].children[i]);
	}
	Aabb aabb = nodes[nodeId].aabb[0] + nodes[nodeId].aabb[1];
	int32_t id = nodes[nodeId].parent;
	int32_t childId = nodeId;
	while (id > 0) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		if (((Aabb)nodes[id].aabb[i]).ContainsAll(aabb)) {
			return;
		}
		nodes[id].aabb[i] = aabb;
		aabb = aabb + nodes[id].aabb[i ^ 1];
		RebalanceNodesRecursively(id, 1);
		childId = id;
		id = nodes[childId].parent;
	}
}

void HashLooseOctree::UpdateAabbSimple(const int32_t nodeId)
{
	if (nodeId > OFFSET) {
		UpdateAabbSimple(data[nodeId - OFFSET].parent);
		return;
	}
	for (int i = 0; i < 2; ++i) {
		nodes[nodeId].aabb[i] = GetDirectAabb(nodes[nodeId].children[i]);
	}
	Aabb aabb = nodes[nodeId].aabb[0] + nodes[nodeId].aabb[1];
	int32_t id = nodes[nodeId].parent;
	int32_t childId = nodeId;
	while (id > 0) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		if (((Aabb)nodes[id].aabb[i]).ContainsAll(aabb)) {
			return;
		}
		nodes[id].aabb[i] = aabb;
		aabb = aabb + nodes[id].aabb[i ^ 1];
		childId = id;
		id = nodes[childId].parent;
	}
}

void HashLooseOctree::UpdateMask(const int32_t nodeId)
{
	if (nodeId > OFFSET) {
		UpdateMask(data[nodeId - OFFSET].parent);
		return;
	}
	MaskType mask = 0;
	for (int i = 0; i < 2; ++i) {
		mask |= GetDirectMask(nodes[nodeId].children[i]);
	}
	int32_t id = nodes[nodeId].parent;
	int32_t childId = nodeId;
	while (id > 0) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		MaskType m2 = GetDirectMask(nodes[id].children[i ^ 1]);
		if (m2 == mask) {
			break;
		}
		mask |= m2;
		nodes[id].mask = mask;
		childId = id;
		id = nodes[childId].parent;
	}
}

void HashLooseOctree::UpdateAabbAndMask(const int32_t nodeId)
{
	if (nodeId > OFFSET) {
		UpdateAabbAndMask(data[nodeId - OFFSET].parent);
		return;
	}
	MaskType mask = 0;
	for (int i = 0; i < 2; ++i) {
		nodes[nodeId].aabb[i] = GetDirectAabb(nodes[nodeId].children[i]);
		mask |= GetDirectMask(nodes[nodeId].children[i]);
	}
	Aabb aabb = nodes[nodeId].aabb[0] + nodes[nodeId].aabb[1];
	int32_t id = nodes[nodeId].parent;
	int32_t childId = nodeId;
	while (id > 0) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		MaskType m2 = GetDirectMask(nodes[id].children[i ^ 1]);
		if (m2 == mask && ((Aabb)nodes[id].aabb[i]).ContainsAll(aabb)) {
			return;
		}
		mask |= m2;
		nodes[id].mask = mask;
		nodes[id].aabb[i] = aabb;
		aabb = aabb + nodes[id].aabb[i ^ 1];
		RebalanceNodesRecursively(id, 1);
		childId = id;
		id = nodes[childId].parent;
	}
}
*/
} // namespace spp

#endif
