// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>

#include "../include/spatial_partitioning/Dbvh.hpp"

namespace spp
{
Dbvh::Dbvh() { Clear(); }
Dbvh::~Dbvh() {}

const char *Dbvh::GetName() const { return "Dbvh"; }

void Dbvh::Clear()
{
	data.Clear();
	nodes.Clear();
	rootNode = nodes.Add({});
	fastRebalance = false;
}

size_t Dbvh::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.GetMemoryUsage();
}

void Dbvh::ShrinkToFit()
{
	data.ShrinkToFit();
	nodes.ShrinkToFit();
}

void Dbvh::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(rootNode != 0);
	
	const int32_t offset = data.Add(entity, {aabb, entity, mask});

	for (int32_t i = 0; i < 2; ++i) {
		if (nodes[rootNode].children[i] <= 0) {
			nodes[rootNode].children[i] = offset + OFFSET;
			nodes[rootNode].aabb[i] = aabb;
			nodes[rootNode].mask |= mask;
			data[offset].parent = rootNode;
			return;
		}
	}

	++modificationsSinceLastRebuild;

	int32_t node = rootNode;

	for (size_t _depth = 0;; ++_depth) {
		float dv[2] = {0.0f, 0.0f};

		const int32_t parentNodeId = node;

		for (int i = 0; i < 2; ++i) {
			Aabb ab = nodes[parentNodeId].aabb[i];
			dv[i] = (ab + aabb).GetVolume();
		}

		int c = 0;
		if (dv[1] < dv[0]) {
			c = 1;
		}

		const int32_t n = nodes[parentNodeId].children[c];

		if (n <= 0) {
			assert(!"Should not happen");
		} else if (n < OFFSET) {
			nodes[parentNodeId].mask |= mask;
			nodes[parentNodeId].aabb[c] = nodes[parentNodeId].aabb[c] + aabb;
			node = nodes[parentNodeId].children[c];
			continue;
		} else {
			const int32_t otherChildId = n;

			const int32_t newNodeId = nodes.Add({});
			NodeData &parentNode = nodes[parentNodeId];
			NodeData &newNode = nodes[newNodeId];

			newNode.parent = parentNodeId;
			newNode.mask = parentNode.mask | mask;

			newNode.children[0] = parentNode.children[c];
			newNode.aabb[0] = parentNode.aabb[c];

			newNode.children[1] = offset + OFFSET;
			newNode.aabb[1] = aabb;

			parentNode.mask |= mask;
			parentNode.aabb[c] = parentNode.aabb[c] + aabb;
			parentNode.children[c] = newNodeId;

			SetParent(otherChildId, newNodeId);
			data[offset].parent = newNodeId;

			/*
			if (int32_t c = CountEntities() - data.Size()) {
				printf("                      $$$$$$$$$$$$$$$$$$$$$$$$$$$$ "
					   "tree count after adding before rebalance %6i =?= %6i\n",
					   c + data.Size(), data.Size());
				printf("\n");
				fflush(stdout);
			}
			*/

// 			if (_depth > 40) {
// 				RebalanceUpToRoot(newNodeId, 1);
// 			}

			/*
			if (int32_t c = CountEntities() - data.Size()) {
				printf("                      $$$$$$$$$$$$$$$$$$$$$$$$$$$$ "
					   "tree count after adding and rebalance    %6i =?= %6i\n",
					   c + data.Size(), data.Size());
				fflush(stdout);
			}
			*/

			break;
		}
	}
}

int32_t Dbvh::CountNodes() const
{
	static int32_t (*fun)(const Dbvh *s, int32_t node) =
		+[](const Dbvh *s, int32_t node) -> int32_t {
		if (node <= 0 || node > OFFSET) {
			return 0;
		}
		return 1 + fun(s, s->nodes[node].children[0]) +
			   fun(s, s->nodes[node].children[1]);
	};
	return fun(this, rootNode);
}

int32_t Dbvh::CountEntities() const
{
	static int32_t (*fun)(const Dbvh *s, int32_t node) =
		+[](const Dbvh *s, int32_t node) -> int32_t {
		if (node <= 0) {
			return 0;
		} else if (node < OFFSET) {
			return fun(s, s->nodes[node].children[0]) +
				   fun(s, s->nodes[node].children[1]);
		} else {
			return 1;
		}
	};
	return fun(this, rootNode);
}

int32_t Dbvh::CountDepth() const
{
	static int32_t (*fun)(const Dbvh *s, int32_t node) =
		+[](const Dbvh *s, int32_t node) -> int32_t {
		if (node <= 0 || node >= OFFSET) {
			return 1;
		}
		return 1 + std::max(fun(s, s->nodes[node].children[0]),
							fun(s, s->nodes[node].children[1]));
	};
	return fun(this, rootNode);
}

void Dbvh::Update(EntityType entity, Aabb aabb)
{
	++modificationsSinceLastRebuild;

	int32_t offset = data.GetOffset(entity);
	data[offset].aabb = aabb;
	UpdateAabb(offset);
// 	RebalanceUpToRoot(data[offset].parent, 1);
}

void Dbvh::Remove(EntityType entity)
{
	assert(!"Not implemented");

	++modificationsSinceLastRebuild;

	const int32_t offset = data.GetOffset(entity);
	const int32_t id = data[offset].parent;
	data.RemoveByKey(entity);
	const int32_t childId = offset + OFFSET;

	const int i1 = nodes[id].children[0] == childId ? 1 : 0;

	const Aabb aabb = nodes[id].aabb[i1];
	const int32_t child2 = nodes[id].children[i1];

	if (id == rootNode) {

	} else {
	}

	/*
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
	*/
}

void Dbvh::SetMask(EntityType entity, MaskType mask)
{
	assert(!"Not implemented");
	/*
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
	*/
}

Aabb Dbvh::GetAabb(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	if (offset > 0) {
		return data[offset].aabb;
	}
	return {};
}

MaskType Dbvh::GetMask(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	if (offset > 0) {
		return data[offset].mask;
	}
	return 0;
}

void Dbvh::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}
	cb.broadphase = this;
	_Internal_IntersectAabb(cb, rootNode);
}

void Dbvh::_Internal_IntersectAabb(IntersectionCallback &cb, const int32_t node)
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
	} else {
		if (data[node - OFFSET].mask & cb.mask) {
			++cb.nodesTestedCount;
			++cb.testedCount;
			cb.callback(&cb, data[node - OFFSET].entity);
		}
	}
}

void Dbvh::IntersectRay(RayCallback &cb)
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

void Dbvh::_Internal_IntersectRay(RayCallback &cb, const int32_t node)
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

void Dbvh::Rebuild()
{
	printf(" Bdvh: depth of tree = %i\n", CountDepth());
	FastRebalance();
}

void Dbvh::FastRebalance()
{
	fastRebalance = false;

	RebalanceNodesRecursively(rootNode, -1);
}

void Dbvh::RebalanceUpToRoot(int32_t node, int32_t rebalancingDepth)
{
	while (node > 0 && node != rootNode) {
		RebalanceNodesRecursively(node, rebalancingDepth);
		node = nodes[node].parent;
	}
}

void Dbvh::RebalanceNodesRecursively(int32_t node, int32_t depth)
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

void Dbvh::DoBestNodeRotation(int32_t node)
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

bool Dbvh::GetRotationIntersectionVolume(int32_t parentNode, int32_t lId,
										 int32_t rId, float *resultValue) const
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

void Dbvh::DoRotation(int32_t parentNode, int32_t lId, int32_t rId)
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

void Dbvh::SetParent(int32_t node, int32_t parent)
{
	if (node <= 0) {
		assert(!"Should not happen");
	} else if (node < OFFSET) {
		nodes[node].parent = parent;
	} else {
		data[node - OFFSET].parent = parent;
	}
}

bool Dbvh::GetNodeOffsetsAndInfo(int32_t rootNodeId, int32_t id,
								 int32_t *nodeId, int32_t *parentNodeId,
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

int32_t Dbvh::GetIndirectMask(int32_t node) const
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

int32_t Dbvh::GetDirectMask(int32_t node) const
{
	if (node <= 0) {
		return 0;
	} else if (node > OFFSET) {
		return data[node - OFFSET].mask;
	} else {
		return nodes[node].mask;
	}
}

Aabb Dbvh::GetIndirectAabb(int32_t node) const
{
	if (node <= 0) {
		assert(!"cannot happen");
		return {};
	} else if (node > OFFSET) {
		return data[node - OFFSET].aabb;
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

Aabb Dbvh::GetDirectAabb(int32_t node) const
{
	if (node <= 0) {
		assert(!"cannot happen");
		return {};
	} else if (node > OFFSET) {
		return data[node - OFFSET].aabb;
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

/*
void Dbvh::Rebuild()
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

void Dbvh::RebuildNode(int32_t nodeId)
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
			return l.aabb.center.x < r.aabb.center.x;
		}
		inline static bool SortY(const Data &l, const Data &r)
		{
			return l.aabb.center.y < r.aabb.center.y;
		}
		inline static bool SortZ(const Data &l, const Data &r)
		{
			return l.aabb.center.z < r.aabb.center.z;
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

void Dbvh::PruneEmptyEntitiesAtEnd()
{
	for (int32_t i = entitiesData.size() - 1; i >= 0; --i) {
		if (entitiesData[i].entity != EMPTY_ENTITY) {
			entitiesData.resize(i + 1);
			return;
		}
	}
}
*/

void Dbvh::UpdateAabb(const int32_t offset)
{
	Aabb aabb = data[offset].aabb;
	int32_t id = data[offset].parent;
	int32_t childId = offset + OFFSET;
	while (id > 0) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		nodes[id].aabb[i] = aabb;
		aabb = aabb + nodes[id].aabb[i^1];
		RebalanceNodesRecursively(id, 2);
		childId = id;
		id = nodes[childId].parent;
	}
}
} // namespace spp
