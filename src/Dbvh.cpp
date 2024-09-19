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
	int32_t offset = data.Add(entity, {aabb, entity, mask});
	
	for (int32_t i=0; i<2; ++i) {
		if (nodes[rootNode].children[i] <= 0) {
			nodes[rootNode].children[i] = offset + OFFSET;
			nodes[rootNode].aabb[i] = aabb;
			nodes[rootNode].mask |= mask;
			return;
		}
	}
	
	++modificationsSinceLastRebuild;

	static void (*fun)(Dbvh &, int32_t, Aabb, MaskType, int32_t) =
		+[](Dbvh &self, int32_t node, Aabb aabb,
			MaskType mask, int32_t entityOffset) -> void {
		float dv[2] = {0.0f, 0.0f};

		for (int i = 0; i < 2; ++i) {
			Aabb ab = self.nodes[node].aabb[i];
			dv[i] = (ab + aabb).GetVolume() - ab.GetVolume();
		}
		
		int c = 0;
		if (dv[1] < dv[0]) {
			c = 1;
		}
		
		const int32_t n = self.nodes[node].children[c];
		
		if (n < OFFSET) {
			self.nodes[node].mask |= mask;
			self.nodes[node].aabb[c] = self.nodes[node].aabb[c] + aabb;
			fun(self, n, aabb, mask, entityOffset);
			return;
		} else {
			const int32_t parentNodeId = node;
			NodeData &parentNode = self.nodes[parentNodeId];
			const int32_t newNodeId = self.nodes.Add({});
			NodeData &newNode = self.nodes[newNodeId];
			const int32_t otherNodeId = n;
			
			parentNode.mask |= mask;
			newNode.mask = parentNode.mask;
			
			newNode.parent = parentNodeId;
			
			newNode.children[0] = otherNodeId;
			newNode.aabb[0] = parentNode.aabb[c];
			
			parentNode.aabb[c] = parentNode.aabb[c] + aabb;
			parentNode.children[c] = newNodeId;
			
			newNode.children[1] = entityOffset;
			newNode.aabb[1] = aabb;
			return;
		}
	};
	fun(*this, rootNode, aabb, mask, offset);
}

void Dbvh::Update(EntityType entity, Aabb aabb)
{
	++modificationsSinceLastRebuild;
	
	int32_t offset = data.GetOffset(entity);
	data[offset].aabb = aabb;
	UpdateAabb(offset);
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

void Dbvh::_Internal_IntersectAabb(IntersectionCallback &cb,
												 const int32_t node)
{
	if (node <= 0) {
		return;
	} else if (node <= OFFSET) {
		if (nodes[node].mask & cb.mask) {
			++cb.nodesTestedCount;
			for (int i=0; i<2; ++i) {
				if (nodes[node].aabb[i] && cb.aabb) {
					_Internal_IntersectAabb(cb, nodes[node].children[i]);
				}
			}
		}
	} else {
		if (data[node-OFFSET].mask & cb.mask) {
			++cb.nodesTestedCount;
			++cb.testedCount;
			cb.callback(&cb, data[node-OFFSET].entity);
		}
	}
}

void Dbvh::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	Rebuild();

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
	} else if(node <= OFFSET) {
		if (nodes[node].mask & cb.mask) {
			float __n[2], __f[2];
			int __has = 0;
			for (int i = 0; i < 2; ++i) {
				++cb.nodesTestedCount;
				if (nodes[node].aabb[i].FastRayTest(
						cb.start, cb.dirNormalized, cb.invDir, cb.length,
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
			if (data[offset].aabb.FastRayTest(
						cb.start, cb.dirNormalized, cb.invDir, cb.length, _n,
						_f)) {
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

void Dbvh::UpdateAabb(int32_t offset)
{
	Aabb aabb = data[offset].aabb;
	
	int32_t id = data[offset].parent;
	int32_t childId = offset + OFFSET;
	while (id != rootNode) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		nodes[id].aabb[i] = aabb;
		aabb = aabb + nodes[id].aabb[i^1];
		childId = id;
		id = nodes[childId].parent;
	}
}

void Dbvh::UpdateNodeAabb(int32_t nodeId)
{
	int32_t id = nodes[nodeId].parent;
	int32_t childId = nodeId;
	Aabb aabb = nodes[nodeId].aabb[0] + nodes[nodeId].aabb[1];
	while (id != rootNode) {
		const int i = nodes[id].children[0] == childId ? 0 : 1;
		nodes[id].aabb[i] = aabb;
		aabb = aabb + nodes[id].aabb[i^1];
		childId = id;
		id = nodes[childId].parent;
	}
}
} // namespace spp
