// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cassert>
#include <cstring>

#include "../include/spatial_partitioning/LooseOctree.hpp"

namespace spp
{
LooseOctree::LooseOctree(glm::vec3 centerOffset, float sizeScale,
						 int32_t levels, float loosnessFactor)
	: data(), nodes(), offset(centerOffset), scale(sizeScale), levels(levels),
	  loosnessFactor(loosnessFactor), invLoosenessFactor(1.0f / loosnessFactor),
	  maxExtent(1 << (levels - 1)), margin((loosnessFactor - 1.0) / 2.0)
{
	Clear();
}
LooseOctree::~LooseOctree() {}

const char *LooseOctree::GetName() const { return "LooseOctree"; }

void LooseOctree::Clear()
{
	data.Clear();
	nodes.Clear();
	rootNode = nodes.Add({});
	nodes[rootNode].level = levels;
	nodes[rootNode].pos = offset;
}

size_t LooseOctree::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.GetMemoryUsage();
}

void LooseOctree::ShrinkToFit()
{
	data.ShrinkToFit();
	nodes.ShrinkToFit();
}

void LooseOctree::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	const int32_t offset = data.Add(entity, {aabb, entity, mask});
	data[offset].prev = -1;
	const IPosLevel id = CalcIPosLevel(aabb);
	const int32_t nodeId = GetNodeIdAt(id.ipos, id.level);
	
	const int32_t nextEntity = nodes[nodeId].firstEntity;
	data[offset].next = nextEntity;
	if (nextEntity >= 0) {
		data[nextEntity].prev = offset;
	}
	nodes[nodeId].firstEntity = offset;
	data[offset].parent = nodeId;
}

void LooseOctree::Update(EntityType entity, Aabb aabb)
{
	const int32_t offset = data.GetOffset(entity);
	const IPosLevel id = CalcIPosLevel(aabb);
	if (GetAabbOfNode(data[offset].parent).ContainsAll(aabb)) {
		return;
	}

	const int32_t nodeId = GetNodeIdAt(id.ipos, id.level);
	const int32_t nextEntity = nodes[nodeId].firstEntity;
	nodes[nodeId].firstEntity = offset;

	RemoveStructureFor(offset);

	data[offset].next = nextEntity;
	if (nextEntity >= 0) {
		data[nextEntity].prev = offset;
	}
	data[offset].parent = nodeId;
}

void LooseOctree::Remove(EntityType entity)
{
	const int32_t offset = data.GetOffset(entity);
	RemoveStructureFor(offset);
	data.RemoveByKey(entity);
}

LooseOctree::IPosLevel LooseOctree::CalcIPosLevel(Aabb aabb) const
{
#define max_comp(V) glm::max(V.x, glm::max(V.y, V.z))
	const glm::vec3 sizes = aabb.GetSizes() * scale;
	const float size = max_comp(sizes);
	const glm::vec3 center = aabb.GetCenter() * scale - this->offset;

	const int32_t level = std::bit_width<uint32_t>(
		std::bit_ceil<uint32_t>(size / (loosnessFactor - 1)));

	const glm::ivec3 ipos = center;
	return {ipos, level};
}

Aabb LooseOctree::GetAabbOfNode(int32_t nodeId) const
{
	glm::vec3 min = glm::vec3(nodes[nodeId].pos) - margin;
	float size = 1 << nodes[nodeId].level;
	glm::vec3 max = min + size * loosnessFactor;
	return {min, max};
}

int32_t LooseOctree::GetNodeIdAt(glm::ivec3 pos, const int32_t level)
{
	if (level > levels) {
		return rootNode;
	}
	const glm::ivec3 pp = glm::abs(pos);
	if (max_comp(pp) > maxExtent) {
		return rootNode;
	}
	
	int32_t i = levels;
	int32_t n = rootNode;
	
	for (; i>level; --i) {
		int32_t cid = CalcChildId(nodes[n].pos, pos, i-1);
		int32_t c = nodes[n].children[cid];
		if (c < 0) {
			nodes[n].children[cid] = 
			c = nodes.Add({});
			nodes[c].level = i-1;
			nodes[c].pos = (pos>>(i-1))<<(i-1);
		}
		n = c;
	}
	return n;
}

void LooseOctree::RemoveStructureFor(int32_t offset)
{
	int32_t n = data[offset].parent;

	const int32_t prev = data[offset].prev;
	const int32_t next = data[offset].next;

	if (prev >= 0) {
		data[prev].next = next;
	} else {
		nodes[n].firstEntity = next;
	}
	if (next >= 0) {
		data[next].prev = prev;
	}

	while (n >= 0 && nodes[n].parentId >= 0) {
		if (nodes[n].HasData()) {
			break;
		}
		const int32_t parentId = nodes[n].parentId;
		if (nodes[n].firstEntity < 0) {
			const int32_t childId = CalcChildId(nodes[parentId].pos, nodes[n].pos, nodes[n].level);
			assert(nodes[parentId].children[childId] == n && "Failed calculating child id");
			nodes[parentId].children[childId] = -1;
			nodes.Remove(n);
		} else {
			return;
		}
		n = parentId;
	}
}

int32_t LooseOctree::CalcChildId(glm::ivec3 parentPos, glm::ivec3 childPos, int32_t childLevel)
{
	glm::ivec3 p = (parentPos - childPos) >> childLevel;
	return (p.x&1) | ((p.y>>1)&1) | ((p.z>>2)&1);
}

bool LooseOctree::NodeData::HasData() const
{
	if (firstEntity >= 0) {
		return true;
	}
	for (int32_t i = 0; i < 8; ++i) {
		if (children[i] >= 0) {
			return true;
		}
	}
	return false;
}

void LooseOctree::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = data.GetOffset(entity);
	data[offset].mask = mask;

	// TODO: update masks of tree
}

Aabb LooseOctree::GetAabb(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	return data[offset].aabb;
}

MaskType LooseOctree::GetMask(EntityType entity) const
{
	int32_t offset = data.GetOffset(entity);
	return data[offset].mask;
}

void LooseOctree::Rebuild() {}

void LooseOctree::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	
	Aabb cbaabb = cb.aabb;
	cbaabb.min *= scale;
	cbaabb.max *= scale;
	
	_Internal_IntersectAabb(cb, rootNode, cbaabb);
}

void LooseOctree::_Internal_IntersectAabb(IntersectionCallback &cb,
		const int32_t n, const Aabb &cbaabb)
{
	++cb.nodesTestedCount;
	if (n == rootNode || (GetAabbOfNode(n) && cbaabb)) {
		for (int32_t c = nodes[n].firstEntity; c >= 0; c = data[c].next) {
			++cb.nodesTestedCount;
			if (data[c].mask & cb.mask) {
				if (data[c].aabb && cbaabb) {
					cb.callback(&cb, data[c].entity);
					++cb.testedCount;
				}
			}
		}
		for (int32_t i=0; i<8; ++i) {
			const int32_t c = nodes[n].children[i];
			if (c >= 0) {
				_Internal_IntersectAabb(cb, c, cbaabb);
			}
		}
	}
}

void LooseOctree::IntersectRay(RayCallback &cb)
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
	
	_Internal_IntersectRay(cb, rootNode, levels);
}

void LooseOctree::_Internal_IntersectRay(RayCallback &cb, const int32_t n, int32_t level)
{
	const Aabb aabb = GetAabbOfNode(n);
	
	float __n, __f;
	if (n != rootNode && !aabb.FastRayTest(cb.start*scale, cb.dirNormalized, cb.invDir,
				cb.length*scale, __n, __f)) {
		return;
	}
	++cb.nodesTestedCount;
	
	for (int32_t c = nodes[n].firstEntity; c >= 0; c = data[c].next) {
		Data &N = data[c];
		++cb.testedCount;
		if (N.mask & cb.mask) {
			if (N.aabb.FastRayTest(cb.start*scale, cb.dirNormalized,
						cb.invDir, cb.length*scale,
						__n, __f)) {
				auto res = cb.callback(&cb, N.entity);
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
	
	if (level == 0) {
		return;
	}
	
	struct Ords {
		float near;
		int32_t n;
	} ords[8];
	int32_t ordsCount = 0;
	
	for (int32_t i=0; i<8; ++i) {
		const int32_t c = nodes[n].children[i];
		if (c < 0) {
			continue;
		}
		
		const glm::ivec3 is = {i&1,(i<<1)&1,(i<<2)&1};
		
		if (GetAabbOfNode(c).FastRayTest(cb.start*scale, cb.dirNormalized,
					cb.invDir, cb.length*scale,
					__n, __f)) {
			
			int j=0;
			for (; i<ordsCount; ++j) {
				if (ords[j].near > __n) {
					break;
				}
			}
			if (ordsCount != j) {
				memmove(ords+j+1, ords+j, (ordsCount-j)*sizeof(Ords));
			}
			ords[j] = {__n*cb.length, nodes[n].children[i]};
		}
	}
	
	for (int32_t _i=0; _i<ordsCount; ++_i) {
		int32_t c = ords[_i].n;
		if (ords[_i].near < cb.length) {
			_Internal_IntersectRay(cb, c, level - 1);
		} else {
			return;
		}
	}
}
} // namespace spp
