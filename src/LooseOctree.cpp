// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cassert>
#include <cstring>

#include "../../thirdparty/glm/glm/ext/vector_uint3.hpp"
#include "../../thirdparty/glm/glm/vector_relational.hpp"

#include "../include/spatial_partitioning/LooseOctree.hpp"

namespace spp
{
LooseOctree::LooseOctree(glm::vec3 centerOffset,
						 int32_t levels, float loosnessFactor)
	: data(), nodes(), centerOffset(centerOffset), levels(levels),
	  loosnessFactor(loosnessFactor), invLoosenessFactor(1.0f / loosnessFactor),
	  maxExtent(1 << (levels - 1)), margin((loosnessFactor - 1.0) / 2.0)
{
	Clear();
}
LooseOctree::~LooseOctree() {}

const char *LooseOctree::GetName() const { return "LooseOctree"; }

void LooseOctree::Clear()
{
// 	printf("                          ^^^^^^^^^^^^^^^^^^^^^^^ nodes count: %i\n", nodes.Size());
	data.Clear();
	nodes.Clear();
	rootNode = nodes.Add({});
	nodes[rootNode].level = levels;
	nodes[rootNode].center = centerOffset;
}

size_t LooseOctree::GetMemoryUsage() const
{
	return data.GetMemoryUsage() + nodes.GetMemoryUsage();
}

void LooseOctree::ShrinkToFit()
{
	data.ShrinkToFit();
	nodes.ShrinkToFit();
// 	printf("                          ^^^^^^^^^^^^^^^^^^^^^^^ nodes count: %i\n", nodes.Size());
}

void LooseOctree::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	const int32_t did = data.Add(entity, {aabb, entity, mask});
	data[did].prev = 0;
	const int32_t nodeId = GetNodeIdAt(aabb);
	
	const int32_t nextEntity = nodes[nodeId].firstEntity;
	data[did].next = nextEntity;
	if (nextEntity) {
		data[nextEntity].prev = did;
	}
	nodes[nodeId].firstEntity = did;
	data[did].parent = nodeId;
}

void LooseOctree::Update(EntityType entity, Aabb aabb)
{
	const int32_t did = data.GetOffset(entity);
	if (GetAabbOfNode(data[did].parent).ContainsAll(aabb)) {
		return;
	}

	const int32_t nodeId = GetNodeIdAt(aabb);
	
	if (nodeId == data[did].parent) {
		// TODO: make tests/checks, because this should not happen
		return;
	}
	
	const int32_t nextEntity = nodes[nodeId].firstEntity;
	nodes[nodeId].firstEntity = did;

	RemoveStructureFor(did);

	data[did].next = nextEntity;
	if (nextEntity) {
		data[nextEntity].prev = did;
	}
	data[did].prev = 0;
	data[did].parent = nodeId;
	data[did].aabb = aabb;
}

void LooseOctree::Remove(EntityType entity)
{
	const int32_t did = data.GetOffset(entity);
	RemoveStructureFor(did);
	data.RemoveByKey(entity);
}

LooseOctree::IPosLevel LooseOctree::CalcIPosLevel(Aabb aabb) const
{
#define max_comp(V) glm::max(V.x, glm::max(V.y, V.z))
	const glm::vec3 sizes = (aabb.max - aabb.min) * 0.5f;
	const float size = max_comp(sizes);
	const glm::vec3 center = ((aabb.max + aabb.min) * 0.5f);

	const int32_t level = std::bit_width<uint32_t>(
		std::bit_ceil<uint32_t>(glm::ceil(size / (loosnessFactor - 1.0f))));
	
	glm::ivec3 c = center;
	c = ((c>>level)<<level);
	
	const IPosLevel ret{c, level};

	printf("Calc IPos = {{%i %i %i} %i}    for  %f.1 %.1f %.1f  (%.1f)\n",
			ret.ipos.x, ret.ipos.y, ret.ipos.z, ret.level,
			center.x, center.y, center.z, size);
	
	return ret;
}

AabbCentered LooseOctree::GetAabbOfNode(int32_t nodeId) const
{
	glm::vec3 center = nodes[nodeId].center;
	float extent = (1 << (nodes[nodeId].level-1)) * loosnessFactor;
	return {center, glm::vec3(extent, extent, extent)};
}

int32_t LooseOctree::GetNodeIdAt(Aabb aabb)
{
	const IPosLevel id = CalcIPosLevel(aabb);
	
	if (id.level > levels) {
		return rootNode;
	}
	const glm::ivec3 pp = glm::abs(id.ipos);
	if (max_comp(pp) > maxExtent) {
		return rootNode;
	}
	
	int32_t i = levels;
	int32_t n = rootNode;
	
	for (; i>id.level; --i) {
		int32_t cid = CalcChildId(nodes[n].center, id.ipos, i-1);
		int32_t c = nodes[n].children[cid];
		if (c == 0) {
			c = nodes.Add({});
			nodes[n].children[cid] = c;
			nodes[c].level = i-1;
			
			const glm::ivec3 par = nodes[n].center;
			
			
			const glm::ivec3 s = glm::sign(id.ipos - par) | glm::ivec3(1,1,1);
			const glm::ivec3 of = s * (1<<(i-1));
			
			
			nodes[c].center = nodes[n].center + of;
			printf("                          ^^^^^^^^^^^^^^^^^^^^^^^ nodes count: %i\n", nodes.Size());
			
			auto a = nodes[n].center;
			auto b = nodes[c].center;
			auto c = id.ipos;
			printf(" new child: %i %i %i -[%i]> %i %i %i   for ipos: %i %i %i\n",
					a.x, a.y, a.z,
					cid,
					b.x, b.y, b.z,
					c.x, c.y, c.z);
// 		} else {
// 			auto a = nodes[n].pos;
// 			auto b = nodes[c].pos;
// 			printf(" parent %i %i %i has child [%i] %i %i %i\n",
// 					a.x, a.y, a.z,
// 					cid,
// 					b.x, b.y, b.z);
		}
		n = c;
	}
	return n;
}

void LooseOctree::RemoveStructureFor(int32_t did)
{
	int32_t n = data[did].parent;

	const int32_t prev = data[did].prev;
	const int32_t next = data[did].next;

	if (prev) {
		data[prev].next = next;
	} else {
		nodes[n].firstEntity = next;
	}
	if (next) {
		data[next].prev = prev;
	}

	while (n && nodes[n].parentId) {
		if (nodes[n].HasData()) {
			break;
		}
		const int32_t parentId = nodes[n].parentId;
		if (nodes[n].firstEntity == 0) {
// 			printf("                          ^^^^^^^^^^^^^^^^^^^^^^^ nodes count: %i\n", nodes.Size());
			const int32_t childId = CalcChildId(nodes[parentId].center, nodes[n].center, nodes[n].level);
			assert(nodes[parentId].children[childId] == n && "Failed calculating child id");
			nodes[parentId].children[childId] = 0;
			nodes.Remove(n);
		} else {
			return;
		}
		n = parentId;
	}
}

int32_t LooseOctree::CalcChildId(glm::ivec3 parentCenter, glm::ivec3 childCenter, int32_t childLevel)
{
	const glm::ivec3 s = glm::sign(childCenter - parentCenter) | glm::ivec3(1,1,1);
	const glm::ivec3 p = (s+1)>>1;

	
// 	const glm::ivec3 p = (parentCenter - childCenter) >> childLevel;
	const int32_t ret = (p.x&1) | ((p.y<<1)&2) | ((p.z<<2)&4);
// 	printf(" calc child diff: %i %i %i -> %i         from %i %i %i  >  %i %i %i\n", p.x, p.y, p.z, ret,
// 			parentPos.x, parentPos.y, parentPos.z,
// 			childPos.x, childPos.y, childPos.z
// 			);
	return ret;
}

bool LooseOctree::NodeData::HasData() const
{
	if (firstEntity) {
		return true;
	}
	for (int32_t i = 0; i < 8; ++i) {
		if (children[i]) {
			return true;
		}
	}
	return false;
}

void LooseOctree::SetMask(EntityType entity, MaskType mask)
{
	int32_t did = data.GetOffset(entity);
	data[did].mask = mask;

	// TODO: update masks of tree
}

Aabb LooseOctree::GetAabb(EntityType entity) const
{
	int32_t did = data.GetOffset(entity);
	return data[did].aabb;
}

MaskType LooseOctree::GetMask(EntityType entity) const
{
	int32_t did = data.GetOffset(entity);
	return data[did].mask;
}

void LooseOctree::Rebuild() {
	printf("                          ^^^^^^^^^^^^^^^^^^^^^^^ nodes count: %i\n", nodes.Size());
}

void LooseOctree::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	
	_Internal_IntersectAabb(cb, rootNode);
}

void LooseOctree::_Internal_IntersectAabb(IntersectionCallback &cb,
		const int32_t n)
{
	++cb.nodesTestedCount;
	if (n == rootNode || (GetAabbOfNode(n) && cb.aabb)) {
		for (int32_t c = nodes[n].firstEntity; c; c = data[c].next) {
			++cb.nodesTestedCount;
			if (data[c].mask & cb.mask) {
				if (data[c].aabb && cb.aabb) {
					cb.callback(&cb, data[c].entity);
					++cb.testedCount;
				}
			}
		}
		for (int32_t i=0; i<8; ++i) {
			const int32_t c = nodes[n].children[i];
			if (c) {
				_Internal_IntersectAabb(cb, c);
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
	cb.dirNormalized = glm::normalize(cb.dir);
	cb.invDir = glm::vec3(1.f, 1.f, 1.f) / cb.dirNormalized;
	
	_Internal_IntersectRay(cb, rootNode, levels);
}

void LooseOctree::_Internal_IntersectRay(RayCallback &cb, const int32_t n, int32_t level)
{
	const Aabb aabb = GetAabbOfNode(n);
	
	++cb.nodesTestedCount;
	float __n, __f;
	if (n != rootNode && !aabb.FastRayTest(cb.start, cb.dirNormalized,
				cb.invDir, cb.length, __n, __f)) {
		return;
	}
	
	for (int32_t c = nodes[n].firstEntity; c; c = data[c].next) {
		Data &N = data[c];
		++cb.testedCount;
		if (N.mask & cb.mask) {
			if (N.aabb.FastRayTest(cb.start, cb.dirNormalized,
						cb.invDir, cb.length,
						__n, __f)) {
				auto res = cb.callback(&cb, N.entity);
				if (res.intersection) {
					if (res.dist + 0.00000001f < 1.0f) {
						if (res.dist < 0.0f)
							res.dist = 0.0f;
						cb.length *= res.dist;
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
		if (c == 0) {
			continue;
		}
		
		if (GetAabbOfNode(c).FastRayTest(cb.start, cb.dirNormalized,
					cb.invDir, cb.length,
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
