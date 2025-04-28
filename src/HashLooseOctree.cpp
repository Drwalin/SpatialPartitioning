// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>
#include <cstring>

#include <bit>

#include "../../thirdparty/glm/glm/ext/vector_int3.hpp"

#include "../include/spatial_partitioning/HashLooseOctree.hpp"

namespace spp
{
namespace experimental
{
HashLooseOctree::HashLooseOctree(float resolution, int32_t levels,
								 float loosenessFactor)
	: nodes(12289, Key::Hash(this)), loosenessFactor(loosenessFactor),
	  invLoosenessFactor(1.0f / loosenessFactor), resolution(resolution),
	  invResolution(1.0f / resolution), levels(levels), iterator(*this)
{
	Clear();
}
HashLooseOctree::~HashLooseOctree() {}

void HashLooseOctree::Rebuild() {}

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

uint64_t HashLooseOctree::Hash(const glm::vec3 pos, int32_t level) const
{
	if (level > levels) {
		return 3141592653589793238lu;
	}

	const glm::ivec3 p = pos * invResolution;
	return Hash(p, level);
}

uint64_t HashLooseOctree::Hash(const glm::ivec3 pos, int32_t level) const
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

	if (offset <= 0) {
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
		nd.mask |= mask;
		nd.firstChild = offset;
	}

	for (++level; level <= levels; ++level) {
		glm::ivec3 p = glm::ivec3(pos) >> (level - 1);
		const int32_t childId = (p.x & 1) | ((p.y & 1) << 1) | ((p.y & 1) << 2);
		NodeData &nd = nodes[Key(this, pos, level)];
		nd.childrenInNodesCounts[childId]++;
		nd.mask |= mask;
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
		nd.mask |= data[offset].mask;
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
		nd.mask |= data[offset].mask;
	}
}

void HashLooseOctree::Remove(EntityType entity)
{
	const int32_t offset = data.GetOffset(entity);
	const Aabb aabb = data[offset].aabb;
	// 	MaskType mask = data[offset].mask;
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
	const Aabb aabb = data[offset].aabb;
	data[offset].mask = mask;
	data[offset] = {};
	int32_t level = CalcHashMinLevel(aabb);

	const glm::ivec3 pos = aabb.GetCenter() * invResolution;

	NodeData &nd = nodes[Key(this, pos, level)];
	nd.mask |= mask;

	for (; level <= levels; ++level) {
		NodeData &nd = nodes[Key(this, pos, level)];
		if ((nd.mask & mask) == mask) {
			break;
		}
		nd.mask |= mask;
	}
}

int32_t HashLooseOctree::GetCount() const { return data.Size(); }

bool HashLooseOctree::Exists(EntityType entity) const
{
	return data.GetOffset(entity) > 0;
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

Aabb HashLooseOctree::CalcLocalAabbOfNode(glm::ivec3 pos, int32_t level)
{
	Aabb aabb;
	pos &= ~((1 << level) - 1);
	aabb.min = pos;
	int32_t size = 1 << level;
	aabb.max = pos + glm::ivec3(size, size, size);
	return aabb;
}

void HashLooseOctree::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}
	cb.broadphase = this;

	Aabb cbaabb = cb.aabb;
	cbaabb.min *= invResolution;
	cbaabb.max *= invResolution;

	// iterate over unfit objects
	_Internal_IntersectAabb(cb, {0, 0, 0}, levels + 1, cbaabb);

	/*
	 * iterate over structured objects, usually up to 2 iterations in each
	 * direction should happen
	 */
	const float topLevelBorder =
		((1 << (levels - 1)) * (loosenessFactor - 1.0));
	glm::ivec3 a = CalcLocalAabbOfNode(cbaabb.min - topLevelBorder, levels).min;
	glm::ivec3 b = CalcLocalAabbOfNode(cbaabb.max + topLevelBorder, levels).max;
	const int32_t stride = 1 << levels;
	for (glm::vec3 p = a; p.x <= b.x; p.x += stride) {
		for (p.y = a.y; p.y <= b.y; p.y += stride) {
			for (p.z = a.z; p.z <= b.z; p.z += stride) {
				_Internal_IntersectAabb(cb, p, levels, cbaabb);
			}
		}
	}
}

void HashLooseOctree::_Internal_IntersectAabb(IntersectionCallback &cb,
											  glm::ivec3 pos, int32_t level,
											  const Aabb &cbaabb)
{
	auto it = nodes.find(Key(this, pos, level));
	if (it == nodes.end()) {
		return;
	}

	auto nodeData = it->second;

	_Inernal_IntersectAabbIterateOverData(cb, nodeData.firstChild, cbaabb);

	if (level == 0 || level > levels) {
		return;
	}

	const float halfSize = 1 << (level - 1);
	const int32_t isize = 1 << level;
	const int32_t ihalfSize = 1 << (level - 1);
	const int32_t ihalf = ihalfSize;
	const float overlap = (loosenessFactor)*halfSize;

	glm::ivec3 incl = {0, 0, 0};

	const Aabb totalAabb = {(glm::vec3)pos - overlap,
							(glm::vec3)(pos + isize) + overlap};
	const glm::ivec3 mid = pos + ihalf;

	for (int32_t i = 0; i < 3; ++i) {
		Aabb t = totalAabb;
		t.max[i] = mid[i] + overlap;
		if (t && cbaabb) {
			incl[i] += 1;
		}
		t = totalAabb;
		t.min[i] = mid[i] - overlap;
		if (t && cbaabb) {
			incl[i] += 2;
		}
	}

	for (int32_t i = 0; i < 8; ++i) {
		if (nodeData.childrenInNodesCounts[i] == 0) {
			continue;
		}

		const glm::ivec3 is = {i & 1, (i << 1) & 1, (i << 2) & 1};
		const glm::ivec3 isd = is + 1;

		int32_t j = 0;
		for (; j < 3 && (incl[j] & isd[j]); ++j) {
		};
		if (j != 3) {
			continue;
		}

		_Internal_IntersectAabb(cb, pos + is * ihalf, level - 1, cbaabb);
	}
}

void HashLooseOctree::_Inernal_IntersectAabbIterateOverData(
	IntersectionCallback &cb, int32_t firstNode, const Aabb &cbaabb)
{
	int32_t n = firstNode;
	while (n >= 0) {
		auto &N = data[n];
		if (N.mask & cb.mask) {
			++cb.nodesTestedCount;
			if (N.aabb && cbaabb) {
				++cb.testedCount;
				cb.callback(&cb, N.entity);
			}
		}
		n = data[n].next;
	}
}

void HashLooseOctree::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	cb.InitVariables();

	// iterate over unfit objects
	_Internal_IntersectRay(cb, {0, 0, 0}, levels + 1);

	glm::ivec3 dirs = glm::sign(cb.dir);

	const float topLevelBorder =
		((1 << (levels - 1)) * (loosenessFactor - 1.0));
	glm::ivec3 p =
		CalcLocalAabbOfNode(glm::min(cb.start, cb.end) - topLevelBorder, levels)
			.min;
	glm::ivec3 b =
		CalcLocalAabbOfNode(glm::max(cb.start, cb.end) + topLevelBorder, levels)
			.max;
	glm::ivec3 strides = dirs * (1 << levels);

	const float size = 1 << levels;
	const float margin = (size * (loosenessFactor - 1.0f) * 0.5);

	glm::vec3 d = glm::abs(cb.dir);

	glm::ivec3 axesImportance = {0, 1, 2};
	{
		int32_t _a = 0, _b = 0;
		for (int32_t i = 1; i < 3; ++i) {
			if (d[i] < d[_a]) {
				_a = i;
			} else if (d[i] > d[_b]) {
				_b = i;
			}
		}
		if (_a != _b) {
			int32_t _c = 3 ^ _a ^ _b;
			axesImportance = {_a, _c, _b};
		}
	}
	const glm::ivec3 ai = axesImportance;

#define FOR(X)                                                                 \
	for (; dirs[X] > 0 ? p[X] <= b[X] : p[X] >= b[X]; p[X] += strides[X])
	FOR(ai[0])
	{
		const float X = p[ai[0]];
		const float t1 = (X - cb.start[ai[0]]) / cb.dirNormalized[ai[0]];
		p[ai[1]] = cb.dirNormalized[ai[1]] * t1 + cb.start[ai[1]] -
				   margin * dirs[ai[1]];
		const float t2 =
			(X + strides[ai[0]] - cb.start[ai[0]]) / cb.dirNormalized[ai[0]];
		b[ai[1]] = cb.dirNormalized[ai[1]] * t2 + cb.start[ai[1]] +
				   margin * dirs[ai[1]];
		FOR(ai[1])
		{
			float Y = p[ai[1]];
			const float t1 = (Y - cb.start[ai[1]]) / cb.dirNormalized[ai[1]];
			p[ai[2]] = cb.dirNormalized[ai[2]] * t1 + cb.start[ai[2]] -
					   margin * dirs[ai[2]];
			const float t2 = (X + strides[ai[1]] - cb.start[ai[1]]) /
							 cb.dirNormalized[ai[1]];
			b[ai[2]] = cb.dirNormalized[ai[2]] * t2 + cb.start[ai[2]] +
					   margin * dirs[ai[2]];
			FOR(ai[2])
			{
				Aabb aabb = CalcLocalAabbOfNode(p, levels);
				float __n, __f;
				if (aabb.FastRayTestCenter(
						cb.start * invResolution, cb.dirNormalized, cb.invDir,
						cb.length * invResolution * cb.cutFactor, __n, __f)) {
					_Internal_IntersectRay(cb, p, levels);
				}
			}
		}
	}
}

void HashLooseOctree::_Internal_IntersectRay(RayCallback &cb, glm::ivec3 pos,
											 int32_t level)
{
	auto it = nodes.find(Key(this, pos, level));
	if (it == nodes.end()) {
		return;
	}

	auto nodeData = it->second;

	_Inernal_IntersectRayIterateOverData(cb, nodeData.firstChild);

	if (level == 0 || level > levels) {
		return;
	}

	const int32_t ihalfSize = 1 << (level - 1);
	const int32_t ihalf = ihalfSize;

	struct Ords {
		float near;
		int32_t i;
	} ords[8];
	int32_t ordsCount = 0;

	for (int32_t i = 0; i < 8; ++i) {
		if (nodeData.childrenInNodesCounts[i] == 0) {
			continue;
		}

		const glm::ivec3 is = {i & 1, (i << 1) & 1, (i << 2) & 1};

		glm::ivec3 p = pos + is * ihalfSize;
		Aabb aabb = {p, p + ihalfSize};
		float __n, __f;
		if (aabb.FastRayTestCenter(cb.start * invResolution, cb.dirNormalized,
							 cb.invDir, cb.length * invResolution, __n, __f)) {

			if (__n < cb.cutFactor) {
				int j = 0;
				for (; i < ordsCount; ++j) {
					if (ords[j].near > __n) {
						break;
					}
				}
				if (ordsCount != j) {
					memmove(ords + j + 1, ords + j,
							(ordsCount - j) * sizeof(Ords));
				}
				ords[j] = {__n, i};
			}
		}
	}

	for (int32_t _i = 0; _i < ordsCount; ++_i) {
		int32_t i = ords[_i].i;
		const glm::ivec3 is = {i & 1, (i << 1) & 1, (i << 2) & 1};
		if (ords[_i].near < cb.cutFactor) {
			_Internal_IntersectRay(cb, pos + is * ihalf, level - 1);
		}
	}
}

void HashLooseOctree::_Inernal_IntersectRayIterateOverData(RayCallback &cb,
														   int32_t firstNode)
{
	int32_t n = firstNode;
	while (n >= 0) {
		auto &N = data[n];
		if (N.mask & cb.mask) {
			++cb.nodesTestedCount;

			float __n, __f;
			if (N.aabb.FastRayTestCenter(cb.start * invResolution, cb.dirNormalized,
								   cb.invDir, cb.length * invResolution, __n,
								   __f)) {
				cb.ExecuteCallback(N.entity);
			}
		}
		n = data[n].next;
	}
}

BroadphaseBaseIterator *HashLooseOctree::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

HashLooseOctree::Iterator::Iterator(HashLooseOctree &bp)
{
	data = &bp.data._Data()._Data();
	it = 0;
	Next();
}

HashLooseOctree::Iterator::~Iterator() {}

bool HashLooseOctree::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

bool HashLooseOctree::Iterator::FetchData()
{
	if (Valid()) {
		entity = (*data)[it].entity;
		aabb = (*data)[it].aabb;
		mask = (*data)[it].mask;
		return true;
	}
	return false;
}

bool HashLooseOctree::Iterator::Valid() { return it < data->size(); }
} // namespace experimental
} // namespace spp
