// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include <unordered_map>

#include "../../thirdparty/glm/glm/ext/vector_int3.hpp"

#include "AssociativeArray.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
namespace experimental
{
/*
 * Max objects count: 2<<30
 *
 * Restrict positions ranges to less than 2^21 * resolution.
 * Position ranges cannot be more than +-2^31 * resolution.
 */
class HashLooseOctree final : public BroadphaseBase
{
public:
	HashLooseOctree(float resolution, int32_t levels,
					float loosenessFactor = 1.5);
	virtual ~HashLooseOctree();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual int32_t GetCount() const override;
	virtual bool Exists(EntityType entity) const override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

	virtual BroadphaseBaseIterator *RestartIterator() override;

private:
	static int32_t FindBestPrime(int32_t capacity);
	// value of level should be 0 initially
	int32_t CalcHashMinLevel(Aabb aabb);
	uint64_t Hash(const glm::vec3 pos, int32_t level) const;
	uint64_t Hash(const glm::ivec3 pos, int32_t level) const;

	static Aabb CalcLocalAabbOfNode(glm::ivec3 pos, int32_t level);

	void _Internal_IntersectAabb(IntersectionCallback &cb, glm::ivec3 pos,
								 int32_t level, const Aabb &cbaabb);
	void _Inernal_IntersectAabbIterateOverData(IntersectionCallback &cb,
											   int32_t firstNode,
											   const Aabb &cbaabb);

	void _Internal_IntersectRay(RayCallback &cb, glm::ivec3 pos, int32_t level);
	void _Inernal_IntersectRayIterateOverData(RayCallback &cb,
											  int32_t firstNode);

private:
	struct Data {
		AabbCentered aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t prev = -1;
		int32_t next = -1;
	};

	struct NodeData {
		int32_t childrenInNodesCounts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		int32_t directChildrenCount = 0;
		int32_t firstChild = -1;
		MaskType mask = 0;
		inline bool HasIndirectChildren() const
		{
			const int64_t *v = (const int64_t *)childrenInNodesCounts;
			return v[0] | v[0] | v[2] | v[3];
		};
	};

	AssociativeArray<EntityType, int32_t, Data> data;

	struct Key {
		Key(const Key &o) : pos(o.pos), level(o.level) {}
		Key(const HashLooseOctree *bp, glm::ivec3 pos, int32_t level)
			: pos(pos >> level), level(level)
		{
			if (level > bp->levels) {
				pos = {0, 0, 0};
				level = bp->levels + 1;
			}
		}
		inline bool operator==(const Key &o) const
		{
			return pos == o.pos && level == o.level;
		}
		struct Hash {
		public:
			Hash(const Hash &o) : bp(o.bp) {}
			Hash(HashLooseOctree *bp) : bp(bp) {}
			HashLooseOctree *bp;
			size_t operator()(const Key &v) const
			{
				return bp->Hash(v.pos, v.level);
			}
		};

	private:
		glm::ivec3 pos;
		int32_t level;
	};

	std::unordered_map<Key, NodeData, HashLooseOctree::Key::Hash> nodes;

public:
	const float loosenessFactor;
	const float invLoosenessFactor;
	const float resolution;
	const float invResolution;

	const int32_t levels;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(HashLooseOctree &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		std::vector<Data> *data;
		int it;
	} iterator;
};
} // namespace experimental
} // namespace spp
