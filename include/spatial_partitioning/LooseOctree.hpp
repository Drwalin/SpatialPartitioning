// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include <unordered_map>
#include <vector>

#include "BroadPhaseBase.hpp"

namespace spp
{
/*
 * Limit of entities count is 268435456 (2^28-1)
 *
 * Tree is perfectly balanced due to heap use as nodes storage
 */
class LooseOctree final : public BroadphaseBase
{
public:
	LooseOctree(spp::Aabb aabb, int32_t depth, float cellExtensionFactor);
	virtual ~LooseOctree();

	void Resize(spp::Aabb aabb, int32_t depth, float cellExtensionFactor);

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

private:
	void PruneEmptyEntitiesAtEnd();
	void UpdateAabb(int32_t entityId);

	void _Internal_IntersectAabb(IntersectionCallback &cb,
								 const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

private:
	struct alignas(64) Data {
		Aabb aabb;
		int32_t nextEntity = -1;
		EntityType entity = 0;
		MaskType mask = 0;
	};

	struct alignas(32) NodePartialData {
		Aabb aabb;
		MaskType mask = 0;
	};

	struct alignas(32) NodeData {
		NodePartialData children[8];
		int32_t childrenNodesOrEntitiesOffsets[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	};

	std::unordered_map<EntityType, int32_t> entitiesOffsets;
	// [0] - ignored, because heap works faster starting from 1
	std::vector<NodeData> nodes;
	std::vector<Data> entitiesData;
	std::vector<int32_t> emptyNodesOffsets;

	int32_t entitiesCount = 0;

	Aabb aabb;
	int32_t depth;
	float cellExtensionFactor;
};
} // namespace spp
