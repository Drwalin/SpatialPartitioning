// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "EntityTypes.hpp"
#include "Aabb.hpp"
#include "IntersectionCallbacks.hpp"

namespace spp
{
class BroadphaseBase
{
public:
	BroadphaseBase();
	virtual ~BroadphaseBase();
	
	virtual const char* GetName() const = 0;

public:
	virtual void Clear() = 0;
	virtual size_t GetMemoryUsage() const = 0;
	virtual void ShrinkToFit() = 0;
	
	virtual void Add(EntityType entity, Aabb aabb,
					 MaskType mask) = 0;
	virtual void Update(EntityType entity, Aabb aabb) = 0;
	virtual void Remove(EntityType entity) = 0;
	virtual void SetMask(EntityType entity, MaskType mask) = 0;
	
	virtual Aabb GetAabb(EntityType entity) const = 0;
	virtual MaskType GetMask(EntityType entity) const = 0;
	
	virtual void Rebuild() = 0;

	// returns number of tested entities
	virtual void IntersectAabb(IntersectionCallback &callback) = 0;
	virtual void IntersectRay(RayCallback &callback) = 0;
};
} // namespace spp
