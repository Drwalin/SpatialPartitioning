// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "IntersectionCallbacks.hpp"
#include "EntityTypes.hpp"

namespace spp
{
SPP_TEMPLATE_DECL
class BroadphaseBase;

SPP_TEMPLATE_DECL
class BroadphaseBaseIterator
{
public:
	virtual ~BroadphaseBaseIterator();

	virtual bool Next() = 0;
	virtual bool Valid() = 0;

	Aabb aabb;
	EntityType entity = EMPTY_ENTITY;
	MaskType mask = 0;
};

SPP_TEMPLATE_DECL
class BroadphaseBase
{
public:
	BroadphaseBase();
	virtual ~BroadphaseBase();

	virtual const char *GetName() const = 0;
	
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator = spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;

public:
	virtual void Clear() = 0;
	virtual size_t GetMemoryUsage() const = 0;
	virtual void ShrinkToFit() = 0;

	virtual void StartFastAdding();
	virtual void StopFastAdding();

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) = 0;
	virtual void Update(EntityType entity, Aabb aabb) = 0;
	virtual void Remove(EntityType entity) = 0;
	virtual void SetMask(EntityType entity, MaskType mask) = 0;

	virtual int32_t GetCount() const = 0;
	virtual bool Exists(EntityType entity) const = 0;

	virtual Aabb GetAabb(EntityType entity) const = 0;
	virtual MaskType GetMask(EntityType entity) const = 0;

	// maybe rename/add Optimize() function
	virtual void Rebuild() = 0;

	// returns number of tested entities
	virtual void IntersectAabb(AabbCallback &callback) = 0;
	virtual void IntersectRay(RayCallback &callback) = 0;

	virtual BroadphaseBaseIterator *RestartIterator() = 0;
};

SPP_EXTERN_VARIANTS(BroadphaseBaseIterator)
SPP_EXTERN_VARIANTS(BroadphaseBase)

} // namespace spp
