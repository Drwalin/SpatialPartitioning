// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <memory>
#include <atomic>
#include <unordered_set>

#include "BroadPhaseBase.hpp"

namespace spp
{
class ThreeStageDbvh final : public BroadphaseBase
{
public:
	ThreeStageDbvh(std::shared_ptr<BroadphaseBase> optimised,
				   std::shared_ptr<BroadphaseBase> rebuilding,
				   std::unique_ptr<BroadphaseBase> &&dynamic,
				   std::unique_ptr<BroadphaseBase> &&dynamic2);
	virtual ~ThreeStageDbvh();

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

	void SetRebuildSchedulerFunction(
		void (*_ScheduleRebuildFunc)(
			std::shared_ptr<std::atomic<bool>> finishedRebuilding,
			std::shared_ptr<BroadphaseBase> dbvh, std::shared_ptr<void> data),
		std::shared_ptr<void> scheduleUpdateUserData = nullptr);
	
private:
	
	bool TryIntegrateOptimised();
	void TryScheduleRebuild();

private:
	std::shared_ptr<BroadphaseBase> dbvhs[2];

	bool wasScheduled = false;
	std::shared_ptr<std::atomic<bool>> _finishedRebuilding;
	std::atomic<bool> *finishedRebuilding;
	std::shared_ptr<BroadphaseBase> _rebuild;
	std::unordered_set<EntityType> toRemoveAfterRebuild;
	std::unordered_map<EntityType, MaskType> setMaskAfterRebuild;
	int32_t optimisedUpdates = 0;
	int32_t tests = 0;
	bool clear = false;

	std::shared_ptr<BroadphaseBase> _optimised;

	std::unique_ptr<BroadphaseBase> _dynamic;
	std::unique_ptr<BroadphaseBase> _dynamic2;
	
	int32_t dynamicUpdates = 0;
	
	BroadphaseBase *rebuild = nullptr;
	BroadphaseBase *optimised = nullptr;
	BroadphaseBase *dynamic = nullptr;
	BroadphaseBase *dynamic2 = nullptr;

	void (*scheduleRebuildFunc)(
		std::shared_ptr<std::atomic<bool>> finishedRebuilding,
		std::shared_ptr<BroadphaseBase> dbvh,
		std::shared_ptr<void> data) = nullptr;
	std::shared_ptr<void> scheduleUpdateUserData;
	
	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(ThreeStageDbvh &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		ThreeStageDbvh *bp = nullptr;
		BroadphaseBaseIterator *it = nullptr;
		int32_t stage = 0;
	} iterator;
};
} // namespace spp
