// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <memory>
#include <atomic>

#include "HashMap.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
SPP_TEMPLATE_DECL
class ThreeStageDbvh final : public BroadphaseBase<SPP_TEMPLATE_ARGS>
{
public:
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator =
		spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;

	ThreeStageDbvh(
		std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> optimised,
		std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> rebuilding,
		std::unique_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> &&dynamic);
	virtual ~ThreeStageDbvh();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void StartFastAdding() override;
	virtual void StopFastAdding() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual int32_t GetCount() const override;
	virtual bool Exists(EntityType entity) const override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(AabbCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

	virtual BroadphaseBaseIterator *RestartIterator() override;

	void SetRebuildSchedulerFunction(
		void (*_ScheduleRebuildFunc)(
			std::shared_ptr<std::atomic<bool>> finishedRebuilding,
			std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> dbvh,
			std::shared_ptr<void> data),
		std::shared_ptr<void> scheduleUpdateUserData = nullptr);

private:
	void TryIntegrateOptimised();
	void TryScheduleRebuild();

private:
	std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> dbvhs[2];

	std::shared_ptr<std::atomic<bool>> _finishedRebuilding;
	std::atomic<bool> *finishedRebuilding;
	std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> _rebuild;
	std::vector<EntityType> toRemoveAfterRebuild;
	HashMap<EntityType, MaskType> setMaskAfterRebuild;
	int32_t optimisedUpdates = 0;
	int32_t tests = 0;
	bool clear = false;
	bool fastAdding = false;

	std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> _optimised;

	std::unique_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> _dynamic;

	int32_t dynamicUpdates = 0;

	BroadphaseBase<SPP_TEMPLATE_ARGS> *rebuild = nullptr;
	BroadphaseBase<SPP_TEMPLATE_ARGS> *optimised = nullptr;
	BroadphaseBase<SPP_TEMPLATE_ARGS> *dynamic = nullptr;

	void (*scheduleRebuildFunc)(
		std::shared_ptr<std::atomic<bool>> finishedRebuilding,
		std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> dbvh,
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

SPP_EXTERN_VARIANTS(ThreeStageDbvh)

} // namespace spp
