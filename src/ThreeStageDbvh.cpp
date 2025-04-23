// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/ThreeStageDbvh.hpp"

namespace spp
{

ThreeStageDbvh::ThreeStageDbvh(std::shared_ptr<BroadphaseBase> optimised,
							   std::shared_ptr<BroadphaseBase> rebuilding,
							   std::unique_ptr<BroadphaseBase> &&dynamic,
							   std::unique_ptr<BroadphaseBase> &&dynamic2)
	: iterator(*this)
{
	this->_finishedRebuilding = std::make_shared<std::atomic<bool>>();
	this->finishedRebuilding = this->_finishedRebuilding.get();
	this->dbvhs[0] = optimised;
	this->dbvhs[1] = rebuilding;
	this->_rebuild = rebuilding;
	this->_optimised = optimised;
	this->_dynamic = std::move(dynamic);
	this->_dynamic2 = std::move(dynamic2);
	this->rebuild = nullptr;
	this->optimised = _optimised.get();
	this->dynamic = _dynamic.get();
	this->dynamic2 = _dynamic2.get();
}

ThreeStageDbvh::~ThreeStageDbvh() {}

const char *ThreeStageDbvh::GetName() const { return "ThreeStageDbvh"; }
void ThreeStageDbvh::Clear()
{
	dynamic->Clear();
	dynamic2->Clear();
	optimised->Clear();
	if (rebuild) {
		clear = true;
		TryIntegrateOptimised();
	}
}

size_t ThreeStageDbvh::GetMemoryUsage() const
{
	return dbvhs[0]->GetMemoryUsage() + dbvhs[1]->GetMemoryUsage() +
		   dynamic->GetMemoryUsage() + dynamic2->GetMemoryUsage() +

		   sizeof(*this) + 6 * 32 +

		   toRemoveAfterRebuild.bucket_count() * sizeof(void *) +
		   toRemoveAfterRebuild.size() *
			   (sizeof(void *) * 2lu + sizeof(EntityType)) +

		   setMaskAfterRebuild.bucket_count() * sizeof(void *) +
		   setMaskAfterRebuild.size() *
			   (sizeof(void *) * 2lu + sizeof(EntityType) + sizeof(MaskType));
}

void ThreeStageDbvh::ShrinkToFit()
{
	dbvhs[0]->ShrinkToFit();
	dbvhs[1]->ShrinkToFit();
	dynamic->ShrinkToFit();
	toRemoveAfterRebuild.rehash((toRemoveAfterRebuild.size() * 3 + 7) / 2);
}

void ThreeStageDbvh::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	dynamicUpdates++;
	dynamic->Add(entity, aabb, mask);
}

void ThreeStageDbvh::Update(EntityType entity, Aabb aabb)
{
	if (TryIntegrateOptimised()) {
		if (rebuild->Exists(entity)) {
			toRemoveAfterRebuild.insert(entity);
		}
	}

	if (dynamic->Exists(entity)) {
		dynamicUpdates++;
		dynamic->Update(entity, aabb);
	} else {
		MaskType mask = 0;
		bool inOptimised = false;
		if (dynamic2->Exists(entity)) {
			mask = dynamic2->GetMask(entity);
		} else {
			inOptimised = true;
			mask = optimised->GetMask(entity);
		}

		optimisedUpdates++;
		dynamicUpdates++;
		dynamic->Add(entity, aabb, mask);
		if (inOptimised) {
			optimised->Remove(entity);
		} else {
			dynamic2->Remove(entity);
		}
	}

	if (dynamicUpdates > 1000 || optimisedUpdates > 100) {
		TryScheduleRebuild();
	}
}

void ThreeStageDbvh::Remove(EntityType entity)
{
	if (TryIntegrateOptimised()) {
		if (rebuild->Exists(entity)) {
			toRemoveAfterRebuild.insert(entity);
		}
	}

	if (dynamic->Exists(entity)) {
		dynamicUpdates++;
		dynamic->Remove(entity);
	} else if (dynamic2->Exists(entity)) {
		optimisedUpdates++;
		dynamic2->Remove(entity);
	} else if (optimised->Exists(entity)) {
		optimisedUpdates++;
		optimised->Remove(entity);
	}
}

void ThreeStageDbvh::SetMask(EntityType entity, MaskType mask)
{
	if (rebuild) {
		if (rebuild->Exists(entity)) {
			setMaskAfterRebuild[entity] = mask;
		}
	}

	if (dynamic->Exists(entity)) {
		dynamic->SetMask(entity, mask);
	} else if (dynamic2->Exists(entity)) {
		dynamic2->SetMask(entity, mask);
	} else {
		optimised->SetMask(entity, mask);
	}
}

bool ThreeStageDbvh::TryIntegrateOptimised()
{
	bool done = finishedRebuilding->load();
	if (done) {
		if (wasScheduled) {
			finishedRebuilding->store(false);
			wasScheduled = false;

			if (clear) {
				clear = false;
				rebuild->Clear();
				rebuild = nullptr;
			} else {
				std::swap(_optimised, _rebuild);
				optimised = rebuild;
				rebuild = nullptr;

				for (auto entity : toRemoveAfterRebuild) {
					optimised->Remove(entity);
				}
				for (auto it : setMaskAfterRebuild) {
					optimised->SetMask(it.first, it.second);
				}
				dynamic2->Clear();
			}
		}
		return false;
	} else {
		return wasScheduled;
	}
}

void ThreeStageDbvh::TryScheduleRebuild()
{
	if (wasScheduled) {
		if (finishedRebuilding->load() == false) {
			return;
		}
	}

	if (scheduleRebuildFunc) {
		rebuild = _rebuild.get();
		rebuild->Clear();
		for (auto it = optimised->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}
		for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}

		std::swap(_dynamic, _dynamic2);
		std::swap(dynamic, dynamic2);

		tests = 0;
		dynamicUpdates = 0;
		optimisedUpdates = 0;
		wasScheduled = true;
		finishedRebuilding->store(false);

		scheduleRebuildFunc(_finishedRebuilding, _rebuild,
							scheduleUpdateUserData);
	} else {
		Rebuild();
	}
}

int32_t ThreeStageDbvh::GetCount() const
{
	return dynamic->GetCount() + optimised->GetCount() + dynamic2->GetCount();
}

bool ThreeStageDbvh::Exists(EntityType entity) const
{
	return optimised->Exists(entity) || dynamic->Exists(entity) ||
		   dynamic2->Exists(entity);
}

Aabb ThreeStageDbvh::GetAabb(EntityType entity) const
{
	if (optimised->Exists(entity)) {
		return optimised->GetAabb(entity);
	} else if (dynamic->Exists(entity)) {
		return dynamic->GetAabb(entity);
	} else {
		return dynamic2->GetAabb(entity);
	}
}

MaskType ThreeStageDbvh::GetMask(EntityType entity) const
{
	if (optimised->Exists(entity)) {
		return optimised->GetMask(entity);
	} else if (dynamic->Exists(entity)) {
		return dynamic->GetMask(entity);
	} else {
		return dynamic2->GetMask(entity);
	}
}

void ThreeStageDbvh::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	++tests;
	if (tests > 100000) {
		TryScheduleRebuild();
	}

	dynamic2->IntersectAabb(cb);
	dynamic->IntersectAabb(cb);
	optimised->IntersectAabb(cb);
}

void ThreeStageDbvh::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	++tests;
	if (tests > 100000) {
		TryScheduleRebuild();
	}

	dynamic2->IntersectRay(cb);
	dynamic->IntersectRay(cb);
	optimised->IntersectRay(cb);
}

void ThreeStageDbvh::Rebuild() {
	
	if (wasScheduled) {
		clear = true;
	}
	
	for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
		optimised->Add(it->entity, it->aabb, it->mask);
	}
	for (auto it = dynamic2->RestartIterator(); it->Valid(); it->Next()) {
		optimised->Add(it->entity, it->aabb, it->mask);
	}
	optimised->Rebuild();

	dynamic->Clear();
	dynamic2->Clear();

	dynamicUpdates = 0;
	optimisedUpdates = 0;
	wasScheduled = false;
	finishedRebuilding->store(false);
	tests = 0;
}

BroadphaseBaseIterator *ThreeStageDbvh::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

ThreeStageDbvh::Iterator::Iterator(ThreeStageDbvh &bp)
{
	this->bp = &bp;
	stage = 0;
	if (bp.optimised) {
		it = bp.optimised->RestartIterator();
		while (it && it->Valid() == false) {
			Next();
		}

		FetchData();
	}
}

ThreeStageDbvh::Iterator::~Iterator() {}

bool ThreeStageDbvh::Iterator::Next()
{
	if (it == nullptr) {
		return false;
	}

	if (it->Next() == false) {
		switch (stage) {
		case 0:
			stage = 1;
			it = bp->dynamic2->RestartIterator();
			if (it->Valid()) {
				break;
			}
		case 1:
			stage = 2;
			it = bp->dynamic->RestartIterator();
			if (it->Valid()) {
				break;
			}
		default:
			return false;
		}
	}
	return FetchData();
}

bool ThreeStageDbvh::Iterator::FetchData()
{
	if (Valid()) {
		entity = it->entity;
		aabb = it->aabb;
		mask = it->mask;
		return true;
	}
	return false;
}

bool ThreeStageDbvh::Iterator::Valid() { return it; }

void ThreeStageDbvh::SetRebuildSchedulerFunction(
	void (*_ScheduleRebuildFunc)(
		std::shared_ptr<std::atomic<bool>> finishedRebuilding,
		std::shared_ptr<BroadphaseBase> dbvh, std::shared_ptr<void> data),
	std::shared_ptr<void> scheduleUpdateUserData)
{
	scheduleRebuildFunc = _ScheduleRebuildFunc;
	this->scheduleUpdateUserData = scheduleUpdateUserData;
}
} // namespace spp
