// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BroadPhaseBase.hpp"

namespace spp
{
SPP_TEMPLATE_DECL
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>::~BroadphaseBaseIterator() {}
SPP_TEMPLATE_DECL
BroadphaseBase<SPP_TEMPLATE_ARGS>::BroadphaseBase() {}
SPP_TEMPLATE_DECL
BroadphaseBase<SPP_TEMPLATE_ARGS>::~BroadphaseBase() {}

SPP_TEMPLATE_DECL
void BroadphaseBase<SPP_TEMPLATE_ARGS>::StartFastAdding() {}
SPP_TEMPLATE_DECL
void BroadphaseBase<SPP_TEMPLATE_ARGS>::StopFastAdding() {}

SPP_DEFINE_VARIANTS(BroadphaseBaseIterator)
SPP_DEFINE_VARIANTS(BroadphaseBase)

} // namespace spp
