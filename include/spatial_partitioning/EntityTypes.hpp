// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

namespace spp
{
#ifndef SPP_ENTITY_TYPE
using EntityType = uint64_t;
#else
using EntityType = SPP_ENTITY_TYPE
#endif

#ifndef SPP_MASK_TYPE
using MaskType = uint32_t;
#else
	using MaskType = SPP_MASK_TYPE
#endif

inline const EntityType EMPTY_ENTITY = 0;
} // namespace spp
