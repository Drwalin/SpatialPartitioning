// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#define SPP_TEMPLATE_DECL template<typename Aabb, typename EntityType, typename MaskType, EntityType EMPTY_ENTITY>
#define SPP_TEMPLATE_ARGS Aabb, EntityType, MaskType, EMPTY_ENTITY
#define SPP_TEMPLATE_DECL_OFFSET template<typename Aabb, typename EntityType, typename MaskType, EntityType EMPTY_ENTITY, typename OffsetType>
#define SPP_TEMPLATE_ARGS_OFFSET Aabb, EntityType, MaskType, EMPTY_ENTITY, OffsetType

#define SPP_EXTERN_VARIANTS(CLASS) \
	extern template class CLASS<spp::Aabb, uint32_t, uint32_t, 0>; \
	extern template class CLASS<spp::Aabb, uint64_t, uint32_t, 0>;

#define SPP_EXTERN_VARIANTS_OFFSET(CLASS) \
	extern template class CLASS<spp::Aabb, uint32_t, uint32_t, 0, uint32_t>; \
	extern template class CLASS<spp::Aabb, uint64_t, uint32_t, 0, uint32_t>; \
	extern template class CLASS<spp::Aabb, uint32_t, uint32_t, 0, uint16_t>; \
	extern template class CLASS<spp::Aabb, uint64_t, uint32_t, 0, uint16_t>;

#define SPP_DEFINE_VARIANTS(CLASS) \
	template class CLASS<spp::Aabb, uint32_t, uint32_t, 0>; \
	template class CLASS<spp::Aabb, uint64_t, uint32_t, 0>;

#define SPP_DEFINE_VARIANTS_OFFSET(CLASS) \
	template class CLASS<spp::Aabb, uint32_t, uint32_t, 0, uint32_t>; \
	template class CLASS<spp::Aabb, uint64_t, uint32_t, 0, uint32_t>; \
	template class CLASS<spp::Aabb, uint32_t, uint32_t, 0, uint16_t>; \
	template class CLASS<spp::Aabb, uint64_t, uint32_t, 0, uint16_t>;
