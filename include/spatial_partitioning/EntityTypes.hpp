// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#define __EXT_TEMPL_CLS extern template class
#define __TEMPL_CLS template class

#define SPP_TEMPLATE_DECL                                                      \
	template <typename Aabb, typename EntityType, typename MaskType,           \
			  EntityType EMPTY_ENTITY>
#define SPP_TEMPLATE_ARGS Aabb, EntityType, MaskType, EMPTY_ENTITY

#define SPP_TEMPLATE_DECL_MORE(...)                                            \
	template <typename Aabb, typename EntityType, typename MaskType,           \
			  EntityType EMPTY_ENTITY, __VA_ARGS__>
#define SPP_TEMPLATE_ARGS_MORE(...)                                            \
	Aabb, EntityType, MaskType, EMPTY_ENTITY, __VA_ARGS__

#define SPP_TEMPLATE_DECL_OFFSET SPP_TEMPLATE_DECL_MORE(typename OffsetType)
#define SPP_TEMPLATE_ARGS_OFFSET SPP_TEMPLATE_ARGS_MORE(OffsetType)

#define SPP_EXTERN_VARIANTS(CLASS)                                             \
	__EXT_TEMPL_CLS CLASS<spp::Aabb, uint32_t, uint32_t, 0>;                   \
	__EXT_TEMPL_CLS CLASS<spp::Aabb, uint64_t, uint32_t, 0>;

#define SPP_EXTERN_VARIANTS_MORE(CLASS, ...)                                   \
	__EXT_TEMPL_CLS CLASS<spp::Aabb, uint32_t, uint32_t, 0, __VA_ARGS__>;      \
	__EXT_TEMPL_CLS CLASS<spp::Aabb, uint64_t, uint32_t, 0, __VA_ARGS__>;

#define SPP_EXTERN_VARIANTS_OFFSET(CLASS)                                      \
	SPP_EXTERN_VARIANTS_MORE(CLASS, uint32_t);                                        \
	SPP_EXTERN_VARIANTS_MORE(CLASS, uint16_t);

#define SPP_DEFINE_VARIANTS(CLASS)                                             \
	__TEMPL_CLS CLASS<spp::Aabb, uint32_t, uint32_t, 0>;                       \
	__TEMPL_CLS CLASS<spp::Aabb, uint64_t, uint32_t, 0>;

#define SPP_DEFINE_VARIANTS_MORE(CLASS, ...)                                   \
	__TEMPL_CLS CLASS<spp::Aabb, uint32_t, uint32_t, 0, __VA_ARGS__>;          \
	__TEMPL_CLS CLASS<spp::Aabb, uint64_t, uint32_t, 0, __VA_ARGS__>;

#define SPP_DEFINE_VARIANTS_OFFSET(CLASS)                                      \
	SPP_DEFINE_VARIANTS_MORE(CLASS, uint32_t)                                  \
	SPP_DEFINE_VARIANTS_MORE(CLASS, uint16_t)
