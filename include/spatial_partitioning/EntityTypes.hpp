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

#define _SPP_EXTERN_VARIANTS_AABBS(CLASS, ...)                                 \
	__EXT_TEMPL_CLS CLASS<spp::Aabb, __VA_ARGS__>;                                     \
	__EXT_TEMPL_CLS CLASS<spp::Aabb_i16, __VA_ARGS__>;

#define _SPP_EXTERN_VARIANTS_ENTITY_TYPES(CLASS, ...)                          \
	_SPP_EXTERN_VARIANTS_AABBS(CLASS, uint16_t, __VA_ARGS__)                   \
	_SPP_EXTERN_VARIANTS_AABBS(CLASS, uint32_t, __VA_ARGS__)                   \
	_SPP_EXTERN_VARIANTS_AABBS(CLASS, uint64_t, __VA_ARGS__)

#define _SPP_EXTERN_VARIANTS_MASK_TYPES(CLASS, ...)                            \
	_SPP_EXTERN_VARIANTS_ENTITY_TYPES(CLASS, uint8_t, __VA_ARGS__)             \
	_SPP_EXTERN_VARIANTS_ENTITY_TYPES(CLASS, uint16_t, __VA_ARGS__)            \
	_SPP_EXTERN_VARIANTS_ENTITY_TYPES(CLASS, uint32_t, __VA_ARGS__)            \
	_SPP_EXTERN_VARIANTS_ENTITY_TYPES(CLASS, uint64_t, __VA_ARGS__)

#define SPP_EXTERN_VARIANTS(CLASS) _SPP_EXTERN_VARIANTS_MASK_TYPES(CLASS, 0)

#define SPP_EXTERN_VARIANTS_MORE(CLASS, ...)                                   \
	_SPP_EXTERN_VARIANTS_MASK_TYPES(CLASS, 0, __VA_ARGS__)

#define SPP_EXTERN_VARIANTS_OFFSET(CLASS)                                      \
	SPP_EXTERN_VARIANTS_MORE(CLASS, uint32_t);                                 \
	SPP_EXTERN_VARIANTS_MORE(CLASS, uint16_t);

#define _SPP_DEFINE_VARIANTS_AABBS(CLASS, ...)                                 \
	__TEMPL_CLS CLASS<spp::Aabb, __VA_ARGS__>;                                 \
	__TEMPL_CLS CLASS<spp::Aabb_i16, __VA_ARGS__>;

#define _SPP_DEFINE_VARIANTS_ENTITY_TYPES(CLASS, ...)                          \
	_SPP_DEFINE_VARIANTS_AABBS(CLASS, uint16_t, __VA_ARGS__)                   \
	_SPP_DEFINE_VARIANTS_AABBS(CLASS, uint32_t, __VA_ARGS__)                   \
	_SPP_DEFINE_VARIANTS_AABBS(CLASS, uint64_t, __VA_ARGS__)

#define _SPP_DEFINE_VARIANTS_MASK_TYPES(CLASS, ...)                            \
	_SPP_DEFINE_VARIANTS_ENTITY_TYPES(CLASS, uint8_t, __VA_ARGS__)             \
	_SPP_DEFINE_VARIANTS_ENTITY_TYPES(CLASS, uint16_t, __VA_ARGS__)            \
	_SPP_DEFINE_VARIANTS_ENTITY_TYPES(CLASS, uint32_t, __VA_ARGS__)            \
	_SPP_DEFINE_VARIANTS_ENTITY_TYPES(CLASS, uint64_t, __VA_ARGS__)

#define SPP_DEFINE_VARIANTS(CLASS) _SPP_DEFINE_VARIANTS_MASK_TYPES(CLASS, 0)

#define SPP_DEFINE_VARIANTS_MORE(CLASS, ...)                                   \
	_SPP_DEFINE_VARIANTS_MASK_TYPES(CLASS, 0, __VA_ARGS__)

#define SPP_DEFINE_VARIANTS_OFFSET(CLASS)                                      \
	SPP_DEFINE_VARIANTS_MORE(CLASS, uint32_t)                                  \
	SPP_DEFINE_VARIANTS_MORE(CLASS, uint16_t)
