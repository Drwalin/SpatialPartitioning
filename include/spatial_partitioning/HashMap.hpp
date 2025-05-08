// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <unordered_map>

namespace spp
{
template <typename K, typename V, typename Hash = std::hash<K>,
		  typename Equal = std::equal_to<K>>
class HashMap : public std::unordered_map<K, V, Hash, Equal>
{
public:
	using Map = std::unordered_map<K, V, Hash, Equal>;

	template <typename... Args>
	HashMap(Args &&...args) : Map(std::forward(args)...)
	{
	}

	inline size_t GetMemoryUsage() const
	{
		using KV = std::pair<K, V>;
		constexpr size_t two_ptr = 2 * sizeof(void *);
		return this->bucket_count() * sizeof(void *) +
			   this->size() *
				   (two_ptr +
					(((sizeof(KV) + two_ptr - 1) / two_ptr) * two_ptr));
	}
};
} // namespace spp
