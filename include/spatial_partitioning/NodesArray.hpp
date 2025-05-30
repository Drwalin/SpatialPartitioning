// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>

namespace spp
{
/*
 * Offsets start from 1
 */
template <typename OffsetType, typename ValueType> class NodesArray final
{
public:
	inline NodesArray() { Clear(); }
	inline ~NodesArray() {}

	inline NodesArray(const NodesArray &) = default;
	inline NodesArray(NodesArray &) = default;
	inline NodesArray(NodesArray &&) = default;

	inline OffsetType Add(ValueType &&value)
	{
		OffsetType offset = -1;
		if (freeOffsets.size() > 0) {
			offset = freeOffsets.back();
			freeOffsets.resize(freeOffsets.size() - 1);
			data[offset] = std::move(value);
		} else {
			offset = data.size();
			data.push_back(value);
		}
		return offset;
	}

	inline void Remove(OffsetType offset)
	{
		if (offset == data.size() - 1) {
			data.resize(data.size() - 1);
		} else {
			data[offset] = {};
			freeOffsets.push_back(offset);
		}
	}

	inline void Clear()
	{
		freeOffsets.clear();
		data.resize(1);
	}

	__attribute__((noinline))
	void ShrinkToFit()
	{
		data.shrink_to_fit();
		freeOffsets.shrink_to_fit();
	}

	inline void Reserve(OffsetType capacity) { data.reserve(capacity); }

	inline OffsetType Size() const
	{
		return data.size() - freeOffsets.size() - 1;
	}

	inline ValueType &operator[](OffsetType offset) { return data[offset]; }
	inline const ValueType &operator[](OffsetType offset) const
	{
		return data[offset];
	}

	inline size_t GetMemoryUsage() const
	{
		return data.capacity() * sizeof(ValueType) +
			   freeOffsets.capacity() * sizeof(OffsetType);
	}

	std::vector<ValueType> &_Data() { return data; }
	std::vector<OffsetType> &_FreeOffsets() { return freeOffsets; }

private:
	std::vector<ValueType> data;
	std::vector<OffsetType> freeOffsets;
};
} // namespace spp
