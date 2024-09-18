// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>
#include <queue>

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
		if (freeOffsets.size() > 0) {
			OffsetType offset = freeOffsets.top();
			freeOffsets.pop();
			data[offset] = std::move(value);
			return offset;
		} else {
			OffsetType offset = data.size();
			data.push_back(value);
			return offset;
		}
	}

	inline void Remove(OffsetType offset)
	{
		if (offset == data.size() - 1) {
			data.resize(data.size() - 1);
		} else {
			data[offset] = {};
			freeOffsets.push(offset);
		}
	}

	inline void Clear()
	{
		data.clear();
		freeOffsets.clear();
		data.resize(1);
	}

	inline void ShrinkToFit()
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

	inline size_t GetMemoryUsage() const
	{
		return data.capacity() * sizeof(ValueType) +
			   freeOffsets.capacity() * sizeof(OffsetType);
	}

private:
	std::vector<ValueType> data;
	std::priority_queue<OffsetType, std::vector<OffsetType>,
						std::greater<OffsetType>>
		freeOffsets;
};
} // namespace spp
