#ifndef PERCEPTION_CORE__GRID_KEY_HPP_
#define PERCEPTION_CORE__GRID_KEY_HPP_

#include <cstddef>
#include <cstdint>

namespace perception_core
{

// Packs integer cell coordinates into one 64-bit key, 21 bits per axis
// (+-1M cells: +-50 km at 5 cm). Out-of-range coordinates wrap, which only
// merges cells that far apart; callers that need exact neighbours still
// compare real distances.
inline std::uint64_t packGridKey(std::int64_t x, std::int64_t y, std::int64_t z)
{
  constexpr std::uint64_t kMask = (1ULL << 21) - 1;
  return ((static_cast<std::uint64_t>(x) & kMask) << 42) |
         ((static_cast<std::uint64_t>(y) & kMask) << 21) |
         (static_cast<std::uint64_t>(z) & kMask);
}

// splitmix64 finalizer. std::hash<integer> is the identity in libstdc++, so
// neighbouring cells landed in neighbouring buckets and collided heavily.
struct GridKeyHash
{
  std::size_t operator()(std::uint64_t key) const noexcept
  {
    key += 0x9E3779B97F4A7C15ULL;
    key = (key ^ (key >> 30)) * 0xBF58476D1CE4E5B9ULL;
    key = (key ^ (key >> 27)) * 0x94D049BB133111EBULL;
    return static_cast<std::size_t>(key ^ (key >> 31));
  }
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__GRID_KEY_HPP_
