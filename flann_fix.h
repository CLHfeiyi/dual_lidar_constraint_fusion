#ifndef FLANN_FIX_H_
#define FLANN_FIX_H_

// Include full FLANN headers first so Serializer is declared
#include <flann/flann.hpp>
#include <unordered_map>
#include <vector>

namespace flann {
namespace serialization {

// Specialize Serializer for the specific unordered_map type that PCL uses
// This provides empty load/save, avoiding the broken access::serialize path
template<>
struct Serializer<std::unordered_map<unsigned int, std::vector<unsigned int>>> {
    template<typename InputArchive>
    static inline void load(InputArchive&, std::unordered_map<unsigned int, std::vector<unsigned int>>&) {}
    template<typename OutputArchive>
    static inline void save(OutputArchive&, const std::unordered_map<unsigned int, std::vector<unsigned int>>&) {}
};

} // namespace serialization
} // namespace flann

#endif
