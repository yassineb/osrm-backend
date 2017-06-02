#include "engine/routing_algorithms/alternative_path.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

#include "util/static_assert.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <iterator>
#include <memory>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/function_output_iterator.hpp>

// TODO: debug, remove
#include <iostream>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

// Unqualified calls below are from the mld namespace.
// This alternative implementation works only for mld.
using namespace mld;

using Heap = SearchEngineData<Algorithm>::QueryHeap;
using Partition = partition::MultiLevelPartitionView;
using Facade = datafacade::ContiguousInternalMemoryDataFacade<Algorithm>;

// Implementation details
namespace
{

// Alternative paths candidate via nodes are taken from overlapping search spaces.
// Overlapping by a third guarantees us taking candidate nodes "from the middle".
const constexpr auto kSearchSpaceOverlapFactor = 1.66;
// Maximum number of alternative paths to return.
const constexpr auto kMaxAlternatives = 3;
// Alternative paths length requirement (stretch).
// At most 25% longer then the shortest path.
const constexpr auto kEpsilon = 0.25;
// Alternative paths similarity requirement (sharing).
// At least 15% different than the shortest path.
const constexpr auto kGamma = 0.85;
// Alternative paths are still reasonable around the via node candidate (local optimality).
// At least optimal around 10% sub-paths around the via node candidate.
const /*constexpr*/ auto kAlpha = 0.10;
// Note: ^ ICEs gcc 7.1, just leave it out for better times

// Represents a via middle node where forward (from s) and backward (from t)
// search spaces overlap and the weight a path (made up of s,via and via,t) has.
struct WeightedViaNode
{
    NodeID node;
    EdgeWeight weight;
};

// Represents a complete packed path (made up of s,via and via,t)
// its total weight and the via node used to construct the path.
struct WeightedViaNodePackedPath
{
    WeightedViaNode via;
    PackedPath path;
};

// Represents a high-detail unpacked path (s, .., via, .., t)
// its total weight and the via node used to construct the path.
struct WeightedViaNodeUnpackedPath
{
    WeightedViaNode via;
    UnpackedNodes nodes;
    UnpackedEdges edges;
};

// Filters candidates which are on not unique.
// Returns an iterator to the uniquified range's new end.
// Note: mutates the range in-place invalidating iterators.
template <typename RandIt> RandIt filterViaCandidatesByUniqueNodeIds(RandIt first, RandIt last)
{
    util::static_assert_iter_category<RandIt, std::random_access_iterator_tag>();
    util::static_assert_iter_value<RandIt, WeightedViaNode>();

    std::sort(first, last, [](auto lhs, auto rhs) { return lhs.node < rhs.node; });
    return std::unique(first, last, [](auto lhs, auto rhs) { return lhs.node == rhs.node; });
}

// Filters candidates which are on un-important roads.
// Returns an iterator to the filtered range's new end.
template <typename RandIt>
RandIt filterViaCandidatesByRoadImportance(RandIt first, RandIt last, const Facade &facade)
{
    util::static_assert_iter_category<RandIt, std::random_access_iterator_tag>();
    util::static_assert_iter_value<RandIt, WeightedViaNode>();

    // Todo: the idea here is to filter out alternatives where the via candidate is not on a
    // high-priority road. We should experiment if this is really needed or if the boundary
    // nodes the mld search space gives us already provides us with reasonable via candidates.
    //
    // Implementation: we already have `RoadClassification` from guidance. We need to serialize
    // it to disk and then in the facades read it in again providing `IsImportantRoad(NodeID)`.
    // Note: serialize out bit vector keyed by node id with 0/1 <=> unimportant/important.
    (void)first;
    (void)last;
    (void)facade;

    return last;
}

// Filters candidates with much higher weight than the primary route. Mutates range in-place.
// Returns an iterator to the filtered range's new end.
template <typename RandIt>
RandIt filterViaCandidatesByStretch(RandIt first, RandIt last, EdgeWeight weight)
{
    util::static_assert_iter_category<RandIt, std::random_access_iterator_tag>();
    util::static_assert_iter_value<RandIt, WeightedViaNode>();

    // Todo: scale epsilon with weight. Higher epsilon for short routes are reasonable.
    //
    //  - shortest path 10 minutes, alternative 13 minutes => 0.30 epsilon Ok
    //  - shortest path 10 hours, alternative 13 hours     => 0.30 epsilon Unreasonable
    //
    // We only have generic weights here and no durations without unpacking.
    // How do we scale the epsilon in a setup where users can pass us anything as weight?

    const auto stretch_weight_limit = (1. + kEpsilon) * weight;

    const auto over_weight_limit = [=](const auto via) {
        return via.weight > stretch_weight_limit;
    };

    return std::remove_if(first, last, over_weight_limit);
}

// The packed paths' sharing in [0, 1] for no sharing and equality, respectively.
inline double normalizedPackedPathSharing(const Partition &partition,
                                          const PackedPath &lhs,
                                          const PackedPath &rhs)
{
    if (lhs.empty() || rhs.empty())
        return 0.;

    const auto number_of_levels = partition.GetNumberOfLevels();
    (void)number_of_levels;
    BOOST_ASSERT(number_of_levels >= 1);

    // Todo: on which level does it make sense to compute sharing on? Should sharing be a
    // linear combination based on level and sharing on each level? Needs experimentation.
    const auto level = 1;

    const auto get_cell = [&](auto node) { return partition.GetCell(level, node); };

    // Todo: might benefit from stack alloc, we're creating two small vecs everytime here
    std::vector<CellID> lhs_cells(lhs.size() + 1);
    std::vector<CellID> rhs_cells(rhs.size() + 1);

    // Transform edges (from, to) to node ids and then to cell ids for each node id.

    lhs_cells[0] = get_cell(std::get<0>(lhs.front()));
    rhs_cells[0] = get_cell(std::get<0>(rhs.front()));

    for (std::size_t i = 0; i < lhs.size(); ++i)
        lhs_cells[i + 1] = get_cell(std::get<1>(lhs[i]));

    for (std::size_t i = 0; i < rhs.size(); ++i)
        rhs_cells[i + 1] = get_cell(std::get<1>(rhs[i]));

    std::sort(begin(lhs_cells), end(lhs_cells));
    std::sort(begin(rhs_cells), end(rhs_cells));

    // Todo: do we need to scale sharing with edge weights in some sort?
    // Is sharing based on cells only already good enough? Needs experimentation.

    std::size_t num_different = 0;

    // Needs to be wrapped in std::function for Windows compiler bug
    auto out = boost::make_function_output_iterator(
        std::function<void(CellID)>([&](auto) { num_different += 1; }));

    std::set_difference(begin(lhs_cells), end(lhs_cells), begin(rhs_cells), end(rhs_cells), out);

    const auto difference = static_cast<double>(num_different) / lhs.size();

    BOOST_ASSERT(difference >= 0.);
    BOOST_ASSERT(difference <= 1.);

    return 1. - difference;
}

// Filters packed paths with similar cells compared to the primary route. Mutates range in-place.
// Returns an iterator to the filtered range's new end.
template <typename RandIt>
RandIt filterPackedPathsByCellSharing(const WeightedViaNodePackedPath &path,
                                      const Partition &partition,
                                      RandIt first,
                                      RandIt last)
{
    util::static_assert_iter_category<RandIt, std::random_access_iterator_tag>();
    util::static_assert_iter_value<RandIt, WeightedViaNodePackedPath>();

    const auto over_sharing_limit = [&](const auto &packed) {
        return normalizedPackedPathSharing(partition, path.path, packed.path) > kGamma;
    };

    return std::remove_if(first, last, over_sharing_limit);
}

// Filters packed paths based on local optimality. Mutates range in-place.
// Returns an iterator to the filtered range's new end.
template <typename RandIt>
RandIt filterPackedPathsByLocalOptimality(const WeightedViaNodePackedPath &path,
                                          const Heap &forward_heap,
                                          const Heap &reverse_heap,
                                          RandIt first,
                                          RandIt last)
{
    util::static_assert_iter_category<RandIt, std::random_access_iterator_tag>();
    util::static_assert_iter_value<RandIt, WeightedViaNodePackedPath>();

    // Check sub-path optimality on alternative path crossing the via node candidate.
    //
    //   s - - - v - - - t  our packed path made up of (from, to) edges
    //        |--|--|       sub-paths "left" and "right" of v based on threshold
    //        f  v  l       nodes in [v,f] and in [v, l] have to match predecessor in heaps

    // Todo: this approach is efficient but works on packed paths only. Do we need to do a
    // thorough check on the unpacked paths instead? Or do we even need to introduce two
    // new thread-local heaps for the mld SearchEngineData and do proper s-t routing here?

    BOOST_ASSERT(path.via.weight != INVALID_EDGE_WEIGHT);

    const EdgeWeight threshold = kAlpha * path.via.weight;

    const auto is_not_locally_optimal = [&](const auto &packed) {
        BOOST_ASSERT(packed.via.weight != INVALID_EDGE_WEIGHT);
        BOOST_ASSERT(packed.via.node != SPECIAL_NODEID);
        BOOST_ASSERT(!packed.path.empty());
        // Todo: we hit path.empty() when e.g. extracting bavaria but querying in berlin

        const NodeID via = packed.via.node;

        // Todo: check the plateaux property not only for one-hop parents but for all in
        // the ranges [f, via] and [via, l] which we determined based on the threshold.
        // See below for plateaux property explanation and the one-hop implementation.

        /*
        const PackedPath &path = packed.path;

        const EdgeWeight via_weight_in_forward_search = forward_heap.GetKey(via);
        const EdgeWeight via_weight_in_reverse_search = reverse_heap.GetKey(via);

        BOOST_ASSERT(via_weight_in_forward_search != INVALID_EDGE_WEIGHT);
        BOOST_ASSERT(via_weight_in_reverse_search != INVALID_EDGE_WEIGHT);

        // Todo: Triggers. Why? I think we don't have the stepping fully correct yet.
        //
        // BOOST_ASSERT(via_weight_in_forward_search + via_weight_in_reverse_search ==
        //              packed.via.weight);

        // Gather sub-paths around node v with weights smaller than weight(v)
        // in both the forward and the reverse search. Then check predecessors.

        const EdgeWeight forward_min_weight =
            std::max(EdgeWeight{0}, via_weight_in_forward_search - threshold);
        const EdgeWeight reverse_min_weight =
            std::max(EdgeWeight{0}, via_weight_in_reverse_search - threshold);

        // Todo: can we do this without the transformation and the new vector here?
        // If not we should benchmark and think about a stack allocator.
        std::vector<NodeID> path_nodes(path.size() + 1);

        path_nodes[0] = std::get<0>(path.front());

        for (std::size_t i = 0; i < path.size(); ++i)
            path_nodes[i + 1] = std::get<1>(path[i]);

        const auto via_it = std::find(begin(path_nodes), end(path_nodes), via);
        BOOST_ASSERT(via_it != end(path_nodes));

        const std::size_t via_at = via_it - begin(path_nodes);
        BOOST_ASSERT(via_at >= 0 && via_at < path_nodes.size());

        // From via go towards s and find the node where we are over the threshold.
        // Then, from via go towards t and fine the node where we are over the threshold.

        // Todo: refactor into using iterators and reverse_iterators
        // Todo: refactor into using lower_bound; nodes on path are sorted wrt. their weight

        std::size_t forward_min_index = via_at;
        std::size_t reverse_min_index = via_at;

        for (std::size_t i = via_at; i > 0; --i)
        {
            const auto node = path_nodes[i];
            const auto weight = forward_heap.GetKey(node);

            if (weight < forward_min_weight)
            {
                forward_min_index = i;
                break;
            }
        }

        for (std::size_t i = via_at; i < path_nodes.size(); ++i)
        {
            const auto node = path_nodes[i];
            const auto weight = reverse_heap.GetKey(node);

            if (weight < reverse_min_weight)
            {
                reverse_min_index = i;
                break;
            }
        }

        // We now have the ranges [f, via] and [via, l] which we have to test for optimality.
        // We can do this by checking the node predecessors in the heaps, these are optimal.

        const auto forward_subpath_first = begin(path_nodes) + forward_min_index;
        const auto forward_subpath_last = begin(path_nodes) + via_at;

        const auto reverse_subpath_first = begin(path_nodes) + via_at;
        const auto reverse_subpath_last = begin(path_nodes) + reverse_min_index;
        */

        // Plateaux iff via == parent_in_reverse_search(parent_in_forward_search(via))
        //
        // Forward search starts from s, reverse search starts from t:
        //  - parent_in_forward_search(via) = a
        //  - parent_in_reverse_search(a) = b  != via and therefore not local optimal
        //
        //        via
        //      .'   '.
        // s - a - - - b - t
        //
        // Care needs to be taken for the edge case where the via node is on the border
        // of the search spaces and therefore parent pointers may not be valid in heaps.
        // In these cases we know we can't have local optimality around the via already.

        const auto parent_in_forward = forward_heap.GetData(via).parent;
        const auto parent_in_reverse = reverse_heap.GetData(via).parent;

        // Edge case where via is at the border of the search spaces;
        // we may not have seen the node's parent in the dual heap.
        if (!forward_heap.WasInserted(parent_in_reverse))
            return true;
        if (!reverse_heap.WasInserted(parent_in_forward))
            return true;

        const auto parent_parent_in_forward = forward_heap.GetData(parent_in_reverse).parent;
        const auto parent_parent_in_reverse = reverse_heap.GetData(parent_in_forward).parent;

        const auto plateaux = parent_parent_in_forward == via && parent_parent_in_reverse == via;

        return !plateaux;
    };

    return std::remove_if(first, last, is_not_locally_optimal);
}

// The unpacked paths' sharing in [0, 1] for no sharing and equality, respectively.
inline double normalizedUnpackedPathSharing(const UnpackedNodes &lhs, const UnpackedNodes &rhs)
{
    if (lhs.empty() || rhs.empty())
        return 0.;

    // Todo: stack alloc?
    std::vector<NodeID> lhs_nodes{begin(lhs), end(lhs)};
    std::vector<NodeID> rhs_nodes{begin(rhs), end(rhs)};

    std::sort(begin(lhs_nodes), end(lhs_nodes));
    std::sort(begin(rhs_nodes), end(rhs_nodes));

    std::size_t num_different = 0;

    // Needs to be wrapped in std::function for Windows compiler bug
    auto out = boost::make_function_output_iterator(
        std::function<void(NodeID)>([&](auto) { num_different += 1; }));

    std::set_difference(begin(lhs_nodes), end(lhs_nodes), begin(rhs_nodes), end(rhs_nodes), out);

    const auto difference = static_cast<double>(num_different) / lhs.size();

    BOOST_ASSERT(difference >= 0.);
    BOOST_ASSERT(difference <= 1.);

    // Todo: do we need to scale sharing with edge weights in some sort?

    return 1. - difference;
}

// Filters unpacked paths compared to the primary route. Mutates range in-place.
// Returns an iterator to the filtered range's new end.
template <typename RandIt>
RandIt
filterUnpackedPathsBySharing(const WeightedViaNodeUnpackedPath &path, RandIt first, RandIt last)
{
    util::static_assert_iter_category<RandIt, std::random_access_iterator_tag>();
    util::static_assert_iter_value<RandIt, WeightedViaNodeUnpackedPath>();

    const auto over_sharing_limit = [&](const auto &unpacked) {
        return normalizedUnpackedPathSharing(path.nodes, unpacked.nodes) > kGamma;
    };

    return std::remove_if(first, last, over_sharing_limit);
}

} // anon. ns

// Alternative Routes for MLD.
//
// Start search from s and continue "for a while" when midd was found. Save all vertices.
// Start search from t and continue "for a while" when s was found. Save all vertices.
// Intersect both vertex sets: these are the candidate vertices.
// For all candidate vertices c a (potentially arbitrarily bad) alternative route is (s, c, t).
// Apply heuristic to evaluate alternative route based on stretch, overlap, how reasonable it is.
//
// For MLD specifically we can pull off some tricks to make evaluating alternatives fast:
//   Only consider (s, c, t) with c border vertex: re-use MLD search steps.
//   Add meta data to border vertices: consider (s, c, t) only when c is e.g. on a highway.
//   Prune based on vertex cell id
//
// https://github.com/Project-OSRM/osrm-backend/issues/3905
InternalManyRoutesResult
alternativePathSearch(SearchEngineData<Algorithm> &search_engine_data,
                      const datafacade::ContiguousInternalMemoryDataFacade<Algorithm> &facade,
                      const PhantomNodes &phantom_node_pair)
{
    const Partition &partition = facade.GetMultiLevelPartition();

    search_engine_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());

    Heap &forward_heap = *search_engine_data.forward_heap_1;
    Heap &reverse_heap = *search_engine_data.reverse_heap_1;

    insertNodesInHeaps(forward_heap, reverse_heap, phantom_node_pair);

    // Saves nodes in the forward and backward search space overlap as candidates

    // The single via node in the shortest paths s,via and via,t sub-paths and
    // the weight for the shortest path s,t we return and compare alternatives to.
    NodeID shortest_path_via = SPECIAL_NODEID;
    EdgeWeight shortest_path_weight = INVALID_EDGE_WEIGHT;

    // The current via node during search spaces overlap stepping and an artificial
    // weight (overlap factor * shortest path weight) we use as a termination criteria.
    NodeID overlap_via = SPECIAL_NODEID;
    EdgeWeight overlap_weight = INVALID_EDGE_WEIGHT;

    // All via nodes in the overlapping search space (except the shortest path via node).
    // Will be filtered and ranked and then used for s,via and via,t alternative paths.
    std::vector<WeightedViaNode> candidate_vias;

    // The logic below is a bit weird - here's why: we want to re-use the MLD routingStep for
    // stepping our search space from s and from t. We don't know how far to overlap until we have
    // the shortest path. Once we have the shortest path we can use its weight to terminate when
    // we're over factor * weight. We have to set the weight for routingStep to INVALID_EDGE_WEIGHT
    // so that stepping will continue even after we reached the shortest path upper bound.

    const bool force_loop_forward = needsLoopForward(phantom_node_pair);
    const bool force_loop_backward = needsLoopBackwards(phantom_node_pair);

    EdgeWeight forward_heap_min = forward_heap.MinKey();
    EdgeWeight reverse_heap_min = reverse_heap.MinKey();

    while (forward_heap.Size() + reverse_heap.Size() > 0)
    {
        if (shortest_path_weight != INVALID_EDGE_WEIGHT)
            overlap_weight = shortest_path_weight * kSearchSpaceOverlapFactor;

        // Termination criteria - when we have a shortest path this will guarantee for our overlap.
        const bool keep_going = forward_heap_min + reverse_heap_min < overlap_weight;

        if (!keep_going)
            break;

        // Force forward step to not break early when we reached the middle, continue for overlap.
        overlap_weight = INVALID_EDGE_WEIGHT;

        if (!forward_heap.Empty())
        {
            routingStep<FORWARD_DIRECTION>(facade,
                                           forward_heap,
                                           reverse_heap,
                                           overlap_via,
                                           overlap_weight,
                                           force_loop_forward,
                                           force_loop_backward,
                                           phantom_node_pair);

            if (!forward_heap.Empty())
                forward_heap_min = forward_heap.MinKey();

            if (overlap_weight != INVALID_EDGE_WEIGHT && overlap_via != SPECIAL_NODEID)
            {
                if (shortest_path_via != SPECIAL_NODEID)
                {
                    candidate_vias.push_back(WeightedViaNode{overlap_via, overlap_weight});
                }
                else
                {
                    shortest_path_via = overlap_via;
                }
            }
        }

        // Adjusting upper bound for forward search
        shortest_path_weight = std::min(shortest_path_weight, overlap_weight);
        // Force reverse step to not break early when we reached the middle, continue for overlap.
        overlap_weight = INVALID_EDGE_WEIGHT;

        if (!reverse_heap.Empty())
        {
            routingStep<REVERSE_DIRECTION>(facade,
                                           reverse_heap,
                                           forward_heap,
                                           overlap_via,
                                           overlap_weight,
                                           force_loop_forward,
                                           force_loop_backward,
                                           phantom_node_pair);

            if (!reverse_heap.Empty())
                reverse_heap_min = reverse_heap.MinKey();

            if (overlap_weight != INVALID_EDGE_WEIGHT && overlap_via != SPECIAL_NODEID)
            {
                if (shortest_path_via != SPECIAL_NODEID)
                {
                    candidate_vias.push_back(WeightedViaNode{overlap_via, overlap_weight});
                }
                else
                {
                    shortest_path_via = overlap_via;
                }
            }
        }

        // Adjusting upper bound for reverse search
        shortest_path_weight = std::min(shortest_path_weight, overlap_weight);
    }

    const bool has_valid_shortest_path_weight = shortest_path_weight != INVALID_EDGE_WEIGHT;
    const bool has_valid_shortest_path_via = shortest_path_via != SPECIAL_NODEID;
    const bool has_shortest_path = has_valid_shortest_path_weight && has_valid_shortest_path_via;

    if (!has_shortest_path)
        return InternalManyRoutesResult{};

    std::cout << ">>> shortest path weight: " << shortest_path_weight << std::endl;
    std::cout << ">>> number of candidates: " << candidate_vias.size() << std::endl;

    // Filters via candidate nodes with heuristics

    // Note: filter pipeline below only makes range smaller; no need to erase items
    // from the vector when we can mutate in-place and for filtering adjust iterators.
    auto it = end(candidate_vias);

    it = filterViaCandidatesByUniqueNodeIds(begin(candidate_vias), it);
    it = filterViaCandidatesByRoadImportance(begin(candidate_vias), it, facade);
    it = filterViaCandidatesByStretch(begin(candidate_vias), it, shortest_path_weight);

    // Pre-rank by weight; sharing filtering below then discards by similarity.
    std::sort(begin(candidate_vias), it, [](const auto lhs, const auto rhs) {
        return lhs.weight < rhs.weight;
    });

    // Filtered and ranked candidate range
    const auto first = begin(candidate_vias);
    const auto last = it;
    const auto number_of_candidate_vias = last - first;

    std::cout << ">>> number of filtered candidates: " << number_of_candidate_vias << std::endl;

    // Reconstruct packed paths from the heaps.
    // The recursive path unpacking below destructs heaps.
    // We need to save all packed paths from the heaps upfront.

    const auto extract_packed_path_from_heaps = [&](WeightedViaNode via) {
        auto packed_path = retrievePackedPathFromHeap(forward_heap, reverse_heap, via.node);
        return WeightedViaNodePackedPath{std::move(via), std::move(packed_path)};
    };

    std::vector<WeightedViaNodePackedPath> weighted_packed_paths;
    weighted_packed_paths.reserve(1 + number_of_candidate_vias);

    // Store shortest path
    WeightedViaNode shortest_path_weighted_via{shortest_path_via, shortest_path_weight};
    weighted_packed_paths.push_back(extract_packed_path_from_heaps(shortest_path_weighted_via));

    // Store all alternative packed paths (if there are any).
    auto into = std::back_inserter(weighted_packed_paths);
    std::transform(first, last, into, extract_packed_path_from_heaps);

    // Filter packed paths with heuristics

    auto alternative_paths_last = end(weighted_packed_paths);

    alternative_paths_last = filterPackedPathsByLocalOptimality(weighted_packed_paths[0],
                                                                forward_heap, // paths for s, via
                                                                reverse_heap, // paths for via, t
                                                                begin(weighted_packed_paths) + 1,
                                                                alternative_paths_last);

    const auto number_of_alternative_paths =
        alternative_paths_last - (begin(weighted_packed_paths) + 1);

    // Todo: refactor - this is ugly af

    auto number_of_requested_alternative_paths =
        std::min(static_cast<std::size_t>(kMaxAlternatives),
                 static_cast<std::size_t>(number_of_alternative_paths));

    for (std::size_t i = 1; i < number_of_requested_alternative_paths; ++i)
    {
        for (std::size_t j = 0; j < i; ++j)
        {
            alternative_paths_last =
                filterPackedPathsByCellSharing(weighted_packed_paths[j],
                                               partition,
                                               begin(weighted_packed_paths) + i,
                                               alternative_paths_last);
        }
    }

    number_of_requested_alternative_paths = std::min(
        static_cast<std::size_t>(kMaxAlternatives),
        static_cast<std::size_t>(alternative_paths_last - (begin(weighted_packed_paths) + 1)));

    // ^ refactor

    const auto paths_first = begin(weighted_packed_paths);
    const auto paths_last =
        begin(weighted_packed_paths) + 1 + number_of_requested_alternative_paths;
    const auto number_of_packed_paths = paths_last - paths_first;

    // Todo: pick x times more paths to unpack here than the user requested.
    // Todo: benchmark how many packed paths we can unnpack in our time budget.

    std::vector<WeightedViaNodeUnpackedPath> unpacked_paths;
    unpacked_paths.reserve(number_of_packed_paths);

    for (auto it = paths_first; it != paths_last; ++it)
    {
        const auto packed_path_weight = it->via.weight;
        const auto packed_path_via = it->via.node;

        const auto &packed_path = it->path;

        //
        // Todo: dup. code with mld::search except for level entry: we run a slight mld::search
        //       adaption here and then dispatch to mld::search for recursively descending down.
        //

        std::vector<NodeID> unpacked_nodes;
        std::vector<EdgeID> unpacked_edges;
        unpacked_nodes.reserve(packed_path.size());
        unpacked_edges.reserve(packed_path.size());

        // Beware the edge case when start, via, end are all the same.
        // In this case we return a single node, no edges. We also don't unpack.
        if (packed_path.empty())
        {
            const auto source_node = packed_path_via;
            unpacked_nodes.push_back(source_node);
        }
        else
        {
            const auto source_node = std::get<0>(packed_path.front());
            unpacked_nodes.push_back(source_node);
        }

        for (auto const &packed_edge : packed_path)
        {
            NodeID source, target;
            bool overlay_edge;
            std::tie(source, target, overlay_edge) = packed_edge;
            if (!overlay_edge)
            { // a base graph edge
                unpacked_nodes.push_back(target);
                unpacked_edges.push_back(facade.FindEdge(source, target));
            }
            else
            { // an overlay graph edge
                LevelID level = getNodeQueryLevel(partition, source, phantom_node_pair); // XXX
                CellID parent_cell_id = partition.GetCell(level, source);
                BOOST_ASSERT(parent_cell_id == partition.GetCell(level, target));

                LevelID sublevel = level - 1;

                // Here heaps can be reused, let's go deeper!
                forward_heap.Clear();
                reverse_heap.Clear();
                forward_heap.Insert(source, 0, {source});
                reverse_heap.Insert(target, 0, {target});

                // TODO: when structured bindings will be allowed change to
                // auto [subpath_weight, subpath_source, subpath_target, subpath] = ...
                EdgeWeight subpath_weight;
                std::vector<NodeID> subpath_nodes;
                std::vector<EdgeID> subpath_edges;
                std::tie(subpath_weight, subpath_nodes, subpath_edges) = search(search_engine_data,
                                                                                facade,
                                                                                forward_heap,
                                                                                reverse_heap,
                                                                                force_loop_forward,
                                                                                force_loop_backward,
                                                                                INVALID_EDGE_WEIGHT,
                                                                                sublevel,
                                                                                parent_cell_id);
                BOOST_ASSERT(!subpath_edges.empty());
                BOOST_ASSERT(subpath_nodes.size() > 1);
                BOOST_ASSERT(subpath_nodes.front() == source);
                BOOST_ASSERT(subpath_nodes.back() == target);
                unpacked_nodes.insert(
                    unpacked_nodes.end(), std::next(subpath_nodes.begin()), subpath_nodes.end());
                unpacked_edges.insert(
                    unpacked_edges.end(), subpath_edges.begin(), subpath_edges.end());
            }
        }

        //
        // Filter and rank a second time. This time instead of being fast and doing
        // heuristics on the packed path only we now have the detailed unpacked path.
        //

        WeightedViaNodeUnpackedPath unpacked_path{
            WeightedViaNode{packed_path_via, packed_path_weight},
            std::move(unpacked_nodes),
            std::move(unpacked_edges)};

        unpacked_paths.push_back(std::move(unpacked_path));
    }

    //
    // Annotate the unpacked path and transform to proper internal route result.
    //

    std::vector<InternalRouteResult> routes;
    routes.reserve(unpacked_paths.size());

    const auto unpacked_path_to_route = [&](const WeightedViaNodeUnpackedPath &path) {
        return extractRoute(facade, path.via.weight, phantom_node_pair, path.nodes, path.edges);
    };

    std::transform(begin(unpacked_paths),
                   end(unpacked_paths),
                   std::back_inserter(routes),
                   unpacked_path_to_route);

    return InternalManyRoutesResult{std::move(routes)};
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm}
