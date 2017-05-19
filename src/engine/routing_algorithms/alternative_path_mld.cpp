#include "engine/routing_algorithms/alternative_path.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

#include "util/integer_range.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <iterator>
#include <memory>
#include <unordered_set>
#include <vector>

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

// Alternative Routes for MLD.
//
// Start search from s and continue "for a while" when t was found. Save all vertices.
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
    const auto &partition = facade.GetMultiLevelPartition();

    search_engine_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());

    auto &forward_heap = *search_engine_data.forward_heap_1;
    auto &reverse_heap = *search_engine_data.reverse_heap_1;

    insertNodesInHeaps(forward_heap, reverse_heap, phantom_node_pair);

    //
    // Save nodes in the forward and backward search space overlap as candidates
    //

    // The single via node in the shortest paths s,via and via,t sub-paths and
    // the weight for the shortest path s,t we return and compare alternatives to.
    NodeID shortest_path_via = SPECIAL_NODEID;
    EdgeWeight shortest_path_weight = INVALID_EDGE_WEIGHT;

    // The current via node during search spaces overlap stepping and an artificial
    // weight (overlap factor * shortest path weight) we use as a termination criteria.
    NodeID overlap_via = SPECIAL_NODEID;
    EdgeWeight overlap_weight = INVALID_EDGE_WEIGHT;

    const auto overlap_factor = 1.66;

    // Represents a via middle node where search spaces touch and
    // the total weight a path (made up of s,via and via,t) has.
    struct WeightedViaNode
    {
        NodeID node;
        EdgeWeight weight;
    };

    // All via nodes in the overlapping search space (except the shortest path via node).
    // Will be filtered and ranked and then used for s,via and via,t alternative paths.
    std::vector<WeightedViaNode> candidate_vias;

    // The logic below is a bit weird - here's why: we want to re-use the MLD routingStep for
    // stepping our search space from s and from t. We don't know how far to overlap until we have
    // the shortest path. Once we have the shortest path we can use its weight to terminate when
    // we're over factor * weight. We have to set the weight for routingStep to INVALID_EDGE_WEIGHT
    // so that stepping will continue even after we reached the shortest path upper bound.

    const auto force_loop_forward = needsLoopForward(phantom_node_pair);
    const auto force_loop_backward = needsLoopBackwards(phantom_node_pair);

    EdgeWeight forward_heap_min = forward_heap.MinKey();
    EdgeWeight reverse_heap_min = reverse_heap.MinKey();

    while (forward_heap.Size() + reverse_heap.Size() > 0)
    {
        if (shortest_path_weight != INVALID_EDGE_WEIGHT)
            overlap_weight = shortest_path_weight * overlap_factor;

        // Termination criteria - when we have a shortest path this will guarantee for our overlap.
        const auto keep_going = forward_heap_min + reverse_heap_min < overlap_weight;

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

            if (overlap_weight != INVALID_EDGE_WEIGHT)
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

            if (overlap_weight != INVALID_EDGE_WEIGHT)
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

    const auto has_valid_shortest_path_weight = shortest_path_weight != INVALID_EDGE_WEIGHT;
    const auto has_valid_shortest_path_via = shortest_path_via != SPECIAL_NODEID;
    const auto has_shortest_path = has_valid_shortest_path_weight && has_valid_shortest_path_via;

    if (!has_shortest_path)
        return InternalManyRoutesResult{};

    //
    // Todo: filter via candidate nodes with heuristics
    //

    // Note: unique and remove_if calls below only re-shuffle candidates in-place so that
    // we never have to call candidate_vias.erase() and can avoid memory (re-) allocations.

    // We only care for unique via nodes on the paths s,via and via,t.

    std::cout << ">>> shortest path weight: " << shortest_path_weight << std::endl;
    std::cout << ">>> number of candidates: " << candidate_vias.size() << std::endl;

    std::sort(begin(candidate_vias), end(candidate_vias), [](auto lhs, auto rhs) {
        return lhs.node < rhs.node;
    });

    auto it = std::unique(begin(candidate_vias), end(candidate_vias), [](auto lhs, auto rhs) {
        return lhs.node == rhs.node;
    });

    // Filter by stretch - alternative must not be longer than x times the primary route

    // Todo: scale epsilon with weight. Higher epsilon for short routes are reasonable.
    //
    //  - shortest path 10 minutes, alternative 13 minutes => 0.30 epsilon Ok
    //  - shortest path 10 hours, alternative 13 hours     => 0.30 epsilon Unreasonable
    //
    // We only have generic weights here and no durations without unpacking.
    // How do we scale the epsilon in a setup where users can pass us anything as weight?
    const auto over_stretch_limit = [shortest_path_weight](const auto via) {
        const auto epsilon = 0.05;
        const auto stretch_weight_limit = (1. + epsilon) * shortest_path_weight;
        return via.weight > stretch_weight_limit;
    };

    it = std::remove_if(begin(candidate_vias), it, over_stretch_limit);

    // Filtered and ranked candidate range
    const auto first = begin(candidate_vias);
    const auto last = it;
    const auto number_of_candidate_vias = last - first;

    std::cout << ">>> number of filtered candidates: " << number_of_candidate_vias << std::endl;

    //
    // Reconstruct paths
    //

    // The recursive path unpacking below destructs heaps.
    // We need to save all packed paths from the heaps upfront.

    // Represents a complete packed path (made up of s,via,t) and its total weight.
    struct WeightedViaNodePackedPath
    {
        WeightedViaNode via;
        PackedPath path;
    };

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

    // We have at least one shortest path and potentially many alternative paths in the response.
    std::vector<InternalRouteResult> routes;
    routes.reserve(weighted_packed_paths.size());

    for (const auto weighted_packed_path : weighted_packed_paths)
    {
        const auto packed_path_weight = weighted_packed_path.via.weight;
        const auto packed_path_via = weighted_packed_path.via.node;

        const auto &packed_path = weighted_packed_path.path;

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

        // Annotate the unpacked path and transform to proper internal route result.
        routes.push_back(extractRoute(
            facade, packed_path_weight, phantom_node_pair, unpacked_nodes, unpacked_edges));
    }

    return InternalManyRoutesResult{std::move(routes)};
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm}
