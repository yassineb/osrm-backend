#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"

#include <boost/assert.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

namespace ch
{

using ManyToManyQueryHeap = SearchEngineData<Algorithm>::ManyToManyQueryHeap;

namespace
{
struct NodeBucket
{
    unsigned target_id; // essentially a row in the weight matrix
    EdgeWeight weight;
    EdgeWeight duration;
    NodeBucket(const unsigned target_id, const EdgeWeight weight, const EdgeWeight duration)
        : target_id(target_id), weight(weight), duration(duration)
    {
    }
};

// FIXME This should be replaced by an std::unordered_multimap, though this needs benchmarking
using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;

template <bool DIRECTION>
void relaxOutgoingEdges(const datafacade::ContiguousInternalMemoryDataFacade<Algorithm> &facade,
                        const NodeID node,
                        const EdgeWeight weight,
                        const EdgeWeight duration,
                        ManyToManyQueryHeap &query_heap)
{
    for (auto edge : facade.GetAdjacentEdgeRange(node))
    {
        const auto &data = facade.GetEdgeData(edge);
        if (DIRECTION == FORWARD_DIRECTION ? data.forward : data.backward)
        {
            const NodeID to = facade.GetTarget(edge);
            const EdgeWeight edge_weight = data.weight;
            const EdgeWeight edge_duration = data.duration;

            BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
            const EdgeWeight to_weight = weight + edge_weight;
            const EdgeWeight to_duration = duration + edge_duration;

            // New Node discovered -> Add to Heap + Node Info Storage
            if (!query_heap.WasInserted(to))
            {
                query_heap.Insert(to, to_weight, {node, to_duration});
            }
            // Found a shorter Path -> Update weight
            else if (to_weight < query_heap.GetKey(to))
            {
                // new parent
                query_heap.GetData(to) = {node, to_duration};
                query_heap.DecreaseKey(to, to_weight);
            }
        }
    }
}

void forwardRoutingStep(const datafacade::ContiguousInternalMemoryDataFacade<Algorithm> &facade,
                        const unsigned row_idx,
                        const unsigned number_of_targets,
                        ManyToManyQueryHeap &query_heap,
                        const SearchSpaceWithBuckets &search_space_with_buckets,
                        std::vector<EdgeWeight> &weights_table,
                        std::vector<EdgeWeight> &durations_table)
{
    const NodeID node = query_heap.DeleteMin();
    const EdgeWeight source_weight = query_heap.GetKey(node);
    const EdgeWeight source_duration = query_heap.GetData(node).duration;

    // check if each encountered node has an entry
    const auto bucket_iterator = search_space_with_buckets.find(node);
    // iterate bucket if there exists one
    if (bucket_iterator != search_space_with_buckets.end())
    {
        const std::vector<NodeBucket> &bucket_list = bucket_iterator->second;
        for (const NodeBucket &current_bucket : bucket_list)
        {
            // get target id from bucket entry
            const unsigned column_idx = current_bucket.target_id;
            const EdgeWeight target_weight = current_bucket.weight;
            const EdgeWeight target_duration = current_bucket.duration;

            auto &current_weight = weights_table[row_idx * number_of_targets + column_idx];
            auto &current_duration = durations_table[row_idx * number_of_targets + column_idx];

            // check if new weight is better
            const EdgeWeight new_weight = source_weight + target_weight;
            if (new_weight < 0)
            {
                const EdgeWeight loop_weight = ch::getLoopWeight<false>(facade, node);
                const EdgeWeight new_weight_with_loop = new_weight + loop_weight;
                if (loop_weight != INVALID_EDGE_WEIGHT && new_weight_with_loop >= 0)
                {
                    current_weight = std::min(current_weight, new_weight_with_loop);
                    current_duration = std::min(current_duration,
                                                source_duration + target_duration +
                                                    ch::getLoopWeight<true>(facade, node));
                }
            }
            else if (new_weight < current_weight)
            {
                current_weight = new_weight;
                current_duration = source_duration + target_duration;
            }
        }
    }
    if (ch::stallAtNode<FORWARD_DIRECTION>(facade, node, source_weight, query_heap))
    {
        return;
    }

    relaxOutgoingEdges<FORWARD_DIRECTION>(facade, node, source_weight, source_duration, query_heap);
}

void backwardRoutingStep(const datafacade::ContiguousInternalMemoryDataFacade<Algorithm> &facade,
                         const unsigned column_idx,
                         ManyToManyQueryHeap &query_heap,
                         SearchSpaceWithBuckets &search_space_with_buckets)
{
    const NodeID node = query_heap.DeleteMin();
    const EdgeWeight target_weight = query_heap.GetKey(node);
    const EdgeWeight target_duration = query_heap.GetData(node).duration;

    // store settled nodes in search space bucket
    search_space_with_buckets[node].emplace_back(column_idx, target_weight, target_duration);

    if (ch::stallAtNode<REVERSE_DIRECTION>(facade, node, target_weight, query_heap))
    {
        return;
    }

    relaxOutgoingEdges<REVERSE_DIRECTION>(facade, node, target_weight, target_duration, query_heap);
}
}

std::vector<EdgeWeight>
manyToManySearch(SearchEngineData<Algorithm> &engine_working_data,
                 const datafacade::ContiguousInternalMemoryDataFacade<Algorithm> &facade,
                 const std::vector<PhantomNode> &phantom_nodes,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices)
{
    const auto number_of_sources =
        source_indices.empty() ? phantom_nodes.size() : source_indices.size();
    const auto number_of_targets =
        target_indices.empty() ? phantom_nodes.size() : target_indices.size();
    const auto number_of_entries = number_of_sources * number_of_targets;

    std::vector<EdgeWeight> weights_table(number_of_entries, INVALID_EDGE_WEIGHT);
    std::vector<EdgeWeight> durations_table(number_of_entries, MAXIMAL_EDGE_DURATION);

    engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(facade.GetNumberOfNodes());

    auto &query_heap = *(engine_working_data.many_to_many_heap);

    SearchSpaceWithBuckets search_space_with_buckets;

    unsigned column_idx = 0;
    const auto search_target_phantom = [&](const PhantomNode &phantom) {
        // clear heap and insert target nodes
        query_heap.Clear();
        insertTargetInHeap(query_heap, phantom);

        // explore search space
        while (!query_heap.Empty())
        {
            backwardRoutingStep(facade, column_idx, query_heap, search_space_with_buckets);
        }
        ++column_idx;
    };

    // for each source do forward search
    unsigned row_idx = 0;
    const auto search_source_phantom = [&](const PhantomNode &phantom) {
        // clear heap and insert source nodes
        query_heap.Clear();
        insertSourceInHeap(query_heap, phantom);

        // explore search space
        while (!query_heap.Empty())
        {
            forwardRoutingStep(facade,
                               row_idx,
                               number_of_targets,
                               query_heap,
                               search_space_with_buckets,
                               weights_table,
                               durations_table);
        }
        ++row_idx;
    };

    if (target_indices.empty())
    {
        for (const auto &phantom : phantom_nodes)
        {
            search_target_phantom(phantom);
        }
    }
    else
    {
        for (const auto index : target_indices)
        {
            const auto &phantom = phantom_nodes[index];
            search_target_phantom(phantom);
        }
    }

    if (source_indices.empty())
    {
        for (const auto &phantom : phantom_nodes)
        {
            search_source_phantom(phantom);
        }
    }
    else
    {
        for (const auto index : source_indices)
        {
            const auto &phantom = phantom_nodes[index];
            search_source_phantom(phantom);
        }
    }

    return durations_table;
}

} // namespace ch


namespace mld
{
template<typename QueryHeap>
void forwardRoutingStep(const datafacade::ContiguousInternalMemoryDataFacade<Algorithm> &facade,
                        const unsigned row_idx,
                        const unsigned number_of_targets,
                        QueryHeap &query_heap,
                        std::vector<EdgeWeight> &weights_table,
                        std::vector<EdgeWeight> &durations_table)
{
    const NodeID node = query_heap.DeleteMin();
    const EdgeWeight source_weight = query_heap.GetKey(node);

    /* TODO uni-directional MLD search */
}
}

std::vector<EdgeWeight>
manyToManySearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                 const datafacade::ContiguousInternalMemoryDataFacade<ch::Algorithm> &facade,
                 const std::vector<PhantomNode> &phantom_nodes,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices)
{
    ch::manyToManySearch(engine_working_data, facade, phantom_nodes, source_indices, target_indices);
}

std::vector<EdgeWeight>
manyToManySearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                 const datafacade::ContiguousInternalMemoryDataFacade<mld::Algorithm> &facade,
                 const std::vector<PhantomNode> &phantom_nodes,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices)
{
    const auto number_of_sources =
        source_indices.empty() ? phantom_nodes.size() : source_indices.size();
    const auto number_of_targets =
        target_indices.empty() ? phantom_nodes.size() : target_indices.size();
    const auto number_of_entries = number_of_sources * number_of_targets;

    const auto &grasp_storage = facade.GetGraspStorage();

    std::vector<EdgeWeight> weights_table(number_of_entries, INVALID_EDGE_WEIGHT);
    std::vector<EdgeWeight> durations_table(number_of_entries, MAXIMAL_EDGE_DURATION);
    std::vector<bool> is_target(facade.GetNumberOfNodes(), false);

    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());

    auto &query_heap = *(engine_working_data.forward_heap_1);

    std::deque<NodeID> selection_queue;
    const auto select_node = [&](const NodeID node) {
        if (!is_target[node])
        {
            is_target[node] = true;
            for (const auto edge : grasp_storage.GetDownwardEdgeRange(node))
            {
                const auto source = grasp_storage.GetSource(edge);
                selection_queue.emplace_back(source);
            }
        }
    };

    const auto search_target_phantom = [&](const PhantomNode &phantom) {
        if (phantom.forward_segment_id.enabled)
            select_node(phantom.forward_segment_id.id);

        if (phantom.reverse_segment_id.enabled)
            select_node(phantom.reverse_segment_id.id);
    };

    // for each source do forward search
    unsigned row_idx = 0;
    const auto search_source_phantom = [&](const PhantomNode &phantom) {
        // clear heap and insert source nodes
        query_heap.Clear();
        insertNodesInHeap<FORWARD_DIRECTION>(query_heap, phantom);

        // explore search space
        while (!query_heap.Empty())
        {
            mld::forwardRoutingStep(facade,
                               row_idx,
                               number_of_targets,
                               query_heap,
                               weights_table,
                               durations_table);
        }
        ++row_idx;
    };

    if (target_indices.empty())
    {
        for (const auto &phantom : phantom_nodes)
        {
            search_target_phantom(phantom);
        }
    }
    else
    {
        for (const auto index : target_indices)
        {
            const auto &phantom = phantom_nodes[index];
            search_target_phantom(phantom);
        }
    }

    // this marks all nodes that are connected by downward edges to the target nodes
    while (!selection_queue.empty())
    {
        const auto node = selection_queue.front();
        selection_queue.pop_front();
        select_node(node);
    }

    if (source_indices.empty())
    {
        for (const auto &phantom : phantom_nodes)
        {
            search_source_phantom(phantom);
        }
    }
    else
    {
        for (const auto index : source_indices)
        {
            const auto &phantom = phantom_nodes[index];
            search_source_phantom(phantom);
        }
    }

    return durations_table;
}



} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
