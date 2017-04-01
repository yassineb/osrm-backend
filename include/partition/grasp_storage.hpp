#ifndef OSRM_PARTITION_GRASP_STORAGE_HPP
#define OSRM_PARTITION_GRASP_STORAGE_HPP

#include "partition/cell_storage.hpp"

#include "customizer/grasp_customization_graph.hpp"

#include "storage/io_fwd.hpp"

#include "util/assert.hpp"
#include "util/log.hpp"
#include "util/static_graph.hpp"
#include "util/vector_view.hpp"

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace osrm
{
namespace partition
{
namespace detail
{
template <storage::Ownership Ownership> class GRASPStorageImpl;
}
using GRASPStorage = detail::GRASPStorageImpl<storage::Ownership::Container>;
using GRASPStorageView = detail::GRASPStorageImpl<storage::Ownership::View>;

namespace serialization
{
template <storage::Ownership Ownership>
inline void read(storage::io::FileReader &reader, detail::GRASPStorageImpl<Ownership> &storage);
template <storage::Ownership Ownership>
inline void write(storage::io::FileWriter &writer,
                  const detail::GRASPStorageImpl<Ownership> &storage);
}

namespace detail
{

template <storage::Ownership Ownership> class GRASPStorageImpl
{
    using GRASPData = customizer::GRASPData;
    using DownwardsGraph = util::StaticGraph<GRASPData, Ownership>;
    using DownwardEdge = typename DownwardsGraph::InputEdge;

  public:
    GRASPStorageImpl() = default;

    template <typename GraphT,
              typename = std::enable_if<Ownership == storage::Ownership::Container>,
              typename CellStorageT>
    GRASPStorageImpl(const partition::MultiLevelPartition &partition,
                     const GraphT &base_graph,
                     const CellStorageT &cell_storage)
    {
        std::vector<DownwardEdge> edges;

        // Level 1: We need to insert an arc to all nodes in the base graph
        // TODO: This is super wasteful IMHO, the paper wants it that way
        // but we could also try to fall back to a search on the base graph
        for (auto node : util::irange<NodeID>(0, base_graph.GetNumberOfNodes()))
        {
            auto parent_cell = cell_storage.GetCell(1, partition.GetCell(1, node));
            for (const auto source : parent_cell.GetSourceNodes())
            {
                edges.emplace_back(node, source, INVALID_EDGE_WEIGHT);
            }
        }

        for (auto level : util::irange<LevelID>(2, partition.GetNumberOfLevels()))
        {
            for (auto cell : util::irange<CellID>(0, partition.GetNumberOfCells(level)))
            {
                auto parent = cell_storage.GetCell(level, cell);

                for (auto sub_cell = partition.BeginChildren(level, cell);
                     sub_cell < partition.EndChildren(level, cell);
                     ++sub_cell)
                {
                    auto child = cell_storage.GetCell(level - 1, sub_cell);

                    for (auto child_source : child.GetSourceNodes())
                    {
                        for (auto parent_source : parent.GetSourceNodes())
                        {
                            edges.emplace_back(child_source, parent_source, INVALID_EDGE_WEIGHT);
                        }
                    }
                }
            }
        }

        tbb::parallel_sort(edges.begin(), edges.end());
        auto new_end = std::unique(edges.begin(), edges.end());
        edges.resize(new_end - edges.begin());
        downwards_graph = DownwardsGraph{base_graph.GetNumberOfNodes(), edges};
    }

    customizer::GRASPCustomizationGraph GetCustomizationGraph() const
    {
        auto edges = downwards_graph.ToEdges();

        auto edge_range = tbb::blocked_range<std::size_t>(0, edges.size());
        tbb::parallel_for(edge_range, [&](const auto &range) {
            for (auto idx = range.begin(); idx < range.end(); ++idx)
            {
                std::swap(edges[idx].source, edges[idx].target);
            }
        });

        tbb::parallel_sort(edges.begin(), edges.end());

        return customizer::GRASPCustomizationGraph(downwards_graph.GetNumberOfNodes(), edges);
    }

    void SetDownwardEdges(const customizer::GRASPCustomizationGraph &customization_graph)
    {
        auto node_range = tbb::blocked_range<NodeID>(0, customization_graph.GetNumberOfNodes());
        tbb::parallel_for(node_range, [&, this](const auto &range) {
            for (auto node = range.begin(); node < range.end(); ++node)
            {
                for (auto edge : customization_graph.GetAdjacentEdgeRange(node))
                {
                    const auto target = customization_graph.GetTarget(edge);
                    const auto rev_edge = downwards_graph.FindEdge(target, node);
                    auto &data = customization_graph.GetEdgeData(edge);
                    auto &rev_data = downwards_graph.GetEdgeData(rev_edge);
                    rev_data.weight = data.weight;
                }
            }
        });
    }

  private:
    friend void serialization::read<Ownership>(storage::io::FileReader &reader,
                                               GRASPStorageImpl &storage);
    friend void serialization::write<Ownership>(storage::io::FileWriter &writer,
                                                const GRASPStorageImpl &storage);

    // maps a node to its incoming edges "downward edges"
    DownwardsGraph downwards_graph;
};
}
}
}

#endif // OSRM_CUSTOMIZE_CELL_STORAGE_HPP
