#include "common/range_tools.hpp"
#include <boost/test/unit_test.hpp>

#include "customizer/grasp_cell_customizer.hpp"
#include "partition/grasp_storage.hpp"
#include "partition/cell_storage.hpp"
#include "partition/multi_level_graph.hpp"
#include "partition/multi_level_partition.hpp"
#include "util/static_graph.hpp"

using namespace osrm;
using namespace osrm::customizer;
using namespace osrm::partition;
using namespace osrm::util;

namespace
{
struct MockEdge
{
    NodeID start;
    NodeID target;
    EdgeWeight weight;
};

auto makeGraph(const MultiLevelPartition &mlp, const std::vector<MockEdge> &mock_edges)
{
    struct EdgeData
    {
        EdgeWeight weight;
        bool forward;
        bool backward;
    };
    using Edge = static_graph_details::SortableEdgeWithData<EdgeData>;
    std::vector<Edge> edges;
    std::size_t max_id = 0;
    for (const auto &m : mock_edges)
    {
        max_id = std::max<std::size_t>(max_id, std::max(m.start, m.target));
        edges.push_back(Edge{m.start, m.target, m.weight, true, false});
        edges.push_back(Edge{m.target, m.start, m.weight, false, true});
    }
    std::sort(edges.begin(), edges.end());
    return partition::MultiLevelGraph<EdgeData, osrm::storage::Ownership::Container>(
        mlp, max_id + 1, edges);
}
}

BOOST_AUTO_TEST_SUITE(grasp_customization_tests)

BOOST_AUTO_TEST_CASE(two_level_test)
{
}

BOOST_AUTO_TEST_SUITE_END()
