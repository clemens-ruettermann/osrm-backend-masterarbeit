#ifndef OSRM_CONTRACTOR_GRAPH_CONTRACTION_ADAPTORS_HPP_
#define OSRM_CONTRACTOR_GRAPH_CONTRACTION_ADAPTORS_HPP_

#include "contractor/contractor_graph.hpp"

#include "util/log.hpp"
#include "util/percent.hpp"

#include <tbb/parallel_sort.h>

#include <vector>

namespace osrm
{
namespace contractor
{

// Make sure to move in the input edge list!
template <typename InputEdgeContainer>
ContractorGraph toContractorGraph(NodeID number_of_nodes, InputEdgeContainer input_edge_list)
{
    std::vector<ContractorEdge> edges;
    edges.reserve(input_edge_list.size() * 2);
	auto max_consumption = -2147483647;
	auto min_consumption = 2147483648;

    for (const auto &input_edge : input_edge_list)
    {
        if (input_edge.data.weight == INVALID_EDGE_WEIGHT) {
	        continue;
		}

		if (input_edge.data.consumption == INVALID_EDGE_CONSUMPTION) {
			util::Log(logWARNING) << "INVALID EDGE CONSUMPTION" << std::endl;
		}

#ifndef NDEBUG
        const unsigned int constexpr DAY_IN_DECI_SECONDS = 24 * 60 * 60 * 10;
        if (static_cast<unsigned int>(std::max(input_edge.data.weight, 1)) > DAY_IN_DECI_SECONDS)
        {
            util::Log(logWARNING) << "Edge weight large -> "
                                  << static_cast<unsigned int>(std::max(input_edge.data.weight, 1))
                                  << " : " << static_cast<unsigned int>(input_edge.source) << " -> "
                                  << static_cast<unsigned int>(input_edge.target);
        }

#endif

		if (max_consumption < input_edge.data.consumption) {
			max_consumption = input_edge.data.consumption;
		}
		if (min_consumption > input_edge.data.consumption) {
			min_consumption = input_edge.data.consumption;
		}
        auto new_edge1 = edges.emplace_back(ContractorEdge {input_edge.source,
                           input_edge.target,
                           std::max(input_edge.data.weight, 1),
                           input_edge.data.duration,
                           input_edge.data.distance,
						   input_edge.data.consumption,
                           1,
                           input_edge.data.turn_id,
                           false,
                           input_edge.data.forward,
                           input_edge.data.backward});

#ifdef NON_ZERO_CONSUMPTION
		BOOST_ASSERT(new_edge1.consumption != 0);
#endif

        auto new_edge2 = edges.emplace_back(ContractorEdge {input_edge.target,
                           input_edge.source,
                           std::max(input_edge.data.weight, 1),
                           input_edge.data.duration,
                           input_edge.data.distance,
						   input_edge.data.consumption,
                           1,
                           input_edge.data.turn_id,
                           false,
                           input_edge.data.backward,
                           input_edge.data.forward});
#ifdef NON_ZERO_CONSUMPTION
	    BOOST_ASSERT(new_edge2.consumption != 0);
#endif
    };

	util::Log(logWARNING) << "Edge consumption max: " << max_consumption << std::flush;
	util::Log(logWARNING) << "Edge consumption min: " << min_consumption << std::flush;

    tbb::parallel_sort(edges.begin(), edges.end());

    NodeID edge = 0;
    for (NodeID i = 0; i < edges.size();)
    {
        const NodeID source = edges[i].source;
        const NodeID target = edges[i].target;
        const NodeID id = edges[i].data.id;
        // remove eigenloops
        if (source == target)
        {
            ++i;
            continue;
        }
        ContractorEdge forward_edge;
        ContractorEdge reverse_edge;
        forward_edge.source = reverse_edge.source = source;
        forward_edge.target = reverse_edge.target = target;
        forward_edge.data.forward = reverse_edge.data.backward = true;
        forward_edge.data.backward = reverse_edge.data.forward = false;
        forward_edge.data.shortcut = reverse_edge.data.shortcut = false;
        forward_edge.data.id = reverse_edge.data.id = id;
        forward_edge.data.originalEdges = reverse_edge.data.originalEdges = 1;
        forward_edge.data.weight = reverse_edge.data.weight = INVALID_EDGE_WEIGHT;
        forward_edge.data.duration = reverse_edge.data.duration = MAXIMAL_EDGE_DURATION;
        forward_edge.data.distance = reverse_edge.data.distance = MAXIMAL_EDGE_DISTANCE;
		forward_edge.data.consumption = reverse_edge.data.consumption = INVALID_EDGE_CONSUMPTION;
	    // remove parallel edges
        while (i < edges.size() && edges[i].source == source && edges[i].target == target)
        {
            if (edges[i].data.forward) {

	            forward_edge.data.weight = std::min(forward_edge.data.weight, edges[i].data.weight);
	            forward_edge.data.duration = std::min(forward_edge.data.duration, edges[i].data.duration);
	            forward_edge.data.distance = std::min(forward_edge.data.distance, edges[i].data.distance);
	            forward_edge.data.consumption = std::min(forward_edge.data.consumption, edges[i].data.consumption);

            }
			if (edges[i].data.backward) {
				reverse_edge.data.weight = std::min(reverse_edge.data.weight, edges[i].data.weight);
				reverse_edge.data.duration = std::min(reverse_edge.data.duration, edges[i].data.duration);
				reverse_edge.data.distance = std::min(reverse_edge.data.distance, edges[i].data.distance);
				reverse_edge.data.consumption = std::min(reverse_edge.data.consumption, edges[i].data.consumption);
            }
            ++i;
        }
        // merge edges (s,t) and (t,s) into bidirectional edge
        if (forward_edge.data.weight == reverse_edge.data.weight && forward_edge.data.consumption == reverse_edge.data.consumption) {
            if ((int)forward_edge.data.weight != INVALID_EDGE_WEIGHT)
            {
                forward_edge.data.backward = true;
                edges[edge++] = forward_edge;
            }
        } else { // insert seperate edges
            if (((int)forward_edge.data.weight) != INVALID_EDGE_WEIGHT) {
                edges[edge++] = forward_edge;
				if (forward_edge.data.consumption == INVALID_EDGE_CONSUMPTION
#ifdef NON_ZERO_CONSUMPTION
		            || forward_edge.data.consumption == 0
#endif
				) {
					util::Log(logWARNING) << "Inserted edge with invalid consumption";
				}
            }
            if ((int)reverse_edge.data.weight != INVALID_EDGE_WEIGHT) {
                edges[edge++] = reverse_edge;
	            if (reverse_edge.data.consumption == INVALID_EDGE_CONSUMPTION
#ifdef NON_ZERO_CONSUMPTION
		            || reverse_edge.data.consumption == 0
#endif
				) {
		            util::Log(logWARNING) << "Inserted edge with invalid consumption";
	            }
            }
        }
    }
    util::Log() << "merged " << edges.size() - edge << " edges out of " << edges.size();
    edges.resize(edge);

    return ContractorGraph{number_of_nodes, edges};
}

template <class Edge, typename GraphT> inline std::vector<Edge> toEdges(GraphT graph)
{
    util::Log() << "Converting contracted graph with " << graph.GetNumberOfEdges()
                << " to edge list (" << (graph.GetNumberOfEdges() * sizeof(Edge)) << " bytes)";
    std::vector<Edge> edges(graph.GetNumberOfEdges());

    {
        util::UnbufferedLog log;
        log << "Getting edges of minimized graph ";
        util::Percent p(log, graph.GetNumberOfNodes());
        const NodeID number_of_nodes = graph.GetNumberOfNodes();
        std::size_t edge_index = 0;
        for (const auto node : util::irange(0u, number_of_nodes))
        {
            p.PrintStatus(node);
            for (auto edge : graph.GetAdjacentEdgeRange(node))
            {
                const NodeID target = graph.GetTarget(edge);
                const auto &data = graph.GetEdgeData(edge);
                auto &new_edge = edges[edge_index++];
                new_edge.source = node;
                new_edge.target = target;
                BOOST_ASSERT_MSG(SPECIAL_NODEID != new_edge.target, "Target id invalid");
                new_edge.data.weight = data.weight;
                new_edge.data.duration = data.duration;
                new_edge.data.distance = data.distance;
				new_edge.data.consumption = data.consumption;
                new_edge.data.shortcut = data.shortcut;
                new_edge.data.turn_id = data.id;
                BOOST_ASSERT_MSG(new_edge.data.turn_id != INT_MAX, // 2^31
                                 "edge id invalid");
                new_edge.data.forward = data.forward;
                new_edge.data.backward = data.backward;
            }
        }
        BOOST_ASSERT(edge_index == edges.size());
    }

    tbb::parallel_sort(edges.begin(), edges.end());

    return edges;
}

} // namespace contractor
} // namespace osrm

#endif // OSRM_CONTRACTOR_GRAPH_CONTRACTION_ADAPTORS_HPP_
