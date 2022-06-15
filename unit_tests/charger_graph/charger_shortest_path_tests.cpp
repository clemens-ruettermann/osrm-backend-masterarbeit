//
// Created by kenspeckle on 3/28/22.
//
#include <boost/test/unit_test.hpp>
#include <vector>
#include "charger_graph/charger_graph.hpp"

using namespace osrm;
using namespace osrm::engine;
using namespace osrm::enav;
using namespace std;


// WARNING: The edges in adj_list MUST be sorted

BOOST_AUTO_TEST_SUITE(charger_shortest_path_tests)


std::shared_ptr<ChargerGraph> createChargerShortestPathWithoutCharging() {
	std::shared_ptr<vector<ChargerGraphEdge>> adj_list = make_shared<vector<ChargerGraphEdge>>();
	adj_list->emplace_back(0, 1, 1, 1);
	adj_list->emplace_back(1, 0, 1, 1);

	adj_list->emplace_back(1, 2, 1, 1);
	adj_list->emplace_back(1, 3, 2, 2);
	adj_list->emplace_back(2, 1, 1, 1);
	adj_list->emplace_back(2, 4, 1, 1);
	adj_list->emplace_back(3, 1, 2, 2);
	adj_list->emplace_back(3, 5, 5, 5);
	adj_list->emplace_back(4, 2, 1, 1);
	adj_list->emplace_back(4, 5, 1, 1);
	adj_list->emplace_back(5, 3, 5, 5);
	adj_list->emplace_back(5, 4, 1, 1);

	vector<Charger> charger_list;
	vector<Plug> plug_list{Plug(0, {PlugType::Typ_2})};
	charger_list.emplace_back(Charger{0, engine::PhantomNode{}, Coordinate{}, "", 0, false, plug_list});
	charger_list.emplace_back(Charger{1, engine::PhantomNode{}, Coordinate{}, "", 0, false, plug_list});
	charger_list.emplace_back(Charger{2, engine::PhantomNode{}, Coordinate{}, "", 0, false, plug_list});
	charger_list.emplace_back(Charger{3, engine::PhantomNode{}, Coordinate{}, "", 0, false, plug_list});
	charger_list.emplace_back(Charger{4, engine::PhantomNode{}, Coordinate{}, "", 0, false, plug_list});
	charger_list.emplace_back(Charger{5, engine::PhantomNode{}, Coordinate{}, "", 0, false, plug_list});

	std::shared_ptr<enav::Car> car = make_shared<enav::Car>();
	std::shared_ptr<ChargerGraph> chargerGraph = std::make_shared<ChargerGraph>(car, adj_list, charger_list);
	return chargerGraph;
}

BOOST_AUTO_TEST_CASE(shortest_tree_from_node0_no_charging_test)
{
	auto chargerGraph = createChargerShortestPathWithoutCharging();

	std::shared_ptr<enav::Car> car = make_shared<enav::Car>();

	ReachableStartNode start_node{0, 0, 0};
	auto tree_tuple = chargerGraph->buildShortestPathTree(start_node);
	auto prev = get<0>(tree_tuple);
	auto consumptions = get<1>(tree_tuple);

	BOOST_CHECK_EQUAL(prev.size(), 6);
	BOOST_CHECK_EQUAL(consumptions.size(), 6);

	BOOST_CHECK_EQUAL(prev[0], 0);
	BOOST_CHECK_EQUAL(prev[1], 0);
	BOOST_CHECK_EQUAL(prev[2], 1);
	BOOST_CHECK_EQUAL(prev[3], 1);
	BOOST_CHECK_EQUAL(prev[4], 2);
	BOOST_CHECK_EQUAL(prev[5], 4);

	BOOST_CHECK_EQUAL(consumptions[0], 0);
	BOOST_CHECK_EQUAL(consumptions[1], 1);
	BOOST_CHECK_EQUAL(consumptions[2], 2);
	BOOST_CHECK_EQUAL(consumptions[3], 3);
	BOOST_CHECK_EQUAL(consumptions[4], 3);
	BOOST_CHECK_EQUAL(consumptions[5], 4);
}


BOOST_AUTO_TEST_CASE(shortest_tree_from_node1_no_charging_test)
{
	auto chargerGraph = createChargerShortestPathWithoutCharging();

	std::shared_ptr<Car> car = make_shared<Car>();

	ReachableStartNode start{1, 0, 0};
	auto tree_tuple = chargerGraph->buildShortestPathTree(start);
	auto prev = get<0>(tree_tuple);
	auto consumptions = get<1>(tree_tuple);

	BOOST_CHECK_EQUAL(prev.size(), 6);
	BOOST_CHECK_EQUAL(consumptions.size(), 6);

	BOOST_CHECK_EQUAL(prev[0], 1);
	BOOST_CHECK_EQUAL(prev[1], 1);
	BOOST_CHECK_EQUAL(prev[2], 1);
	BOOST_CHECK_EQUAL(prev[3], 1);
	BOOST_CHECK_EQUAL(prev[4], 2);
	BOOST_CHECK_EQUAL(prev[5], 4);

	BOOST_CHECK_EQUAL(consumptions[0], 1);
	BOOST_CHECK_EQUAL(consumptions[1], 0);
	BOOST_CHECK_EQUAL(consumptions[2], 1);
	BOOST_CHECK_EQUAL(consumptions[3], 2);
	BOOST_CHECK_EQUAL(consumptions[4], 2);
	BOOST_CHECK_EQUAL(consumptions[5], 3);
}



BOOST_AUTO_TEST_CASE(shortest_tree_from_node0_with_charging_test) {
	// When setting the battery capacity and the power in the plug/charger we add additional weight.
	std::shared_ptr<vector<ChargerGraphEdge>> adj_list = make_shared<vector<ChargerGraphEdge>>();
	adj_list->emplace_back(0, 1, 200, 2);
	adj_list->emplace_back(0, 2, 500, 5);
	adj_list->emplace_back(1, 0, 200, 2);
	adj_list->emplace_back(1, 2, 200, 2);
	adj_list->emplace_back(2, 1, 200, 2);
	adj_list->emplace_back(2, 0, 500, 5);

	vector<Charger> charger_list;
	vector<Plug> plug_list{Plug(18, {PlugType::Typ_2})};
	charger_list.emplace_back(Charger{0, engine::PhantomNode{}, Coordinate{}, "", 1, false, plug_list});
	charger_list.emplace_back(Charger{1, engine::PhantomNode{}, Coordinate{}, "", 1, false, plug_list});

	// we want Charger 2 to be more powerfull, so that it will charge faster and thus resulting in a direct path between node 0 and node 2
	vector<Plug> plug_list2{Plug(180, {PlugType::Typ_2})};
	charger_list.emplace_back(Charger{2, engine::PhantomNode{}, Coordinate{}, "", 1, false, plug_list2});

	std::shared_ptr<Car> car = make_shared<Car>();
	car->base_battery_capacity_milli_watt_h = 5;
	std::shared_ptr<ChargerGraph> chargerGraph = std::make_shared<ChargerGraph>(car, adj_list, charger_list);


	ReachableStartNode start{0, 0, 0};
	auto tree_tuple = chargerGraph->buildShortestPathTree(start);
	auto prev = get<0>(tree_tuple);
	auto consumptions = get<1>(tree_tuple);

	BOOST_CHECK_EQUAL(prev.size(), 3);
	BOOST_CHECK_EQUAL(consumptions.size(), 3);

	BOOST_CHECK_EQUAL(prev[0], 0);
	BOOST_CHECK_EQUAL(prev[1], 0);
	BOOST_CHECK_EQUAL(prev[2], 0);

	BOOST_CHECK_EQUAL(consumptions[0], 0);
	BOOST_CHECK_EQUAL(consumptions[1], 2);
	BOOST_CHECK_EQUAL(consumptions[2], 5);

}

BOOST_AUTO_TEST_SUITE_END()
