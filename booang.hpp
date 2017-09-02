#include <iostream>                  // for std::cout 
#include <cassert>
#include <vector> 
#include <type_traits> 
#include <fstream>
#include <sstream>


// Boost
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/graphviz.hpp>

#include <boost/graph/undirected_graph.hpp>// A subclass to provide reasonable arguments to adjacency_list for a typical undirected graph
#include <boost/graph/dijkstra_shortest_paths.hpp>


#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/property_map/property_map.hpp> 
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>

// plan to do lists
// DFS, BFS, Prim, Kruskal, Bellman-Ford, Floyd, Warshall, Dijkstra, Bipartite, Maximum Flow

#if (defined(_WIN32) || defined(WIN32))
#ifndef BOOPATH
#error must be set BOOPATH (#define) BOOPATH 하하하
#endif 
#endif

#define DOTPATH BOOPATH##"\\graphviz\\bin\\dot.exe" 

#if !(defined(_WIN32) || defined(WIN32))
#include <gvc.h>
int dot2png(const std::string& dot, const std::string& png) {
	//return 0;
	GVC_t *gvc;
	graph_t *g;
	FILE *fp;
	FILE* out;

	gvc = gvContext();
	printf("%d\n", __LINE__);
	std::cout << dot << std::endl;
	fp = fopen(dot.c_str(), "r");
	out = fopen(png.c_str(), "w");
#ifdef WITH_CGRAPH
	g = agread(fp, 0);
#else
	g = agread(fp);
#endif
	gvLayout(gvc, g, "dot");
	gvRender(gvc, g, "plain", stdout);;
	gvRender(gvc, g, "png", out);;
	gvFreeLayout(gvc, g);
	agclose(g);
	return (gvFreeContext(gvc));
}

#endif


namespace {
	using namespace boost;
	template <typename T>
	class has_toString {
	private:
		typedef char Yes;
		typedef Yes No[2];

		template <typename U, U> struct really_has;
		template <typename C> static Yes& Test(really_has <std::string(C::*)() const,
			&C::toString>*);
		// EDIT: and you can detect one of several overloads... by overloading :)
		template <typename C> static Yes& Test(really_has <std::string(C::*)(),
			&C::toString>*);
		template <typename> static No& Test(...);

	public:
		static bool const value = sizeof(Test<T>(0)) == sizeof(Yes);
	};

	template < typename TimeMap > class bfs_time_visitor :public default_bfs_visitor {
		typedef typename property_traits < TimeMap >::value_type T;
	public:
		bfs_time_visitor(TimeMap tmap, T & t) :m_timemap(tmap), m_time(t) { }
		template < typename Vertex, typename Graph >
		void discover_vertex(Vertex u, const Graph & g) const
		{
			put(m_timemap, u, m_time++);
		}
		TimeMap m_timemap;
		T & m_time;
	};

	template<typename edgeType = int, typename vertexProperty = no_property>
	class BGraph {
	public:
		typedef adjacency_list<
			setS,
			vecS,
			directedS,
			vertexProperty,
			property<edge_weight_t, edgeType>> graphType;

		typedef typename graphType::vertex_descriptor vertex_descriptor;
		typedef typename graphType::edge_descriptor edge_descriptor;
		typedef typename graph_traits<graphType>::vertex_iterator vertex_iterator;
		typedef typename graphType::vertex_bundled vertex_bundled;
		typedef typename graphType::edge_bundled edge_bundled;
		typedef typename graphType::vertices_size_type vertices_size_type;
		struct ToWeight {
			vertex_descriptor to;
			edgeType weight;
		};

		
#if (defined(_WIN32) || defined(WIN32))
#ifndef BOOPATH 
			assert(false, "must be set BOOPATH (#define)");
#endif  
#endif

		

		graphType G;
		BGraph(vertices_size_type s) {
			G = graphType(s);
		}
		BGraph() {
		}

		auto addVertex() {
			return add_vertex(G);
			//if(toDescriptor.find(v0) == toDescriptor.end()) {
			//toDescriptor[v0] = 
			//toVit[toDescriptor[v0]] = v0;
			//}
		}
		auto addVertex(const vertexProperty& vp) {
			add_vertex(vp, G);
			//if(toDescriptor.find(v0) == toDescriptor.end()) {
			//typename property_map<graphType, vertex_index_t>::type imap = 
			//get(vertex_index, G);
			//toDescriptor[v0] = 
			//G[toDescriptor[v0]] = vp;
			//std::cout << imap[toDescriptor[v0]] << std::endl;
			////std::cout << G[
			//toVit[toDescriptor[v0]] = v0;
			//}
		}
		void removeVertex(vertex_descriptor v0) {
			//assert(false);
			/*bool isExist = toDescriptor.find(v0) != toDescriptor.end();
			assert(isExist == true);*/
			remove_vertex(v0, G);
		}

		void addEdge(vertex_descriptor v0,
			vertex_descriptor v1, edgeType e) {
			add_edge(v0, v1, e, G);
			//bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
			//toDescriptor.find(v1) != toDescriptor.end();
			//assert(isExist == true);
		}
		void addEdge(vertex_descriptor v0,
			vertex_descriptor v1) {
			add_edge(v0, v1, G);
			//bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
			//toDescriptor.find(v1) != toDescriptor.end();
			//assert(isExist == true);
			//add_edge(toDescriptor[v0], toDescriptor[v1], G); 
		}

		void removeEdge(vertex_descriptor v0, vertex_descriptor v1) {
			remove_edge(v0, v1, G);
			//bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
			//toDescriptor.find(v1) != toDescriptor.end();
			//assert(isExist == true);
		}


		std::vector<ToWeight> operator[](vertex_descriptor v) {
			auto EdgeWeightMap = get(edge_weight_t(), G);

			//std::vector<std::pair<vertex_descriptor, edgeType>> ret;
			std::vector<ToWeight> ret;
			auto outEdgeIters = out_edges(v, G);
			bool vertexisExist = outEdgeIters.first != outEdgeIters.second;
			assert(vertexisExist);

			for (; outEdgeIters.first != outEdgeIters.second; ++outEdgeIters.first) {
				//ret.push_back(std::make_pair((*outEdgeIters.first).m_target, 
				//EdgeWeightMap[*outEdgeIters.first]));
				ret.push_back({ (*outEdgeIters.first).m_target,
					EdgeWeightMap[*outEdgeIters.first] });
			}
			return ret;
		}

		vertex_bundled& getVertex(vertex_descriptor v) {
			return get(vertex_bundle, G)[v];
		}
		edge_bundled& operator[](edge_descriptor e)
		{
			return get(edge_bundle, G)[e];
		}


		bool hasEdge(vertex_descriptor v0, vertex_descriptor v1) {
			std::pair<edge_descriptor, bool> ed = edge(v0, v1, G);
			return ed.second;
		}


		edgeType getWeight(vertex_descriptor v0, vertex_descriptor v1) {

			static_assert(!std::is_same<edgeType, no_property>::value, "edgeType must not be no_property");
			std::pair<edge_descriptor, bool> ed = edge(v0, v1, G);
			edgeType weight = get(edge_weight_t(), G, ed.first);
			return weight;
			//return no_property();
		}


		edgeType putWeight(vertex_descriptor v0, vertex_descriptor v1, edgeType weight) {
			std::pair<edge_descriptor, bool> ed = edge(v0, v1, G);
			this->addEdge(v0, v1, weight);
			if (!ed.second) {
				this->addEdge(v0, v1, weight);
			}
			else {
				put(edge_weight_t(), G, ed.first, weight);
			}
		}

		void loopOutEdges(vertex_descriptor v, const std::function<void(int, int, edgeType)>& f) {
			auto outEdgeIter = out_edges(v, G);
			auto edgeWeightMap = get(edge_weight_t(), G);

			for (; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first) {
				f((*outEdgeIter.first).m_source, (*outEdgeIter.first).m_target,
					edgeWeightMap[*outEdgeIter.first]);
			}
		}

		void loopAllEdges(const std::function<bool(int, int, edgeType)>& f) {
			auto EdgeWeightMap = get(edge_weight_t(), G);
			auto edgesVector = edges(G);
			for (; edgesVector.first != edgesVector.second; ++edgesVector.first) {
				auto tt = *edgesVector.first;
				if (f((*edgesVector.first).m_source, (*edgesVector.first).m_target, EdgeWeightMap[*edgesVector.first]) == false)
					break;
			}
		}

		void loopAllVertices() {
		}

		// type
		std::vector<ToWeight> dijk(vertex_descriptor v0) {
			auto& g = G;
			std::vector<vertex_descriptor> p(num_vertices(g));
			std::vector<vertex_descriptor> d(num_vertices(g));
			dijkstra_shortest_paths(g, v0,
				predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
				distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))));

			std::cout << "distances and parents:" << std::endl;
			vertex_iterator vi, vend;
			std::vector<ToWeight> ret;
			for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
				ret.push_back(ToWeight{ *vi, d[*vi] });
			}

			return ret;
		}

		// unsigned int is ordinary
		std::vector<vertices_size_type> bfs() {			
			std::vector < vertices_size_type > dtime(num_vertices(g));
			typedef
				iterator_property_map<typename std::vector<vertices_size_type>::iterator, typename property_map<graphType, vertex_index_t>::const_type>
				dtime_pm_type;
			dtime_pm_type dtime_pm(dtime.begin(), get(vertex_index, g));

			vertices_size_type time = 0;
			bfs_time_visitor < dtime_pm_type >vis(dtime_pm, time);
			breadth_first_search(g, vertex(0, g), visitor(vis));
			vertices_size_type N = num_vertices(G);
			std::vector<vertices_size_type> discover_order(N);
			integer_range < vertices_size_type >range(0, N);
			std::copy(range.begin(), range.end(), discover_order.begin());
			std::sort(discover_order.begin(), discover_order.end(),
				indirect_cmp < dtime_pm_type, std::less < vertices_size_type > >(dtime_pm));
			return discover_order;
		}

		auto getAllVertices() {
			auto verticesVector = vertices(G);
			std::vector<vertex_descriptor> ret;
			for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
				ret.push_back(*verticesVector.first);
			}
			return ret;
		}

		auto getNumVertices() {
			return num_vertices(G);
		}

		void print() {
			std::cout << "vertex list" << std::endl << std::endl;
			{
				auto verticesVector = vertices(G);

				for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
					std::cout << *verticesVector.first << "vertex" << std::endl;
				}
			}
			std::cout << std::endl;
			printEdgeList();
			std::cout << "---------------------------------------" << std::endl;
			std::cout << std::endl;

		}

		template<typename U = edgeType>
		typename std::enable_if<!std::is_same<U, no_property>::value, void>::type printEdgeList() {
			std::cout << "edge list" << std::endl;
			{
				auto EdgeWeightMap = get(edge_weight_t(), G);
				auto edgesVector = edges(G);
				for (; edgesVector.first != edgesVector.second; ++edgesVector.first) {
					auto tt = *edgesVector.first;
					std::cout << (*edgesVector.first).m_source << "===>" << (*edgesVector.first).m_target << std::endl;
					std::cout << "===> weight : " << EdgeWeightMap[*edgesVector.first] << std::endl;
				}
			}
		}
		template<typename U = edgeType>
		typename std::enable_if<std::is_same<U, no_property>::value, void>::type printEdgeList() {
			std::cout << "edge list" << std::endl;
			{
				auto EdgeWeightMap = get(edge_weight_t(), G);
				auto edgesVector = edges(G);
				for (; edgesVector.first != edgesVector.second; ++edgesVector.first) {
					auto tt = *edgesVector.first;
					std::cout << (*edgesVector.first).m_source << "===>" << (*edgesVector.first).m_target << std::endl;
					//std::cout << "===> weight : " << EdgeWeightMap[*edgesVector.first] << std::endl;
				}
			}
		}

		auto getGraph() {
			return G;
		}

		// return Graph
		auto boost_prim(vertex_descriptor v0) {
			typename std::remove_reference<decltype(*this)>::type ret;
			size_t verticesCount = num_vertices(g);

			std::vector<vertex_descriptor> p(verticesCount);
			auto EdgeWeightMap = get(edge_weight_t(), G);

			typename boost::property_map<graphType, vertex_index_t>::type id = get(vertex_index, G);
			vertex_descriptor v0Index = id[v0];

			prim_minimum_spanning_tree(G, &p[v0Index]);

			for (vertex_descriptor i = 0; i < verticesCount; i++) {
				if (i != p[i]) {
					auto edgeIter = edge(p[i], i, G);
					auto t = EdgeWeightMap[edgeIter.first];
					ret.addEdge(p[i], i, t);
				}
			}
			return ret;
		}


		void writeSimpleViz(const std::string filename) {
			// std::ostringstream dot;

			std::ofstream dot("graph.dot");
			write_graphviz(dot, G);
#if defined(_WIN32) || defined(WIN32)

			std::string fff = DOTPATH;

			auto cmd = std::string("\"") + fff + std::string("\" -Tpng graph.dot -o" + filename);
			std::cout << cmd << std::endl;
			system(cmd.c_str());
#else
			dot2png("graph.dot", filename);

			// system(("./dot -Tpng graph.dot > " + filename).c_str());
#endif
		}
		void writeSimpleViz2(const std::string filename) {
			std::ofstream dot("graph.dot");

			auto verticesVector = vertices(G);
			std::vector<std::string> names;
			std::vector<vertex_descriptor> ret;
			for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
				std::ostringstream oss;
				oss << getVertex(*verticesVector.first).toString();
				names.push_back(oss.str());
			}
			write_graphviz(dot, G, make_label_writer(&names[0]));
#if defined(_WIN32) || defined(WIN32)
			std::string fff = DOTPATH;

			system((std::string("\"") + fff + std::string("\" -Tpng graph.dot -o" + filename)).c_str());
#else
			dot2png("graph.dot", filename);
#endif
		}
		void writeSimpleViz3(const std::string filename) {
			std::ofstream dot("graph.dot");
			write_graphviz(dot, G, make_label_writer(get(vertex_index, G)),
				make_label_writer(get(edge_weight_t(), G))

			);
#if defined(_WIN32) || defined(WIN32)
			std::string fff = DOTPATH;


			system((std::string("\"") + fff + std::string("\" -Tpng graph.dot -o" + filename)).c_str());
#else
			dot2png("graph.dot", filename);
#endif
		}
		void writeSimpleViz4(const std::string filename) {
			std::ofstream dot("graph.dot");

			auto verticesVector = vertices(G);
			std::vector<std::string> names;
			std::vector<vertex_descriptor> ret;
			for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
				std::ostringstream oss;
				oss << getVertex(*verticesVector.first).toString();
				names.push_back(oss.str());
			}
			write_graphviz(dot, G, make_label_writer(&names[0]),
				make_label_writer(get(edge_weight_t(), G))

			);
			// write_graphviz(dot, G, make_label_writer(&names[0])


			//     ); 

#if defined(_WIN32) || defined(WIN32)
			std::string fff = DOTPATH;


			system((std::string("\"") + fff + std::string("\" -Tpng graph.dot -o" + filename)).c_str());
#else
			dot2png("graph.dot", filename);
#endif
		}

		/////////////////////////////////////////////////////
		template<typename U = vertexProperty,
			typename C = edgeType
		>
			void printGraphViz(const std::string filename, typename std::enable_if<!std::is_same<U, no_property>::value, int>::type = 0,
				typename std::enable_if<!has_toString<U>::value, int>::type = 0,
				typename std::enable_if<!std::is_same<C, no_property>::value, int>::type = 0
			) {
			std::cout << "yes prop, no toString, yes weight" << std::endl;
			writeSimpleViz3(filename);
		}
		template<typename U = vertexProperty,
			typename C = edgeType
		>
			void printGraphViz(
				const std::string filename,


				typename std::enable_if<std::is_same<U, no_property>::value, int>::type = 0,
				typename std::enable_if<!std::is_same<C, no_property>::value, int>::type = 0
			) {
			std::cout << "no prop, yes weight" << std::endl;
			writeSimpleViz3(filename);
		}
		template<typename U = vertexProperty,
			typename C = edgeType
		>
			void printGraphViz(
				const std::string filename,
				typename std::enable_if<has_toString<U>::value, int>::type = 0,
				typename std::enable_if<!std::is_same<C, no_property>::value, int>::type = 0
			) {
			std::cout << "yes prop, yes toString, yes weight" << std::endl;
			writeSimpleViz4(filename);
			// writeSimpleViz3(); 
		}

		/////////////////////////////////////////
		template<typename U = vertexProperty,
			typename C = edgeType
		>
			void printGraphViz(
				const std::string filename,
				typename std::enable_if<!std::is_same<U, no_property>::value, int>::type = 0,
				typename std::enable_if<!has_toString<U>::value, int>::type = 0,
				typename std::enable_if<std::is_same<C, no_property>::value, int>::type = 0
			) {
			std::cout << "yes prop, no toString, no weight" << std::endl;
			writeSimpleViz(filename);
		}
		template<typename U = vertexProperty,
			typename C = edgeType
		>
			void printGraphViz(
				const std::string filename,
				typename std::enable_if<std::is_same<U, no_property>::value, int>::type = 0,
				typename std::enable_if<std::is_same<C, no_property>::value, int>::type = 0
			) {
			std::cout << "no prop, no weight" << std::endl;
			writeSimpleViz(filename);
			// writeSimpleViz();
		}
		template<typename U = vertexProperty,
			typename C = edgeType
		>
			void printGraphViz(
				const std::string filename,
				typename std::enable_if<has_toString<U>::value, int>::type = 0,
				typename std::enable_if<std::is_same<C, no_property>::value, int>::type = 0
			) {
			std::cout << "yes prop, yes toString, no weight" << std::endl;
			writeSimpleViz2(filename);
			// writeSimpleViz3(); 
		}
	};
}

typedef BGraph<no_property> SimpleGraph;
typedef BGraph<int> WeightedGraph;
template<typename vertexProperty>
class PropGraph : public BGraph<int, vertexProperty> {};
//typedef BGraph<property<vertex_index_t, int, _>> TestedGraph; 
