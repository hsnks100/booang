#include <iostream>                  // for std::cout 
#include <cassert>
#include <vector> 
#include <type_traits> 
#include <fstream>
#include <sstream>
#include <queue>


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
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

// plan to do lists
// DFS, BFS, Prim, Kruskal, Bellman-Ford, Floyd, Warshall, Dijkstra, Bipartite, Maximum Flow

#if (defined(_WIN32) || defined(WIN32))
#ifndef BOOPATH
#error must be set BOOPATH. (ex. #define BOOPATH "C:\\Users\\pc1\\Documents\\graphviz-2.38" )
#endif 
#endif

#define DOTPATH BOOPATH##"\\graphviz\\bin\\dot.exe" 

#if !(defined(_WIN32) || defined(WIN32))
#include <gvc.h>
int dot2png(const string& dot, const string& png) {
    //return 0;
    GVC_t *gvc;
    graph_t *g;
    FILE *fp;
    FILE* out;


    gvc = gvContext();
    cout << dot << endl;
    fp = fopen(dot.c_str(), "r");
    out = fopen(png.c_str(), "w");
#ifdef WITH_CGRAPH
    g = agread(fp, 0);
#else
    g = agread(fp);
#endif
    gvLayout(gvc, g, "dot");
    // gvRender(gvc, g, "plain", stdout);;
    gvRender(gvc, g, "png", out);;
    gvFreeLayout(gvc, g);
    agclose(g);
    return (gvFreeContext(gvc));
}

#endif


namespace {
    using namespace boost;
    using namespace std;
    template <typename T>
    class has_toString {
    private:
        typedef char Yes;
        typedef Yes No[2];

        template <typename U, U> struct really_has;
        template <typename C> static Yes& Test(really_has <string(C::*)() const, &C::toString>*);
        template <typename C> static Yes& Test(really_has <string(C::*)(), &C::toString>*);
        template <typename> static No& Test(...);
    public:
        static bool const value = sizeof(Test<T>(0)) == sizeof(Yes);
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
        class boo_dfs_visitor : public default_dfs_visitor
        {
        public:
            boo_dfs_visitor(vector<vertex_descriptor>& r)
                : recv(r) {
                ;
            } 
            void discover_vertex(vertex_descriptor v, graphType const& g) const {
                recv.push_back(v);
            }
            vector<vertex_descriptor>& recv;
        };
        class boo_bfs_visitor : public default_bfs_visitor
        {
        public:
            boo_bfs_visitor(vector<vertex_descriptor>& r)
                : recv(r) {
                ;
            } 
            void discover_vertex(vertex_descriptor v, graphType const& g) const {
                recv.push_back(v);
            }
            vector<vertex_descriptor>& recv;
        };

        struct ShortestPath {
            vector<ToWeight> toWeight;
            vector<vertex_descriptor> path;
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
        }
        auto addVertex(const vertexProperty& vp) {
            add_vertex(vp, G);
        }
        void removeVertex(vertex_descriptor v0) {
            remove_vertex(v0, G);
        }

        void addEdge(vertex_descriptor v0, vertex_descriptor v1, edgeType e) {
            add_edge(v0, v1, e, G);
        }
        void addEdge(vertex_descriptor v0, vertex_descriptor v1) {
            add_edge(v0, v1, G);
        }

        void removeEdge(vertex_descriptor v0, vertex_descriptor v1) {
            remove_edge(v0, v1, G);
        }


        vector<ToWeight> operator[](vertex_descriptor v) {
            auto EdgeWeightMap = get(edge_weight_t(), G);
            vector<ToWeight> ret;
            auto outEdgeIters = out_edges(v, G);
            bool vertexisExist = outEdgeIters.first != outEdgeIters.second;
            assert(vertexisExist);

            for (; outEdgeIters.first != outEdgeIters.second; ++outEdgeIters.first) {
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
            pair<edge_descriptor, bool> ed = edge(v0, v1, G);
            return ed.second;
        }


        edgeType getWeight(vertex_descriptor v0, vertex_descriptor v1) {

            static_assert(!is_same<edgeType, no_property>::value, "edgeType must not be no_property");
            pair<edge_descriptor, bool> ed = edge(v0, v1, G);
            edgeType weight = get(edge_weight_t(), G, ed.first);
            return weight;
        }


        edgeType putWeight(vertex_descriptor v0, vertex_descriptor v1, edgeType weight) {
            pair<edge_descriptor, bool> ed = edge(v0, v1, G);
            this->addEdge(v0, v1, weight);
            if (!ed.second) {
                this->addEdge(v0, v1, weight);
            }
            else {
                put(edge_weight_t(), G, ed.first, weight);
            }
        }

        void loopOutEdges(vertex_descriptor v, const function<void(int, int, edgeType)>& f) {
            auto outEdgeIter = out_edges(v, G);
            auto edgeWeightMap = get(edge_weight_t(), G);

            for (; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first) {
                f((*outEdgeIter.first).m_source, (*outEdgeIter.first).m_target,
                    edgeWeightMap[*outEdgeIter.first]);
            }
        }

        void loopAllEdges(const function<bool(int, int, edgeType)>& f) {
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
        ShortestPath dijk(vertex_descriptor v0) {
            auto& g = G;
            vector<vertex_descriptor> p(num_vertices(g));
            vector<edgeType> d(num_vertices(g));
            dijkstra_shortest_paths(g, v0,
                predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
                distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))));

            cout << "distances and parents:" << endl;
            vertex_iterator vi, vend;

            ShortestPath ret;
            for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
                ret.toWeight.push_back(ToWeight{ *vi, d[*vi] });
            }

            ret.path = p;

            return ret;
        }

        // unsigned int is ordinary
        vector<vertex_descriptor> bfs(vertex_descriptor beg) {
            vector<vertex_descriptor> recv;
            boo_bfs_visitor vis(recv); 
            vector<default_color_type> color_map(num_vertices(G));
            boost::queue<vertex_descriptor> bf;
            breadth_first_visit(G,
                                beg,
                                bf,
                                vis,
                                make_iterator_property_map(color_map.begin(), get(vertex_index, G), color_map[0]));

            return recv; 
        }


        vector<vertex_descriptor> dfs(vertex_descriptor beg) {
            vector<vertex_descriptor> recv;
            boo_dfs_visitor vis(recv); 
            vector<default_color_type> color_map(num_vertices(G));
            depth_first_visit(G,
                              beg,
                              vis,
                              make_iterator_property_map(color_map.begin(), get(vertex_index, G), color_map[0]));

            return recv; 
        }

        auto getAllVertices() {
            auto verticesVector = vertices(G);
            vector<vertex_descriptor> ret;
            for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
                ret.push_back(*verticesVector.first);
            }
            return ret;
        }

        auto getNumVertices() {
            return num_vertices(G);
        }

        void print() {
            cout << "vertex list" << endl << endl;
            {
                auto verticesVector = vertices(G);

                for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
                    cout << *verticesVector.first << "vertex" << endl;
                }
            }
            cout << endl;
            printEdgeList();
            cout << "---------------------------------------" << endl;
            cout << endl;

        }

        template<typename U = edgeType>
        typename enable_if<!is_same<U, no_property>::value, void>::type printEdgeList() {
            cout << "edge list" << endl;
            {
                auto EdgeWeightMap = get(edge_weight_t(), G);
                auto edgesVector = edges(G);
                for (; edgesVector.first != edgesVector.second; ++edgesVector.first) {
                    auto tt = *edgesVector.first;
                    cout << (*edgesVector.first).m_source << "===>" << (*edgesVector.first).m_target << endl;
                    cout << "===> weight : " << EdgeWeightMap[*edgesVector.first] << endl;
                }
            }
        }
        template<typename U = edgeType>
        typename enable_if<is_same<U, no_property>::value, void>::type printEdgeList() {
            cout << "edge list" << endl;
            {
                auto EdgeWeightMap = get(edge_weight_t(), G);
                auto edgesVector = edges(G);
                for (; edgesVector.first != edgesVector.second; ++edgesVector.first) {
                    auto tt = *edgesVector.first;
                    cout << (*edgesVector.first).m_source << "===>" << (*edgesVector.first).m_target << endl;
                    //cout << "===> weight : " << EdgeWeightMap[*edgesVector.first] << endl;
                }
            }
        }

        auto getGraph() {
            return G;
        }

        void removeAllEdges() {
            auto verticesVector = vertices(G);
            for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
                clear_vertex(*verticesVector.first, G);
            }
        }

        // kruskal은 시작하는 vertex가 필요 없기 때문에 parameter X
        auto boost_kruskal() {
            typename remove_reference<decltype(*this)>::type ret = *this;
            ret.removeAllEdges();
            size_t verticesCount = num_vertices(G);
            size_t edgesCount = num_edges(G);

            auto EdgeWeightMap = get(edge_weight, G);
            auto weight = get(edge_weight, G);

            vector < edge_descriptor > spanning_tree;

            kruskal_minimum_spanning_tree(G, back_inserter(spanning_tree));


            for (typename vector < edge_descriptor >::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei) {
                ret.addEdge(source(*ei, G), target(*ei, G), weight[*ei]);
            }

            return ret;
        }


        // return Graph
        auto boost_prim(vertex_descriptor v0) {
            typename remove_reference<decltype(*this)>::type ret = *this;
            ret.removeAllEdges();

            size_t verticesCount = num_vertices(G);

            vector<vertex_descriptor> p(verticesCount);
            auto EdgeWeightMap = get(edge_weight_t(), G);

            typename property_map<graphType, vertex_index_t>::type id = get(vertex_index, G);
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


        void writeSimpleViz(const string filename) {
            ofstream dot("graph.dot");
            write_graphviz(dot, G);
#if defined(_WIN32) || defined(WIN32)

            string fff = DOTPATH;
            auto cmd = string("\"") + fff + string("\" -Tpng graph.dot -o" + filename);
            cout << cmd << endl;
            system(cmd.c_str());
#else
            dot2png("graph.dot", filename);
#endif
        }
        void writeSimpleViz2(const string filename) {
            ofstream dot("graph.dot");

            auto verticesVector = vertices(G);
            vector<string> names;
            vector<vertex_descriptor> ret;
            for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
                ostringstream oss;
                oss << getVertex(*verticesVector.first).toString();
                names.push_back(oss.str());
            }
            write_graphviz(dot, G, make_label_writer(&names[0]));
#if defined(_WIN32) || defined(WIN32)
            string fff = DOTPATH;

            system((string("\"") + fff + string("\" -Tpng graph.dot -o" + filename)).c_str());
#else
            dot2png("graph.dot", filename);
#endif
        }
        void writeSimpleViz3(const string filename) {
            ofstream dot("graph.dot");
            write_graphviz(dot, G, make_label_writer(get(vertex_index, G)),
                make_label_writer(get(edge_weight_t(), G))

            );
#if defined(_WIN32) || defined(WIN32)
            string fff = DOTPATH;


            system((string("\"") + fff + string("\" -Tpng graph.dot -o" + filename)).c_str());
#else
            dot2png("graph.dot", filename);
#endif
        }
        void writeSimpleViz4(const string filename) {
            ofstream dot("graph.dot");

            auto verticesVector = vertices(G);
            vector<string> names;
            vector<vertex_descriptor> ret;
            for (; verticesVector.first != verticesVector.second; ++verticesVector.first) {
                ostringstream oss;
                oss << getVertex(*verticesVector.first).toString();
                names.push_back(oss.str());
            }
            write_graphviz(dot, G, make_label_writer(&names[0]),
                make_label_writer(get(edge_weight_t(), G))

            );
            // write_graphviz(dot, G, make_label_writer(&names[0])


            //     ); 

#if defined(_WIN32) || defined(WIN32)
            string fff = DOTPATH;


            system((string("\"") + fff + string("\" -Tpng graph.dot -o" + filename)).c_str());
#else
            dot2png("graph.dot", filename);
#endif
        }

        /////////////////////////////////////////////////////
        template<typename U = vertexProperty,
            typename C = edgeType
        >
            void printGraphViz(const string filename, typename enable_if<!is_same<U, no_property>::value, int>::type = 0,
                typename enable_if<!has_toString<U>::value, int>::type = 0,
                typename enable_if<!is_same<C, no_property>::value, int>::type = 0
            ) {
            cout << "yes prop, no toString, yes weight" << endl;
            writeSimpleViz3(filename);
        }
        template<typename U = vertexProperty,
            typename C = edgeType
        >
            void printGraphViz(
                const string filename,


                typename enable_if<is_same<U, no_property>::value, int>::type = 0,
                typename enable_if<!is_same<C, no_property>::value, int>::type = 0
            ) {
            cout << "no prop, yes weight" << endl;
            writeSimpleViz3(filename);
        }
        template<typename U = vertexProperty,
            typename C = edgeType
        >
            void printGraphViz(
                const string filename,
                typename enable_if<has_toString<U>::value, int>::type = 0,
                typename enable_if<!is_same<C, no_property>::value, int>::type = 0
            ) {
            cout << "yes prop, yes toString, yes weight" << endl;
            writeSimpleViz4(filename);
        }

        /////////////////////////////////////////
        template<typename U = vertexProperty,
            typename C = edgeType
        >
            void printGraphViz(
                const string filename,
                typename enable_if<!is_same<U, no_property>::value, int>::type = 0,
                typename enable_if<!has_toString<U>::value, int>::type = 0,
                typename enable_if<is_same<C, no_property>::value, int>::type = 0
            ) {
            cout << "yes prop, no toString, no weight" << endl;
            writeSimpleViz(filename);
        }
        template<typename U = vertexProperty,
            typename C = edgeType
        >
            void printGraphViz(
                const string filename,
                typename enable_if<is_same<U, no_property>::value, int>::type = 0,
                typename enable_if<is_same<C, no_property>::value, int>::type = 0
            ) {
            cout << "no prop, no weight" << endl;
            writeSimpleViz(filename);
            // writeSimpleViz();
        }
        template<typename U = vertexProperty,
            typename C = edgeType
        >
            void printGraphViz(
                const string filename,
                typename enable_if<has_toString<U>::value, int>::type = 0,
                typename enable_if<is_same<C, no_property>::value, int>::type = 0
            ) {
            cout << "yes prop, yes toString, no weight" << endl;
            writeSimpleViz2(filename);
        }
    };
}

typedef BGraph<no_property> SimpleGraph;
typedef BGraph<int> WeightedGraph;
template<typename vertexProperty>
class PropGraph : public BGraph<int, vertexProperty> {};
//typedef BGraph<property<vertex_index_t, int, _>> TestedGraph; 
