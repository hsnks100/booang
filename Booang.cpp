// STL
#include <iostream>                  // for std::cout

// Boost
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/undirected_graph.hpp>// A subclass to provide reasonable arguments to adjacency_list for a typical undirected graph
#include <boost/graph/dijkstra_shortest_paths.hpp>

template<typename edgeType, typename vertexProperty>
class BGraph{
    public:
        typedef boost::adjacency_list<boost::vecS, boost::vecS, 
            boost::directedS, 
            vertexProperty,
            boost::property<boost::edge_weight_t, edgeType>> graphType;
        
        graphType G;
        typename graphType::vertex_descriptor addVertex(){
            return boost::add_vertex(G); 
        }
        void addEdge(typename graphType::vertex_descriptor v0, 
                typename graphType::vertex_descriptor v1, edgeType e){
            boost::add_edge(v0, v1, e, G); 
        }
        void addEdge(typename graphType::vertex_descriptor v0, 
                typename graphType::vertex_descriptor v1){
            boost::add_edge(v0, v1, G); 
        }
        auto operator[](typename graphType::vertex_descriptor v){
            return boost::out_edges(v, G); 
        }

        typename graphType::vertex_bundled& getVertex(typename graphType::vertex_descriptor v){
             return get(boost::vertex_bundle, G)[v]; 
        }
        typename graphType::edge_bundled& operator[](typename graphType::edge_descriptor e)
        { return get(boost::edge_bundle, G)[e]; }

        void print(){ 
            std::cout << "vertex list" << std::endl << std::endl;;
            {
                auto vertices = boost::vertices(G);
                for(; vertices.first != vertices.second; ++vertices.first){
                    std::cout << *vertices.first << "번 vertex 가 존재" << std::endl;
                }
            }
            std::cout << std::endl;
            std::cout << "edge list" << std::endl;
            {
                auto EdgeWeightMap = get(boost::edge_weight_t(), G);
                auto edges = boost::edges(G);
                for(; edges.first != edges.second; ++edges.first){
                    auto tt = *edges.first;
                    std::cout << (*edges.first).m_source << "to" << (*edges.first).m_target;
                    std::cout << "===> weight : " << EdgeWeightMap[*edges.first] << std::endl;
                }
            }
            std::cout << std::endl;

            std::cout << "edge list each vertex" << std::endl;
            {
                auto vertices = boost::vertices(G);
                for(; vertices.first != vertices.second; ++vertices.first){
                    std::cout << *vertices.first << "번 vertex 가 존재" << std::endl;

                    auto outEdgeIters = boost::out_edges(*vertices.first, G);
                    for(; outEdgeIters.first != outEdgeIters.second; ++outEdgeIters.first){
                        std::cout << "... to" << (*outEdgeIters.first).m_target << std::endl;
                    }
                }
            } 
        } 

        void loopOutEdges(typename graphType::vertex_descriptor v, const std::function<void(int, int, edgeType)>& f){
            auto outEdgeIter = boost::out_edges(v, G); 
            auto edgeWeightMap = get(boost::edge_weight_t(), G);

            for(; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first){
                f((*outEdgeIter.first).m_source, (*outEdgeIter.first).m_target, 
                        edgeWeightMap[*outEdgeIter.first]); 
            }
        }
        void dijk(typename graphType::vertex_descriptor v0){
            // Create things for Dijkstra
            //
            typedef typename boost::graph_traits < graphType >::vertex_descriptor vertex_descriptor;
            typedef typename boost::graph_traits < graphType >::edge_descriptor edge_descriptor;
            typedef std::pair<int, int> Edge;
            std::vector<vertex_descriptor> parents(boost::num_vertices(G)); // To store parents
            std::vector<int> distances(boost::num_vertices(G)); // To store distances

            // Compute shortest paths from v0 to all vertices, and store the output in parents and distances
            boost::dijkstra_shortest_paths(G, v0, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));

            // Output results
            std::cout << "distances and parents:" << std::endl;
            typename boost::graph_traits < graphType >::vertex_iterator vertexIterator, vend;
            for (boost::tie(vertexIterator, vend) = boost::vertices(G); vertexIterator != vend; ++vertexIterator) 
            {
                std::cout << "distance(" << *vertexIterator << ") = " << distances[*vertexIterator] << ", ";
                std::cout << "parent(" << *vertexIterator << ") = " << parents[*vertexIterator] << std::endl;
            }
            std::cout << std::endl;
        }
};


 

int main(int,char*[])
{ 
    struct VertexProperty{
        int Id;
    };
    
    BGraph<int, VertexProperty> G;
        
    auto v0 = G.addVertex();
    auto v1 = G.addVertex();
    auto v2 = G.addVertex();
    auto v3 = G.addVertex();
    G.getVertex(v0).Id = 2;
    // G[v0].Id = 2;
    
    G.addEdge(v0, v1, 100);
    G.addEdge(v1, v0, 200);
    G.addEdge(v0, v2, 300);
    G.addEdge(v2, v0, 400);
    G.addEdge(v1, v0, 50);
    G.addEdge(v2, v3, 10000);
    G.print();



    // method 1
    auto outEdgeIter = G[v0];

    std::cout << "----------" << std::endl;
    for(; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first){
        std::cout << "... to" << (*outEdgeIter.first).m_target << std::endl;
    }


    // method 2
    G.loopOutEdges(v0, [](int from, int to, int weight){
            std::cout << from << " " << to << " " << weight << std::endl;
            }); 


    // dijkstra algorithm
    G.dijk(v0);



    return 0;
}
