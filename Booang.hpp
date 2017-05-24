// STL
#include <iostream>                  // for std::cout

// Boost
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/undirected_graph.hpp>// A subclass to provide reasonable arguments to adjacency_list for a typical undirected graph

template<typename edgeType, typename vertexProperty>
class BGraph{
    public:
        typedef boost::adjacency_list<boost::vecS, boost::vecS, 
            boost::directedS, 
            boost::vertexProperty,
            boost::property<boost::edge_weight_t, edgeType>> graphType;
        graphType G;
        typename graphType::vertex_descriptor addVertex(){
            return boost::add_vertex(G); 
        }
        // void addEdge(typename graphType::vertex_descriptor v0, 
                // typename graphType::vertex_descriptor v1, int ){
            // // boost::add_edge(v0, v1, G); 
        // }
        void addEdge(typename graphType::vertex_descriptor v0, 
                typename graphType::vertex_descriptor v1, edgeType e){
            boost::add_edge(v0, v1, e, G); 
        }
        void print(){ 
            typename boost::graph_traits<graphType>::vertex_iterator i, end; 
            typename boost::graph_traits<graphType>::out_edge_iterator ei, edge_end; 
            typename boost::property_map < graphType,
                boost::edge_weight_t >::type EdgeWeightMap = get(boost::edge_weight, G);
            for(boost::tie(i, end) = boost::vertices(G); i != end; i++) { 
                std::cout << *i << " 번 vertex에서 출발하는 edges들은 다음과 같다\n"; 
                for(boost::tie(ei, edge_end) = boost::out_edges(*i, G); ei != edge_end; ei++) { 
                    std::cout << ei->m_source << " to " << ei->m_target << "\n"; 
                    std::cout << EdgeWeightMap[*ei] << std::endl;
                } 
                std::cout << "\n"; 
            }
        } 
};


int main(int,char*[])
{

    BGraph<int, int> G;
        
    auto v0 = G.addVertex();
    auto v1 = G.addVertex();
    G.addEdge(v0, v1, 2);
    G.addEdge(v1, v0, 1);
    G.print();




    return 0;
}


