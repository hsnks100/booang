// STL
#include <iostream>                  // for std::cout 
#include <cassert>
#include <vector> 
#include <type_traits> 
//using namespace std;

// Boost
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/undirected_graph.hpp>// A subclass to provide reasonable arguments to adjacency_list for a typical undirected graph
#include <boost/graph/dijkstra_shortest_paths.hpp>


template<typename edgeType, typename vertexProperty = boost::no_property, typename vertexIndexType = int>
class BGraph{
  public:
    typedef boost::adjacency_list<
      boost::setS,
      boost::listS, 
      boost::directedS, 
      vertexProperty,
      boost::property<boost::edge_weight_t, edgeType>> graphType;

    typedef typename graphType::vertex_descriptor vertex_descriptor;
    typedef typename graphType::edge_descriptor edge_descriptor;

    std::map<vertexIndexType, typename graphType::vertex_descriptor> toDescriptor;
    std::map<typename graphType::vertex_descriptor, vertexIndexType> toVit;

    graphType G;

    void addVertex(vertexIndexType v0) {
      if(toDescriptor.find(v0) == toDescriptor.end()) {
        toDescriptor[v0] = boost::add_vertex(G); 
        toVit[toDescriptor[v0]] = v0;
      }
    }
    void addVertex(vertexIndexType v0, const vertexProperty& vp) {
      if(toDescriptor.find(v0) == toDescriptor.end()) {
        toDescriptor[v0] = boost::add_vertex(vp, G); 
        toVit[toDescriptor[v0]] = v0;
      }
    }
    void removeVertex(vertexIndexType v0) {
      bool isExist = toDescriptor.find(v0) != toDescriptor.end();
      assert(isExist == true);
      boost::remove_vertex(toDescriptor[v0], G);
      toVit.erase(toVit.find(toDescriptor[v0]));
      toDescriptor.erase(v0);
    } 

    void addEdge(vertexIndexType v0, 
        vertexIndexType v1, edgeType e) {
      bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
        toDescriptor.find(v1) != toDescriptor.end();
      assert(isExist == true);
      boost::add_edge(toDescriptor[v0], toDescriptor[v1], e, G); 
    }
    void addEdge(vertexIndexType v0, 
        vertexIndexType v1){
      bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
        toDescriptor.find(v1) != toDescriptor.end();
      assert(isExist == true);
      boost::add_edge(toDescriptor[v0], toDescriptor[v1], G); 
    }

    void removeEdge(vertexIndexType v0, vertexIndexType v1){
      
      bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
        toDescriptor.find(v1) != toDescriptor.end();
      assert(isExist == true);
      boost::remove_edge(toDescriptor[v0], toDescriptor[v1], G); 
    }


    auto operator[](vertexIndexType v){
      auto EdgeWeightMap = get(boost::edge_weight_t(), G);

      std::vector<std::pair<vertexIndexType, edgeType>> ret;
      auto outEdgeIters = boost::out_edges(toDescriptor[v], G);
      bool vertexisExist = outEdgeIters.first != outEdgeIters.second;
      assert(vertexisExist);

      for(; outEdgeIters.first != outEdgeIters.second; ++outEdgeIters.first){
        ret.push_back(std::make_pair(toVit[(*outEdgeIters.first).m_target], 
              EdgeWeightMap[*outEdgeIters.first]));
      }
      return ret; 
    }

    typename graphType::vertex_bundled& getVertex(vertexIndexType v){
      return get(boost::vertex_bundle, G)[toDescriptor[v]]; 
    }
    typename graphType::edge_bundled& operator[](typename graphType::edge_descriptor e)
    { return get(boost::edge_bundle, G)[e]; }


    bool hasEdge(vertexIndexType v0, vertexIndexType v1) {
      std::pair<edge_descriptor, bool> ed = boost::edge(toDescriptor[v0], toDescriptor[v1], G); 
      return ed.second; 
    }


    edgeType getWeight(vertexIndexType v0, vertexIndexType v1) {

      static_assert(!std::is_same<edgeType, boost::no_property>::value, "edgeType must not be boost::no_property");
      std::pair<edge_descriptor, bool> ed = boost::edge(toDescriptor[v0], toDescriptor[v1], G); 
      edgeType weight = get(boost::edge_weight_t(), G, ed.first);
      return weight;
      //return boost::no_property();
    } 


    edgeType putWeight(vertexIndexType v0, vertexIndexType v1, edgeType weight){
      std::pair<edge_descriptor, bool> ed = boost::edge(toDescriptor[v0], toDescriptor[v1], G);
      this->addEdge(toDescriptor[v0], toDescriptor[v1], weight);
      if(!ed.second){
        this->addEdge(toDescriptor[v0], toDescriptor[v1], weight);
      }
      else{
        boost::put(boost::edge_weight_t(), G, ed.first, weight);
      }
    }

    void loopOutEdges(vertexIndexType v, const std::function<void(int, int, edgeType)>& f){
      auto outEdgeIter = boost::out_edges(toDescriptor[v], G); 
      auto edgeWeightMap = get(boost::edge_weight_t(), G);

      for(; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first){
        f(toVit[(*outEdgeIter.first).m_source], toVit[(*outEdgeIter.first).m_target], 
            edgeWeightMap[*outEdgeIter.first]); 
      }
    }

    void loopAllEdges(const std::function<bool(int, int, edgeType)>& f){ 
      auto EdgeWeightMap = get(boost::edge_weight_t(), G);
      auto edges = boost::edges(G);
      for(; edges.first != edges.second; ++edges.first){
        auto tt = *edges.first;
        if( f((*edges.first).m_source, (*edges.first).m_target, EdgeWeightMap[*edges.first]) == false)
          break;
      }
    }

    void loopAllVertices(){
    }
    void dijk(vertexIndexType v0){
      // Create things for Dijkstra
      //
      typedef std::pair<int, int> Edge;

      typedef graph_traits < graphType >::vertex_descriptor vertex_descriptor;

      std::vector<vertex_descriptor> parents(boost::num_vertices(G)); // To store parents
      std::vector<int> distances(boost::num_vertices(G)); // To store distances

      std::vector<vertex_descriptor> p(num_vertices(G));
      std::vector<int> d(num_vertices(G));
      vertex_descriptor s = toDescriptor[v0]; // vertex(A, g);

      boost::dijkstra_shortest_paths(G, s,
          boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, G))).
          distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, G))));

      //boost::dijkstra_shortest_paths(G, toDescriptor[v0], 
          //boost::predecessor_map(&parents[0]).distance_map(&distances[0]));

      //std::cout << "distances and parents:" << std::endl;
      //typename boost::graph_traits < graphType >::vertex_iterator vertexIterator, vend;
      //for (boost::tie(vertexIterator, vend) = boost::vertices(G); vertexIterator != vend; ++vertexIterator) 
      //{
        //std::cout << "distance(" << *vertexIterator << ") = " << distances[*vertexIterator] << ", ";
        //std::cout << "parent(" << *vertexIterator << ") = " << parents[*vertexIterator] << std::endl;
      //}
      //std::cout << std::endl;
    }

    auto getAllVertices() {
      auto vertices = boost::vertices(G);

      std::vector<vertexIndexType> ret;
      for(; vertices.first != vertices.second; ++vertices.first){ 
        ret.push_back(toVit[*vertices.first]);
      } 
      return ret;
    }

    void print() { 
      std::cout << "vertex list" << std::endl << std::endl;
      {
        auto vertices = boost::vertices(G);
        for(; vertices.first != vertices.second; ++vertices.first){
          std::cout << toVit[*vertices.first] << "번 vertex 가 존재" << std::endl;
        }
      }
      std::cout << std::endl;
      printEdgeList();
      std::cout << "---------------------------------------" << std::endl;
      std::cout << std::endl;

      //std::cout << "edge list each vertex" << std::endl;
      //{
        //auto vertices = boost::vertices(G);
        //for(; vertices.first != vertices.second; ++vertices.first){
          //std::cout << toVit[*vertices.first] << "번 vertex 가 존재" << std::endl;

          //auto outEdgeIters = boost::out_edges(*vertices.first, G);
          //if(outEdgeIters.first == outEdgeIters.second) {
            //std::cout << "... nothing!" << std::endl;
          //}
          //else for(; outEdgeIters.first != outEdgeIters.second; ++outEdgeIters.first){
            //std::cout << "... ===>" << toVit[(*outEdgeIters.first).m_target] << std::endl;
          //}
        //}
      //} 
    }

    template<typename U = edgeType>
    typename std::enable_if<!std::is_same<U, boost::no_property>::value, void>::type printEdgeList() { 
      std::cout << "edge list" << std::endl;
      {
        auto EdgeWeightMap = get(boost::edge_weight_t(), G);
        auto edges = boost::edges(G);
        for(; edges.first != edges.second; ++edges.first){
          auto tt = *edges.first;
          std::cout << toVit[(*edges.first).m_source] << "===>" << toVit[(*edges.first).m_target] << std::endl;
          std::cout << "===> weight : " << EdgeWeightMap[*edges.first] << std::endl;
        }
      }
    }
    template<typename U = edgeType>
    typename std::enable_if<std::is_same<U, boost::no_property>::value, void>::type printEdgeList() { 
      std::cout << "edge list" << std::endl;
      {
        auto EdgeWeightMap = get(boost::edge_weight_t(), G);
        auto edges = boost::edges(G);
        for(; edges.first != edges.second; ++edges.first){
          auto tt = *edges.first;
          std::cout << toVit[(*edges.first).m_source] << "===>" << toVit[(*edges.first).m_target] << std::endl;
          //std::cout << "===> weight : " << EdgeWeightMap[*edges.first] << std::endl;
        }
      }
    }
}; 

typedef BGraph<boost::no_property> SimpleGraph;
typedef BGraph<int> WeightedGraph; 
