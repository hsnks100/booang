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


#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>




template<typename edgeType = int, typename vertexProperty = boost::no_property>
class BGraph{
  public:
    typedef boost::adjacency_list<
      boost::listS,
      boost::vecS, 
      boost::directedS, 
      vertexProperty,
      boost::property<boost::edge_weight_t, edgeType>> graphType;

    typedef typename graphType::vertex_descriptor vertex_descriptor;
    typedef typename graphType::edge_descriptor edge_descriptor;
    struct ToWeight{
      vertex_descriptor to;
      edgeType weight; 
    };

    //std::map<vertexIndexType, typename graphType::vertex_descriptor> toDescriptor;
    //std::map<typename graphType::vertex_descriptor, vertexIndexType> toVit;

    graphType G;

    void resize(size_t s) {
      G = graphType(s);
    }

    auto addVertex() {
      return boost::add_vertex(G); 
      //if(toDescriptor.find(v0) == toDescriptor.end()) {
        //toDescriptor[v0] = 
        //toVit[toDescriptor[v0]] = v0;
      //}
    }
    auto addVertex(const vertexProperty& vp) {
      boost::add_vertex(vp, G); 
      //if(toDescriptor.find(v0) == toDescriptor.end()) {
        //typename boost::property_map<graphType, boost::vertex_index_t>::type imap = 
          //boost::get(boost::vertex_index, G);
        //toDescriptor[v0] = 
        //G[toDescriptor[v0]] = vp;
        //std::cout << imap[toDescriptor[v0]] << std::endl;
        ////std::cout << G[
        //toVit[toDescriptor[v0]] = v0;
      //}
    }
    //void removeVertex(vertexIndexType v0) {
      //assert(false);
      //bool isExist = toDescriptor.find(v0) != toDescriptor.end();
      //assert(isExist == true);
      //boost::remove_vertex(toDescriptor[v0], G);
      //toVit.erase(toVit.find(toDescriptor[v0]));
      //toDescriptor.erase(v0);
    //} 

    void addEdge(vertex_descriptor v0, 
        vertex_descriptor v1, edgeType e) {
      boost::add_edge(v0, v1, e, G);
      //bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
        //toDescriptor.find(v1) != toDescriptor.end();
      //assert(isExist == true);
    }
    void addEdge(vertex_descriptor v0, 
        vertex_descriptor v1){
      boost::add_edge(v0, v1, G);
      //bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
        //toDescriptor.find(v1) != toDescriptor.end();
      //assert(isExist == true);
      //boost::add_edge(toDescriptor[v0], toDescriptor[v1], G); 
    }

    void removeEdge(vertex_descriptor v0, vertex_descriptor v1){ 
      boost::remove_edge(v0, v1, G);
      //bool isExist = toDescriptor.find(v0) != toDescriptor.end() && 
        //toDescriptor.find(v1) != toDescriptor.end();
      //assert(isExist == true);
    }


    std::vector<ToWeight> operator[](vertex_descriptor v){
      auto EdgeWeightMap = get(boost::edge_weight_t(), G);

      //std::vector<std::pair<vertex_descriptor, edgeType>> ret;
      std::vector<ToWeight> ret;
      auto outEdgeIters = boost::out_edges(v, G);
      bool vertexisExist = outEdgeIters.first != outEdgeIters.second;
      assert(vertexisExist);

      for(; outEdgeIters.first != outEdgeIters.second; ++outEdgeIters.first){
        //ret.push_back(std::make_pair((*outEdgeIters.first).m_target, 
              //EdgeWeightMap[*outEdgeIters.first]));
        ret.push_back({(*outEdgeIters.first).m_target, 
              EdgeWeightMap[*outEdgeIters.first]});
      }
      return ret; 
    }

    typename graphType::vertex_bundled& getVertex(vertex_descriptor v){
      return get(boost::vertex_bundle, G)[v]; 
    }
    typename graphType::edge_bundled& operator[](typename graphType::edge_descriptor e)
    { return get(boost::edge_bundle, G)[e]; }


    bool hasEdge(vertex_descriptor v0, vertex_descriptor v1) {
      std::pair<edge_descriptor, bool> ed = boost::edge(v0, v1, G); 
      return ed.second; 
    }


    edgeType getWeight(vertex_descriptor v0, vertex_descriptor v1) {

      static_assert(!std::is_same<edgeType, boost::no_property>::value, "edgeType must not be boost::no_property");
      std::pair<edge_descriptor, bool> ed = boost::edge(v0, v1, G); 
      edgeType weight = get(boost::edge_weight_t(), G, ed.first);
      return weight;
      //return boost::no_property();
    } 


    edgeType putWeight(vertex_descriptor v0, vertex_descriptor v1, edgeType weight){
      std::pair<edge_descriptor, bool> ed = boost::edge(v0, v1, G);
      this->addEdge(v0, v1, weight);
      if(!ed.second){
        this->addEdge(v0, v1, weight);
      }
      else{
        boost::put(boost::edge_weight_t(), G, ed.first, weight);
      }
    }

    void loopOutEdges(vertex_descriptor v, const std::function<void(int, int, edgeType)>& f){
      auto outEdgeIter = boost::out_edges(v, G); 
      auto edgeWeightMap = get(boost::edge_weight_t(), G);

      for(; outEdgeIter.first != outEdgeIter.second; ++outEdgeIter.first){
        f((*outEdgeIter.first).m_source, (*outEdgeIter.first).m_target, 
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

    // type 에 대한 SFINAE 적용이 필요함.
    std::vector<ToWeight> dijk(vertex_descriptor v0){ 
      using namespace boost;

      auto& g = G;
      std::vector<vertex_descriptor> p(num_vertices(g));
      std::vector<int> d(num_vertices(g));
      dijkstra_shortest_paths(g, v0,
          predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
          distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g)))); 

      std::cout << "distances and parents:" << std::endl;
      typename graph_traits < graphType >::vertex_iterator vi, vend;
      std::vector<ToWeight> ret;
      for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
        ret.push_back(ToWeight{*vi, d[*vi]});
      }

      return ret;
    }

    auto getAllVertices() {
      auto vertices = boost::vertices(G); 
      std::vector<vertex_descriptor> ret;
      for(; vertices.first != vertices.second; ++vertices.first){ 
        ret.push_back(*vertices.first);
      } 
      return ret;
    }

    auto getNumVertices() {
        return boost::num_vertices(G);
    }

    void print() { 
      std::cout << "vertex list" << std::endl << std::endl;
      {
        auto vertices = boost::vertices(G);
        for(; vertices.first != vertices.second; ++vertices.first){
          std::cout << *vertices.first << "번 vertex 가 존재" << std::endl;
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
          std::cout << (*edges.first).m_source << "===>" << (*edges.first).m_target << std::endl;
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
          std::cout << (*edges.first).m_source << "===>" << (*edges.first).m_target << std::endl;
          //std::cout << "===> weight : " << EdgeWeightMap[*edges.first] << std::endl;
        }
      }
    }
}; 

typedef BGraph<boost::no_property> SimpleGraph;
typedef BGraph<int> WeightedGraph; 
template<typename vertexProperty>
class PropGraph : public BGraph<int, vertexProperty>{};

//typedef BGraph<property<vertex_index_t, int, _>> TestedGraph; 
