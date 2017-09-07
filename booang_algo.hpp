
#ifndef BOO_MST_H_
#define BOO_MST_H_


namespace booang {
    template<typename edgeType, typename vertexProperty>
    auto BGraph<edgeType, vertexProperty>::bfs(vertex_descriptor beg) {
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


    template<typename edgeType, typename vertexProperty>
    auto BGraph<edgeType, vertexProperty>::dfs(vertex_descriptor beg) {
        vector<vertex_descriptor> recv;
        boo_dfs_visitor vis(recv); 
        vector<default_color_type> color_map(num_vertices(G));
        depth_first_visit(G,
                          beg,
                          vis,
                          make_iterator_property_map(color_map.begin(), get(vertex_index, G), color_map[0]));

        return recv; 
    }
    template<typename edgeType, typename vertexProperty>
    auto BGraph<edgeType, vertexProperty>::boost_kruskal() {
        typename std::remove_reference<decltype(*this)>::type ret = *this;
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
    template<typename edgeType, typename vertexProperty>
    auto BGraph<edgeType, vertexProperty>::boost_prim() {
        typename std::remove_reference<decltype(*this)>::type ret = *this;
        ret.removeAllEdges();

        size_t verticesCount = num_vertices(G);

        
        auto EdgeWeightMap = get(edge_weight_t(), G);  
        vector<vertex_descriptor> p(num_vertices(G));
        prim_minimum_spanning_tree(G, &p[0]);     

        for (vertex_descriptor i = 0; i < verticesCount; i++) {
            if (i != p[i]) {
                auto edgeIter = edge(p[i], i, G);
                auto t = EdgeWeightMap[edgeIter.first];
                ret.addEdge(p[i], i, t);
            }
        }
        return ret;
    }
}

#endif
