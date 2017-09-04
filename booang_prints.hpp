#ifndef BOO_PRINT_H_
#define BOO_PRINT_H_


namespace booang {


    template<typename edgeType, typename vertexProperty>
    void BGraph<edgeType, vertexProperty>::print() {
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


    template<typename edgeType, typename vertexProperty>
    template<typename U>
    typename std::enable_if<std::is_same<U, no_property>::value, void>::type
    BGraph<edgeType, vertexProperty>::printEdgeList() {
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

    template<typename edgeType, typename vertexProperty>
    template<typename U>
    typename std::enable_if<!std::is_same<U, no_property>::value>::type
    BGraph<edgeType, vertexProperty>::printEdgeList() {
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
}

#endif
