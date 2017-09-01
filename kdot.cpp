#include <gvc.h>
#include <iostream>
#include <string>
using namespace std;

#define DOTPATH "C:\\Users\\dlwpd\\Documents\\Visual Studio 2017\\Projects\\Booang\\graphviz-2.38\\release\\bin\\dot.exe"

int main(int argc, char **argv)
{

    system((std::string("\"") + DOTPATH + std::string("\" -Tpng graph.dot -o output.png")).c_str());

    /*GVC_t *gvc;
    graph_t *g;
    FILE *fp;
    FILE* out;

    gvc = gvContext();
    if (argc > 2) {
        fp = fopen(argv[1], "r");
        out = fopen(argv[2], "w");
    }
    else {
        printf("argc > 2\n");
        return 0;
    }
#ifdef WITH_CGRAPH
    cout << __LINE__ << endl;
    g = agread(fp, 0);
    cout << __LINE__ << endl;
#else
    g = agread(fp);
#endif

    gvLayout(gvc, g, "dot");

    gvRender(gvc, g, "plain", stdout);;
    gvRender(gvc, g, "png", out);;

    gvFreeLayout(gvc, g);

    agclose(g);

    */
    //return (gvFreeContext(gvc));
    return 0;
}
