#include <gvc.h>

int main(int argc, char **argv)
{
    GVC_t *gvc;
    graph_t *g;
    FILE *fp;
    FILE* out;

    gvc = gvContext();

    if (argc > 2){
        fp = fopen(argv[1], "r");
        out = fopen(argv[2], "w"); 
    }
    else{
        printf("argc > 2\n");
        return 0;
    }
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
