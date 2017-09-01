CC = g++
CXXFLAGS = -std=c++14 -I./ -I/usr/include/graphviz




GRAPHVIZ = graph_viz
GRAPHVIZ_SRC = graphviz.cpp

BASIC = basic 
BASIC_SRC = basic.cpp
all : $(BASIC)

$(GRAPHVIZ): booang.hpp $(GRAPHVIZ_SRC)
	$(CC) $(CXXFLAGS) -o $@.o -c $(GRAPHVIZ_SRC)
	$(CC) $(CXXFLAGS) -o $@ $@.o -lcgraph -lcdt -lgvc
	./$(GRAPHVIZ)

$(BASIC): booang.hpp $(BASIC_SRC)
	$(CC) $(CXXFLAGS) -o $@.o -c $(BASIC_SRC)
	$(CC) $(CXXFLAGS) -o $@ $@.o -lcgraph -lcdt -lgvc
	# ./$@
clean :
	rm $(BASIC) $(GRAPHVIZ)

kdot : kdot.cpp
	g++ -c kdot.cpp -o kdot.o -I/usr/include/graphviz
	g++ kdot.o -o kdot 

	# $(CC) $(CXXFLAGS2) -o kdot kdot.cpp




