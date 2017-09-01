CC = g++
CXXFLAGS = -std=c++14 -I./ -I/usr/include/graphviz
GRAPHVIZ = -std=c++14 
TARGET = booang
OBJECTS = booang.hpp graphviz.cpp 
SRCS = graphviz.cpp

all : $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CXXFLAGS) -o $(TARGET).o -c $(SRCS)
	$(CC) $(CXXFLAGS) $(GRAPHVIZ) -o $(TARGET) $(TARGET).o `pkg-config libcgraph libgvc --cflags --libs`

clean :
	rm *.o 

kdot : kdot.cpp
	g++ -c kdot.cpp -o kdot.o -I/usr/include/graphviz
	g++ kdot.o -o kdot 

	# $(CC) $(CXXFLAGS2) -o kdot kdot.cpp




