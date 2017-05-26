CC = g++
CXXFLAGS = -std=c++14 -I./
TARGET = booang
OBJECTS = Booang.o

all : $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ 
	./$(TARGET)

clean :
	rm *.o $(TARGET)
