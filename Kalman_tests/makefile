all: test

CFLAGS=-fPIC -g3 -O0 -Wall `pkg-config --cflags opencv` -std=c++11 -fpermissive
LIBS = `pkg-config --libs opencv`
INCLUDE = 
FREE_LIBS = -L/usr/local/lib -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect

test:  *.cpp 
	g++ $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	g++ -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test
