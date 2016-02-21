all: test

CFLAGS=-fPIC -g -Wall `pkg-config --cflags opencv -lpthread`
LIBS = `pkg-config --libs pthread opencv libusb-1.0`
INCLUDE = -I/usr/local/include/libfreenect
INCLUDE = -I/usr/include
INCLUDE = -I/usr/include/opencv
FREE_LIBS = -L/usr/local/lib -lfreenect

test:  test.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test
