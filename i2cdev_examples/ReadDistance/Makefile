readDistance: readDistance.o
	g++ -std=c++11 -o readDistance readDistance.o -li2c

readDistance.o: readDistance.cpp ../../src/*.hpp
	g++ -std=c++11 -Wall -I../../src/ -c readDistance.cpp

run: readDistance
	./readDistance

edit:
	vim readDistance.cpp

clean:
	rm -f readDistance *.o *~
