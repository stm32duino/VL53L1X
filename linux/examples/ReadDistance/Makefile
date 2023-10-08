readDistance: readDistance.o i2c_linux.o
	g++ -std=c++11 -o readDistance readDistance.o i2c_linux.o -li2c

readDistance.o: readDistance.cpp
	g++ -std=c++11 -Wall -I../../src/ -I../../../src/ -c readDistance.cpp

i2c_linux.o: ../../src/*.*
	g++ -std=c++11 -Wall -I../../src/ -I../../../src/ -c ../../src/i2c_linux.cpp

run: readDistance
	./readDistance

edit:
	vim readDistance.cpp

clean:
	rm -f readDistance *.o *~
