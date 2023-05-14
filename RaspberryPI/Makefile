all: program

program: main.cpp pi2c.o
	g++ -g -Wall main.cpp pi2c.o -o program

pi2c.o: pi2c.cpp pi2c.h
	g++ -g -Wall -c pi2c.cpp -o pi2c.o
	
clean:
	rm -f *.o program
