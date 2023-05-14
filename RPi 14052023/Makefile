all: program

program: main.cpp pi2c.o StewartPlatform.o myMatrixMath.o
	g++ -g -Wall main.cpp pi2c.o StewartPlatform.o myMatrixMath.o -o program

pi2c.o: pi2c.cpp pi2c.h
	g++ -g -Wall -c pi2c.cpp -o pi2c.o

StewartPlatform.o: StewartPlatform.cpp StewartPlatform.h
	g++ -g -Wall -c StewartPlatform.cpp -o StewartPlatform.o

myMatrixMath.o: myMatrixMath.cpp myMatrixMath.h
	g++ -g -Wall -c myMatrixMath.cpp -o myMatrixMath.o
	
clean:
	rm -f *.o program
