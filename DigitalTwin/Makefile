cxx = g++
CFLAGS = -Wall
INCLUDEPATH = -I "./../lib"
DEPS = ./../lib/sendReceive.h ./../lib/platformControll.h ./../lib/myMatrixMath.h ./../lib/circularBuffer.h
LIBS = -lm -lpthread
OBJ = controller.o ./../lib/sendReceive.o ./../lib/platformControll.o ./../lib/myMatrixMath.o ./../lib/circularBuffer.o
TARGET = DTControll.out

${TARGET}: ${OBJ} ${DEPS}
	${cxx} ${CFLAGS} -o $@ ${OBJ} ${LIBS}
clean:
	rm -f ${OBJ} *~
