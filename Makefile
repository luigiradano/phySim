CC		= gcc
CFLAGS	= -lSDL -lSDL2_ttf -lm
FFLAGS	= -lSDL -lSDL2_ttf -lm
RM		= rm -rf
OBJS	= ./build/main.o ./build/plot.o ./build/constraint.o ./build/rigid.o ./build/matrix.o
INCLUDE = main.h plot.h constraintSolve.h rigidBody.h matrixOps.h


main: main.c main.h
	$(CC) $(CFLAGS) -c main.c -o ./build/main.o
plot: plot.c plot.h main.h
	$(CC) $(CFLAGS) -c plot.c -o ./build/plot.o
constraintSolve: constraintSolve.c constraintSolve.h main.h
	$(CC) $(CFLAGS) -c constraintSolve.c -o ./build/constraint.o
rigidBody: rigidBody.c rigidBody.h
	$(CC) $(CFLAGS) -c rigidBody.c -o ./build/rigid.o
matrixOps: matrixOps.c matrixOps.h
	$(CC) $(CFLAGS) -c matrixOps.c -o ./build/matrix.o

build: init plot constraintSolve rigidBody matrixOps main final

init:
	mkdir build -p

final:
	$(CC) ./build/main.o ./build/plot.o ./build/constraint.o ./build/rigid.o ./build/matrix.o -o phySim -L /usr/lib/ -I /usd/lib

clean:
	$(RM) phySim
	$(RM) build	
