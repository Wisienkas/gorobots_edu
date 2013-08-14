OPT= -g -Wall

compile_all: main

main.o: main.cpp
	g++ $(OPT) -c main.cpp

dmp.o: dmp.cpp dmp.h
	g++ $(OPT) -c dmp.cpp


main: main.o dmp.o
	g++ -lncurses main.o dmp.o $(OPT) -o main
	chmod a+x main

clean:
	rm -f main
	rm -f *.o


