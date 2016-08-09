CC=g++
INCLUDE=
LIB=
OUTBIN=nrrICP

all: clean main.o 
	${CC} ${LIB} *.o -lGL -lglut -lGLU -o ${OUTBIN}

main.o:
	g++ ${INCLUDE} -Wno-deprecated -c -Wunused -g *.cpp 
clean:
	rm -rf *.o
	rm -rf *.*~
	rm -rf ${OUTBIN}
