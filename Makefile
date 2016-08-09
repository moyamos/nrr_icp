CC=g++
INCLUDE=-I/usr/X11R6/include
LIB=-L/usr/X11R6/lib
OUTBIN=nrrICP

all: clean main.o 
#	g++ ${LIB} -lGL -lglut -lXmu -lXi -lstdc++ -lm -lpthread -lncurses -o ${OUTBIN} *.o
	${CC} ${LIB} *.o -lGL -lglut -lGLU -lstdc++ -lpthread -lm -o ${OUTBIN}

main.o:
	g++ ${INCLUDE} -Wno-deprecated -c -Wunused -g *.cpp 
clean:
	rm -rf *.o
	rm -rf *.*~
	rm -rf ${OUTBIN}
