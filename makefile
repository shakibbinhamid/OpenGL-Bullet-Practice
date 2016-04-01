BIN = courseworkp
CC = g++
FLAGS = -std=c++11 -stdlib=libc++
INC = -I/usr/local/include -I/usr/local/include/bullet -I ./include
LIB_PATH = ./lib/
LOC_LIB = $(LIB_PATH)libGLEW.a $(LIB_PATH)libglfw3.a $(LIB_PATH)libBulletDynamics.a $(LIB_PATH)libBulletCollision.a $(LIB_PATH)libLinearMath.a
FRAMEWORKS = -framework Cocoa -framework OpenGL -framework IOKit
SRC = main.cpp gl_util.cpp

all: compile

compile:
	${CC} ${FLAGS} ${FRAMEWORKS} -o ${BIN} ${SRC} ${INC} ${LOC_LIB}