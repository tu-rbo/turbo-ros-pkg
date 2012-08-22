# Makefile for matrix library

CC=gcc
OPTFLAGS=-O3
OTHERFLAGS=-Wall -D__NO_MKL__

INCLUDE_PATH = -I../imagelib -I../cminpack -I../clapack/INCLUDE -I../../include/visual_net/sfm/bundler
CFLAGS = $(OPTFLAGS) $(OTHERFLAGS) $(INCLUDE_PATH)

TARGET = libmatrix.a
OBJS = matrix.o vector.o svd.o

all: $(TARGET)

$(TARGET): $(OBJS)
	ar r $@ $(OBJS)
	cp $@ ..

clean:
	rm -f *.o $(TARGET) *~
