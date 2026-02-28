# BeagleBone Black Makefile
CC = gcc
CFLAGS = -O2 -Wall
LDFLAGS = -lm -lrt

SRC_DIR = ..
SRCS = poc_bbb.c $(SRC_DIR)/pid.c $(SRC_DIR)/kalman.c

.PHONY: all clean run

all: poc_bbb

poc_bbb: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

run: poc_bbb
	sudo ./poc_bbb | tee data_log.csv

clean:
	rm -f poc_bbb data_log.csv
