# Raspberry Pi Makefile
CC = gcc
CFLAGS = -shared -fPIC -O2
LDFLAGS = -lm

SRC_DIR = ..
SRCS = $(SRC_DIR)/pid.c $(SRC_DIR)/kalman.c $(SRC_DIR)/ekf_attitude.c

.PHONY: all clean run

all: libcontrol.so

libcontrol.so: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

run: libcontrol.so
	python3 poc_rpi.py

clean:
	rm -f libcontrol.so kalman_pid_result.png
