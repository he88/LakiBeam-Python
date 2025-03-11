CC = gcc
CFLAGS = -Wall -Wextra -fPIC -I.

TARGET = rb_lidar.so
SOURCES = rb_lidar.c
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -shared -o $@ $^ -lpthread

%.o: %.c rb_lidar.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all clean