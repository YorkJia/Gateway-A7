CROSS = arm-linux-gnueabihf-

CC = $(CROSS)gcc

STRIP = $(CROSS)strip 

CFLAGS = -g -O2 -Wall

# Aliyun SDK 
SRCDIR := Aliiot

IOT_SDK_SOURCE_FILES_C := $(shell find $(SRCDIR) -name "*.c" -not -path "*wrappers*")
IOT_SDK_WRAPPER_IMPL_S := $(shell find $(SRCDIR) -name "*.c" -path "*wrappers*")

IOT_SDK_HDRDIR := $(shell find $(SRCDIR) -type d)
IOT_SDK_HDRDIR := $(addprefix -I,$(IOT_SDK_HDRDIR))

#include
INC = -I./bsp -I./bsp/wraperror \
       -I./bsp/wrappthread \
       -I./bsp/event $(IOT_SDK_HDRDIR)

#lib
LIBS = -lpthread -lrt

#src
SRC = main.c  bsp/wraperror/wraperror.c \
      bsp/wrappthread/wrappthread.c \
      bsp/event/portevent.c \
      $(IOT_SDK_SOURCE_FILES_C) \
      $(IOT_SDK_WRAPPER_IMPL_S)

#target
TARGET = test

#objs
OBJS = $(SRC:.c=.o)

$(TARGET):$(OBJS)
	$(CC) -o $@ $^ $(LIBS)

.PHONY: clean

clean:
	rm -f $(OBJS)

install: $(TARGET) clean
	@echo start compile...
	@echo end.

%.o:%.c
	$(CC) $(CFLAGS) $(INC) $(LIBS) -o $@ -c $<



