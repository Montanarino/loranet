CC      = gcc
TARGET  = loracall
SRC     = loracall.c
 
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -lopus -lpthread -ldl -lm
 
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    LDFLAGS += -framework CoreAudio -framework AudioToolbox \
               -framework CoreFoundation
endif
 
.PHONY: all clean deps
 
all: miniaudio.h $(TARGET)
 
$(TARGET): $(SRC) miniaudio.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@echo ""
	@echo "  Build OK!"
	@echo "  Uso: ./$(TARGET) /dev/ttyUSB0 9600 433.000"
	@echo ""
 
miniaudio.h:
	@echo "Scarico miniaudio.h..."
	@curl -fsSL https://raw.githubusercontent.com/mackron/miniaudio/master/miniaudio.h \
	  -o miniaudio.h 2>/dev/null || \
	  wget -q https://raw.githubusercontent.com/mackron/miniaudio/master/miniaudio.h
 
deps:
ifeq ($(UNAME), Linux)
	sudo apt-get install -y libopus-dev
else ifeq ($(UNAME), Darwin)
	brew install opus
endif
 
clean:
	rm -f $(TARGET)
