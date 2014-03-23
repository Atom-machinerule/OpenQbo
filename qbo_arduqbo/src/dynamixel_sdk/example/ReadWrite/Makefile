TARGET		= ReadWrite
OBJS		= ReadWrite.o
SRCS		= $(OBJS:.o=.c)
INCLUDEDIRS	+= -I../../include
LIBDIRS		+= -L../../lib
CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2
LDFLAGS		= $(LIBDIRS) -lm -ldxl

CC			= gcc

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	
.c.o:
	$(CC) -c $< $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

