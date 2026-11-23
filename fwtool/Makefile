CC := gcc
CFLAGS := -O2 -Wall

TARGET := fwtool
OBJS := fwtool.o tsv.o crc32.o
ASV5211FW := asv5211fw

all: $(TARGET) $(ASV5211FW)

clean:
	rm -vf $(TARGET) $(OBJS) $(ASV5211FW)

depend:
	$(CC) -MM $(OBJS:.o=.c) > Makefile.dep

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS)

$(ASV5211FW): asv5211fw.c
	$(CC) $(CFLAGS) -o $@ $<

-include Makefile.dep
