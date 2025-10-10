CC       = gcc
CFLAGS   = -fPIC -O3
LFLAGS   =

SRCDIR   = src
OBJDIR   = build

SOURCES  := $(wildcard $(SRCDIR)/*.c)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)
rm       = rm -Rf

PREFIX := /usr
	
all: build_buildDir $(OBJECTS)
	$(CC) -shared -Wl,-soname,libpynq.so -o libpynq.so $(OBJECTS) $(LFLAGS)
	ar rcs libpynq.a $(OBJECTS) $(LFLAGS)

install: 
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	cp libpynq.so $(PREFIX)/lib/.
	cp libpynq.a $(PREFIX)/lib/.
	cp src/pynq_api.h $(PREFIX)/include/.

uninstall:
	rm -f $(PREFIX)/lib/libpynq.so
	rm -f $(PREFIX)/lib/libpynq.a
	rm -f $(PREFIX)/include/pynq_api.h

build_buildDir:
	@mkdir -p $(OBJDIR)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONEY: clean
clean:
	$(rm) $(OBJDIR)	
	$(rm) libpynq.so
	$(rm) libpynq.a
