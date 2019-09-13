CC       = gcc
CFLAGS   = -fPIC -O3
LFLAGS   =

SRCDIR   = src
OBJDIR   = build

SOURCES  := $(wildcard $(SRCDIR)/*.c)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)
rm       = rm -Rf
	
all: build_buildDir $(OBJECTS)
	$(CC) -shared -Wl,-soname,libpynq.so -o libpynq.so $(OBJECTS) $(LFLAGS)
	ar rcs libpynq.a $(OBJECTS) $(LFLAGS)

install: 
	cp libpynq.so /usr/lib/.
	cp libpynq.a /usr/lib/.
	cp src/pynq_api.h /usr/include/.

uninstall:
	rm -f /usr/lib/libpynq.so
	rm -f /usr/lib/libpynq.a
	rm -f /usr/include/pynq_api.h

build_buildDir:
	@mkdir -p $(OBJDIR)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONEY: clean
clean:
	$(rm) $(OBJDIR)	
	$(rm) libpynq.so
	$(rm) libpynq.a
