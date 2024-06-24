BINDIR := $(PREFIX)/bin
CFLAGS := -Os -Wall

ucom: ucom.c Makefile
	$(CC) $(CFLAGS) -o $@ $(filter %.c,$^)

install: ucom
	mkdir -p $(DESTDIR)$(BINDIR)
	install -s ucom $(DESTDIR)$(BINDIR)

clean:
	rm -f ucom

.PHONY: install clean
