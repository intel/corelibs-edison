
ifdef CROSS_COMPILE
CXX := $(CROSS_COMPILE)g++
endif

# Runtime daemon
CSRCS_IZRD = izrd/main.cpp

CFLAGS_EXTRA :=-m32 -Os -march=pentium -Wall -g -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -std=c++98 -Werror -Iinclude
LDFLAGS_EXTRA :=
OBJS_CXX = $(CSRCS_IZRD:.cpp=.o)
DEPEND_CXX = $(CSRCS_IZRD:.cpp=.d)
BIN = ../../bin
DESTDIR = /opt/izmir/bin

%.o : %.c 
	$(CC) $(CFLAGS) $(CFLAGS_EXTRA) -c -o $@ $<

all: $(OBJS_CXX)
	mkdir -p $(BIN)
	$(CXX) -o $(BIN)/izrd $(OBJS) $(CFLAGS) $(CFLAGS_EXTRA) $(LDFLAGS) $(LDFLAGS_EXTRA)

clean:
	${RM} $(BIN)/* $(DEPEND)
	${RM} $(OBJS)
	${RM} -r ${DESTDIR}

%.d: %.c
	@echo DEPEND $<; \
        rm -f $@; \
        $(CC) -MM $(CPPFLAGS) $(CFLAGS) $< > $@.$$$$; \
        sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
        rm -f $@.$$$$

ifneq ($(MAKECMDGOALS), clean)
-include $(DEPEND)
endif

install: all
	install -d $(DESTDIR)
	install -m 0755 bin/* $(DESTDIR)
	install -m 0755 *.sh $(DESTDIR)
	install -m 0755 runscript-cln $(DESTDIR)
