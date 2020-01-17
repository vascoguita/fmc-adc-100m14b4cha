-include Makefile.specific
# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/..
-include $(REPO_PARENT)/parent_common.mk

DIRS = drivers

all clean install: $(DIRS)

.PHONY: all clean install $(DIRS)

clean:   TARGET = clean
install: TARGET = install

$(DIRS):
	$(MAKE) -C $@ $(TARGET)
