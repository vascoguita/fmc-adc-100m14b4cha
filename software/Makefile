# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
-include $(REPO_PARENT)/parent_common.mk

all: kernel tools

DIRS =kernel tools

.PHONY: all clean modules install modules_install cppcheck $(DIRS)

install modules_install:

all clean modules install modules_install: $(DIRS)

clean: TARGET = clean
modules: TARGET = modules
install: TARGET = install
modules_install: TARGET = modules_install


$(DIRS):
	$(MAKE) -C $@ $(TARGET)

cppcheck:
	@$(MAKE) -C tools $@
