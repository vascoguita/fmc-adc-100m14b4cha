# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/..
-include $(REPO_PARENT)/parent_common.mk

all: kernel tools

ZIO ?= zio
SVEC_SW ?= svec-sw
VMEBUS ?= $(REPO_PARENT)/vmebridge

# Use the absolute path so it can be used by submodule
# FMC_ABS and ZIO_ABS has to be absolut path,
# due to beeing passed to the Kbuild
FMC_ABS ?= $(abspath $(FMC) )
ZIO_ABS ?= $(abspath $(ZIO) )
SVEC_SW_ABS ?= $(abspath $(SVEC_SW) )
VMEBUS_ABS ?= $(abspath $(VMEBUS) )

export FMC_ABS
export ZIO_ABS
export SVEC_SW_ABS
export VMEBUS_ABS

DIRS = $(FMC_ABS) $(ZIO_ABS) kernel tools

kernel: $(FMC_ABS) $(ZIO_ABS)

.PHONY: all clean modules install modules_install $(DIRS)
.PHONY: gitmodules prereq_install prereq_install_warn

install modules_install: prereq_install_warn

all clean modules install modules_install: $(DIRS)

clean: TARGET = clean
modules: TARGET = modules
install: TARGET = install
modules_install: TARGET = modules_install


$(DIRS):
	$(MAKE) -C $@ $(TARGET)


SUBMOD = $(ZIO_ABS)

prereq_install_warn:
	@test -f .prereq_installed || \
		echo -e "\n\n\tWARNING: Consider \"make prereq_install\"\n"

prereq_install:
	for d in $(SUBMOD); do $(MAKE) -C $$d modules_install || exit 1; done
	touch .prereq_installed

$(ZIO_ABS): zio-init_repo

# init submodule if missing
zio-init_repo:
	@test -d $(ZIO_ABS)/doc || ( echo "Checking out submodule $(ZIO_ABS)" && git submodule update --init $(ZIO_ABS) )
