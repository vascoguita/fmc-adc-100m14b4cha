
.PHONY: all clean modules install modules_install clean_all
.PHONY: gitmodules prereq prereq_install prereq_install_warn prereq_clean

DIRS = kernel tools lib libtools

all clean modules install modules_install: gitmodules
	@if echo $@ | grep -q install; then $(MAKE) prereq_install_warn; fi
	for d in $(DIRS); do $(MAKE) -C $$d $@ || exit 1; done

all modules: prereq

clean_all: clean prereq_clean

#### The following targets are used to manage prerequisite repositories
gitmodules:
	@test -d fmc-bus/doc || echo "Checking out submodules"
	@test -d fmc-bus/doc || git submodule update --init

# The user can override, using environment variables, the place for our
# three submodules. Note that svec-sw is not built, as it uses cern-internal
# pathnames, and thus won't build elsewhere. We have it as a submodule to
# find needed headers to build kernel code.
FMC_BUS ?= fmc-bus
ZIO ?= zio
SPEC_SW ?= spec-sw
SUBMOD = $(FMC_BUS) $(ZIO) $(SPEC_SW)

prereq:
	for d in $(SUBMOD); do $(MAKE) -C $$d || exit 1; done

prereq_install_warn:
	@test -f .prereq_installed || \
		echo -e "\n\n\tWARNING: Consider \"make prereq_install\"\n"

prereq_install:
	for d in $(SUBMOD); do $(MAKE) -C $$d modules_install || exit 1; done
	touch .prereq_installed

prereq_clean:
	for d in $(SUBMOD); do $(MAKE) -C $$d clean || exit 1; done
