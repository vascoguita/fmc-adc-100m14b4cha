# Copyright (C) 2020 CERN
# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

TOP_DIR ?= $(shell pwd)/../

NAME := fmc-adc-100m14b4ch
VERSION := $(shell git describe --abbrev=0 | tr -d 'v')

DIR_NAME := $(NAME)-$(VERSION)
KEEP_TEMP ?= n
BUILD ?= $(abspath build)
SOURCES ?= $(abspath sources)
PKG_SOURCES := $(SOURCES)/$(DIR_NAME)
BUILD_DKMS := $(BUILD)/dkms
BUILD_DKMSSOURCE := $(BUILD_DKMS)/source
BUILD_DKMSTREE := $(BUILD_DKMS)/tree
RPM_SPEC := $(shell mktemp /tmp/$(NAME)-$(VERSION)-XXXXXX.spec)

DKMS_OPT := --dkmstree $(BUILD_DKMSTREE) -m $(NAME)/$(VERSION)
FIND_OPT := -type f -regextype posix-extended

all: kernel

kernel: dkms-tar dkms-rpm

dkms-tree:
	@mkdir -p $(BUILD_DKMSSOURCE)
	@mkdir -p $(BUILD_DKMSTREE)

dkms-src: dkms-tree sources-build
	@cp -a $(PKG_SOURCES) $(BUILD_DKMSSOURCE)

dkms-add: dkms-src
	@dkms add $(DKMS_OPT) --sourcetree $(BUILD_DKMSSOURCE)

dkms-tar: dkms-add
	@dkms mktarball $(DKMS_OPT) --source-only

dkms-rpm: dkms-add
	@dkms mkrpm $(DKMS_OPT) --source-only

dkms-build: dkms-add
	@dkms build $(DKMS_OPT)

sources-build:
	@mkdir -p $(PKG_SOURCES)
	@find $(TOP_DIR)/software/kernel $(FIND_OPT) -regex '.+/([^.]+[.](c|h)|Makefile|Kbuild)' -exec cp \{} $(PKG_SOURCES) \;
	@mkdir -p $(PKG_SOURCES)/platform_data
	@cp $(TOP_DIR)/software/kernel/platform_data/* $(PKG_SOURCES)/platform_data/
	@cp $(TOP_DIR)/software/kernel/fmc-adc-100m14b4cha.h $(PKG_SOURCES)/
	@cp $(TOP_DIR)/LICENSES/GPL-2.0-or-later.txt $(PKG_SOURCES)/LICENSE
	@cp -a $(TOP_DIR)/CHANGELOG.rst $(PKG_SOURCES)/CHANGELOG.rst
	@cp -a $(TOP_DIR)/distribution/dkms.conf $(PKG_SOURCES)/
	@sed -r -i -e "s/^GIT_VERSION\s=\s.*/GIT_VERSION = $(VERSION)/" $(PKG_SOURCES)/Makefile
	@sed -r -i -e "s/@PKGVER@/$(VERSION)/g" -e "s/@PKGNAME@/$(NAME)/g" $(PKG_SOURCES)/dkms.conf

sources-tar: sources-build
	cd $(SOURCES); tar czf $(DIR_NAME).tar.gz --exclude-vcs $(DIR_NAME)

sources: sources-tar

srpm-changelog: sources
	$(eval $@_pattern := ^([0-9]+\.[0-9]+\.[0-9]+)\s-\s([0-9]{4}-[0-9]{2}-[0-9]{2})$)
	$(eval $@_replace := echo -e "\n"\\* `date --date="\2" "+%a %b %d %Y"` "\1")

	@cp -a $(PKG_SOURCES)/CHANGELOG.rst $(SOURCES)/CHANGELOG
	@sed -r -i -e "/^[.]{2}/d" -e "/^\s{2}.*$$/d" $(SOURCES)/CHANGELOG
	@sed -r -i -e "/Changelog/d" -e "/^(=|-|\s)*$$/d" $(SOURCES)/CHANGELOG
	@sed -r -i -e 's,$($@_pattern),$($@_replace),e' $(SOURCES)/CHANGELOG

rpm-spec: fmc-adc-100m14b4ch.spec
# The fmc-adc-100m14b4ch.spec file is good when we can define
# variables. With tools like koji it is not possible, so we need to
# create a .spec file without using package-specific variables.
	@cp $< $(RPM_SPEC)
	@sed -i -e "s/%{?_build_version}/$(VERSION)/" $(RPM_SPEC)

srpm: sources rpm-spec srpm-changelog
	@rpmbuild -bs --define "_topdir $(BUILD)" \
		--define '_sourcedir $(SOURCES)' \
		--define "_build_version $(VERSION)" \
		$(RPM_SPEC)
ifeq ($(KEEP_TEMP), n)
	@rm -f $(RPM_SPEC)
endif

clean:
	@rm -rf $(BUILD) $(SOURCES) *.tar.gz $(RPM_SPEC)

.PHONY: sources sources-tar sources-build
.PHONY: dkmstree dkms-add kernel-dkms-tar
