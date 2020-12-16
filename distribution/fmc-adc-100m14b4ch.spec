# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

# Mainline copied from the template, added requirements

name:		fmc-adc-100m14b4ch
version:	%{?_build_version}
Release:	1%{?dist}
Summary:	Linux kernel FMC-ADC-10M support
BuildArch:	noarch


Group:		System/Kernel
License:	GPL-2.0-or-later
URL:		https://ohwr.org/project/fmc-adc-100m14b4cha
Source0:	%{name}-%{version}.tar.gz
Source1:	CHANGELOG

BuildRequires: dkms
Requires: dkms >= 1.95, zio >= 1.4, fmc >= 1.1

%description
Kernel modules for FMC-ADC-100M in a DKMS wrapper.

%prep
%setup -q

%install
mkdir -p %{buildroot}/usr/src/%{name}-%{version}
cp -ra * %{buildroot}/usr/src/%{name}-%{version}/

%post
dkms add -m %{name} -v %{version} --rpm_safe_upgrade
dkms build -m %{name} -v %{version} --rpm_safe_upgrade
dkms install -m %{name} -v %{version} --rpm_safe_upgrade

%preun
dkms remove -m %{name} -v %{version} --rpm_safe_upgrade --all ||:

%files
/usr/src/%{name}-%{version}/

%changelog
%include %{SOURCE1}
