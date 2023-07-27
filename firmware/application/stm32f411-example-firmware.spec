Name: stm32f411-example-firmware

# disable dynamic rpmbuild checks
%global __os_install_post /bin/true
%global __arch_install_post /bin/true
%global _build_id_links none

AutoReqProv: no

%global firmwareStore /usr/share/fwupd/remotes.d/vendor/firmware

Version: 2.2.0
Release: 2

Summary: STM32F411 example firmware for use with fwupd.
License: MIT
Url: https://github.com/redhat-et/firmware-on-the-edge

Source0: stm32f411-example-vcp-2.2.cab

Requires: fwupd

%description

This package provides a firmware package for fwupd, this is
an example package.


%prep

if [ -d  %{buildroot}%{firmwareStore} ]
then
   rm -rf  %{buildroot}%{firmwareStore}
fi

%clean
rm -rf  %{buildroot}

%install
mkdir -p %{buildroot}%{firmwareStore}
cp %{SOURCE0} %{buildroot}%{firmwareStore}

%pre
# only on install (1), not on upgrades (2)
if [ $1 -eq 1 ]; then
   sed -i 's/Enabled=false/Enabled=true/' /etc/fwupd/remotes.d/vendor-directory.conf
   # WORKAROUND ALERT: remove once we have the new fwupd that fixed the signature issue with vendor-directory
   sed -i 's/OnlyTrusted=true/OnlyTrusted=false/' /etc/fwupd/daemon.conf

   systemctl is-active --quiet fwupd && systemctl restart --quiet fwupd || true
fi
%postun


%files
%{firmwareStore}/*


%changelog
* Wed Mar 22 2023 Miguel Angel Ajo Pelayo <majopela@redhat.com> . 2.2.0-1
Update to 2.2

* Tue Mar 21 2023 Miguel Angel Ajo Pelayo <majopela@redhat.com> . 2.1.0-2
Create the firmware package
