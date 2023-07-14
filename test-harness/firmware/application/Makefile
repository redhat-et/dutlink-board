# Copyright (C) 2015 Richard Hughes <richard@hughsie.com>
# Copyright (C) 2023 Miguel Angel Ajo <majopela@redhat.com>
#
# Licensed under the GNU General Public License Version 2
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

TARGET      = jumpstarter
TARGET_ELF  = target/thumbv7em-none-eabihf/release/jumpstarter
VERSION     = 0.01


all: ${TARGET}-$(VERSION).cab

CAB_FILES=							\
	jumpstarter.bin			\
	firmware.metainfo.xml

check: firmware.metainfo.xml
	appstream-util validate-relax $<

$(TARGET_ELF): src/main.rs Cargo.toml memory.x
	cargo build --release

jumpstarter.bin: $(TARGET_ELF)
	arm-none-eabi-objcopy -Obinary $(TARGET_ELF) jumpstarter.bin

jumpstarter.hex: $(TARGET_ELF)
	arm-none-eabi-objcopy -Oihex $(TARGET_ELF) jumpstarter.hex

rpmbuild/SOURCES:
	mkdir -p ./rpmbuild/{BUILD,RPMS,SOURCES,SPECS,SRPMS}

rpmbuild/SOURCES/${TARGET}-${VERSION}.cab: rpmbuild/SOURCES ${TARGET}-$(VERSION).cab
	cp ${TARGET}-${VERSION}.cab rpmbuild/SOURCES

rpm: rpmbuild/SOURCES/${TARGET}-${VERSION}.cab jumpstarter-firmware.spec
	rpmbuild -bb --define "_topdir $(CURDIR)/rpmbuild" jumpstarter-firmware.spec

srpm: rpmbuild/SOURCES/${TARGET}-${VERSION}.cab jumpstarter-firmware.spec
	rpmbuild -bs --define "_topdir $(CURDIR)/rpmbuild" jumpstarter-firmware.spec

${TARGET}-${VERSION}.cab: $(CAB_FILES) check
	gcab --create --nopath $@ $(CAB_FILES)

copr: clean srpm
	copr-cli build @redhat-et/jumpstarter rpmbuild/SRPMS/jumpstarter*.src.rpm


all: ${TARGET}-${VERSION}.cab

clean:
	rm -rf target jumpstarter.hex jumpstarter.bin *.cab rpmbuild

