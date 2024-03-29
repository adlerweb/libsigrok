##
## This file is part of the libsigrok project.
##
## Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>
## Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

# We require at least autoconf 2.63 (AC_INIT format changed there).
AC_PREREQ([2.63])

# libsigrok package version number (NOT the same as shared lib version!).
m4_define([sr_package_version_major], [0])
m4_define([sr_package_version_minor], [2])
m4_define([sr_package_version_micro], [0])
m4_define([sr_package_version], [sr_package_version_major.sr_package_version_minor.sr_package_version_micro])

AC_INIT([libsigrok], [sr_package_version], [sigrok-devel@lists.sourceforge.net],
	[libsigrok], [http://www.sigrok.org])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_MACRO_DIR([autostuff])
AC_CONFIG_AUX_DIR([autostuff])

# We require at least automake 1.11 (needed for 'silent rules').
AM_INIT_AUTOMAKE([1.11 -Wall -Werror check-news color-tests])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

AH_TOP([#ifndef SR_CONFIG_H
#define SR_CONFIG_H    /* To stop multiple inclusions. */])
AH_BOTTOM([#endif /* SR_CONFIG_H */])

# Enable more compiler warnings via -Wall and -Wextra. Add -fvisibility=hidden
# and enforce use of SR_API to explicitly mark all public API functions.
CFLAGS="$CFLAGS -Wall -Wextra -fvisibility=hidden"

# Checks for programs.
AC_PROG_CC
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S

# Required for per-target flags or subdir-objects with C sources.
AM_PROG_CC_C_O

# Initialize libtool.
LT_INIT

# Initialize pkg-config.
# We require at least 0.22, as "Requires.private" behaviour changed there.
PKG_PROG_PKG_CONFIG([0.22])

# Library version for libsigrok (NOT the same as the package version).
# Carefully read the libtool docs before updating these numbers!
# The algorithm for determining which number to change (and how) is nontrivial!
# http://www.gnu.org/software/libtool/manual/libtool.html#Updating-version-info
SR_LIB_VERSION_CURRENT=1
SR_LIB_VERSION_REVISION=0
SR_LIB_VERSION_AGE=0
SR_LIB_VERSION="$SR_LIB_VERSION_CURRENT:$SR_LIB_VERSION_REVISION:$SR_LIB_VERSION_AGE"
SR_LIB_LDFLAGS="-version-info $SR_LIB_VERSION"
AC_SUBST(SR_LIB_VERSION_CURRENT)
AC_SUBST(SR_LIB_VERSION_REVISION)
AC_SUBST(SR_LIB_VERSION_AGE)
AC_SUBST(SR_LIB_VERSION)
AC_SUBST(SR_LIB_LDFLAGS)

# Hardware support '--enable' options.

AC_ARG_ENABLE(all-drivers, AC_HELP_STRING([--enable-all-drivers],
	[enable all drivers by default [default=yes]]),
	[HW_ENABLED_DEFAULT="$enableval"],
	[HW_ENABLED_DEFAULT="yes"])

AC_ARG_ENABLE(agilent-dmm, AC_HELP_STRING([--enable-agilent-dmm],
	[enable Agilent DMM support [default=yes]]),
	[HW_AGILENT_DMM="$enableval"],
	[HW_AGILENT_DMM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(alsa, AC_HELP_STRING([--enable-alsa],
	[enable ALSA driver support [default=yes]]),
	[HW_ALSA="$enableval"],
	[HW_ALSA=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(asix-sigma, AC_HELP_STRING([--enable-asix-sigma],
	[enable ASIX SIGMA/SIGMA2 support [default=yes]]),
	[HW_ASIX_SIGMA="$enableval"],
	[HW_ASIX_SIGMA=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(brymen-dmm, AC_HELP_STRING([--enable-brymen-dmm],
	[enable Brymen DMM support [default=yes]]),
	[HW_BRYMEN_DMM="$enableval"],
	[HW_BRYMEN_DMM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(cem-dt-885x, AC_HELP_STRING([--enable-cem-dt-885x],
	[enable CEM DT-885x support [default=yes]]),
	[HW_CEM_DT_885X="$enableval"],
	[HW_CEM_DT_885X=yes])

AC_ARG_ENABLE(chronovu-la8, AC_HELP_STRING([--enable-chronovu-la8],
	[enable ChronoVu LA8 support [default=yes]]),
	[HW_CHRONOVU_LA8="$enableval"],
	[HW_CHRONOVU_LA8=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(colead-slm, AC_HELP_STRING([--enable-colead-slm],
	[enable Colead SLM support [default=yes]]),
	[HW_COLEAD_SLM="$enableval"],
	[HW_COLEAD_SLM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(demo, AC_HELP_STRING([--enable-demo],
	[enable demo driver support [default=yes]]),
	[HW_DEMO="$enableval"],
	[HW_DEMO=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(fluke-dmm, AC_HELP_STRING([--enable-fluke-dmm],
	[enable Fluke DMM support [default=yes]]),
	[HW_FLUKE_DMM="$enableval"],
	[HW_FLUKE_DMM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(fx2lafw, AC_HELP_STRING([--enable-fx2lafw],
	[enable fx2lafw support (for FX2 LAs). [default=yes]]),
	[HW_FX2LAFW="$enableval"],
	[HW_FX2LAFW=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(hantek-dso, AC_HELP_STRING([--enable-hantek-dso],
	[enable Hantek DSO support [default=yes]]),
	[HW_HANTEK_DSO="$enableval"],
	[HW_HANTEK_DSO=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(ikalogic-scanalogic2, AC_HELP_STRING([--enable-ikalogic-scanalogic2],
	[enable IKALOGIC Scanalogic2 support [default=yes]]),
	[HW_IKALOGIC_SCANALOGIC2="$enableval"],
	[HW_IKALOGIC_SCANALOGIC2=yes])

AC_ARG_ENABLE(lascar-el-usb, AC_HELP_STRING([--enable-lascar-el-usb],
	[enable Lascar EL-USB support [default=yes]]),
	[HW_LASCAR_EL_USB="$enableval"],
	[HW_LASCAR_EL_USB=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(link-mso19, AC_HELP_STRING([--enable-link-mso19],
	[enable Link Instruments MSO-19 support [default=yes]]),
	[HW_LINK_MSO19="$enableval"],
	[HW_LINK_MSO19=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(mic-985xx, AC_HELP_STRING([--enable-mic-985xx],
	[enable MIC 985xx support [default=yes]]),
	[HW_MIC_985XX="$enableval"],
	[HW_MIC_985XX=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(nexus-osciprime, AC_HELP_STRING([--enable-nexus-osciprime],
	[enable Nexus Osciprime support [default=yes]]),
	[HW_NEXUS_OSCIPRIME="$enableval"],
	[HW_NEXUS_OSCIPRIME=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(ols, AC_HELP_STRING([--enable-ols],
	[enable OpenBench Logic Sniffer (OLS) support [default=yes]]),
	[HW_OLS="$enableval"],
	[HW_OLS=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(rigol-ds1xx2, AC_HELP_STRING([--enable-rigol-ds1xx2],
	[enable Rigol DS1xx2 support [default=yes]]),
	[HW_RIGOL_DS1XX2="$enableval"],
	[HW_RIGOL_DS1XX2=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(serial-dmm, AC_HELP_STRING([--enable-serial-dmm],
	[enable serial DMM support [default=yes]]),
	[HW_SERIAL_DMM="$enableval"],
	[HW_SERIAL_DMM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(tondaj-sl-814, AC_HELP_STRING([--enable-tondaj-sl-814],
	[enable Tondaj SL-814 support [default=yes]]),
	[HW_TONDAJ_SL_814="$enableval"],
	[HW_TONDAJ_SL_814=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(uni-t-dmm, AC_HELP_STRING([--enable-uni-t-dmm],
	[enable UNI-T DMM support [default=yes]]),
	[HW_UNI_T_DMM="$enableval"],
	[HW_UNI_T_DMM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(victor-dmm, AC_HELP_STRING([--enable-victor-dmm],
	[enable victor-dmm support [default=yes]]),
	[HW_VICTOR_DMM="$enableval"],
	[HW_VICTOR_DMM=$HW_ENABLED_DEFAULT])

AC_ARG_ENABLE(zeroplus-logic-cube,
	AC_HELP_STRING([--enable-zeroplus-logic-cube],
	[enable ZEROPLUS Logic Cube support [default=yes]]),
	[HW_ZEROPLUS_LOGIC_CUBE="$enableval"],
	[HW_ZEROPLUS_LOGIC_CUBE=$HW_ENABLED_DEFAULT])

# Checks for libraries.

# This variable collects the pkg-config names of all detected libs.
# It is then used to construct the "Requires.private:" field in the
# libsigrok.pc file.
SR_PKGLIBS=""

# libm (the standard math library) is always needed.
AC_SEARCH_LIBS([pow], [m])

# libglib-2.0 is always needed. Abort if it's not found.
# Note: glib-2.0 is part of the libsigrok API (hard pkg-config requirement).
# We require at least 2.32.0 due to e.g. g_variant_new_fixed_array().
AM_PATH_GLIB_2_0([2.32.0],
	[CFLAGS="$CFLAGS $GLIB_CFLAGS"; LIBS="$LIBS $GLIB_LIBS"])

# libzip is always needed. Abort if it's not found.
PKG_CHECK_MODULES([libzip], [libzip >= 0.8],
	[CFLAGS="$CFLAGS $libzip_CFLAGS"; LIBS="$LIBS $libzip_LIBS";
	SR_PKGLIBS="$SR_PKGLIBS libzip"])

# libusb-1.0 is only needed for some hardware drivers. Disable the respective
# drivers if it is not found.
case "$host" in
*freebsd*)
	# FreeBSD comes with an "integrated" libusb-1.0-style USB API.
	# This means libusb-1.0 is always available, no need to check for it,
	# and no need to (potentially) disable any drivers if it's not found.
	AC_DEFINE_UNQUOTED(HAVE_LIBUSB_1_0, [1],
		[Specifies whether we have a libusb.h header.])
	;;
*)
	PKG_CHECK_MODULES([libusb], [libusb-1.0 >= 1.0.9],
		[have_libusb1_0="yes"; CFLAGS="$CFLAGS $libusb_CFLAGS";
		LIBS="$LIBS $libusb_LIBS";
		SR_PKGLIBS="$SR_PKGLIBS libusb-1.0"],
		[have_libusb1_0="no"; HW_FX2LAFW="no"; HW_HANTEK_DSO="no";
		HW_IKALOGIC_SCANALOGIC2="no"; HW_LASCAR_EL_USB="no";
		HW_NEXUS_OSCIPRIME="no"; HW_UNI_T_DMM="no"; HW_VICTOR_DMM="no";
		HW_ZEROPLUS_LOGIC_CUBE="no"])

	# Define HAVE_LIBUSB_1_0 in config.h if we found libusb-1.0.
	if test "x$have_libusb1_0" != "xno"; then
		AC_DEFINE_UNQUOTED(HAVE_LIBUSB_1_0, [1],
			[Specifies whether we have a libusb.h header.])
	fi
	;;
esac

# USB + FX2 firmware helper code is only compiled in if libusb-1.0 was found.
AM_CONDITIONAL(NEED_USB, test "x$have_libusb1_0" != xno)

# libftdi is only needed for some hardware drivers. Disable them if not found.
PKG_CHECK_MODULES([libftdi], [libftdi >= 0.16],
	[CFLAGS="$CFLAGS $libftdi_CFLAGS";
	LIBS="$LIBS $libftdi_LIBS";
	SR_PKGLIBS="$SR_PKGLIBS libftdi"],
	[HW_ASIX_SIGMA="no"; HW_CHRONOVU_LA8="no"])

# libudev is only needed for some hardware drivers. Disable them if not found.
PKG_CHECK_MODULES([libudev], [libudev >= 151],
	[CFLAGS="$CFLAGS $libudev_CFLAGS"; LIBS="$LIBS $libudev_LIBS";
	SR_PKGLIBS="$SR_PKGLIBS libudev"],
	[HW_LINK_MSO19="no"])

# ALSA is only needed for some hardware drivers. Disable them if not found.
PKG_CHECK_MODULES([alsa], [alsa >= 1.0],
	[CFLAGS="$CFLAGS $alsa_CFLAGS"; LIBS="$LIBS $alsa_LIBS";
	SR_PKGLIBS="$SR_PKGLIBS alsa"],
	[HW_ALSA="no"])

# The Check unit testing framework is optional. Disable if not found.
PKG_CHECK_MODULES([check], [check >= 0.9.4],
	[have_check="yes"; CFLAGS="$CFLAGS $check_CFLAGS";
	LIBS="$LIBS $check_LIBS"], [have_check="no"])
AM_CONDITIONAL(HAVE_CHECK, test x"$have_check" = "xyes")

# The Rigol DS1xx2 driver currently uses the Linux kernel usbtmc module
# (though it is planned to rewrite the driver to be portable later).
# Thus, it will be disabled for non-Linux builds for now.
case "$host" in
*linux*)
	# Do nothing. Whether the driver is enabled is determined by the
	# previous --enable-all-drivers/--disable-all-drivers and/or any
	# --enable-rigol-ds1xx2/--disable-rigol-ds1xx2 options.
	;;
*)
	# Disable the driver for builds that don't target Linux.
	HW_RIGOL_DS1XX2="no"
	;;
esac

AC_SUBST(SR_PKGLIBS)

# Now set AM_CONDITIONALs and AC_DEFINEs for the enabled/disabled drivers.

AM_CONDITIONAL(HW_AGILENT_DMM, test x$HW_AGILENT_DMM = xyes)
if test "x$HW_AGILENT_DMM" = "xyes"; then
	AC_DEFINE(HAVE_HW_AGILENT_DMM, 1, [Agilent DMM support])
fi

AM_CONDITIONAL(HW_ALSA, test x$HW_ALSA = xyes)
if test "x$HW_ALSA" = "xyes"; then
	AC_DEFINE(HAVE_HW_ALSA, 1, [ALSA driver support])
fi

AM_CONDITIONAL(HW_ASIX_SIGMA, test x$HW_ASIX_SIGMA = xyes)
if test "x$HW_ASIX_SIGMA" = "xyes"; then
	AC_DEFINE(HAVE_HW_ASIX_SIGMA, 1, [ASIX SIGMA/SIGMA2 support])
fi

AM_CONDITIONAL(HW_BRYMEN_DMM, test x$HW_BRYMEN_DMM = xyes)
if test "x$HW_BRYMEN_DMM" = "xyes"; then
	AC_DEFINE(HAVE_HW_BRYMEN_DMM, 1, [Brymen DMM support])
fi

AM_CONDITIONAL(HW_CEM_DT_885X, test x$HW_CEM_DT_885X = xyes)
if test "x$HW_CEM_DT_885X" = "xyes"; then
	AC_DEFINE(HAVE_HW_CEM_DT_885X, 1, [CEM DT-885x support])
fi

AM_CONDITIONAL(HW_CHRONOVU_LA8, test x$HW_CHRONOVU_LA8 = xyes)
if test "x$HW_CHRONOVU_LA8" = "xyes"; then
	AC_DEFINE(HAVE_HW_CHRONOVU_LA8, 1, [ChronoVu LA8 support])
fi

AM_CONDITIONAL(HW_COLEAD_SLM, test x$HW_COLEAD_SLM = xyes)
if test "x$HW_COLEAD_SLM" = "xyes"; then
	AC_DEFINE(HAVE_HW_COLEAD_SLM, 1, [Colead SLM support])
fi

AM_CONDITIONAL(HW_DEMO, test x$HW_DEMO = xyes)
if test "x$HW_DEMO" = "xyes"; then
	AC_DEFINE(HAVE_HW_DEMO, 1, [Demo driver support])
fi

AM_CONDITIONAL(HW_FLUKE_DMM, test x$HW_FLUKE_DMM = xyes)
if test "x$HW_FLUKE_DMM" = "xyes"; then
	AC_DEFINE(HAVE_HW_FLUKE_DMM, 1, [Fluke DMM support])
fi

AM_CONDITIONAL(HW_FX2LAFW, test x$HW_FX2LAFW = xyes)
if test "x$HW_FX2LAFW" = "xyes"; then
	AC_DEFINE(HAVE_HW_FX2LAFW, 1, [fx2lafw support])
fi

AM_CONDITIONAL(HW_HANTEK_DSO, test x$HW_HANTEK_DSO = xyes)
if test "x$HW_HANTEK_DSO" = "xyes"; then
	AC_DEFINE(HAVE_HW_HANTEK_DSO, 1, [Hantek DSO support])
fi

AM_CONDITIONAL(HW_IKALOGIC_SCANALOGIC2, test x$HW_IKALOGIC_SCANALOGIC2 = xyes)
if test "x$HW_IKALOGIC_SCANALOGIC2" = "xyes"; then
	AC_DEFINE(HAVE_HW_IKALOGIC_SCANALOGIC2, 1, [IKALOGIC Scanalogic2 support])
fi

AM_CONDITIONAL(HW_LASCAR_EL_USB, test x$HW_LASCAR_EL_USB = xyes)
if test "x$HW_LASCAR_EL_USB" = "xyes"; then
	AC_DEFINE(HAVE_HW_LASCAR_EL_USB, 1, [Lascar EL-USB support])
fi

AM_CONDITIONAL(HW_LINK_MSO19, test x$HW_LINK_MSO19 = xyes)
if test "x$HW_LINK_MSO19" = "xyes"; then
	AC_DEFINE(HAVE_HW_LINK_MSO19, 1, [Link Instruments MSO-19 support])
fi

AM_CONDITIONAL(HW_MIC_985XX, test x$HW_MIC_985XX = xyes)
if test "x$HW_MIC_985XX" = "xyes"; then
	AC_DEFINE(HAVE_HW_MIC_985XX, 1, [MIC 985xx support])
fi

AM_CONDITIONAL(HW_NEXUS_OSCIPRIME, test x$HW_NEXUS_OSCIPRIME = xyes)
if test "x$HW_NEXUS_OSCIPRIME" = "xyes"; then
	AC_DEFINE(HAVE_HW_NEXUS_OSCIPRIME, 1, [Nexus Osciprime support])
fi

AM_CONDITIONAL(HW_OLS, test x$HW_OLS = xyes)
if test "x$HW_OLS" = "xyes"; then
	AC_DEFINE(HAVE_HW_OLS, 1, [OpenBench Logic Sniffer (OLS) support])
fi

AM_CONDITIONAL(HW_RIGOL_DS1XX2, test x$HW_RIGOL_DS1XX2 = xyes)
if test "x$HW_RIGOL_DS1XX2" = "xyes"; then
	AC_DEFINE(HAVE_HW_RIGOL_DS1XX2, 1, [Rigol DS1xx2 support])
fi

AM_CONDITIONAL(HW_SERIAL_DMM, test x$HW_SERIAL_DMM = xyes)
if test "x$HW_SERIAL_DMM" = "xyes"; then
	AC_DEFINE(HAVE_HW_SERIAL_DMM, 1, [Serial DMM support])
fi

AM_CONDITIONAL(HW_TONDAJ_SL_814, test x$HW_TONDAJ_SL_814 = xyes)
if test "x$HW_TONDAJ_SL_814" = "xyes"; then
	AC_DEFINE(HAVE_HW_TONDAJ_SL_814, 1, [Tondaj SL-814 support])
fi

AM_CONDITIONAL(HW_UNI_T_DMM, test x$HW_UNI_T_DMM = xyes)
if test "x$HW_UNI_T_DMM" = "xyes"; then
	AC_DEFINE(HAVE_HW_UNI_T_DMM, 1, [UNI-T DMM support])
fi

AM_CONDITIONAL(HW_VICTOR_DMM, test x$HW_VICTOR_DMM = xyes)
if test "x$HW_VICTOR_DMM" = "xyes"; then
	AC_DEFINE(HAVE_HW_VICTOR_DMM, 1, [Victor DMM support])
fi

AM_CONDITIONAL(HW_ZEROPLUS_LOGIC_CUBE, test x$HW_ZEROPLUS_LOGIC_CUBE = xyes)
if test "x$HW_ZEROPLUS_LOGIC_CUBE" = "xyes"; then
	AC_DEFINE(HAVE_HW_ZEROPLUS_LOGIC_CUBE, 1, [ZEROPLUS Logic Cube support])
fi

# Checks for header files.
# These are already checked: inttypes.h stdint.h stdlib.h string.h unistd.h.
AC_CHECK_HEADERS([fcntl.h sys/time.h termios.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_INT8_T
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_UINT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_SIZE_T

# Checks for library functions.
AC_CHECK_FUNCS([gettimeofday memset strchr strcspn strdup strerror strncasecmp strstr strtol strtoul strtoull])

AC_SUBST(FIRMWARE_DIR, "$datadir/sigrok-firmware")
AC_SUBST(MAKEFLAGS, '--no-print-directory')
AC_SUBST(AM_LIBTOOLFLAGS, '--silent')

SR_PACKAGE_VERSION_MAJOR=sr_package_version_major
SR_PACKAGE_VERSION_MINOR=sr_package_version_minor
SR_PACKAGE_VERSION_MICRO=sr_package_version_micro
SR_PACKAGE_VERSION=sr_package_version

AC_SUBST(SR_PACKAGE_VERSION_MAJOR)
AC_SUBST(SR_PACKAGE_VERSION_MINOR)
AC_SUBST(SR_PACKAGE_VERSION_MICRO)
AC_SUBST(SR_PACKAGE_VERSION)

AC_CONFIG_FILES([Makefile version.h hardware/Makefile
		 hardware/agilent-dmm/Makefile
		 hardware/alsa/Makefile
		 hardware/asix-sigma/Makefile
		 hardware/brymen-dmm/Makefile
		 hardware/cem-dt-885x/Makefile
		 hardware/chronovu-la8/Makefile
		 hardware/colead-slm/Makefile
		 hardware/common/Makefile
		 hardware/ikalogic-scanalogic2/Makefile
		 hardware/lascar-el-usb/Makefile
		 hardware/mic-985xx/Makefile
		 hardware/nexus-osciprime/Makefile
		 hardware/rigol-ds1xx2/Makefile
		 hardware/tondaj-sl-814/Makefile
		 hardware/victor-dmm/Makefile
		 hardware/common/dmm/Makefile
		 hardware/demo/Makefile
		 hardware/fluke-dmm/Makefile
		 hardware/fx2lafw/Makefile
		 hardware/hantek-dso/Makefile
		 hardware/link-mso19/Makefile
		 hardware/openbench-logic-sniffer/Makefile
		 hardware/serial-dmm/Makefile
		 hardware/uni-t-dmm/Makefile
		 hardware/zeroplus-logic-cube/Makefile
		 input/Makefile
		 output/Makefile
		 output/text/Makefile
		 libsigrok.pc
		 contrib/Makefile
		 tests/Makefile
		])

AC_OUTPUT

echo
echo "libsigrok configuration summary:"
echo
echo "  - Package version (major.minor.micro):    $SR_PACKAGE_VERSION"
echo "  - Library version (current:revision:age): $SR_LIB_VERSION"
echo "  - Prefix: $prefix"
echo "  - Building on: $build"
echo "  - Building for: $host"
echo
echo "Detected libraries:"
echo

# Note: This only works for libs with pkg-config integration.
for lib in "glib-2.0 >= 2.32.0" "libzip >= 0.8" "libusb-1.0 >= 1.0.9" "libftdi >= 0.16" "libudev >= 151" "alsa >= 1.0" "check >= 0.9.4"; do
	if `$PKG_CONFIG --exists $lib`; then
		ver=`$PKG_CONFIG --modversion $lib`
		answer="yes ($ver)"
	else
		answer="no"
	fi
	echo "  - $lib: $answer"
done

echo -e "\nEnabled hardware drivers:\n"
echo "  - agilent-dmm..................... $HW_AGILENT_DMM"
echo "  - alsa............................ $HW_ALSA"
echo "  - asix-sigma...................... $HW_ASIX_SIGMA"
echo "  - brymen-dmm...................... $HW_BRYMEN_DMM"
echo "  - cem-dt-885x..................... $HW_CEM_DT_885X"
echo "  - chronovu-la8.................... $HW_CHRONOVU_LA8"
echo "  - colead-slm...................... $HW_COLEAD_SLM"
echo "  - demo............................ $HW_DEMO"
echo "  - fluke-dmm....................... $HW_FLUKE_DMM"
echo "  - fx2lafw......................... $HW_FX2LAFW"
echo "  - hantek-dso...................... $HW_HANTEK_DSO"
echo "  - ikalogic-scanalogic2............ $HW_IKALOGIC_SCANALOGIC2"
echo "  - lascar-el-usb................... $HW_LASCAR_EL_USB"
echo "  - link-mso19...................... $HW_LINK_MSO19"
echo "  - mic-985xx....................... $HW_MIC_985XX"
echo "  - nexus-osciprime................. $HW_NEXUS_OSCIPRIME"
echo "  - openbench-logic-sniffer......... $HW_OLS"
echo "  - rigol-ds1xx2.................... $HW_RIGOL_DS1XX2"
echo "  - serial-dmm...................... $HW_SERIAL_DMM"
echo "  - tondaj-sl-814................... $HW_TONDAJ_SL_814"
echo "  - uni-t-dmm....................... $HW_UNI_T_DMM"
echo "  - victor-dmm...................... $HW_VICTOR_DMM"
echo "  - zeroplus-logic-cube............. $HW_ZEROPLUS_LOGIC_CUBE"
echo

