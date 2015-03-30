# OpenEmbedded bitbake recipe for installing ImageMagick with PerlMagick as
# a dynamically-loaded version.  The build process is a two-stage mess-up
# where first ImageMagick is built and installed without Perl support then
# reconfigured with Perl support before PerlMagick is built.

MAINTAINER = "Ole Wolf <ole@naturloven.dk>"
SUMMARY = " ImageMagick with PerlMagick"
DESCRIPTION = "${SUMMARY}"
PROVIDES = "perlmagick imagemagick"
LICENSE = "ImageMagick"
LIC_FILES_CHKSUM = "file://LICENSE;md5=0887b670be7ef0c3cb37092b64d57514"

DEPENDS += " perl-native "
RDEPENDS_${PN} += " \
	perl \
	perl-module-carp \
	perl-module-parent \
	perl-module-dynaloader \
	perl-module-autoloader \
"

SRC_URI = "svn://subversion.imagemagick.org/subversion/ImageMagick;module=trunk;protocol=https"
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/trunk"

inherit autotools binconfig pkgconfig perlnative

# Define ImageMagick files.
PACKAGECONFIG ??= ""
PACKAGECONFIG[jp2] = "--with-jp2,--without-jp2,jasper"

#${libdir}/ImageMagick-${PV}/modules-Q16/*/*.so \
#                ${libdir}/ImageMagick-${PV}/modules-Q16/*/*.la \
#                ${libdir}/ImageMagick-${PV}/modules-Q16/filters \
#                ${libdir}/ImageMagick-${PV}/modules-Q16/coders \
#                ${libdir}/ImageMagick-${PV}/config-Q16

FILES_${PN}-dbg =+ " \
	usr/lib/perl/5.14.3/auto/Image/Magick/.debug \
	usr/lib/perl/5.14.3/auto/Image/Magick/Q16HDRI/.debug/Q16HDRI.so \
	"

FILES_${PN} += " \
	/usr/lib/perl \
	/usr/share \
	/usr/man/man3 \
	/usr/lib/ImageMagick-${PV} \
"

BBCLASSEXTEND = "native"

LEAD_SONAME = "libMagickCore.so.*"

OECONF_OPTIONS = "--without-x --disable-openmp --without-xml --disable-opencl --with-quantum-depth=16 --with-sysroot=${PKG_CONFIG_SYSROOT_DIR} --prefix=${PKG_CONFIG_SYSROOT_DIR}${prefix}"

EXTRA_OECONF = "${OECONF_OPTIONS}"

do_configure_prepend() {
	# Work around libperl.so issue in the Perl recipe.
	if [ ! -h "${STAGING_LIBDIR}/libperl.so" ]; then
		ln -sf libperl.so.5 ${STAGING_LIBDIR}/libperl.so 
	fi
}

do_configure() {
	oe_runconf --without-perl
}

do_install_append() {
	# Build and install PerlMagick after the installation of ImageMagick
	# is complete.
	bbnote ImageMagick built and installed, now building PerlMagick

	PERL="${STAGING_BINDIR_NATIVE}/perl-native/perl"
	PERL_VERSION="$( perl -v 2>/dev/null | sed -n 's/This is perl.*v[a-z ]*\([0-9]\.[0-9][0-9.]*\).*$/\1/p' )"

	PERLMAGICK_INSTALL_DIR="${WORKDIR}/image"
#	PERLMAGICK_INSTALL_DIR="${STAGING_LIBDIR}/perl5/${PERL_VERSION}"

	oe_runconf --with-perl
	oe_runmake perl-sources
	cd ${S}/PerlMagick
	$PERL Makefile.PL PERL="${PERL}" FULLPERL="${PERL}" PREFIX="${PERLMAGICK_INSTALL_DIR}${prefix}" CC="${CC}"
	make PREFIX="${PERLMAGICK_INSTALL_DIR}${prefix}" CC="${CC}"

	bbnote "Running: make PREFIX=\"${PERLMAGICK_INSTALL_DIR}${prefix}\" CC=\"${CC}\" install"

	# Hack to install into the target perl dir.
	mkdir -p ${PERLMAGICK_INSTALL_DIR}/${libdir}/perl
	mkdir ${PERLMAGICK_INSTALL_DIR}${libdir}/perl-native
	ln -s ../perl ${PERLMAGICK_INSTALL_DIR}${libdir}/perl-native/perl

	make PREFIX="${PERLMAGICK_INSTALL_DIR}${prefix}" CC="${CC}" install

	# Unhack.
	rm ${PERLMAGICK_INSTALL_DIR}${libdir}/perl-native/perl
	rmdir ${PERLMAGICK_INSTALL_DIR}${libdir}/perl-native
}

