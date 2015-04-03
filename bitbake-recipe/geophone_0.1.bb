# OpenEmbedded bitbake recipe for installing the geodata logger into an
# embedded Linux system.

MAINTAINER = "Ole Wolf <ole@naturloven.dk>"
HOMEPAGE = "https://github.com/olewolf/geophone"
SUMMARY = "Geophone data logger and visualizer"
DESCRIPTION = "${SUMMARY}"
PROVIDES = "geophone-logger"

PR = "8"

SRC_URI = " \
	git://github.com/olewolf/geophone.git;protocol=https \
	file://geophone-rotate-heatmap \
	file://led-indicator \
	file://geophone-rotate-heatmap.cron \
	file://geophone.logrotate \
	file://geophone.service file://geophone-service.sh file://geophone.default \
	file://www/nav-first.png \
	file://www/nav-previous.png \
	file://www/nav-next.png \
	file://www/nav-last.png \
	file://www/style.css \
	file://www/view-geophone-image.pl \
	file://geophone-cgi.conf \
	file://lighttpd.logrotate \
"
SRCREV = "${AUTOREV}"
LICENSE = "GPLv3"
LIC_FILES_CHKSUM = "file://LICENSE;md5=94a3f3bdf61243b5e5cf569fbfbbea52"
S = "${WORKDIR}/git"

WEB_ROOT = "/www/pages"

DEPENDS += " eglibc "
RDEPENDS_${PN} += " \
	logrotate \
	cronie \
	perl \
	perl-module-time-piece \
	lighttpd \
	lighttpd-module-cgi \
	liberation-fonts \
	imagemagick-perlmagick \
"

RRECOMMENDS_${PN} += " \
	ntp \
"

inherit systemd

SYSTEMD_PACKAGES = "${PN}-systemd"
SYSTEMD_SERVICE_${PN}-systemd = "geophone.service"

FILES_${PN} = " \
	/usr/bin/ \
	/usr/share/geophone/colormaps.png \
	/var/lib/geophone \
	/etc/logrotate.d/geophone \
	/etc/logrotate.d/lighttpd \
	/etc/cron.d/geophone-rotate-heatmap \
	${WEB_ROOT}/ \
	/etc/lighttpd.d/geophone-cgi.conf \
	/etc/default/geophone \
"

PACKAGES =+ "${PN}-systemd"
FILES_${PN}-systemd = "${systemd_unitdir}/system/"
RDEPENDS_${PN}-systemd = "${PN}"


do_patch () {
	# Move the colormaps.png file to the geophone shared resource dir.
	sed -i '/\$heatmap->Read( "colormaps.png" );/c\$heatmap->Read( "\/usr\/share\/geophone\/colormaps.png" );' ${S}/createheatmap.pl
	# Replace Arial with Liberation font.
	sed -i 's/truetype\/msttcorefonts\/arial.ttf/ttf\/LiberationSans-Regular.ttf/g' ${S}/createheatmap.pl
}

do_compile () {
	CFLAGS="${CFLAGS} -std=gnu99 $(pkg-config --cflags glib-2.0)"
	LIBS="$LIBS $(pkg-config --libs glib-2.0)"
	$CC $CFLAGS -o geophone-log-serial read-serial-log.c $LIBS
}

do_install () {
	install -m 0755 -d -D ${D}/usr/bin
	install -m 0755 ${S}/createheatmap.pl ${D}/usr/bin/geophone-create-heatmap
	install -m 0755 ${S}/filter-log.pl ${D}/usr/bin/geophone-filter-log
	install -m 0755 ${S}/geophone-log-serial ${D}/usr/bin/
	install -m 0755 ${WORKDIR}/geophone-rotate-heatmap ${D}/usr/bin/
	install -m 0755 ${WORKDIR}/led-indicator ${D}/usr/bin/

	install -m 0755 -d -D ${D}/var/lib/geophone

	install -m 0755 -d -D ${D}/usr/share/geophone
	install -m 0644 ${S}/colormaps.png ${D}/usr/share/geophone/

	install -m 0755 -d -D ${D}/etc/logrotate.d
	install -m 0644 ${WORKDIR}/geophone.logrotate ${D}/etc/logrotate.d/geophone

	install -m 0755 -d -D ${D}/etc/cron.d
	install -m 0644 ${WORKDIR}/geophone-rotate-heatmap.cron ${D}/etc/cron.d/geophone-rotate-heatmap

	install -m 0755 -d -D ${D}/etc/default
	install -m 0644 ${WORKDIR}/geophone.default ${D}/etc/default/geophone
	install -m 0755 ${WORKDIR}/geophone-service.sh ${D}/usr/bin/geophone-service.sh
	install -m 0755 -d -D ${D}/${systemd_unitdir}/system
	install -m 0644 ${WORKDIR}/geophone.service ${D}/${systemd_unitdir}/system/

	install -m 0755 -d -D ${WORKDIR}/geophone-cfi.conf ${D}/etc/lighttpd.d
	install -m 0644 ${WORKDIR}/geophone-cgi.conf ${D}/etc/lighttpd.d/geophone-cgi.conf
	install -m 0755 -d -D ${D}/${WEB_ROOT}
	install -m 0644 ${WORKDIR}/www/nav-first.png ${D}/${WEB_ROOT}/
	install -m 0644 ${WORKDIR}/www/nav-previous.png ${D}/${WEB_ROOT}/
	install -m 0644 ${WORKDIR}/www/nav-next.png ${D}/${WEB_ROOT}/
	install -m 0644 ${WORKDIR}/www/nav-last.png ${D}/${WEB_ROOT}/
	install -m 0644 ${WORKDIR}/www/style.css ${D}/${WEB_ROOT}/
	install -m 0755 ${WORKDIR}/www/view-geophone-image.pl ${D}/${WEB_ROOT}/
	install -m 0755 -d -D ${D}/${WEB_ROOT}/geophone
	install -m 0644 ${WORKDIR}/lighttpd.logrotate ${D}/etc/logrotate.d/lighttpd
}
