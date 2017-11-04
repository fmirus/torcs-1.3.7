##############################################################################
#
#    file                 : Makefile
#    created              : Sat Nov 4 20:28:35 CET 2017
#    copyright            : (C) 2002 Jonas Natzer
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

# I need those environment variables
# export TORCS_BASE=~/torcs-1.3.7
# export MAKE_DEFAULT=${TORCS_BASE}/Make-default.mk

ROBOT       = pid_driver
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml 155-DTM.rgb logo.rgb
SHIPSUBDIRS =

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-pid_driver_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-pid_driver_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
