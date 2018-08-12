##############################################################################
#
#    file                 : Makefile
#    created              : Mo 8. Jan 00:39:15 CET 2018
#    copyright            : (C) 2002 Jonas Natzer, Michael Heinrich
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = hemic
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp TrackData.cpp Trajectory.cpp optimal_line.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml 155-DTM.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-hemic_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-hemic_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
