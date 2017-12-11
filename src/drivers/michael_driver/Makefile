##############################################################################
#
#    file                 : Makefile
#    created              : Do 2. Nov 08:49:38 CET 2017
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

ROBOT       = test_bot
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp optimal_line.cpp Trajectory.cpp TrackData.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml 155-DTM.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-test_bot_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-test_bot_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
