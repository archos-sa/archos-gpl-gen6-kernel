# File: app/Makefile
#
# (c) Copyright � 2003-2006, Marvell International Ltd. 
#
# This software file (the "File") is distributed by Marvell International 
# Ltd. under the terms of the GNU General Public License Version 2, June 1991 
# (the "License").  You may use, redistribute and/or modify this File in 
# accordance with the terms and conditions of the License, a copy of which 
# is available along with the File in the gpl.txt file or by writing to 
# the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
# 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
#
# THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
# ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
# this warranty disclaimer.
#

# Path to the top directory of the wlan distribution
PATH_TO_TOP = ../..

# Determine how we should copy things to the install directory
ABSPATH := $(filter /%, $(INSTALLDIR))
RELPATH := $(filter-out /%, $(INSTALLDIR))
INSTALLPATH := $(ABSPATH)
ifeq ($(strip $(INSTALLPATH)),)
INSTALLPATH := $(PATH_TO_TOP)/$(RELPATH)
endif

# Override CFLAGS for application sources, remove __ kernel namespace defines
CFLAGS := $(filter-out -D__%, $(CFLAGS))


# Add the wlan driver directory to the preprocessor include path
CFLAGS +=  -I$(PATH_TO_TOP)/wlan


#
# List of application executables to create
#
TARGETS := wlanconfig

#
# Make target rules
#

# All rule compiles list of TARGETS using builtin program target from src rule
all : $(TARGETS)

# Install rule; INSTALLPATH calculated above from passed INSTALLDIR variable
$(INSTALLPATH)/% : ./%
	@cp -f $< $@

# Map build and install invocation to the install rule for each TARGET file
build install: $(addprefix $(INSTALLPATH)/, $(TARGETS))

clean:
	@rm -f $(TARGETS)

distclean: clean
	@rm -f *~ core