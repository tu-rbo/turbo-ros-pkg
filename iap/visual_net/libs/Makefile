#-----------------------------------------------------------------------------
# Top level makefile for Bundler
#
# Bundler: Structure from Motion for Unordered Photo Collections
# Version: 0.4 04/03/2010
#    http://phototour.cs.washington.edu/bundler/
#-----------------------------------------------------------------------------
# Copyright (c) 2008-2010 University of Washington and Noah Snavely
# All Rights Reserved.
#-----------------------------------------------------------------------------

default:
# Make libraries
	cd 5point; $(MAKE) 
	cd ann_1.1_char; $(MAKE) linux-g++-shared 
	cd imagelib; $(MAKE) 
	cd matrix; $(MAKE) 
	cd sba-1.5; $(MAKE) 
	cd sfm-driver; $(MAKE) 
# Auxiliary libraries
	cd minpack; $(MAKE)
	cd cblas; $(MAKE)
	cd f2c; $(MAKE)


clean:
	cd 5point; $(MAKE) clean
	cd ann_1.1_char; $(MAKE) clean
	cd imagelib; $(MAKE) clean
	cd matrix; $(MAKE) clean
	cd sba-1.5; $(MAKE) clean
	cd sfm-driver; $(MAKE) clean
	cd minpack; $(MAKE) clean
	cd cblas; $(MAKE) clean
	cd f2c; $(MAKE) clean
	rm -f lib/*.a
