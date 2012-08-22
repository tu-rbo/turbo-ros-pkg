VISUAL_NET_DIR = $(CURDIR)
include $(shell rospack find mk)/cmake.mk

build-libs:
# Make libraries
	cd $(VISUAL_NET_DIR)/libs; $(MAKE)
	$(MAKE) clean

clean-libs:
# Clean libraries
	cd $(VISUAL_NET_DIR)/libs; $(MAKE) clean
