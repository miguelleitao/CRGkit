
# Makefile for CRGkit

VERSION=`cat version`

# Default install path.
# Final users can override INSTALL_PATH using an environment variable.
INSTALL_PATH?=/usr/local

# Source files
SRCFILES:=*.c *.C

# Shell Scripts
SCRIPTS:=

# Programmes
PROGS:=crg2pts crg2obj crg2osg

# Files to store in archive
FILES:=${SRCFILES} Makefile ${SCRIPTS} version README* .gitignore .gitlab-ci.yml

LINMATH_INC=-I .

CFLAGS=-Wall -O2

all: OpenCRG ${PROGS}

OpenCRG/makefile:
	git submodule update --init --recursive

OpenCRG: OpenCRG/lib

OpenCRG/lib: OpenCRG/makefile
	make -C $@

crg2pts.o: crg2pts.c 
	gcc -c -Wall crg2pts.c -I OpenCRG/inc

crg2pts: crg2pts.o OpenCRG/lib/libOpenCRG.a
	gcc -o $@ $^ -lm

crg2obj.o: crg2obj.c
	gcc -c -Wall crg2obj.c -I OpenCRG/inc

crg2obj: crg2obj.o OpenCRG/lib/libOpenCRG.a
	gcc -o $@ $^ -lm

crg2osg.o: crg2osg.C
	g++ -c -Wall -I OpenCRG/inc $<

crg2osg: crg2osg.o
	g++ -o $@ $^ -l osg -l osgViewer -l osgGA -l osgDB -L OpenCRG/lib -l OpenCRG

OpenCRG/lib/libOpenCRG.a:
	make -C OpenCRG

.PHONY:clean
clean:
	rm ${PROGS} version.h

inc_version: version
	@perl -pe 's/^((\d+\.)*)(\d+)(.*)$$/$$1.($$3+1).$$4/e' < $< >$<.new 
	@mv version.new version
	@echo "new version:" `cat $<`

version.h: version
	@echo "// $@" >$@
	@echo "// This file is automatically generated from file '$<'." >>$@
	@echo "// Use 'make inc_version' to increment file '$<'" >>$@
	@echo "// and 'make $@' update this file." >>$@
	@echo >>$@
	@echo -n "#define VERSION \"${VERSION}\"" >>$@
	@echo >>$@
	@echo >>$@

push: 
	git add ${FILES}
	git commit -m "auto update"
	git push

.PHONY: install
install: ${PROGS} 
	mkdir -p ${INSTALL_PATH}/bin
	cp -p -u ${PROGS} ${SCRIPTS} ${INSTALL_PATH}/bin/

.PHONY: uninstall
uninstall:
	@$(foreach file,$(PROGS) $(SCRIPTS),rm -f ${INSTALL_PATH}/bin/$(file);)
