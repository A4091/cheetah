PROJECT=A4092
PART=XC95144XL-10-TQ144
WORKDIR=tmp
SERIAL=0
DEFINES=makedefines SERIAL=32'h${SERIAL}

CABLE=usb21

#CPLDFITFLAGS=-loc on -slew slow -init low -terminate keeper  -unused ground -power std  -optimize speed -keepio
CPLDFITFLAGS=-loc on -slew slow -init low -terminate keeper  -unused float -power std  -optimize speed -keepio

.PHONY:	all clean timing

# default is DIP switch, not FLASH
#USE_DIP_SWITCH ?= 1
# default is FLASH
USE_DIP_SWITCH ?= 0

ifeq ($(USE_DIP_SWITCH),1)
  DEFINES += USE_DIP_SWITCH
endif

all:	$(PROJECT).jed timing

$(PROJECT).prj: *.v
	-$(shell rm -f $@)
	-$(foreach source,$^,$(shell echo verilog work $(source) >> $@))

$(WORKDIR)/$(PROJECT).ngc:	*.v $(PROJECT).prj
	-@mkdir $(WORKDIR)
	@sed -r "s#^(-ofn).*#\1 ..\/$@#g;s#^(-ifn).*#\1 ../$(PROJECT).prj#g;s#^(-define).*#\1 {$(DEFINES)}#g" template.xst > $@.xst
	cd $(WORKDIR) && xst -ifn ../$@.xst -ofn $(PROJECT)-xst.log

$(WORKDIR)/%.ngd:	$(WORKDIR)/%.ngc $(PROJECT).ucf
	cd $(WORKDIR) && ngdbuild -p $(PART) -uc ../$(PROJECT).ucf ../$< ../$@

$(WORKDIR)/%.rpt $(WORKDIR)/%.vm6:	$(WORKDIR)/%.ngd $(PROJECT).ucf
	cd $(WORKDIR) && cpldfit $(CPLDFITFLAGS) -p $(PART) ../$< | egrep -v "^CS: block|^$$"
	cp $(WORKDIR)/$(PROJECT).rpt .

%.jed:	$(WORKDIR)/%.vm6
	hprep6 -i $<
	cp $@ ../Binary/

fit:	$(WORKDIR)/$(PROJECT).vm6


%.tim:	$(WORKDIR)/%.vm6
	cd $(WORKDIR) && taengine -l ../$@ -f $(PROJECT)


timing:	$(PROJECT).tim

clean:
	-rm -rvf tmp
	-rm *.jed
	-rm *.prj

flash:	$(PROJECT).jed
	@echo "setMode -bs\n"\
	"setCable -p $(CABLE)\n"\
	"identify\n"\
	"assignfile -p 1 -file $<\n"\
	"erase -p 1\n"\
	"program -p 1\n"\
	"verify -p 1" | LD_PRELOAD=/opt/Xilinx/usb-driver/libusb-driver.so impact -batch
