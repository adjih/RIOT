
# "Porting" RIOT to the M3 on "A8 Open Nodes"

See [Differences between "M3 Open Nodes" and "M3 from A8 Open Nodes"](https://github.com/adjih/exp-iotlab/blob/master/doc/README-diff-M3-M3ofA8.md)

The main issue of supporting the M3 on "Open A8 Nodes" is a more or less
clean integration in RIOT build system.

A requirement is to be able to build code for both "Open M3 Nodes" and 
"M3 from Open A8 Nodes" in the same directory, and obtain two different files.

* Simplest way (tried here) is to have ```BOARD=iot-lab_M3ofA8``` instead of ```BOARD-iot-lab_M3```
  The problem is that the ```BOARD``` is used for locating files (in paths
  in the Makefile), so it is necessary to do some redirections here and there.

# Some changes for iot-lab_M3ofA8

### Minimal changes to compile hello-world with BOARD=iot-lab_M3ofA8

The ```hello-world``` program compiled for a "Open M3 Node" will work on a 
"M3 of a Open A8 Node" (no hardware difference impact on this program, the hello-world binary for ```BOARD=iot-lab_M3``` runs on M3 of Open A8).

* The file ```board/iot-lab_M3ofA8/Makefile.include``` is the entry point for 
   the board. It is created and just includes the ```Makefile.include``` 
   from ```iot-lab_M3``` (could be symbolic link?).


* The file ```board/iot-lab_M3/Makefile.include```
  The file uses ```$(BOARD)``` in several places for paths, etc.
  This has been replaced by ```$(BOARDCLASS)``` which is defined to be ```iot-lab_M3```.

* ```includer_board.c```. The base file ```board.c``` is found 
by ```$(RIOTBASE)/Makefile.base```, through list of source files
identified automatically with:
```
ifeq ($(strip $(SRC)),)
    SRC := $(wildcard *.c)
endif
```
A solution is the creation of a ```includer_board.c``` which just includes
```../iot-lab_M3/board.c```; there are other solutions (change Makefiles, symbolic link, ...)


### Thinking of dynamically detecting boards

This is just a few quick thoughts.

Dynamically detecting between "Open M3" and "M3 from Open A8" could be
done in several ways. One can imagine:

* using I2C: send a "I2C START" to the adress of a device and checking whether
  it answers or not. The "Open M3" has a light sensor and a pressure sensor
  which the "M3 from Open A8" does not have.

* using I2C to configure a device so that it sends an interrupt: 
  the GPIO of the gyroscope and of the accelerometer are different, so 
  this could be tested. This is complicated, but cleaner because there
  would be an response.

* maybe sending SPI commands ? the SPI of the AT86RF231 (radio) is changed
  on the M3ofA8 

* ...

However all this would require device initialisation in controlled order.

Alternate way is:
* Just maintaining a table of hardware identifiers of "M3 from Open A8"
  (possibly a bloom filter) and looking up the identifier of the node.
  Probably having a tables of both node types would be more interesting
  since when a new node is introduced it would be detected to be "unknown".

In anycase, dynamic detection would make many constants become variables.

# Comparing

```
cd examples/hello-world
rm -rf bin
BOARD=iot-lab_M3 remake -x 2>&1 > log.M3
BOARD=iot-lab_M3ofA8 remake -x 2>&1 > log.M3ofA8 || true
kdiff3 log.M3 log.M3ofA8
```

or 
```make -f test-changes.mk WHERE=examples diff-hello-world```
```make -f test-changes.mk WHERE=examples send-hello-world```

# Tracking RIOT Makefile inclusions (+parts)

The [partial] inclusion flow of the Makefile(s) 
(plus some useful information from
the Makefile[s]) is found to be as follows (from manual file reading).


<pre>
---------------------------------------------------------------------------
### Makefile.include

# mandatory includes!
include $(RIOTBASE)/Makefile.modules
include $(RIOTBOARD)/$(BOARD)/Makefile.include
include $(RIOTCPU)/$(CPU)/Makefile.include
include $(RIOTBASE)/Makefile.dep

# if you want to publish the board into the sources as an uppercase #define
BOARDDEF := $(shell echo $(BOARD) | tr 'a-z' 'A-Z' | tr '-' '_')
CPUDEF := $(shell echo $(CPU) | tr 'a-z' 'A-Z' | tr '-' '_')
MCUDEF := $(shell echo $(MCU) | tr 'a-z' 'A-Z' | tr '-' '_')
CFLAGS += -DBOARD_$(BOARDDEF)='"$(BOARD)"' -DRIOT_BOARD=BOARD_$(BOARDDEF)
CFLAGS += -DCPU_$(CPUDEF)='"$(CPU)"' -DRIOT_CPU=CPU_$(CPUDEF)
CFLAGS += -DMCU_$(MCUDEF)='"$(MCU)"' -DRIOT_MCU=MCU_$(MCUDEF)

---------------------------------------------------------------------------
### Makefile.modules
include $(RIOTBASE)/Makefile.pseudomodules
include $(RIOTBASE)/Makefile.defaultmodules


---------------------------------------------------------------------------
### Makefile.pseudomodules
DEFAULT_MODULE += cpu core sys

DEFAULT_MODULE += auto_init

---------------------------------------------------------------------------
### Makefile.defaultmodules
DEFAULT_MODULE += cpu core sys

DEFAULT_MODULE += auto_init

---------------------------------------------------------------------------
### board/iot-lab_M3/Makefile.include

export CPU = stm32f1
export CPU_MODEL = stm32f103re

# export board specific includes to the global includes-listing
export INCLUDES += -I$(RIOTBOARD)/$(BOARD)/include/ -I$(RIOTBASE)/drivers/at86rf231/include -I$(RIOTBASE)/sys/net/include

include $(RIOTBOARD)/$(BOARD)/Makefile.dep

---------------------------------------------------------------------------
### board/iot-lab_M3/Makefile.dep

ifneq (,$(filter defaulttransceiver,$(USEMODULE)))
    USEMODULE += at86rf231
    ifeq (,$(filter netdev_base,$(USEMODULE)))
        USEMODULE += transceiver
    endif
endif

---------------------------------------------------------------------------
### cpu/stm32f1/Makefile.include

# tell the build system that the CPU depends on the Cortex-M common files
export USEMODULE += cortex-m3_common

# define path to cortex-m common module, which is needed for this CPU
export CORTEXM_COMMON = $(RIOTCPU)/cortex-m3_common/

# export the peripheral drivers to be linked into the final binary
export USEMODULE += periph

# CPU depends on the cortex-m common module, so include it
include $(CORTEXM_COMMON)Makefile.include

---------------------------------------------------------------------------
### cpu/cortex-m3_common/Makefile.include 

# include module specific includes
export INCLUDES += -I$(RIOTCPU)/cortex-m3_common/include

---------------------------------------------------------------------------
### Makefile.dep

ifneq (,$(filter at86rf231,$(USEMODULE)))
	USEMODULE += netdev_802154
	USEMODULE += ieee802154
endif

ifneq (,$(filter netdev_802154,$(USEMODULE)))
	USEMODULE += netdev_base
endif

ifneq (,$(filter defaulttransceiver,$(USEMODULE)))
	FEATURES_REQUIRED += transceiver
endif

---------------------------------------------------------------------------
</pre>

