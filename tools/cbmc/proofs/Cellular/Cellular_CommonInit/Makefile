H_OBJS_EXCEPT_HARNESS = $(FREERTOS)/vendors/aws/modules/3gpp/ports/cellular/src/cellular_common_api.goto $(FREERTOS)/vendors/aws/modules/3gpp/ports/cellular/src/cellular_common.goto 
H_ENTRY = Cellular_CommonInit
H_FAILURE_BOUND = 2
H_CBMCFLAGS = --unwind 4 --unwindset libClose.0:13 --nondet-static --flush
H_OBJS = $(ENTRY)_harness.goto $(FREERTOS)/vendors/aws/modules/3gpp/ports/cellular/src/cellular_common_api.goto $(FREERTOS)/vendors/aws/modules/3gpp/ports/cellular/src/cellular_common.goto 
H_DEF = -DCBMC_GETNETWORKBUFFER_FAILURE_BOUND=2
H_EXPECTED = SUCCESSFUL

include ../../Makefile.common