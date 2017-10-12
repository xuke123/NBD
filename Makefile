COMPONENT=NbrDiscoveryAppC

PFLAGS += -I/opt/tinyos-2.1.2/apps/my_own_mac_coding/PW_MAC_4
PFLAGS += -I/opt/tinyos-2.1.2/apps/my_own_mac_coding/PW_MAC_4/interface
PFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DPWMAC

include $(MAKERULES)
