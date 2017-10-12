configuration NbrDiscoveryAppC{
}implementation{
components NbrDiscoveryP as app,
           MainC, 
           new TimerMilliC() as CkTimer,
           new TimerMilliC() as RaTimer,
           PWMacSenderC,
           CC2420CsmaC,
           CC2420PacketC,
           LocalTimeMilliC as timeC,
           PWMacWakeupTimeComputeC,
           PWMacBeaconSenderC,
           NeigListC,
           NbrDiscoveryP,
           ActiveMessageC,
           CC2420ReceiveC,
           PWMacSplitControlC,
           PWMacListenerC,
           PWMacInitC,
           CkndManagerP,
           SerialPrintfC;
            
           app.Boot -> MainC;
           app.checkTimer -> CkTimer;
           app.radioTimer -> RaTimer;

           app.sendImmediate -> PWMacSenderC;
           app.ListenBeacon -> ActiveMessageC.AMSend[6];
           app.NotifyData -> PWMacListenerC;//这里不能接收csma的数据
           app.CC2420PacketBody -> CC2420PacketC;
           app.localtime -> timeC;
           app.pseudoWakeupTimeCompute -> 
                  PWMacWakeupTimeComputeC;
           app.beaconAppend -> PWMacBeaconSenderC;
           app.NbrManager -> NeigListC.neigList;
           app.CkndManager-> CkndManagerP;
           app.whetherInDC-> PWMacInitC;
           app.Packet -> ActiveMessageC;
           app.PWMacListener->PWMacListenerC;            

           app.receiveTime -> CC2420ReceiveC;
           app.radioConfig -> PWMacSplitControlC;
}
