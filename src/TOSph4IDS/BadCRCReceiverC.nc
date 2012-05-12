configuration BadCRCReceiverC {
	provides interface ReceiveBadCRC;

} implementation {
	components CC2420ReceiveC;
        ReceiveBadCRC = CC2420ReceiveC.ReceiveBadCRC;
}
