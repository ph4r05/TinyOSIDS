
#include "CtpForwardingEngine.h"
/**
 * Interface for snooping SubSend.SendDone() calls in ForwardingEngine.
 * It is used to measure Carrier Sensing Time for sent packets by Forwarding engine.
 */
interface CtpForwardingSubSendDone {
	/**
	 * @param msg      message sent
	 * @param error    sending error
	 * @param qe       pointer to queue element corresponding to message being sent
	 * @param acked    was ACK received? Is relevant only if error==SUCCESS 
	 * @return void
	 */
    event void CTPSubSendDone(message_t *msg, error_t error, fe_queue_entry_t ONE * qe, am_addr_t dest, bool acked);
}

