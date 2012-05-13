/** 
 * Forward control interface
 * 
 * helps to control forwarding on lower level. 
 */

interface ForwardControl {
	command void postTask();
	command void flushQueue();
	
}