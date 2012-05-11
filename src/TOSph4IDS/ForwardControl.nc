/** 
 * AMTap interface. 
 * 
 * This interface is provided by ForgedActiveMessageP and used by Statistics Manager.
 * Using this interface the Statistics Manager hears all processed messages 
 */

interface ForwardControl {
	command void postTask();
	command void flushQueue();
	
}