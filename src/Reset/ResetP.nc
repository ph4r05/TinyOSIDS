


#include "Reset.h"
module ResetP
{
	provides interface Reset;
	uses interface Timer<TMilli> as ResetTimer;
}
implementation
{
	command void Reset.reset(){
		call ResetTimer.startOneShot(100);
	}

	event void ResetTimer.fired(){
		resetMote();
	}

	command void Reset.resetAfterTimeout(uint8_t mili){
		call ResetTimer.startOneShot(mili);
	}
}