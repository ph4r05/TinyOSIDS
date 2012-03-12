


#include "Reset.h"
module ResetP
{
	provides interface Reset;
	uses interface Timer<TMilli> as ResetTimer;
}
implementation
{
	command void Reset.reset(){
		call ResetTimer.startOneShot(300);
	}

	event void ResetTimer.fired(){
		resetMote();
	}
}