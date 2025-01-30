#include "Arduino.h"

int main(void)
{
	pinMode(PB23, OUTPUT);
	digitalWrite(PB23, HIGH);

    while(1)
    {
    	for(int i = 0; i < 100000L; i++);
    	digitalWrite(PB23, LOW);
    	for(int i = 0; i < 100000L; i++);
		digitalWrite(PB23, HIGH);
    }
}
