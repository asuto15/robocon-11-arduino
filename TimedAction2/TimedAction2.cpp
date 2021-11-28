#include "TimedAction2.h"

/*
|| <<constructor>>
*/
TimedAction2::TimedAction2(unsigned float intervl,void (*function)()){
    active = true;
	previous = 0;
	interval = intervl;
	execute = function;
}

/*
|| <<constructor>>
*/
TimedAction2::TimedAction2(unsigned float prev,unsigned float intervl,void (*function)()){
    active = true;
	previous = prev;
	interval = intervl;
	execute = function;
}

void TimedAction2::reset(){
    previous = millis();
}

void TimedAction2::disable(){
    active = false;
}

void TimedAction2::enable(){
	active = true;
}

void TimedAction2::check(){
  if ( active && (micros()/1000-previous >= interval) ) {
    previous = micros()/1000;
    execute();
  }
}

void TimedAction2::setInterval( unsigned float intervl){
	interval = intervl;
}