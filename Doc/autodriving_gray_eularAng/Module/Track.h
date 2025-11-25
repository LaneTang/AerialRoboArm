#ifndef __TRACK_H
#define __TRACK_H

extern uint8_t tubes[];

void follow_blackline_singleChannel(uint8_t meetTubes);
void follow_blackline_multiChannel(uint8_t meetTubes);
uint8_t meet_black(void);

#endif
