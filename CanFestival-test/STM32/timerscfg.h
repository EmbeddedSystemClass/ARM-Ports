/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
STM32 Port: Ken Sarkies, based on AVR port by Andreas GLAUSER and Peter CHRISTEN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __TIMERSCFG_H__
#define __TIMERSCFG_H__

/* TIMEVAL is set to 32 bits but as the timers are 16 bits and the
maximum count is less than 0xFFFF, it could be set to 16 bits */

#define TIMEVAL UNS32

/* The timer of the STM32 series counts to s preset period then restarts
if it has been placed in upcounting mode. This allows 0.5 seconds cycle
time for an 8us clock period. */

#define TIMEVAL_MAX 62500

// The timer is incrementing every 8 us.
#define MS_TO_TIMEVAL(ms) ((ms) * 125)
#define US_TO_TIMEVAL(us) ((us)>>3)

#endif
