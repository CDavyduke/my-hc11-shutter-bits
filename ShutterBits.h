/*  Filename:       ShutterBits.h
    Author:         Corey Davyduke
    Created:        2012-04-30
    Modified:       2012-04-30
    Description:    This is the header file for the Shutter Bits project.
*/

#ifndef _SHUTTERBITS_H
#define _SHUTTERBITS_H

#include <sys/param.h>
#include <sys/interrupts.h>
#include <sys/sio.h>
#include <sys/locks.h>
#include <stdarg.h>

extern void timer_interrupt (void) __attribute__((interrupt));

inline static void print (const char* msg)
{
  serial_print (msg);
}

#endif
