/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2008  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main library USB management task for both Host and Device mode operations. This contains the master
 *  USB_USBTask task which should be periodically run to service both host and device USB projects.
 */
 
#ifndef __USBTASK_H__
#define __USBTASK_H__

	/* Includes: */
		#include <avr/io.h>
		#include <stdbool.h>
		#include <stddef.h>
		
		#include "../../../Scheduler/Scheduler.h"
		#include "../LowLevel/LowLevel.h"
		#include "../LowLevel/HostChapter9.h"
		#include "../LowLevel/USBMode.h"
		#include "Events.h"
		#include "StdDescriptors.h"
		
	/* Enable C linkage for C++ Compilers: */
		#if defined(__cplusplus)
			extern "C" {
		#endif

	/* Public Interface - May be used in end-application: */
		/* Global Variables: */
			/** Indicates if the USB interface is currently connected to a host if in device mode, or to a
			 *  device while running in host mode.
			 *
			 *  \note This variable should be treated as read-only in the user application, and never manually
			 *        changed in value.
			 */
			extern volatile bool USB_IsConnected;

			/** Indicates if the USB interface is currently initialized but not neccesarily connected to a host
			 *  or device (i.e. if USB_Init() has been run).
			 *
			 *  \note This variable should be treated as read-only in the user application, and never manually
			 *        changed in value.
			 */
			extern volatile bool USB_IsInitialized;

			#if defined(USB_CAN_BE_HOST) || defined(__DOXYGEN__)
			/** Indicates the current host state machine state. When in host mode, this indicates the state
			 *  via one of the values of the USB_Host_States_t enum values in Host.h.
			 *
			 *  This value may be altered by the user application to implement the HOST_STATE_Addressed,
			 *  HOST_STATE_Configured, HOST_STATE_Ready and HOST_STATE_Suspended states which are not implemented
			 *  by the library.
			 *
			 *  \note This global is only present if the user application can be a USB host.
			 */
			extern volatile uint8_t USB_HostState;
			#endif

		/* Throwable Events: */
			#if defined(USB_CAN_BE_HOST) || defined(__DOXYGEN__)
				/** This module raises the USB Connect event when a USB device has been connected whilst in host
				 *  mode, but not yet enumerated.
				 *
				 *  \see Events.h for more information on this event.
				 */
				RAISES_EVENT(USB_Connect);

				/** This module raises the USB Device Attached event when in host mode, and a device is attached
				 *  to the AVR's USB interface.
				 *
				 *  \see Events.h for more information on this event.
				 */
				RAISES_EVENT(USB_DeviceAttached);

				/** This module raises the USB Device Unattached event when in host mode, and a device is removed
				 *  from the AVR's USB interface.
				 *
				 *  \see Events.h for more information on this event.
				 */
				RAISES_EVENT(USB_DeviceUnattached);
				
				/** This module raises the USB Device Enumeration Failed event when in host mode, and an
				 *  attached USB device has failed to successfully enumerated.
				 *
				 *  \see Events.h for more information on this event.
				 */
				RAISES_EVENT(USB_DeviceEnumerationFailed);

				/** This module raises the USB Device Enumeration Complete event when in host mode, and an
				 *  attached USB device has been successfully enumerated and ready to be used by the user
				 *  application.
				 *
				 *  \see Events.h for more information on this event.
				 */
				RAISES_EVENT(USB_DeviceEnumerationComplete);

				/** This module raises the USB Disconnect event when an attached USB device is removed from the USB
				 *  bus.
				 *
				 *  \see Events.h for more information on this event.
				 */
				RAISES_EVENT(USB_Disconnect);
			#endif

		/* Tasks: */
			/** This is the main USB management task. The USB driver requires that this task be executed
			 *  continuously when the USB system is active (device attached in host mode, or attached to a host
			 *  in device mode) in order to manage USB communications. This task may be executed inside an RTOS,
			 *  scheduler (e.g. the simple MyUSB Scheduler), fast timer ISR or the main user application loop.
			 *
			 *  The USB task must be serviced within 50mS in all modes, when needed. The task may be serviced 
			 *  at all times, or (for minimum CPU consumption):
			 *
			 *    > In device mode, it may be disabled at startup, enabled on the firing of the USB_Connect event
			 *    and disabled again on the firing of the USB_Disconnect event.
			 *
			 *    > In host mode, it may be disabled at startup, enabled on the firing of the USB_DeviceAttached
			 *    event and disabled again on the firing of the USB_DeviceUnattached event.
			 *
			 *  \see Events.h for more information on the USB events.
			 */
			TASK(USB_USBTask);

	/* Private Interface - For use in library only: */
	#if !defined(__DOXYGEN__)
		/* Function Prototypes: */
			#if defined(INCLUDE_FROM_USBTASK_C)
				#if defined(USB_CAN_BE_HOST)
					static void USB_HostTask(void);
				#endif
				
				#if defined(USB_CAN_BE_DEVICE)
					static void USB_DeviceTask(void);
				#endif
			#endif

			#if defined(USB_CAN_BE_BOTH)
				void USB_InitTaskPointer(void);
			#endif
	#endif
	
	/* Disable C linkage for C++ Compilers: */
		#if defined(__cplusplus)
			}
		#endif
		
#endif
