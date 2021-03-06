(*
  $Id$
  ------------------------------------------------------------------------------
	Header file for libgba serial communication functions

	Copyright 2003-2004 by Dave Murphy.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Library General Public
	License as published by the Free Software Foundation; either
	version 2 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Library General Public License for more details.

	You should have received a copy of the GNU Library General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
	MA 02110-1301, USA.

	Please report all bugs and problems through the bug tracker at
	"http://sourceforge.net/tracker/?group_id=114505&atid=668551".

  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$


	

*)


{$ifdef GBA_INTERFACE}
//---------------------------------------------------------------------------------
// SIO mode control bits used with REG_SIOCNT
//---------------------------------------------------------------------------------
const
  SIO_8BIT  = $0000;	// Normal 8-bit communication mode
  SIO_32BIT = $1000;	// Normal 32-bit communication mode
  SIO_MULTI = $2000;	// Multi-play communication mode
  SIO_UART  = $3000;	// UART communication mode
  SIO_IRQ   = $4000;	// Enable serial irq


//---------------------------------------------------------------------------------
// baud rate settings
//---------------------------------------------------------------------------------
  SIO_9600   = $0000;
  SIO_38400  = $0001;
  SIO_57600  = $0002;
  SIO_115200 = $0003;

  SIO_CLK_INT  = (1 shl 0);	// Select internal clock
  SIO_2MHZ_CLK = (1 shl 1);	// Select 2MHz clock
  SIO_RDY      = (1 shl 2);	// Opponent SO state
  SIO_SO_HIGH  = (1 shl 3);	// Our SO state
  SIO_START    = (1 shl 7);


//---------------------------------------------------------------------------------
// SIO modes set with REG_RCNT
//---------------------------------------------------------------------------------
  R_NORMAL  = $0000;
  R_MULTI   = $0000;
  R_UART    = $0000;
  R_GPIO    = $8000;
  R_JOYBUS  = $C000;

//---------------------------------------------------------------------------------
// General purpose mode control bits used with REG_RCNT
//---------------------------------------------------------------------------------
  GPIO_SC  = $0001;	// Data
  GPIO_SD	 = $0002;
  GPIO_SI	 = $0004;
  GPIO_SO	 = $0008;
  GPIO_SC_IO  = $0010;	// Select I/O
  GPIO_SD_IO  = $0020;
  GPIO_SI_IO  = $0040;
  GPIO_SO_IO  = $0080;
  GPIO_SC_INPUT  = $0000;	// Input setting
  GPIO_SD_INPUT  = $0000;
  GPIO_SI_INPUT  = $0000;
  GPIO_SO_INPUT  = $0000;
  GPIO_SC_OUTPUT = $0010;	// Output setting
  GPIO_SD_OUTPUT = $0020;
  GPIO_SI_OUTPUT = $0040;
  GPIO_SO_OUTPUT = $0080;


  REG_SIOCNT      : pu16 = pointer(REG_BASE + $128);	// Serial Communication Control
  REG_SIODATA8    : pu16 = pointer(REG_BASE + $12a);	// 8bit Serial Communication Data
  REG_SIODATA32   : pu32 = pointer(REG_BASE + $120);	// 32bit Serial Communication Data
  REG_SIOMLT_SEND : pu16 = pointer(REG_BASE + $12a);	// Multi-play SIO Send Data
  REG_SIOMLT_RECV : pu16 = pointer(REG_BASE + $120);	// Multi-play SIO Receive Data
  REG_SIOMULTI0   : pu16 = pointer(REG_BASE + $120);	// Master data
  REG_SIOMULTI1   : pu16 = pointer(REG_BASE + $122);	// Slave 1 data
  REG_SIOMULTI2   : pu16 = pointer(REG_BASE + $124);	// Slave 2 data
  REG_SIOMULTI3   : pu16 = pointer(REG_BASE + $126);	// Slave 3 data

  REG_RCNT        : pu16 = pointer(REG_BASE + $134);	// SIO Mode Select/General Purpose Data

  REG_HS_CTRL     : pu16 = pointer(REG_BASE + $140);	// SIO JOY Bus Control

  REG_JOYRE       : pu32 = pointer(REG_BASE + $150);	// SIO JOY Bus Receive Data
  REG_JOYRE_L     : pu16 = pointer(REG_BASE + $150);
  REG_JOYRE_H     : pu16 = pointer(REG_BASE + $152);

  REG_JOYTR       : pu32 = pointer(REG_BASE + $154);	// SIO JOY Bus Transmit Data
  REG_JOYTR_L     : pu16 = pointer(REG_BASE + $154);
  REG_JOYTR_H     : pu16 = pointer(REG_BASE + $156);

  REG_JSTAT       : pu16 = pointer(REG_BASE + $158);	// SIO JOY Bus Receive Status

{$endif GBA_INTERFACE}
