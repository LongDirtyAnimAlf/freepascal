{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Karoly Balogh

    timer.device functions for AROS

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


procedure GetSysTime(Dest: PTimeVal); syscall TimerBase 11;
