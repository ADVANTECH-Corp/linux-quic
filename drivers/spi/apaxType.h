/* #############################################################################
   *****************************************************************************
                  Copyright (c) 2011, Advantech eAutomation Division
                               Advantech Co., Ltd.
                               APAX Linux IO Driver
        THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
           INFORMATION WHICH IS THE PROPERTY OF ADVANTECH AUTOMATION CORP.
           
      ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
                 ADVANTECH AUTOMATION CORP., IS STRICTLY PROHIBITED.
   *****************************************************************************
   #############################################################################
   
   File:        apaxtype.h
   Description: This is the type definitions header for APAX Linux IO Driver and Library
   Status:      works
   Author:		Tony Liu
   Change log:	Version 1.00 <2011/4/6>
	 				- Initial version 
	 				
  ----------------------------------------------------------------------------
   Permission to use, copy, modify, and distribute this software and its
   documentation for any purpose and without fee is hereby granted, provided
   that the above copyright notice appear in all copies and that both that
   copyright notice and this permission notice appear in supporting
   documentation.  This software is provided "as is" without express or
   implied warranty.
  ----------------------------------------------------------------------------*/
#ifndef __APAXTYPE_H__
#define __APAXTYPE_H__

//================================================================
 // 2009/01/8, tony liu
#ifndef ULONGLONG
#define ULONGLONG unsigned long
#endif

#ifndef LONGLONG
#define LONGLONG long
#endif

#ifndef DWORD
#define DWORD unsigned int
#endif

#ifndef ULONG
#define ULONG unsigned int
#endif

#ifndef LONG
#define LONG int
#endif

#ifndef UINT
#define UINT unsigned int
#endif

#ifndef WORD
#define WORD unsigned short
#endif

#ifndef USHORT
#define USHORT unsigned short
#endif

#ifndef BYTE
#define BYTE unsigned char
#endif

#ifndef UCHAR
#define UCHAR unsigned char
#endif

#ifndef PUCHAR
#define PUCHAR UCHAR*
#endif

#ifndef VOID
#define VOID void
#endif

#ifndef PVOID
#define PVOID VOID*
#endif

#ifndef BOOL
#define BOOL int
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
#endif
