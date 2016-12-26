/* #############################################################################
   *****************************************************************************
                  Copyright (c) 2016, Advantech eAutomation Division
                               Advantech Co., Ltd.
                               ADAM IO Driver
        THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
           INFORMATION WHICH IS THE PROPERTY OF ADVANTECH AUTOMATION CORP.

      ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
                 ADVANTECH AUTOMATION CORP., IS STRICTLY PROHIBITED.
   *****************************************************************************
   #############################################################################

   File:		adamio.h
   Description:	The On-Board AI driver(SPI) for Qualcomm Dragonboard 410c(APQ8016).
   Status:		works
   Author:		Robert Yin
   Change log:	Version 1.00 <2016/10/27>
	 				- Initial version

  ----------------------------------------------------------------------------
   Permission to use, copy, modify, and distribute this software and its
   documentation for any purpose and without fee is hereby granted, provided
   that the above copyright notice appear in all copies and that both that
   copyright notice and this permission notice appear in supporting
   documentation.  This software is provided "as is" without express or
   implied warranty.
  ----------------------------------------------------------------------------*/

#ifndef ADAMIO_H
#define ADAMIO_H

#include <linux/types.h>
#include "apaxType.h"

// ==========================================================
// Robert(2016/10/28): for Qualcomm EMG project: on-board AI
// ==========================================================
#define ON_BOARD_SLOT_NUMBER 0xFF
#define ON_BOARD_AI_CHANNEL_AMOUNT 2

#define AIO_CHANNEL_VALUE_LENGTH         2
#define AIO_CHANNEL_STATUS_LENGTH        1
#define AIO_CHANNEL_RANGE_TYPE_LENGTH    2
#define AIO_CHANNEL_SAMPLE_RATE_LENGTH   4
#define AIO_CHANNEL_FIRMWARE_VERSION_LENGTH   4

//////////////////////////////////////////////////////////////


/* User space versions of kernel symbols for SPI clocking modes,
 * matching <linux/spi/spi.h>
 */

#define SPI_CPHA		0x01
#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80
#define SPI_TX_DUAL		0x100
#define SPI_TX_QUAD		0x200
#define SPI_RX_DUAL		0x400
#define SPI_RX_QUAD		0x800

/*---------------------------------------------------------------------------*/

/* IOCTL commands */
#define SPI_IOC_MAGIC			'k'

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) (limited to 8 bits) */
#define SPI_IOC_RD_MODE			_IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_WR_MODE			_IOW(SPI_IOC_MAGIC, 1, __u8)

//#if 0
/* Read / Write SPI bit justification */
#define SPI_IOC_RD_LSB_FIRST		_IOR(SPI_IOC_MAGIC, 2, __u8)
#define SPI_IOC_WR_LSB_FIRST		_IOW(SPI_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
#define SPI_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOC_MAGIC, 4, __u32)
#define SPI_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOC_MAGIC, 4, __u32)
//#endif //if 0

/* Read / Write of the SPI mode field */
#define SPI_IOC_RD_MODE32		_IOR(SPI_IOC_MAGIC, 5, __u32)
#define SPI_IOC_WR_MODE32		_IOW(SPI_IOC_MAGIC, 5, __u32)


/////Robert: function code for AdamIO(used by ioctl)////////////////////////
#define SPI_IOC_GET_CHANNEL_VALUE				  _IOR(SPI_IOC_MAGIC, 6, __u32)
#define SPI_IOC_GET_ALL_CHANNEL_VALUE     _IOR(SPI_IOC_MAGIC, 7, __u32)

#define SPI_IOC_GET_CHANNEL_STATUS		    _IOR(SPI_IOC_MAGIC, 8, __u32)
#define SPI_IOC_GET_CHANNEL_RANGE		      _IOR(SPI_IOC_MAGIC, 9, __u32)
#define SPI_IOC_SET_CHANNEL_RANGE		      _IOW(SPI_IOC_MAGIC, 10, __u32)

#define SPI_IOC_SET_ZERO_CALIBRATION	    _IOW(SPI_IOC_MAGIC, 11, __u32)
#define SPI_IOC_SET_SPAN_CALIBRATION	    _IOW(SPI_IOC_MAGIC, 12, __u32)
#define SPI_IOC_GET_SAMPLE_RATE           _IOW(SPI_IOC_MAGIC, 13, __u32)
#define SPI_IOC_SET_SAMPLE_RATE           _IOW(SPI_IOC_MAGIC, 14, __u32)
#define SPI_IOC_GET_MCU_FIRMWARE_VERSION  _IOW(SPI_IOC_MAGIC, 15, __u32)

//#define SPI_IOC_SET_RESET_TRANSACTION	_IOW(SPI_IOC_MAGIC, 15, __u32)

////////////////////////////////////////////////////////////////////////////
#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

//ioctl message structure
struct spi_ioctl_data{
  UCHAR channelNumber;
  WORD  wChanValues[ON_BOARD_AI_CHANNEL_AMOUNT];
  UCHAR byChanStatus[ON_BOARD_AI_CHANNEL_AMOUNT];
  WORD  wChanRanges[ON_BOARD_AI_CHANNEL_AMOUNT];
  DWORD dwSampleRate;
  DWORD wFwVersion;
};

#pragma pack(pop) /* restore original alignment from stack */

/////////////////////////////////////////////////////////////////////

#endif /* ADAMIO_H */
