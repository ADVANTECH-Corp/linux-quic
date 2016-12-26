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

   File:        adamio.c
   Description: The On-Board AI driver(SPI interface) for Qualcomm EMG Project(APQ-8016 chip).
   Status:      works
   Author:      Robert Yin
   Change log:  Version 1.00 <2016/10/27>
                    - Initial version
                Version 1.01 <2016/12/06>
                    - Change SPI Transfer format

  ----------------------------------------------------------------------------
   Permission to use, copy, modify, and distribute this software and its
   documentation for any purpose and without fee is hereby granted, provided
   that the above copyright notice appear in all copies and that both that
   copyright notice and this permission notice appear in supporting
   documentation.  This software is provided "as is" without express or
   implied warranty.
  ----------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include "adamIo.h"


// make mdev/udev create the /dev/adamIO_B.C character device
static struct class *adamio_class;

static struct of_device_id adamio_dts_table[] = {
    { .compatible = "advantech,adamIO",}, //Compatible node must match dts
    { },
};

MODULE_DEVICE_TABLE(of, adamio_dts_table);

//#define BUFFER_SIZE 64
//struct spi_message spi_msg;
//struct spi_transfer spi_xfer;
/////////////////////////////////////////////////////////////////////////////
#define ADAMIO_VER      "1.01"

//SPI transfer parameters
#define SPI_MAX_SPEED_HZ    1*1000*1000 //spi tx/rx speed with adam IO
#define SPI_BITS_PER_WORD_FOR_ADAM_IO  16
#define SPI_TRANSFER_DELAYS  1000 //microseconds to delay after this transfer before changing the chipselect status

//TX/RX max frame length
#define SPI_BUFSIZE     32

//TODO: for test
#define ADVANTECH_DEBUG
//#define ADVANTECH_SPI_SIMULATION

#ifdef ADVANTECH_DEBUG
    #define DEBUGPRINT printk
    u64 gErrorCounter = 0;
#else
    #define DEBUGPRINT(a, ...)
#endif

////Data transfer format on SPI//////////////////////////////////////////////

//total 8 bytes for one SPI transaction(MOSI and MISO)
#define SPI_TRANSACTION_TOTAL_LENGTH    8
// 4 times of chip select low for one SPI transaction
#define SPI_TRANSACTION_CHIP_SELECT_AMOUNT     4
// 2 bytes in each chip select low
#define SPI_TRANSACTION_CHIP_SELECT_LENGTH     2

#define SPI_COMMAND_HEADER_LENGTH           2
#define SPI_RESPONSE_HEADER_LENGTH          1

//fixed value int master command byte 1
#define SPI_MASTER_COMMAND_FIXED_VALUE    0x11
//dummy value
#define SPI_MASTER_SLAVE_DUMMY_VALUE      0x7F
//fixed leading 4 bits in Slave response byte
#define SPI_SLAVE_FIXED_LEADING_VALUE     0xD

#define SPI_COMMAND_READ_CHANNEL_VALUE    0x0
#define SPI_COMMAND_WRITE_CHANNEL_VALUE   0x1
#define SPI_COMMAND_GET_CHANNEL_STATUS    0x2
#define SPI_COMMAND_GET_CHANNEL_RANGE     0x3
#define SPI_COMMAND_SET_CHANNEL_RANGE     0x4
#define SPI_COMMAND_ZERO_CALIBRATION      0x5
#define SPI_COMMAND_SPAN_CALIBRATION      0x6
#define SPI_COMMAND_READ_SAMPLE_RATE      0x7
#define SPI_COMMAND_WRITE_SAMPLE_RATE     0x8
#define SPI_COMMAND_READ_FIRMWARE_VERSION 0xD
#define SPI_COMMAND_RESERVED_VALUE        0xE
#define SPI_COMMAND_RESET_TRANSACTION     0xF

#define SPI_COMMAND_ALL_CHANNEL_ID  0xF

/////////////////////////////////////////////////////////////////////////////
#define SPI_ADAM_IO_MAJOR            153 /* assigned */
#define N_SPI_MINORS            32  /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

//define valid bits in SPI_MODE_MASK
#define SPI_MODE_MASK       (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                | SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
                | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct adamio_data {
    dev_t           devt;
    spinlock_t      spi_lock;
    struct spi_device   *spi;
    struct list_head    device_entry;

    /* TX/RX buffers are NULL unless this device is open (users > 0) */
    struct mutex        buf_lock;
    unsigned        users;
    u32         speed_hz;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/*
//static unsigned bufsiz = 4096;
static unsigned bufsiz = BUFFER_SIZE;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
*/

u16 * pTxData[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];
u16 * pTxDummyData[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];
u16 * pRxData[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];

/////////////////////////////////////////////////////////////////////////////

static int adamio_open(struct inode *inode, struct file *filp)
{
    struct adamio_data  *adamio;
    int         status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(adamio, &device_list, device_entry) {
        if (adamio->devt == inode->i_rdev) {
            status = 0;
            break;
        }
    }

    if (status) {
        pr_debug("adamio: nothing for minor %d\n", iminor(inode));
        goto err_find_dev;
    }

    adamio->users++;
    filp->private_data = adamio;
    nonseekable_open(inode, filp);

    mutex_unlock(&device_list_lock);
    return 0;

err_find_dev:
    mutex_unlock(&device_list_lock);
    return status;
}

static int adamio_release(struct inode *inode, struct file *filp)
{
    struct adamio_data  *adamio;

    mutex_lock(&device_list_lock);
    adamio = filp->private_data;
    filp->private_data = NULL;

    /* last close? */
    adamio->users--;
    if (!adamio->users) {
        int     dofree;

        spin_lock_irq(&adamio->spi_lock);
        if (adamio->spi)
            adamio->speed_hz = adamio->spi->max_speed_hz;

        /* ... after we unbound from the underlying device? */
        dofree = (adamio->spi == NULL);
        spin_unlock_irq(&adamio->spi_lock);

        if (dofree)
            kfree(adamio);
    }
    mutex_unlock(&device_list_lock);

    return 0;
}

static ssize_t adamio_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

static ssize_t adamio_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

static int checkSpiReceiveData(u8 *pRxBuf, u16 spiRxLen)
{

#ifndef ADVANTECH_SPI_SIMULATION

    if(pRxBuf == NULL || ((pRxBuf[0] >> 4) != SPI_SLAVE_FIXED_LEADING_VALUE) || ((pRxBuf[0] & 0x0F) != spiRxLen)){
        gErrorCounter++;
        return -EINVAL;
    }else{
        return 0; //check ok
    }

#else

    return 0;//simulation ok

#endif
}

#ifdef ADVANTECH_DEBUG
static void dump_spi_data(u8 txOrRx, u8 * buf, u16 length)
{
    int i;
    if(txOrRx)
        DEBUGPRINT("Send TX value:\n");
    else
        DEBUGPRINT("Get RX value:\n");
    for(i=0; i < length; i++){
        DEBUGPRINT(" 0x%02X", buf[i]);
    }
    DEBUGPRINT("\n_________\n");
}
#endif //ADVANTECH_DEBUG


static int adamio_transfer_message(struct adamio_data *adamio,
                u8 *txbuf, u16 txlen,
                u8 *rxbuf, u16 rxlen)
{
    struct spi_message msg;
    //struct spi_transfer xfer[2];
    struct spi_transfer xfer[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];
    int counter = 0;
    //int rxCounter = 0;
    int txAmount = 0;
    int rxAmount = 0;
    //int dummyTxLen = SPI_TRANSACTION_CHIP_SELECT_LENGTH;//2 bytes for dummy data
/*
    u16 * pTxData[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];
    u16 * pTxDummyData[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];
    u16 * pRxData[SPI_TRANSACTION_CHIP_SELECT_AMOUNT];
*/
    int     status = -EFAULT;
    int      i;
    //u8      * pLocalBuf;
    //u8      * pDummyTxData; //use dummy data to keep chip select low for SPI slave response data

    if(!txlen || !rxlen)
        return -EINVAL;

    txAmount = txlen / SPI_TRANSACTION_CHIP_SELECT_LENGTH;//each transfer is 2 bytes
    rxAmount = rxlen / SPI_TRANSACTION_CHIP_SELECT_LENGTH;//each rx is 2 bytes
    DEBUGPRINT("adamio_transfer_message: txlen=%d, tx amount=%d\n", txlen, txAmount);
    DEBUGPRINT("adamio_transfer_message: rxlen=%d, rx amount=%d\n", rxlen, rxAmount);
    if(txAmount >= SPI_TRANSACTION_CHIP_SELECT_AMOUNT || rxAmount >= SPI_TRANSACTION_CHIP_SELECT_AMOUNT){
        DEBUGPRINT("Error: tx or rx amount too large, txAmount:%d, rxAmount:%d\n", txAmount, rxAmount);
        return -EINVAL;
    }
/*
    pDummyTxData = kmalloc(dummyTxLen, GFP_KERNEL | GFP_DMA);
    if (!pDummyTxData){
        DEBUGPRINT("Error: pDummyTxData allocate memory failed\n");
        return -ENOMEM;
    }
    memset(pDummyTxData, SPI_MASTER_SLAVE_DUMMY_VALUE, rxlen); //0x7f is for tx dummy data

    pLocalBuf = kmalloc(max((unsigned)SPI_BUFSIZE, txlen + rxlen), GFP_KERNEL | GFP_DMA);
    if (!pLocalBuf){
        DEBUGPRINT("Error: pLocalBuf allocate memory failed\n");
        kfree(pDummyTxData);
        return -ENOMEM;
    }
*/
    //init SPI tx/rx setting value
    //for(i = 0 ; i < (txAmount + rxAmount); i++){ //add one for RX
    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){ //TX and RX
        //DEBUGPRINT("memset xfer for : %d\n", i);
        memset(&xfer[i], 0, sizeof(struct spi_transfer));
    }
/*
    memset(&xfer[0], 0, sizeof(struct spi_transfer));
    memset(&xfer[1], 0, sizeof(struct spi_transfer));
*/
    spi_message_init(&msg);

#if 0
    //tx
    xfer[0].len = txlen;
    xfer[0].bits_per_word = SPI_BITS_PER_WORD_FOR_ADAM_IO;
    memcpy(pLocalBuf, txbuf, txlen);
    xfer[0].tx_buf = pLocalBuf;
    xfer[0].delay_usecs = SPI_TRANSFER_DELAYS;//0;
    xfer[0].speed_hz = adamio->speed_hz;
    spi_message_add_tail(&xfer[0], &msg);

    //rx
    xfer[1].len = rxlen;
    xfer[1].bits_per_word = SPI_BITS_PER_WORD_FOR_ADAM_IO;
    xfer[1].tx_buf = pDummyTxData;
    xfer[1].rx_buf = pLocalBuf + txlen;
    xfer[1].delay_usecs = SPI_TRANSFER_DELAYS;//0;
    xfer[1].speed_hz = adamio->speed_hz;
    spi_message_add_tail(&xfer[1], &msg);
#ifdef ADVANTECH_DEBUG
    dump_spi_data(1, xfer[i].tx_buf, xfer[i].len);
#endif

#else //if 0

    DEBUGPRINT("txAmount value: %d\n", txAmount);
    DEBUGPRINT("rxAmount value: %d\n", rxAmount);
/*
    //allocate DMA safe memory
    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        pTxData[i] = kmalloc(SPI_TRANSACTION_CHIP_SELECT_LENGTH, GFP_KERNEL | GFP_DMA);
        if (!pTxData[i]){
            DEBUGPRINT("Error: pTxData allocate memory failed\n");
            //TODO: free used memory
            return -ENOMEM;
        }
    }

    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        pTxDummyData[i] = kmalloc(SPI_TRANSACTION_CHIP_SELECT_LENGTH, GFP_KERNEL | GFP_DMA);
        if (!pTxData[i]){
            DEBUGPRINT("Error: pTxDummyData allocate memory failed\n");
            //TODO: free used memory
            return -ENOMEM;
        }
        memset(pTxDummyData[i], SPI_MASTER_SLAVE_DUMMY_VALUE, SPI_TRANSACTION_CHIP_SELECT_LENGTH); //0x7f is for tx dummy data
    }

    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        pRxData[i] = kmalloc(SPI_TRANSACTION_CHIP_SELECT_LENGTH, GFP_KERNEL | GFP_DMA);
        if (!pTxData[i]){
            DEBUGPRINT("Error: pRxData allocate memory failed\n");
            //TODO: free used memory
            return -ENOMEM;
        }
    }
*/
    //call spi_message_add_tail() multiple times to build multiple chip select low

    //for TX
    for(i = 0 ; i < txAmount; i++){
        xfer[i].len = SPI_TRANSACTION_CHIP_SELECT_LENGTH; //transfer 2 bytes each time
        xfer[i].bits_per_word = SPI_BITS_PER_WORD_FOR_ADAM_IO;
        //memcpy(pLocalBuf, txbuf, txlen);
        //memcpy(pLocalBuf + i*2, txbuf + i*2, 2);
        //xfer[i].tx_buf = pLocalBuf + i*2;
        memcpy(pTxData[i], txbuf + i * SPI_TRANSACTION_CHIP_SELECT_LENGTH, SPI_TRANSACTION_CHIP_SELECT_LENGTH);
        xfer[i].tx_buf = pTxData[i];
        xfer[i].delay_usecs = SPI_TRANSFER_DELAYS;//0;
        xfer[i].speed_hz = adamio->speed_hz;
        spi_message_add_tail(&xfer[i], &msg);
#ifdef ADVANTECH_DEBUG
    //dump_spi_data(1, xfer[i].tx_buf, xfer[i].len);
        DEBUGPRINT("fill tx data: i:%d, value: 0x%x\n", i, *pTxData[i]);
#endif
        counter++;
    }

    DEBUGPRINT("before fill rx, counter: %d\n", counter);
    //for(i = counter ; i < (counter + rxAmount); i++){
    for(i = counter ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        xfer[i].len = SPI_TRANSACTION_CHIP_SELECT_LENGTH;
        xfer[i].bits_per_word = SPI_BITS_PER_WORD_FOR_ADAM_IO;
        xfer[i].tx_buf = pTxDummyData[i - counter];
        xfer[i].rx_buf = pRxData[i - counter];
        xfer[i].delay_usecs = SPI_TRANSFER_DELAYS;//0;
        xfer[i].speed_hz = adamio->speed_hz;
        spi_message_add_tail(&xfer[i], &msg);
#ifdef ADVANTECH_DEBUG
    //dump_spi_data(1, xfer[i].tx_buf, xfer[i].len);
        //DEBUGPRINT("fill dummy data: i:%d, value: 0x%x\n", i, *pTxDummyData[i - counter]);
#endif
        //rxCounter++;
    }

#endif //if 0

    //DEBUGPRINT("adam_xfer: len=%d\n", xfer.len);
    //DEBUGPRINT("adam_xfer: delay_usecs=%d\n", xfer.delay_usecs);
    //DEBUGPRINT("adam_xfer: bits_per_word=%d\n", xfer.bits_per_word);
    //DEBUGPRINT("adam_xfer: speed_hz=%d\n", xfer.speed_hz);
    //DEBUGPRINT("adam_xfer: tx_nbits:%d\n", xfer.tx_nbits);
    //DEBUGPRINT("adam_xfer: rx_nbits:%d\n", xfer.rx_nbits);

    //spi_message_add_tail(&xfer, &msg);

#ifdef ADVANTECH_DEBUG
    //dump_spi_data(1, xfer[0].tx_buf, xfer[0].len);
    //DEBUGPRINT("Tx Dummy data: ");
    //dump_spi_data(1, pDummyTxData, rxlen);
#endif

    //perform SPI tx and rx
    status = spi_sync(adamio->spi, &msg);//Return: zero on success, else a negative error code.
    if (status < 0){
        DEBUGPRINT("Error: spi_sync status=%d\n", status);
        dev_err(&adamio->spi->dev, "Couldn't send SPI data\n");
    }else{
        //memcpy(rxbuf, xfer[1].rx_buf, rxlen);
        //memcpy(rxbuf, pLocalBuf + txlen, rxlen);
        for(i=0; i < rxAmount; i++){
            memcpy(rxbuf + i * 2, pRxData[i], 2);
            //DEBUGPRINT("0x%x \n", *pRxData[i]);
        }
    }

#ifdef ADVANTECH_DEBUG
    //DEBUGPRINT("spi_sync status return: %d\n", status);
    DEBUGPRINT("Received SPI data:\n");
    //
    for(i=0; i < rxlen; i++){
        DEBUGPRINT("0x%x ", rxbuf[i]);
    }
    DEBUGPRINT("\n");
#endif

    //kfree(pDummyTxData);
    //kfree(pLocalBuf);
/*
    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        kfree(pTxData[i]);
        kfree(pTxDummyData[i]);
        kfree(pRxData[i]);
    }
*/
    DEBUGPRINT("gErrorCounter: %d\n", gErrorCounter);
    DEBUGPRINT("_________\n");

    return status;
}

static long adamio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int         err = 0;
    int         retval = 0;
    struct adamio_data  *adamio;
    struct spi_device   *spi;
//    u8         user_data;
//    u32         tmp;

    //Robert
    struct spi_ioctl_data ioctlData;
    //u8 * pTxBuf; //spi send data buffer
    //u8 * pRxBuf; //spi receive data buffer
    //u8 * pRxVal; //spi receive data buffer
    u16 spiTxLen; //length of spi send data buffer
    u16 spiRxLen; //length of spi receive data buffer
    u16 dataLengthInResponse; //length value in SPI slave byte 0
    int i;
    u8 spiCmdByte; //first spi send byte(command byte)
    u8 channelIndex = 0;
    int counter = 0;


    unsigned char tbuf[SPI_BUFSIZE];
    unsigned char rbuf[SPI_BUFSIZE];

    memset(tbuf, 0 , SPI_BUFSIZE);
    memset(rbuf, 0 , SPI_BUFSIZE);

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
                (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
                (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    //get device data
    adamio = filp->private_data;

    /* guard against device removal before, or while we issue this ioctl. */
    spin_lock_irq(&adamio->spi_lock);
    spi = spi_dev_get(adamio->spi);
    spin_unlock_irq(&adamio->spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* lock for SPI transfer */
    mutex_lock(&adamio->buf_lock);

    switch (cmd) {
    //read requests
    case SPI_IOC_RD_MODE:
        retval = __put_user(spi->mode & SPI_MODE_MASK,
                    (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MODE32:
        retval = __put_user(spi->mode & SPI_MODE_MASK,
                    (__u32 __user *)arg);
        break;
    case SPI_IOC_RD_LSB_FIRST:
        retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
                    (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_BITS_PER_WORD:
        retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
        retval = __put_user(adamio->speed_hz, (__u32 __user *)arg);
        break;
/*
    //write requests
    case SPI_IOC_WR_MODE:
    case SPI_IOC_WR_MODE32:
        if (cmd == SPI_IOC_WR_MODE)
            retval = __get_user(tmp, (u8 __user *)arg);
        else
            retval = __get_user(tmp, (u32 __user *)arg);
        if (retval == 0) {
            u32 save = spi->mode;

            //if not valid mode value: return error
            if (tmp & ~SPI_MODE_MASK) {
                retval = -EINVAL;
                break;
            }

            tmp |= spi->mode & ~SPI_MODE_MASK;
            spi->mode = (u16)tmp;
            retval = spi_setup(spi);//Return: zero on success, else a negative error code.
            if (retval < 0){
                spi->mode = save;
                dev_dbg(&spi->dev, "Error: set spi mode %x failed\n", tmp);
            }else
                dev_dbg(&spi->dev, "set spi mode: %x ok\n", tmp);
        }
        break;
    case SPI_IOC_WR_LSB_FIRST:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u32 save = spi->mode;

            if (tmp)
                spi->mode |= SPI_LSB_FIRST;
            else
                spi->mode &= ~SPI_LSB_FIRST;
            retval = spi_setup(spi);
            if (retval < 0){
                spi->mode = save;
                dev_dbg(&spi->dev, "Error: set LSB_FIRST failed: %x\n", tmp);
            }
            else
                dev_dbg(&spi->dev, "%csb first\n",
                        tmp ? 'l' : 'm');
        }
        break;
    case SPI_IOC_WR_BITS_PER_WORD:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u8  save = spi->bits_per_word;

            spi->bits_per_word = tmp;
            retval = spi_setup(spi);
            if (retval < 0){
                spi->bits_per_word = save;
            }
            else
                dev_dbg(&spi->dev, "%d bits per word\n", tmp);
        }
        break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
        retval = __get_user(tmp, (__u32 __user *)arg);
        if (retval == 0) {
            u32 save = spi->max_speed_hz;
            dev_err(&spi->dev, "SPI_IOC_WR_MAX_SPEED_HZ: save: %d Hz\n", save);
            dev_err(&spi->dev, "SPI_IOC_WR_MAX_SPEED_HZ: tmp: %d Hz\n", tmp);

            spi->max_speed_hz = tmp;
            retval = spi_setup(spi);//Return: zero on success, else a negative error code.
            if (retval == 0){
                dev_err(&spi->dev, "OK: Set MAX Speed to %d Hz\n", tmp);
                adamio->speed_hz = tmp;
            }
            else{
                dev_err(&spi->dev, "Error: Set MAX Speed to %d Hz failed\n", tmp);
            }
            //spi->max_speed_hz = save; //why restore org value ???
            //spi->max_speed_hz = tmp;
        }
        break;
*/
    /////////////////////////////////////////

    case SPI_IOC_GET_CHANNEL_VALUE:
    case SPI_IOC_GET_ALL_CHANNEL_VALUE:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        spiTxLen = SPI_COMMAND_HEADER_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH + AIO_CHANNEL_VALUE_LENGTH;

        //check request is for one or all channels
        if(cmd == SPI_IOC_GET_CHANNEL_VALUE){
            DEBUGPRINT("==adamio_ioctl(): get SPI_IOC_GET_CHANNEL_VALUE channel=0x%x\n", ioctlData.channelNumber);
            channelIndex = ioctlData.channelNumber;
        }else{
            DEBUGPRINT("==adamio_ioctl(): get SPI_IOC_GET_ALL_CHANNEL_VALUE\n");
            channelIndex = 0;
        }
/*
        //check total SPI bytes for one transaction
        if((spiTxLen + spiRxLen) > SPI_TRANSACTION_TOTAL_LENGTH){
            DEBUGPRINT("==adamio_ioctl(): Error: transaction length too long: TX:%d, RX:%d\n", spiTxLen, spiRxLen);
            retval = -EINVAL;
            break;
        }
*/
        for(counter = 0; counter < ON_BOARD_AI_CHANNEL_AMOUNT; counter++){
            //set SPI master command data
            spiCmdByte = (SPI_COMMAND_READ_CHANNEL_VALUE << 4) + (channelIndex & 0xF);

            tbuf[0] = spiCmdByte;
            tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE;

            //send and receive SPI data
            retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
            if (retval < 0) {
                DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
                break;
            }

            //check leading value and length in first RX byte
            if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
                DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
                retval = -EINVAL;
                break;
            }

            ioctlData.wChanValues[channelIndex] = (rbuf[SPI_RESPONSE_HEADER_LENGTH] << 8) +
                                                        rbuf[SPI_RESPONSE_HEADER_LENGTH + 1];
            if(cmd == SPI_IOC_GET_CHANNEL_VALUE){
                break; //if query only one channel
            }
            channelIndex++;
        }//for

        //copy result to user
        retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));

#ifdef ADVANTECH_DEBUG_00
        DEBUGPRINT("Get AI value:\n");
        for(i=0; i < ON_BOARD_AI_CHANNEL_AMOUNT; i++){
            DEBUGPRINT("ch: %d, value: 0x%x\n", i, ioctlData.wChanValues[i]);
        }
        DEBUGPRINT("_________\n");
#endif
        break;

    case SPI_IOC_GET_CHANNEL_STATUS:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        DEBUGPRINT("==adamio_ioctl(): get SPI_IOC_GET_CHANNEL_STATUS\n");
        spiTxLen = SPI_COMMAND_HEADER_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH + AIO_CHANNEL_STATUS_LENGTH;
/*
        //check total SPI frame length
        if((spiTxLen + spiRxLen) > SPI_TRANSACTION_TOTAL_LENGTH){
            DEBUGPRINT("==adamio_ioctl(): Error: transaction length too long: TX:%d, RX:%d\n", spiTxLen, spiRxLen);
            retval = -EINVAL;
            break;
        }
*/
        //init value
        channelIndex = 0;

        //get status for all channels
        for(counter = 0; counter < ON_BOARD_AI_CHANNEL_AMOUNT; counter++){
            //set SPI master command data
            spiCmdByte = (SPI_COMMAND_GET_CHANNEL_STATUS << 4) + (channelIndex & 0xF);

            tbuf[0] = spiCmdByte;
            tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE;

            //transfer and get data from SPI slave
            retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
            if (retval < 0) {
                DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
                break;
            }

            //check leading value and length in first RX byte
            if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
                DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
                retval = -EINVAL;
                break;
            }
            //translate SPI slave data to AI status(1 byes)
            ioctlData.byChanStatus[channelIndex] = rbuf[SPI_RESPONSE_HEADER_LENGTH];
            channelIndex++;
        }//for
        //copy result to user
        retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));

        break;

    case SPI_IOC_GET_CHANNEL_RANGE:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        DEBUGPRINT("==adamio_ioctl(): get SPI_IOC_GET_CHANNEL_RANGE\n");
        spiTxLen = SPI_COMMAND_HEADER_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH + AIO_CHANNEL_RANGE_TYPE_LENGTH;

        //check total SPI frame length
        if((spiTxLen + spiRxLen) > SPI_TRANSACTION_TOTAL_LENGTH){
            DEBUGPRINT("==adamio_ioctl(): Error: transaction length too long: TX:%d, RX:%d\n", spiTxLen, spiRxLen);
            retval = -EINVAL;
            break;
        }

        //init value
        channelIndex = 0;
        //get status for all channels
        for(counter = 0; counter < ON_BOARD_AI_CHANNEL_AMOUNT; counter++){
                //set SPI master command data
                spiCmdByte = (SPI_COMMAND_GET_CHANNEL_RANGE << 4) + (channelIndex & 0xF);

            tbuf[0] = spiCmdByte;
            tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE;

            //transfer and get data from SPI slave
            retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
            if (retval < 0) {
                DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
                break;
            }

            //check leading value and length in first RX byte
            if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
                DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
                retval = -EINVAL;
                break;
            }
            //translate SPI slave data to AI ranges(2 byes)
            ioctlData.wChanRanges[channelIndex] = (rbuf[SPI_RESPONSE_HEADER_LENGTH] << 8) +
                                                            rbuf[SPI_RESPONSE_HEADER_LENGTH + 1];
            channelIndex++;
        }//for
        //copy result to user
        retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));
        break;

    case SPI_IOC_SET_CHANNEL_RANGE:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        DEBUGPRINT("==adamio_ioctl(): SPI_IOC_SET_CHANNEL_RANGE\n");
        spiTxLen = SPI_COMMAND_HEADER_LENGTH + AIO_CHANNEL_RANGE_TYPE_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH; //response of set command does not contain data

        //set init value
        channelIndex = 0;

        //get data for all channels
        for(counter = 0; counter < ON_BOARD_AI_CHANNEL_AMOUNT; counter++){
            //set SPI master command data
            spiCmdByte = (SPI_COMMAND_SET_CHANNEL_RANGE << 4) + (channelIndex & 0xF);

            tbuf[0] = spiCmdByte;
            tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE; //fixed value
            tbuf[SPI_COMMAND_HEADER_LENGTH] = ioctlData.wChanRanges[channelIndex] >> 8; //range value (MSB)
            tbuf[SPI_COMMAND_HEADER_LENGTH + 1] = (u8)(ioctlData.wChanRanges[channelIndex] & 0xff); //range value (LSB)

            //transfer and get data from SPI slave
            retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
            if (retval < 0) {
                DEBUGPRINT("==adamio_ioctl(): Error: SPI_IOC_SET_CHANNEL_RANGE\n");
                break;
            }

            //check leading value and length in first RX byte
            if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
                DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
                retval = -EINVAL;
                break;
            }
            channelIndex++;
        }//for

        break;

    case SPI_IOC_SET_ZERO_CALIBRATION:
    case SPI_IOC_SET_SPAN_CALIBRATION:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        spiTxLen = SPI_COMMAND_HEADER_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH;

        if(cmd == SPI_IOC_SET_ZERO_CALIBRATION){
            DEBUGPRINT("==adamio_ioctl(): SPI_IOC_SET_ZERO_CALIBRATION\n");
            spiCmdByte = (SPI_COMMAND_ZERO_CALIBRATION << 4) + (ioctlData.channelNumber & 0xF);
        }else{
            DEBUGPRINT("==adamio_ioctl(): SPI_IOC_SET_SPAN_CALIBRATION\n");
            spiCmdByte = (SPI_COMMAND_SPAN_CALIBRATION << 4) + (ioctlData.channelNumber & 0xF);
        }

        tbuf[0] = spiCmdByte;
        tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE;//fixed value

        //transfer and get data from SPI slave
        retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
        if (retval < 0) {
            DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
            break;
        }

        //check leading value and length in first RX byte
        if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
            DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
            retval = -EINVAL;
            break;
        }

        //copy result to user
        //retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));
        retval = 0;//success

        break;

    case SPI_IOC_SET_SAMPLE_RATE:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        DEBUGPRINT("adamio_ioctl(): set sample rate: 0x%02X\n", ioctlData.dwSampleRate );

        spiTxLen = SPI_COMMAND_HEADER_LENGTH + AIO_CHANNEL_SAMPLE_RATE_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH;

        //init value
        channelIndex = 0;
        //for(counter = 0; counter < ON_BOARD_AI_CHANNEL_AMOUNT; counter++){
            //set SPI master command data
            spiCmdByte = (SPI_COMMAND_WRITE_SAMPLE_RATE << 4) + (channelIndex & 0xF);

            tbuf[0] = spiCmdByte;
            tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE;//fixed value
            ///sample rate: 4 bytes
            tbuf[SPI_COMMAND_HEADER_LENGTH] = (u8)(ioctlData.dwSampleRate >> 24); //sample value
            tbuf[SPI_COMMAND_HEADER_LENGTH + 1] = (u8)((ioctlData.dwSampleRate & 0x00ff0000) >> 16); //sample value
            tbuf[SPI_COMMAND_HEADER_LENGTH + 2] = (u8)((ioctlData.dwSampleRate & 0x0000ff00) >> 8); //sample value
            tbuf[SPI_COMMAND_HEADER_LENGTH + 3] = (u8)((ioctlData.dwSampleRate & 0x000000ff)); //sample value

            //transfer and get data from SPI slave
            retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
            if (retval < 0) {
                DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
                break;
            }

            //check leading value and length in first RX byte
            if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
                DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
                retval = -EINVAL;
                break;
            }
            channelIndex++;
        //}//for

        //copy result to user
        //retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));
        retval = 0;//success

        break;

    case SPI_IOC_GET_SAMPLE_RATE:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        DEBUGPRINT("==adamio_ioctl(): get SPI_IOC_GET_SAMPLE_RATE\n");
        spiTxLen = SPI_COMMAND_HEADER_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH + AIO_CHANNEL_SAMPLE_RATE_LENGTH;

        channelIndex = 0;
        spiCmdByte = (SPI_COMMAND_READ_SAMPLE_RATE << 4) + (channelIndex & 0xF);

        tbuf[0] = spiCmdByte;
        tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE; //fixed value

        //transfer and get data from SPI slave
        retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
        if (retval < 0) {
            DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
            break;
        }

        //check leading value and length in first RX byte
        if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
            DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
            retval = -EINVAL;
            break;
        }
        //translate SPI slave data to AI status(1 byes)
        ioctlData.dwSampleRate = (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH] << 24) + (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH + 1] << 16) +
                                    (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH + 2] << 8) + (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH + 3]);

        //copy result to user
        retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));

        break;
    case SPI_IOC_GET_MCU_FIRMWARE_VERSION:
        retval = __copy_from_user(&ioctlData, (struct spi_ioctl_data __user *)arg, sizeof(struct spi_ioctl_data));
        if (retval != 0) {
            retval = -EINVAL;
            break;
        }

        DEBUGPRINT("==adamio_ioctl(): get SPI_IOC_GET_MCU_FIRMWARE_VERSION\n");
        spiTxLen = SPI_COMMAND_HEADER_LENGTH;
        spiRxLen = SPI_TRANSACTION_TOTAL_LENGTH - spiTxLen;
        dataLengthInResponse = SPI_RESPONSE_HEADER_LENGTH + AIO_CHANNEL_FIRMWARE_VERSION_LENGTH;

        channelIndex = 0;
        spiCmdByte = (SPI_COMMAND_READ_FIRMWARE_VERSION << 4) + (channelIndex & 0xF);

        tbuf[0] = spiCmdByte;
        tbuf[1] = SPI_MASTER_COMMAND_FIXED_VALUE; //fixed value

        //transfer and get data from SPI slave
        retval = adamio_transfer_message(adamio, tbuf, spiTxLen, rbuf, spiRxLen);
        if (retval < 0) {
            DEBUGPRINT("==adamio_ioctl(): Error: adamio_transfer_message\n");
            break;
        }

        //check leading value and length in first RX byte
        if(checkSpiReceiveData(rbuf, dataLengthInResponse) != 0){
            DEBUGPRINT("==adamio_ioctl(): Error: checkSpiReceiveData failed\n");
            retval = -EINVAL;
            break;
        }
        //translate SPI slave data to AI status(1 byes)
        ioctlData.wFwVersion = (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH] << 24) + (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH + 1] << 16) +
                                    (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH + 2] << 8) + (u32)(rbuf[SPI_RESPONSE_HEADER_LENGTH + 3]);
        //copy result to user
        retval = __copy_to_user((struct spi_ioctl_data __user *)arg, &ioctlData, sizeof(ioctlData));

        break;
    default:
        retval = -EINVAL;
        break;
    }

    //unlock after SPI transfer is complete
    mutex_unlock(&adamio->buf_lock);
    spi_dev_put(spi);

    return retval;
}


static const struct file_operations adamio_fops = {
    .owner =    THIS_MODULE,
    .write =    adamio_write,
    .read =     adamio_read,
    .unlocked_ioctl = adamio_ioctl,
    .open =     adamio_open,
    .release =  adamio_release,
    .llseek =   no_llseek,
};

/////////////////////////////////////////////////////////////////////////////
/*
static int spi_test_transfer(struct spi_device *spi)
{
    spi->mode |= SPI_LOOP; //Enable Loopback mode: defined in spi_device struct
    spi_message_init(&spi_msg);

    spi_xfer.tx_buf = tx_buf;
    spi_xfer.len = BUFFER_SIZE;
    spi_xfer.bits_per_word = 8;
    spi_xfer.speed_hz = spi->max_speed_hz;

    spi_message_add_tail(&spi_xfer, &spi_msg);

    return spi_sync(spi, &spi_msg);
}
*/
static int spi_adamIO_probe(struct spi_device *spi)
{
    struct adamio_data  *adamio;
    int         status;
    unsigned long       minor;
    unsigned int max_speed;

    DEBUGPRINT("===============spi_adamIO_probe ==============\n");

    if(!spi)
        return -ENOMEM;

    /* Allocate driver data */
    adamio = kzalloc(sizeof(*adamio), GFP_KERNEL);
    if (!adamio)
        return -ENOMEM;

    /* Initialize the driver data */
    adamio->spi = spi;
    spin_lock_init(&adamio->spi_lock);
    mutex_init(&adamio->buf_lock);

    INIT_LIST_HEAD(&adamio->device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);//Find the first value of 0 in the memory area
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        //Create device nodes in /dev/
        adamio->devt = MKDEV(SPI_ADAM_IO_MAJOR, minor);
        dev = device_create(adamio_class, &spi->dev, adamio->devt, adamio, "adamIO_%d.%d",
                    spi->master->bus_num, spi->chip_select);
        status = PTR_RET(dev);
    } else {
        dev_dbg(&spi->dev, "no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0) {
        set_bit(minor, minors);
        list_add(&adamio->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    //adamio->speed_hz = spi->max_speed_hz;
/*
    if (status == 0)
        dev_set_drvdata(&spi->dev, adamio);
    else
        kfree(adamio);

    DEBUGPRINT("%s:name=%s,bus_num=%d,cs=%d,mode=%d,speed=%d\n",__func__,spi->modalias, spi->master->bus_num, spi->chip_select, spi->mode, spi->max_speed_hz);//print SPI message
*/

    max_speed = SPI_MAX_SPEED_HZ;

    //note: call spi_setup is necessary to overwrite max speed setting in device tree
    spi->max_speed_hz = max_speed;
    status = spi_setup(spi);//Return: zero on success, else a negative error code.
    if (status == 0){
        adamio->speed_hz = spi->max_speed_hz;
        dev_err(&spi->dev, "Set Max_speed [%d] ok\n", max_speed );
    }else
        dev_err(&spi->dev, "spi_setup %d Hz (max) failed\n", max_speed);

    DEBUGPRINT("%s:name=%s,bus_num=%d,cs=%d,mode=%d,speed=%d\n",__func__,spi->modalias, spi->master->bus_num, spi->chip_select, spi->mode, spi->max_speed_hz);//print SPI message

    if (status == 0)
        dev_set_drvdata(&spi->dev, adamio);
    else
        kfree(adamio);

    return status;
}

static int spi_adamIO_remove(struct spi_device *spi)
{
    struct adamio_data  *adamio = dev_get_drvdata(&spi->dev);

    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&adamio->spi_lock);
    adamio->spi = NULL;
    spin_unlock_irq(&adamio->spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&adamio->device_entry);
    device_destroy(adamio_class, adamio->devt);
    clear_bit(MINOR(adamio->devt), minors);
    if (adamio->users == 0)
        kfree(adamio);
    mutex_unlock(&device_list_lock);

    DEBUGPRINT("========adamIO is removed =======\n");

    return 0;
}

//SPI Driver Info
static struct spi_driver spi_adamIO_driver = {
    .driver = {
        .name  = "adamIO_spi",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(adamio_dts_table),
    },
    .probe  = spi_adamIO_probe,
    .remove = spi_adamIO_remove,
};


static int __init spi_adamIO_init(void)
{
    int status;
    int i;
    int index = 0;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPI_ADAM_IO_MAJOR, "spi", &adamio_fops);
    if (status < 0)
        return status;

    adamio_class = class_create(THIS_MODULE, "spi_adam");
    if (IS_ERR(adamio_class)) {
        unregister_chrdev(SPI_ADAM_IO_MAJOR, spi_adamIO_driver.driver.name);
        return PTR_ERR(adamio_class);
    }

    status = spi_register_driver(&spi_adamIO_driver);
    if (status < 0) {
        class_destroy(adamio_class);
        unregister_chrdev(SPI_ADAM_IO_MAJOR, spi_adamIO_driver.driver.name);
    }

    //allocate DMA safe memory for SPI transfer
    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        pTxData[i] = kmalloc(SPI_TRANSACTION_CHIP_SELECT_LENGTH, GFP_KERNEL | GFP_DMA);
        if (!pTxData[i]){
            DEBUGPRINT("Error: pTxData allocate memory failed\n");
            status = -ENOMEM;
            index = i;
            break;
        }

        pTxDummyData[i] = kmalloc(SPI_TRANSACTION_CHIP_SELECT_LENGTH, GFP_KERNEL | GFP_DMA);
        if (!pTxData[i]){
            DEBUGPRINT("Error: pTxDummyData allocate memory failed\n");
            status = -ENOMEM;
            index = i;
            break;
        }
        memset(pTxDummyData[i], SPI_MASTER_SLAVE_DUMMY_VALUE, SPI_TRANSACTION_CHIP_SELECT_LENGTH); //0x7f is for tx dummy data

        pRxData[i] = kmalloc(SPI_TRANSACTION_CHIP_SELECT_LENGTH, GFP_KERNEL | GFP_DMA);
        if (!pTxData[i]){
            DEBUGPRINT("Error: pRxData allocate memory failed\n");
            status = -ENOMEM;
            index = i;
            break;
        }
    }//for

    if(status == -ENOMEM){
        //free memory
        for(i = 0 ; i <= index; i++){
            kfree(pTxData[i]);
            kfree(pTxDummyData[i]);
            kfree(pRxData[i]);
        }
    }

    if(status >= 0){
        DEBUGPRINT("=====================================================\n");
        DEBUGPRINT("     ADAM IO driver v%s installed\n", ADAMIO_VER);
        DEBUGPRINT("=====================================================\n");
    }

    return status;
}

static void __exit spi_adamIO_exit(void)
{
    int i;
    spi_unregister_driver(&spi_adamIO_driver);
    class_destroy(adamio_class);
    unregister_chrdev(SPI_ADAM_IO_MAJOR, spi_adamIO_driver.driver.name);

    for(i = 0 ; i < SPI_TRANSACTION_CHIP_SELECT_AMOUNT; i++){
        kfree(pTxData[i]);
        kfree(pTxDummyData[i]);
        kfree(pRxData[i]);
    }
}


module_init(spi_adamIO_init);
module_exit(spi_adamIO_exit);
MODULE_DESCRIPTION("ADVANTECH ADAM-IO SPI Driver for Qualcomm APQ-8016");
MODULE_LICENSE("GPL");
