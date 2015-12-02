/*
  flash.c -- Micro-Research Event Receiver TG Linux 2.6 driver
             M25P32/64 flash functions

  Author: Jukka Pietarinen (MRF)
  Date:   6.5.2010

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>     // Needed for the macros
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/sched.h>

#include <asm/uaccess.h>

#include "pci_mrfev.h"

int spi_wait_tmt(struct mrf_dev *ev_dev)
{
  volatile u32 *spi_control = ((ev_dev->pEv) + EV_SPI_CONTROL_OFFSET);
  int retry_count;
  int stat;

#if 0
  printk(KERN_INFO "spi_wait_tmt\n");
#endif

  for (retry_count = SPI_RETRY_COUNT; retry_count; retry_count--)
    if ((stat = be32_to_cpu(*spi_control)) & EV_SPI_CONTROL_TMT)
      break;

  if (retry_count == 0)
    {
      printk(KERN_INFO "spi_write TMT not asserted.\n");
      return -1;
    }
  return 0;
}

void spi_slave_select(struct mrf_dev *ev_dev, int select)
{
  volatile u32 *spi_control = ((ev_dev->pEv) + EV_SPI_CONTROL_OFFSET);
  int stat;

#if 0
  printk(KERN_INFO "spi_slave_select %d.\n", select);
#endif

  spi_wait_tmt(ev_dev);
  if (select)
    {
      *spi_control = be32_to_cpu(EV_SPI_CONTROL_OE);
      stat = be32_to_cpu(*spi_control);
      if ((stat & (EV_SPI_CONTROL_OE | EV_SPI_CONTROL_SELECT)) !=
	  EV_SPI_CONTROL_OE)
	printk(KERN_INFO "spi_slave select OE %02x\n", stat);
      *spi_control = be32_to_cpu(EV_SPI_CONTROL_OE | EV_SPI_CONTROL_SELECT);
      stat = be32_to_cpu(*spi_control);
      if ((stat & (EV_SPI_CONTROL_OE | EV_SPI_CONTROL_SELECT)) !=
	  (EV_SPI_CONTROL_OE | EV_SPI_CONTROL_SELECT))
	printk(KERN_INFO "spi_slave select OE & SELECT %02x\n", stat);
    }
  else
    {
      *spi_control = be32_to_cpu(EV_SPI_CONTROL_OE);
      stat = be32_to_cpu(*spi_control);
      if ((stat & (EV_SPI_CONTROL_OE | EV_SPI_CONTROL_SELECT)) !=
	  EV_SPI_CONTROL_OE)
	printk(KERN_INFO "spi_slave select OE %02x\n", stat);
      *spi_control = be32_to_cpu(0);
      stat = be32_to_cpu(*spi_control);
      if ((stat & (EV_SPI_CONTROL_OE | EV_SPI_CONTROL_SELECT)) !=
	  0)
	printk(KERN_INFO "spi_slave select %02x\n", stat);
    } 
}

int spi_wait_trdy(struct mrf_dev *ev_dev)
{
  volatile u32 *spi_control = ((ev_dev->pEv) + EV_SPI_CONTROL_OFFSET);
  int retry_count;
  int stat;

  for (retry_count = SPI_RETRY_COUNT; retry_count; retry_count--)
    if ((stat = be32_to_cpu(*spi_control)) & EV_SPI_CONTROL_TRDY)
      break;

  if (retry_count == 0)
    {
      printk(KERN_INFO "spi_write TRDY not asserted.\n");
      return -1;
    }
  return 0;
}

int spi_write(struct mrf_dev *ev_dev, int data)
{
  volatile u32 *spi_data = ((ev_dev->pEv) + EV_SPI_DATA_OFFSET);

#if 0
  printk(KERN_INFO "spi_write %02x.\n", data);
#endif

  if (spi_wait_trdy(ev_dev))
    return -1;
  
  *spi_data = be32_to_cpu(data);
  
  return (data & 0x00ff);
}

int spi_wait_rrdy(struct mrf_dev *ev_dev)
{
  volatile u32 *spi_control = ((ev_dev->pEv) + EV_SPI_CONTROL_OFFSET);
  int retry_count;
  int stat;

  for (retry_count = SPI_RETRY_COUNT; retry_count; retry_count--)
    if ((stat = be32_to_cpu(*spi_control)) & EV_SPI_CONTROL_RRDY)
      break;

  if (retry_count == 0)
    {
      printk(KERN_INFO "spi_write RRDY not asserted.\n");
      return -1;
    }
  return 0;
}

int spi_read(struct mrf_dev *ev_dev)
{
  volatile u32 *spi_data = ((ev_dev->pEv) + EV_SPI_DATA_OFFSET);
  int read_data = -1;

  if (spi_wait_rrdy(ev_dev))
    return -1;

  read_data = be32_to_cpu(*spi_data);

#if 0
  printk(KERN_INFO "spi_read %02x.\n", read_data);
#endif

  return read_data;
}

int flash_readstart(struct mrf_dev *ev_dev)
{
  int retval;

  spi_slave_select(ev_dev, 0);
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_FAST_READ);
  if (retval < 0)
    return retval;
  /* Three address bytes */
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    return retval;
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    return retval;
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    return retval;
  /* One dummy write + the first write that actually reads and starts
     the transfer of the first real byte */
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    return retval;
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    return retval;

  return 0;
}

int flash_readbyte(struct mrf_dev *ev_dev)
{
  int retval, read_data;

  /* Read byte shifted in by previous write operation */
  read_data = spi_read(ev_dev);
  if (read_data < 0)
    return read_data;
  /* Write to start next byte transfer */
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    return retval;

  return read_data;
}

void flash_readend(struct mrf_dev *ev_dev)
{
  spi_slave_select(ev_dev, 0);
}

int flash_read_status(struct mrf_dev *ev_dev)
{
  int retval;

  spi_slave_select(ev_dev, 0);
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    goto flash_read_status_end;
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_RDSR);
  if (retval < 0)
    goto flash_read_status_end;
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    goto flash_read_status_end;
  retval = spi_read(ev_dev);
  
 flash_read_status_end:
  spi_slave_select(ev_dev, 0);
  return retval;
}

int flash_fastread(struct mrf_dev *ev_dev,
		   char *data, unsigned int addr, unsigned int size)
{
  int retval, i;

  spi_slave_select(ev_dev, 0);
  retval = spi_write(ev_dev, 0);
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_FAST_READ);
  if (retval < 0)
    goto flash_fastread_out;
  /* Three address bytes */
  retval = spi_write(ev_dev, (addr >> 16) & 0x00ff);
  if (retval < 0)
    goto flash_fastread_out;
  retval = spi_write(ev_dev, (addr >> 8) & 0x00ff);
  if (retval < 0)
    goto flash_fastread_out;
  retval = spi_write(ev_dev, addr & 0x00ff);
  if (retval < 0)
    goto flash_fastread_out;
  /* One dummy write + the first write that actually reads and starts
     the transfer of the first real byte */
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    goto flash_fastread_out;
  retval = spi_write(ev_dev, 0);
  if (retval < 0)
    goto flash_fastread_out;

  for (i = size; i; i--)
    {
      retval = spi_read(ev_dev);
      if (retval < 0)
	goto flash_fastread_out;
      *(data++) = retval & 0x00ff;
      retval = spi_write(ev_dev, 0);
      if (retval < 0)
	goto flash_fastread_out;
    }

 flash_fastread_out:
  spi_slave_select(ev_dev, 0);
  return retval;
}

int flash_bulkerase(struct mrf_dev *ev_dev)
{
  int i, retval;

#if 0
  printk(KERN_INFO "flash_bulkerase\n");
#endif

  /* Dummy write with SS not active */
  spi_slave_select(ev_dev, 0);
  retval = spi_write(ev_dev, M25P_RDID);
  if (retval < 0)
    goto flash_bulkerase_end;

#if 0
  printk(KERN_INFO "flash_bulkerase 2\n");
#endif

  /* Write enable */
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_WREN);
  if (retval < 0)
    goto flash_bulkerase_end;
  spi_slave_select(ev_dev, 0);

#if 0
  printk(KERN_INFO "flash_bulkerase 3\n");
#endif

  /* Bulk erase */
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_BE);
  spi_slave_select(ev_dev, 0);
  if (retval < 0)
    goto flash_bulkerase_end;

#if 0
  printk(KERN_INFO "flash_bulkerase 4\n");
#endif

  for (i = 0; i < SPI_BE_COUNT; i++)
    {
      /* Bulk erasing takes a lot of time so we call schedule() to not
	 block the system for too long */
      schedule();
      if ((i % 100000) == 0)
	printk(KERN_INFO "flash_bulkerase %d\n", i);
      retval = flash_read_status(ev_dev);
      if (!(retval & M25P_STATUS_WIP) && retval >= 0)
	break;
    }

#if 0
  printk(KERN_INFO "flash_bulkerase status read %d times\n", i);
#endif
  if (i == SPI_BE_COUNT)
    retval = -1;

 flash_bulkerase_end:
    spi_slave_select(ev_dev, 0);

  return retval;

  /*
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_RDID);
  retval = spi_write(ev_dev, 0);
  for (i = 0; i < 20; i++)
    {
      s[i] = spi_read(ev_dev);
      retval = spi_write(ev_dev, 0);
    }

  spi_slave_select(ev_dev, 0);

  printk(KERN_INFO "flash read");
  for (i = 0; i < 20; i++)
    printk(" %02x", s[i]);
  printk("\n");

  return 0;
  */
}

int flash_primaryerase(struct mrf_dev *ev_dev)
{
  int sector, i, retval;

  for (sector = FLASH_SECTOR_PRIMARY_START; sector <= FLASH_SECTOR_PRIMARY_END;
       sector += FLASH_SECTOR_SIZE)
    {
#if 0
      printk(KERN_INFO "flash_primary erase\n");
#endif

      /* Dummy write with SS not active */
      spi_slave_select(ev_dev, 0);
      retval = spi_write(ev_dev, M25P_RDID);
      if (retval < 0)
	goto flash_primary_erase_end;
      
#if 0
      printk(KERN_INFO "flash_primary erase 2\n");
#endif
      
      /* Write enable */
      spi_slave_select(ev_dev, 1);
      retval = spi_write(ev_dev, M25P_WREN);
      if (retval < 0)
	goto flash_primary_erase_end;
      spi_slave_select(ev_dev, 0);
      
#if 0
      printk(KERN_INFO "flash_primary erase 3\n");
#endif

      /* Sector erase */
      spi_slave_select(ev_dev, 1);
      retval = spi_write(ev_dev, M25P_SE);
      if (retval < 0)
	goto flash_primary_erase_end;
      /* Three address bytes */
      retval = spi_write(ev_dev, (sector >> 16) & 0x00ff);
      if (retval < 0)
	goto flash_primary_erase_end;
      retval = spi_write(ev_dev, (sector >> 8) & 0x00ff);
      if (retval < 0)
	goto flash_primary_erase_end;
      retval = spi_write(ev_dev, sector & 0x00ff);
      if (retval < 0)
	goto flash_primary_erase_end;
      spi_slave_select(ev_dev, 0);

#if 0
      printk(KERN_INFO "flash_primary erase 4\n");
#endif

      for (i = 0; i < SPI_BE_COUNT; i++)
	{
	  /* Primary  erasing takes a lot of time so we call schedule() to not
	     block the system for too long */
	  schedule();
	  if ((i % 100000) == 99999)
	    printk(KERN_INFO "flash_primary erase sector %d, %d\n", sector, i+1);
	  retval = flash_read_status(ev_dev);
	  if (!(retval & M25P_STATUS_WIP) && retval >= 0)
	    break;
	}

      printk(KERN_INFO "flash_primary erase status read %d times\n", i);
#if 0
#endif
      if (i == SPI_BE_COUNT)
	{
	  retval = -1;
	  goto flash_primary_erase_end;
	}
    }

 flash_primary_erase_end:
    spi_slave_select(ev_dev, 0);

  return retval;

  /*
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_RDID);
  retval = spi_write(ev_dev, 0);
  for (i = 0; i < 20; i++)
    {
      s[i] = spi_read(ev_dev);
      retval = spi_write(ev_dev, 0);
    }

  spi_slave_select(ev_dev, 0);

  printk(KERN_INFO "flash read");
  for (i = 0; i < 20; i++)
    printk(" %02x", s[i]);
  printk("\n");

  return 0;
  */
}

int flash_pageprogram(struct mrf_dev *ev_dev, char *data,
		      int addr, int size)
{
  int i, retval;
  
  /* Check that page size and address valid */
  if (size != FLASH_DATA_ALLOC_SIZE)
    return -1;
  if ((addr & (FLASH_DATA_ALLOC_SIZE - 1)) != 0)
    return -1;

#if 0
  printk(KERN_INFO "flash_pp address %06x, size %d\n", addr, size);
#endif
  
  /* Dummy write with SS not active */
  spi_slave_select(ev_dev, 0);
  retval = spi_write(ev_dev, M25P_RDID);
  if (retval < 0)
    goto flash_pageprogram_end;

  /* Write enable */
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_WREN);
  if (retval < 0)
    goto flash_pageprogram_end;
  spi_slave_select(ev_dev, 0);

  /* Page program */
  spi_slave_select(ev_dev, 1);
  retval = spi_write(ev_dev, M25P_PP);
  if (retval < 0)
    goto flash_pageprogram_end;
  /* Three address bytes */
  retval = spi_write(ev_dev, (addr >> 16) & 0x00ff);
  if (retval < 0)
    goto flash_pageprogram_end;
  retval = spi_write(ev_dev, (addr >> 8) & 0x00ff);
  if (retval < 0)
    goto flash_pageprogram_end;
  retval = spi_write(ev_dev, addr & 0x00ff);
  if (retval < 0)
    goto flash_pageprogram_end;
  
  for (i = 0; i < FLASH_DATA_ALLOC_SIZE; i++)
    {
      retval = spi_write(ev_dev, data[i]);
      if (retval < 0)
	goto flash_pageprogram_end;
    }
  spi_slave_select(ev_dev, 0);

  for (i = 0; i < SPI_RETRY_COUNT; i++)
    {
      retval = flash_read_status(ev_dev);
      if (!(retval & M25P_STATUS_WIP))
	break;
    }

#if 0
  printk(KERN_INFO "flash_pp status read %d times\n", i);
#endif

  if (i == SPI_RETRY_COUNT)
    {
      printk(KERN_INFO "flash_pp status read exceeded retry count %d\n", i);
      retval = -1;
    }

 flash_pageprogram_end:
  spi_slave_select(ev_dev, 0);

  return retval;
}
