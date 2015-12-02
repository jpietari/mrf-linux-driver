/*
  eeprom.c -- Micro-Research Event Receiver Linux 2.6 driver
              PCI9056 EEPROM functions

  Author: Jukka Pietarinen (MRF)
  Date:   27.1.2009

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

/* Descriptions for PCI9056 EEPROM fields */

char *EepromComment9056[] = {
  "Device ID",
  "Vendor ID",
  "Class Code",
  "Class Code / Revision",
  "PCI Maximum Latency / PCI Minimum Grant",
  "PCI Interrupt Pin / PCI Interrupt Line",
  "MSW of Mailbox 0",
  "LSW of Mailbox 0",
  "MSW of Mailbox 1",
  "LSW of Mailbox 1",
  "MSW of Direct Slave Local Address Space 0 Range",
  "LSW of Direct Slave Local Address Space 0 Range",
  "MSW of Direct Slave Local Address Space 0 Local Base Address (Remap)",
  "LSW of Direct Slave Local Address Space 0 Local Base Address (Remap)",
  "MSW of Mode/DMA Arbitration",
  "LSW of Mode/DMA Arbitration",
  "Local Miscellaneous Control 2 / Serial EEPROM Write-Protect Address Boundary",
  "Local Miscellaneous Control 1 / Local Bus Big/Little Endian Descriptor",
  "MSW of Direct Slave Expansion ROM Range",
  "LSW of Direct Slave Expansion ROM Range",
  "MSW of Direct Slave Expansion ROM Local Base Address (Remap) and BREQo Control",
  "LSW of Direct Slave Expansion ROM Local Base Address (Remap) and BREQo Control",
  "MSW of Local Address Space 0/Expansion ROM Bus Region Descriptor",
  "LSW of Local Address Space 0/Expansion ROM Bus Region Descriptor",
  "MSW of Local Range for Direct Master-to-PCI",
  "LSW of Local Range for Direct Master-to-PCI (Reserved)",
  "MSW of Local Base Address for Direct Master-to-PCI Memory",
  "LSW of Local Base Address for Direct Master-to-PCI Memory (Reserved)",
  "MSW of Local Base Address for Direct Master-to-PCI I/O Configuration",
  "LSW of Local Base Address for Direct Master-to-PCI I/O Configuration (Reserved)",
  "MSW of PCI Base Address (Remap) for Direct Master-to-PCI Memory",
  "LSW of PCI Base Address (Remap) for Direct Master-to-PCI Memory",
  "MSW of PCI Configuartion Address for Direct Master-to-PCI I/O Configuration",
  "LSW of PCI Configuartion Address for Direct Master-to-PCI I/O Configuration",
  "Subsystem ID",
  "Subsystem Vendor ID",
  "MSW of Direct Slave Local Address Space 1 Range (1 MB)",
  "LSW of Direct Slave Local Address Space 1 Range (1 MB)",
  "MSW of Direct Slave Local Address Space 1 Local Base Address (Remap)",
  "LSW of Direct Slave Local Address Space 1 Local Base Address (Remap)",
  "MSW of Local Address Space 1 Bus Region Descriptor",
  "LSW of Local Address Space 1 Bus Region Descriptor (Reserved)",
  "Hot Swap Control/Status (Reserved)",
  "Hot Swap Next Capability Pointer / Hot Swap Control",
  "Reserved",
  "PCI Arbiter Control",
  "Power Management Capabilities",
  "Power Management Next Capability Pointer (Reserved)/Power Management Capability ID (Reserved)",
  "Power Management Data / PMCSR Brdge Support Extension (Reserved)",
  "Power Management Control / Status",
  "S/N 0",
  "S/N 2",
  "S/N 4",
  "S/N 6",
  "S/N 8",
  "S/N A",
  NULL };

#define MRF_FCT_EEPROM_SN_BEGIN 50
#define MRF_FCT_EEPROM_SN_END   56

void eeprom_9056_cmd_start(volatile struct Pci9056LocalConf *pLC)
{
  /* CS high, SK low, DI low */
  pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET 
				      | PLX9056_CNTRL_EECS);
}

void eeprom_9056_cmd_stop(volatile struct Pci9056LocalConf *pLC)
{
  /* CS low, SK low, DI low */
  pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET);
}

/* This function is not writting the optimal way - the typical EEPROM
   programming cycle takes 3 ms according to datasheet. This function
   will take 10 - 20 ms depeding on the kernel timer setup */
int eeprom_9056_wait(volatile struct Pci9056LocalConf *pLC)
{
  int timeout_jiffies;
  wait_queue_head_t wq;
  
  /* We try to setup a delay of > 10 ms, but at least 2 jiffies which
     results in a 10-20 ms delay if HZ is 100. Ithis time the
     EEPROM should be ready */

  timeout_jiffies = HZ/100; /* 10 ms timeout */
  if (timeout_jiffies < 2)
    timeout_jiffies = 2;

  init_waitqueue_head(&wq);
  wait_event_interruptible_timeout(wq, 0, timeout_jiffies);

  if (pLC->CNTRL & __constant_cpu_to_le32(PLX9056_CNTRL_EEDO))
    return -ETIME;

  return 0;
}

int eeprom_9056_write_bits(volatile struct Pci9056LocalConf *pLC,
		       int command, int cmdlen)
{
  int bit, data;

  if (cmdlen < 1 || cmdlen > 32)
    return -EFAULT;

  data = command << (32 - cmdlen);

  for (bit = 0; bit < cmdlen; bit++)
    {
      if (data & 0x80000000)
	{
	  /* CS high, SK low, DI high */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET |
					      PLX9056_CNTRL_EECS |
					      PLX9056_CNTRL_EEDI);
	  udelay(1);
	  /* CS high, SK high, DI high */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET |
					      PLX9056_CNTRL_EESK |
					      PLX9056_CNTRL_EECS |
					      PLX9056_CNTRL_EEDI);
	  udelay(1);
	}
      else
	{
	  /* CS high, SK low, DI low */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET |
					      PLX9056_CNTRL_EECS);  
	  udelay(1);
	  /* CS high, SK high, DI low */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET |
					      PLX9056_CNTRL_EESK |
					      PLX9056_CNTRL_EECS);
	}
      data <<= 1;
    } 

  return 0;
}

int eeprom_9056_read_bits(volatile struct Pci9056LocalConf *pLC,
		      int *data, int readlen)
{
  int rddata, bit;

  if (readlen < 1 || readlen > 32)
    return -EFAULT;

  rddata = 0;

  pLC->CNTRL |= __constant_cpu_to_le32(PLX9056_CNTRL_EEDO_IE);

  for (bit = 0; bit < readlen; bit++)
    {
      rddata <<= 1;
      if (pLC->CNTRL & __constant_cpu_to_le32(PLX9056_CNTRL_EEDO))
	{
	  rddata |= 1;
	}
      /* CS high, SK low, DI low */
      pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET |
					  PLX9056_CNTRL_EECS |
					  PLX9056_CNTRL_EEDO_IE);
      udelay(1);
      /* CS high, SK high, DI low, clock in '0' */
      pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET |
					  PLX9056_CNTRL_EESK |
					  PLX9056_CNTRL_EECS |
					  PLX9056_CNTRL_EEDO_IE);
      udelay(1);
    }

  pLC->CNTRL &= __constant_cpu_to_le32(~PLX9056_CNTRL_EEDO_IE);

  *data = rddata;

  return 0;
}

int eeprom_9056_read(struct Pci9056LocalConf *pLC,
		unsigned short *eepromdata, unsigned int wordcount)
{
  int address, eeword;

  /* Limit maximum wordcount */
  if (wordcount > EEPROM_MAX_WORDS)
    wordcount = EEPROM_MAX_WORDS;

  eeprom_9056_cmd_start(pLC);
  eeprom_9056_write_bits(pLC, 0x600, 11);

  /* Now DO should be low, to show that EEPROM is there */
  eeprom_9056_read_bits(pLC, &eeword, 1);
  if (eeword & 1)
    {
      /* Return error if EEPROM missing/not accessible. */
      return -ENODEV;
    }

  /* Now read all EEProm words */
  for (address = 0; address < wordcount; address++)
    {
      eeprom_9056_read_bits(pLC, &eeword, 16);
      eepromdata[address] = eeword;
    }

  eeprom_9056_cmd_stop(pLC);
  
  /* Return number of words read. */
  return address;
}

int eeprom_9056_sprint(struct Pci9056LocalConf *pLC, char *buf, int size)
{
  unsigned short eepromdata[EEPROM_MAX_WORDS];
  int i, result, s;

  result = eeprom_9056_read(pLC, eepromdata, EEPROM_MAX_WORDS);
  if (result < EEPROM_MAX_WORDS)
    return result;

  s = 0;
  s = snprintf(&buf[s], size-s, "PCI9056 EEPROM Contents\nS/N: ");
  for (i = MRF_FCT_EEPROM_SN_BEGIN; i < MRF_FCT_EEPROM_SN_END; i++)
    {
      s += snprintf(&buf[s], size-s, "%c%c",
		    (eepromdata[i] >> 8) >= '0' &&  (eepromdata[i] >> 8) <= 'Z'
		    ? eepromdata[i] >> 8 : ' ', 
		    (eepromdata[i] & 0x0ff) >= '0' && 
		    (eepromdata[i] & 0x0ff) <= 'Z' ?
		    eepromdata[i] & 0x00ff : ' ');
    }
  s += snprintf(&buf[s], size-s, "\n\n");

  for (i = 0; EepromComment9056[i] != NULL; i++)
    if (s < size)
      s += snprintf(&buf[s], size-s, "0x%04x %s\n",
		    eepromdata[i], EepromComment9056[i]);

  /* Return the number of characters written if successful,
     if buffer is too small return 'no space left on device'. */
  if (s < size)
    return s;
  else
    return -ENOSPC;
}

int eeprom_9056_write(struct Pci9056LocalConf *pLC,
		 unsigned short *eepromdata, unsigned int wordcount)
{
  int address, eeword;
  int result = 0, timeout;

  /* Limit maximum wordcount */
  if (wordcount > EEPROM_MAX_WORDS)
    wordcount = EEPROM_MAX_WORDS;

  /* EWEN */
  eeprom_9056_cmd_start(pLC);
  eeprom_9056_write_bits(pLC, 0x4C0, 11);
  eeprom_9056_cmd_stop(pLC);

  for (address = 0; address < wordcount; address++)
    {
      /* Erase location first */
      eeprom_9056_cmd_start(pLC);
      eeprom_9056_write_bits(pLC, 0x700 | (address), 11);
      eeprom_9056_cmd_stop(pLC);
      eeprom_9056_cmd_start(pLC);
      timeout = eeprom_9056_wait(pLC);
      if (timeout)
	result = timeout;
      eeprom_9056_cmd_stop(pLC);

      /* Now program */
      eeprom_9056_cmd_start(pLC);
      eeprom_9056_write_bits(pLC, 0x500 | (address), 11);
      eeword = eepromdata[address];
      eeprom_9056_write_bits(pLC, eeword, 16);
      eeprom_9056_cmd_stop(pLC);
      eeprom_9056_cmd_start(pLC);
      timeout = eeprom_9056_wait(pLC);
      if (timeout)
	result = timeout;
      eeprom_9056_cmd_stop(pLC);
    }

  /* EWDS */
  eeprom_9056_cmd_start(pLC);
  eeprom_9056_write_bits(pLC, 0x400, 11);
  eeprom_9056_cmd_stop(pLC);

  return result;
}

int eeprom_9056_sscan(struct Pci9056LocalConf *pLC, char *buf)
{
  unsigned short eepromdata[EEPROM_MAX_WORDS];
  int datalen, result = 0;
  char *s;

  /* Here we find all lines starting with the number '0' which
     define the words to be written into the EEPROM. */

  datalen = 0;
  for (s = buf; *s && datalen < EEPROM_MAX_WORDS; )
    {
      if (*s == '0')
	eepromdata[datalen++] = simple_strtol(s, NULL, 0);
      do
	s++;
      /* Skip rest of line */
      while (s && *s != '\n');
      /* Skip empty lines */
      while (*s == '\n')
	s++;
    }

#ifdef DEBUG
  printk(KERN_INFO "eeprom_9056_sccan read %d words pLC = %08x.\n", datalen,
	 (unsigned int) pLC);
#endif

  if (datalen)
    {
      result = eeprom_9056_write(pLC, eepromdata, datalen);
      if (!result)
	result = datalen;
    }

  return result;
}
