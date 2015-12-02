/*
  eeprom.c -- Micro-Research Event Receiver Linux 2.6 driver
              PCI9030 EEPROM functions

  Author: Jukka Pietarinen (MRF)
  Date:   30.11.2006

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>     // Needed for the macros
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/sched.h>

#include <asm/uaccess.h>

#include "pci_mrfev.h"

/* Descriptions for PCI9030 EEPROM fields */

char *EepromComment9030[] = {
  "Device ID",
  "Vendor ID",
  "PCI Status",
  "PCI Command",
  "Class Code",
  "Class Code / Revision",
  "Subsystem ID",
  "Subsystem Vendor ID",
  "MSB New Capability Pointer",
  "LSB New Capability Pointer",
  "(Maximum Latency and Minimum Grant are not loadable)",
  "Interrupt Pin (Interrupt Line Routing is not loadable)",
  "MSW of Power Management Capabilities",
  "LSW of Power Management Next Capability Pointer",
  "MSW of Power Management Data / PMCSR Brdge Support Extension",
  "LSW of Power Management Control / Status",
  "MSW of Hot Swap Control / Status",
  "LSW of Hot Swap Next Capability Pointer / Hot Swap Control",
  "PCI Vital Product Data Address",
  "PCI Vital Product Data Next Capability Pointer",
  "MSW of Local Address Space 0 Range",
  "LSW of Local Address Space 0 Range",
  "MSW of Local Address Space 1 Range",
  "LSW of Local Address Space 1 Range",
  "MSW of Local Address Space 2 Range",
  "LSW of Local Address Space 2 Range",
  "MSW of Local Address Space 3 Range",
  "LSW of Local Address Space 3 Range",
  "MSW of Expansion ROM Range", 
  "LSW of Expansion ROM Range",
  "MSW of Local Address Space 0 Local Base Address (Remap)",
  "LSW of Local Address Space 0 Local Base Address (Remap)",
  "MSW of Local Address Space 1 Local Base Address (Remap)",
  "LSW of Local Address Space 1 Local Base Address (Remap)",
  "MSW of Local Address Space 2 Local Base Address (Remap)",
  "LSW of Local Address Space 2 Local Base Address (Remap)",
  "MSW of Local Address Space 3 Local Base Address (Remap)",
  "LSW of Local Address Space 3 Local Base Address (Remap)",
  "MSW of Expansion ROM Local Base Address (Remap)",
  "LSW of Expansion ROM Local Base Address (Remap)",
  "MSW of Local Address Space 0 Bus Region Descriptor",
  "LSW of Local Address Space 0 Bus Region Descriptor",
  "MSW of Local Address Space 1 Bus Region Descriptor",
  "LSW of Local Address Space 1 Bus Region Descriptor",
  "MSW of Local Address Space 2 Bus Region Descriptor",
  "LSW of Local Address Space 2 Bus Region Descriptor",
  "MSW of Local Address Space 3 Bus Region Descriptor",
  "LSW of Local Address Space 3 Bus Region Descriptor",
  "MSW of Expansion ROM Bus Region Descriptor",
  "LSW of Expansion ROM Bus Region Descriptor",
  "MSW of Chip Select 0 Base Address",
  "LSW of Chip Select 0 Base Address",
  "MSW of Chip Select 1 Base Address",
  "LSW of Chip Select 1 Base Address",
  "MSW of Chip Select 2 Base Address",
  "LSW of Chip Select 2 Base Address",
  "MSW of Chip Select 3 Base Address",
  "LSW of Chip Select 3 Base Address",
  "Serial EEPROM Write-Protected Address Boundary",
  "LSW of Interrupt Control/Status",
  "MSW of PCI Target Response, Serial EEPROM, and Initialization Control",
  "LSW of PCI Target Response, Serial EEPROM, and Initialization Control",
  "MSW of General Purpose I/O Control",
  "LSW of General Purpose I/O Control",
  "MSW of Hidden 1 Power Management Data Select",
  "LSW of Hidden 1 Power Management Data Select",
  "MSW of Hidden 2 Power Management Data Select",
  "LSW of Hidden 2 Power Management Data Select",
  "S/N 0",
  "S/N 2",
  "S/N 4",
  "S/N 6",
  "S/N 8",
  "S/N A",
  NULL };

#define MRF_EVX_EEPROM_SN_BEGIN 68
#define MRF_EVX_EEPROM_SN_END   74

void eeprom_9030_cmd_start(volatile struct Pci9030LocalConf *pLC)
{
  /* CS high, SK low, DI low */
  pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET 
				      | PLX9030_CNTRL_EECS);
}

void eeprom_9030_cmd_stop(volatile struct Pci9030LocalConf *pLC)
{
  /* CS low, SK low, DI low */
  pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET);
}

/* This function is not writting the optimal way - the typical EEPROM
   programming cycle takes 3 ms according to datasheet. This function
   will take 10 - 20 ms depeding on the kernel timer setup */
int eeprom_9030_wait(volatile struct Pci9030LocalConf *pLC)
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

  if (pLC->CNTRL & __constant_cpu_to_le32(PLX9030_CNTRL_EEDO))
    return -ETIME;

  return 0;
}

int eeprom_9030_write_bits(volatile struct Pci9030LocalConf *pLC,
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
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET |
					      PLX9030_CNTRL_EECS |
					      PLX9030_CNTRL_EEDI);
	  udelay(1);
	  /* CS high, SK high, DI high */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET |
					      PLX9030_CNTRL_EESK |
					      PLX9030_CNTRL_EECS |
					      PLX9030_CNTRL_EEDI);
	  udelay(1);
	}
      else
	{
	  /* CS high, SK low, DI low */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET |
					      PLX9030_CNTRL_EECS);  
	  udelay(1);
	  /* CS high, SK high, DI low */
	  pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET |
					      PLX9030_CNTRL_EESK |
					      PLX9030_CNTRL_EECS);
	}
      data <<= 1;
    } 

  return 0;
}

int eeprom_9030_read_bits(volatile struct Pci9030LocalConf *pLC,
		      int *data, int readlen)
{
  int rddata, bit;

  if (readlen < 1 || readlen > 32)
    return -EFAULT;

  rddata = 0;

  for (bit = 0; bit < readlen; bit++)
    {
      rddata <<= 1;
      if (pLC->CNTRL & __constant_cpu_to_le32(PLX9030_CNTRL_EEDO))
	{
	  rddata |= 1;
	}
      /* CS high, SK low, DI low */
      pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET |
					  PLX9030_CNTRL_EECS);
      udelay(1);
      /* CS high, SK high, DI low, clock in '0' */
      pLC->CNTRL = __constant_cpu_to_le32(PLX9030_CNTRL_RESET |
					  PLX9030_CNTRL_EESK |
					  PLX9030_CNTRL_EECS);
      udelay(1);
    }

  *data = rddata;

  return 0;
}

int eeprom_9030_read(struct Pci9030LocalConf *pLC,
		unsigned short *eepromdata, unsigned int wordcount)
{
  int address, eeword;

  /* Limit maximum wordcount */
  if (wordcount > EEPROM_MAX_WORDS)
    wordcount = EEPROM_MAX_WORDS;

  eeprom_9030_cmd_start(pLC);
  eeprom_9030_write_bits(pLC, 0x600, 11);

  /* Now DO should be low, to show that EEPROM is there */
  eeprom_9030_read_bits(pLC, &eeword, 1);
  if (eeword & 1)
    {
      /* Return error if EEPROM missing/not accessible. */
      return -ENODEV;
    }

  /* Now read all EEProm words */
  for (address = 0; address < wordcount; address++)
    {
      eeprom_9030_read_bits(pLC, &eeword, 16);
      eepromdata[address] = eeword;
    }

  eeprom_9030_cmd_stop(pLC);
  
  /* Return number of words read. */
  return address;
}

int eeprom_9030_sprint(struct Pci9030LocalConf *pLC, char *buf, int size)
{
  unsigned short eepromdata[EEPROM_MAX_WORDS];
  int i, result, s;

  result = eeprom_9030_read(pLC, eepromdata, EEPROM_MAX_WORDS);
  if (result < EEPROM_MAX_WORDS)
    return result;

  s = 0;
  s = snprintf(&buf[s], size-s, "PCI9030 EEPROM Contents\nS/N: ");
  for (i = MRF_EVX_EEPROM_SN_BEGIN; i < MRF_EVX_EEPROM_SN_END; i++)
    {
      s += snprintf(&buf[s], size-s, "%c%c",
		    (eepromdata[i] >> 8) >= '0' &&  (eepromdata[i] >> 8) <= 'Z'
		    ? eepromdata[i] >> 8 : ' ', 
		    (eepromdata[i] & 0x0ff) >= '0' && 
		    (eepromdata[i] & 0x0ff) <= 'Z' ?
		    eepromdata[i] & 0x00ff : ' ');
    }
  s += snprintf(&buf[s], size-s, "\n\n");

  for (i = 0; EepromComment9030[i] != NULL; i++)
    if (s < size)
      s += snprintf(&buf[s], size-s, "0x%04x %s\n",
		    eepromdata[i], EepromComment9030[i]);

  /* Return the number of characters written if successful,
     if buffer is too small return 'no space left on device'. */
  if (s < size)
    return s;
  else
    return -ENOSPC;
}

int eeprom_9030_write(struct Pci9030LocalConf *pLC,
		 unsigned short *eepromdata, unsigned int wordcount)
{
  int address, eeword;
  int result = 0, timeout;

  /* Limit maximum wordcount */
  if (wordcount > EEPROM_MAX_WORDS)
    wordcount = EEPROM_MAX_WORDS;

  /* EWEN */
  eeprom_9030_cmd_start(pLC);
  eeprom_9030_write_bits(pLC, 0x4C0, 11);
  eeprom_9030_cmd_stop(pLC);

  for (address = 0; address < wordcount; address++)
    {
      /* Erase location first */
      eeprom_9030_cmd_start(pLC);
      eeprom_9030_write_bits(pLC, 0x700 | (address), 11);
      eeprom_9030_cmd_stop(pLC);
      eeprom_9030_cmd_start(pLC);
      timeout = eeprom_9030_wait(pLC);
      if (timeout)
	result = timeout;
      eeprom_9030_cmd_stop(pLC);

      /* Now program */
      eeprom_9030_cmd_start(pLC);
      eeprom_9030_write_bits(pLC, 0x500 | (address), 11);
      eeword = eepromdata[address];
      eeprom_9030_write_bits(pLC, eeword, 16);
      eeprom_9030_cmd_stop(pLC);
      eeprom_9030_cmd_start(pLC);
      timeout = eeprom_9030_wait(pLC);
      if (timeout)
	result = timeout;
      eeprom_9030_cmd_stop(pLC);
    }

  /* EWDS */
  eeprom_9030_cmd_start(pLC);
  eeprom_9030_write_bits(pLC, 0x400, 11);
  eeprom_9030_cmd_stop(pLC);

  return result;
}

int eeprom_9030_sscan(struct Pci9030LocalConf *pLC, char *buf)
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
  printk(KERN_INFO "eeprom_9030_sccan read %d words pLC = %08x.\n", datalen,
	 (unsigned int) pLC);
#endif

  if (datalen)
    {
      result = eeprom_9030_write(pLC, eepromdata, datalen);
      if (!result)
	result = datalen;
    }

  return result;
}
