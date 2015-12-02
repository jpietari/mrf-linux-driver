/*
  jtag.c -- Micro-Research Event Receiver Linux 2.6 driver
            PCI9056 GPIO JTAG interface functions

  Author: Jukka Pietarinen (MRF)
  Date:   10.5.2010

*/

/*
#define JTAG_DEBUG 1
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

/* jtag_9056_init has to be called one time when the driver is beeing
   initialized. Here we clear the reference count to the JTAG pins.
   We only want to drive the JTAG pins when they are really needed
   this allows using the on-board JTAG connector for FPGA debugging
   purposes. */

void jtag_9056_init(struct mrf_dev *ev_dev)
{
  ev_dev->jtag_refcount = 0;
}

int jtag_9056_open(struct mrf_dev *ev_dev)
{
  volatile struct Pci9056LocalConf *pLC = ev_dev->pLC;

  printk(KERN_INFO "jtag_9056_open: refcount %d\n", ev_dev->jtag_refcount+1);
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_open: refcount %d\n", ev_dev->jtag_refcount+1);
#endif
  /* Keep track of how many times open is called.
     If JTAG is already open just increase reference count. */
  if (!ev_dev->jtag_refcount)
    {
      /* Enable TMS, TDI and TCK direction out */
      pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_JTAG);
    }
  ev_dev->jtag_refcount++;

  return ev_dev->jtag_refcount;
}

int jtag_9056_release(struct mrf_dev *ev_dev)
{
  volatile struct Pci9056LocalConf *pLC = ev_dev->pLC;

  printk(KERN_INFO "jtag_9056_release: refcount %d\n", ev_dev->jtag_refcount-1);
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_release: refcount %d\n", ev_dev->jtag_refcount-1);
#endif
  if (ev_dev->jtag_refcount > 0)
    ev_dev->jtag_refcount--;
  
  if (!ev_dev->jtag_refcount)
    {
      /* Disable TMS, TDI and TCK outputs */
      pLC->CNTRL = __constant_cpu_to_le32(PLX9056_CNTRL_RESET);
    }

  return ev_dev->jtag_refcount;
}

void jtag_9056_pulseTCK(volatile struct Pci9056LocalConf *pLC, int n)
{
  int gpioc;

  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TCK);
  for (; n; n--)
    {
      pLC->CNTRL = gpioc;
      pLC->CNTRL = gpioc | __constant_cpu_to_le32(PLX9056_CNTRL_TCK);
    } 
  pLC->CNTRL = gpioc;
}

void jtag_9056_TLR(volatile struct Pci9056LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_TLR\n");
#endif
  
  gpioc = pLC->CNTRL | __constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  pLC->CNTRL = gpioc;

  jtag_9056_pulseTCK(pLC, 5);
}

void jtag_9056_TLRtoRTI(volatile struct Pci9056LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_TLRtoRTI\n");
#endif

  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  pLC->CNTRL = gpioc;

  jtag_9056_pulseTCK(pLC, 1);
}

void jtag_9056_TLRtoSIR(volatile struct Pci9056LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_TLRtoSIR\n");
#endif

  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  pLC->CNTRL = gpioc;
  jtag_9056_pulseTCK(pLC, 1);
  pLC->CNTRL = gpioc | __constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  jtag_9056_pulseTCK(pLC, 2);
  pLC->CNTRL = gpioc;
  jtag_9056_pulseTCK(pLC, 2);
}

void jtag_9056_TLRtoSDR(volatile struct Pci9056LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_TLRtoSDR\n");
#endif

  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  pLC->CNTRL = gpioc;
  jtag_9056_pulseTCK(pLC, 1);
  pLC->CNTRL = gpioc | __constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  jtag_9056_pulseTCK(pLC, 1);
  pLC->CNTRL = gpioc;
  jtag_9056_pulseTCK(pLC, 2);
}

void jtag_9056_EX1toSDR(volatile struct Pci9056LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_EX1toSDR\n");
#endif

  jtag_9056_pulseTCK(pLC, 2);
  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  pLC->CNTRL = gpioc;
  jtag_9056_pulseTCK(pLC, 2);
}

void jtag_9056_EX1toRTI(volatile struct Pci9056LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_EX1toRTI\n");
#endif

  jtag_9056_pulseTCK(pLC, 1);
  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  pLC->CNTRL = gpioc;
  jtag_9056_pulseTCK(pLC, 1);
}

int jtag_9056_SDI(volatile struct Pci9056LocalConf *pLC, int n, int tdi,
	      int *tdo, int last)
{
  int i, gpioc, rdtdo = 0;

#ifdef JTAG_DEBUG
  char bits[33];

  for (i = 0; i < n; i++)
    bits[n-i-1] = '0' + ((tdi >> i) & 1);
  bits[i] = 0;
  
  printk(KERN_INFO "jtag_9056_SDI tdi %s, last %d\n", bits, last);
#endif

  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TDI);

  if (n < 1 || n > 32)
    return -EOVERFLOW;

  for (i = n; i; i--)
    {
      if (last && i == 1)
        gpioc |= __constant_cpu_to_le32(PLX9056_CNTRL_TMS);
      if (tdi & 1)
        pLC->CNTRL = gpioc | __constant_cpu_to_le32(PLX9056_CNTRL_TDI);
      else
        pLC->CNTRL = gpioc;
      rdtdo |= ((pLC->CNTRL & __constant_cpu_to_le32(PLX9056_CNTRL_TDO)) ?
                 1 : 0) << (32 - i);
      jtag_9056_pulseTCK(pLC, 1);
      tdi >>= 1;
    }

#ifdef JTAG_DEBUG
  for (i = 0; i < n; i++)
    bits[i] = '0' + (((rdtdo >> (31 - i)) & 1));
  printk(KERN_INFO "jtag_9056_SDI tdo %s\n", bits);
#endif

  *tdo = rdtdo;

  return 0;
}

void jtag_9056_SCFD(volatile struct Pci9056LocalConf *pLC, char tdi, int last)
{
  int gpioc, n;

#if 0
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_SCFD\n");
#endif
#endif
  gpioc = pLC->CNTRL & ~__constant_cpu_to_le32(PLX9056_CNTRL_TDI);
 
  for (n = 8; n; n--)
    {
      if (last && n == 1)
        gpioc |= __constant_cpu_to_le32(PLX9056_CNTRL_TMS);
      if (tdi & 0x80)
        pLC->CNTRL = gpioc | __constant_cpu_to_le32(PLX9056_CNTRL_TDI);
      else
        pLC->CNTRL = gpioc;
      jtag_9056_pulseTCK(pLC, 1);
      tdi <<= 1;
    }
}

int jtag_9056_validate(struct mrf_dev *ev_dev)
{
  volatile struct Pci9056LocalConf *pLC = ev_dev->pLC;

  int dr[3] = {0, 0, 0};
  int did[3] = {0, 0, 0};

  ev_dev->fpgairlen = 0;

  /* JTAG state Test Logic Reset (TLR) loads the 32 bit device ids into
     the devices' DR and we can recognize the devices based on the IDs */
  jtag_9056_TLR(pLC);
  jtag_9056_SDR(pLC, 0, 64, (char *) dr, 0, (char *) did, 1);

  if ((__cpu_to_be32(did[1]) & 0x0fffffff) == XC5FX70T_DID)
    ev_dev->fpgairlen = XC5FXT_IRLEN;
  ev_dev->fpgaid = __cpu_to_be32(did[1]);

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_");
#endif
  printk(KERN_INFO "jtag_9056_validate id_fpga: %08x\n",
	 ev_dev->fpgaid);

  /* If the chain has more than two devices or the devices are not
     recognized return error. */
  if (did[0] != 0 || ev_dev->fpgairlen == 0)
    {
      return -EIO;
    }

  return 0;
}

/*
  jtag_SIR - JTAG Shift Instruction Register

  hir    - IR chain header length in bits for devices in chain to be
           bypassed
  irlen  - Instruction Register length
  tdi    - Instruction Code to be loaded
  tir    - IR chain trailer length in bits for devices in chain to be
           bypassed
  tdo    - pointer to TDO output or NULL if not required
  tlr    - set to true if end state is TLR, else EXIT1
*/

int jtag_9056_SIR(volatile struct Pci9056LocalConf *pLC, int hir, int irlen,
	     int tdi, int tir, int *tdo, int tlr)
{
  int irout;

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_SIR\n");
#endif
  if (hir < 0 || hir > 32 || tir < 0 || tir > 32 || irlen < 1 || irlen > 32)
    return -EOVERFLOW;

  jtag_9056_TLRtoSIR(pLC);
  if (hir > 0)
    jtag_9056_SDI(pLC, hir, 0xffffffff, &irout, 0);

  jtag_9056_SDI(pLC, irlen, tdi, &irout, !tir);
  if (tdo != NULL)
    *tdo = irout;

  if (tir > 0)
    jtag_9056_SDI(pLC, tir, 0xffffffff, &irout, 1);
  
  if (tlr)
    jtag_9056_EX1toRTI(pLC);

  return 0;
}

int jtag_9056_SDR(volatile struct Pci9056LocalConf *pLC, int hdr, int drlen,
	      char *tdi, int tdr, char *tdo, int tlr)
{
  int drout;
  int bytes = (drlen + 7) >> 3;
  int i, dr, last;

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9056_SDR\n");
#endif
  if (hdr < 0 || hdr > 32|| tdr < 0 || tdr > 32 || drlen < 0)
    return -EOVERFLOW;

  jtag_9056_TLRtoSDR(pLC);
  if (hdr > 0)
    jtag_9056_SDI(pLC, hdr, 0, &drout, 0);

  dr = 8;
  last = 0;
  for (i = bytes - 1; i >= 0; i--)
    {
      if (i == 0)
        {
          if (drlen & 0x7)
            dr = (drlen & 0x7);
          if (tdr == 0)
            last = 1;
        }
      jtag_9056_SDI(pLC, dr, tdi[i], &drout, last);
      if (tdo != NULL)
        tdo[i] = (drout >> (32 - dr));
    }

  if (tdr > 0)
    jtag_9056_SDI(pLC, tdr, 0, &drout, 1);

  if (tlr)
    jtag_9056_EX1toRTI(pLC);

  return 0;
}

/* Read FPGA status register (see Configuration Details in Xilinx
   Virtex-II Pro Users Guide, ug012.pdf) */

int jtag_9056_read_fpga_status(struct mrf_dev *ev_dev)
{
  volatile struct Pci9056LocalConf *pLC = ev_dev->pLC;

  int result;
  char dout[40];
  int tdo, bitrev, fpgastat, i;
  char dinRDSTAT[] = {0x00, 0x00, 0x00, 0x00,
                      0x80, 0x07, 0x00, 0x14,
                      0x00, 0x00, 0x00, 0x04,
                      0x66, 0xaa, 0x99, 0x55,
                      0xff, 0xff, 0xff, 0xff};
  /*
  char dinRCRC[] = {0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                    0xe0, 0x00, 0x00, 0x00,
                    0x80, 0x01, 0x00, 0x0c};
  */
  jtag_9056_open(ev_dev);

  jtag_9056_TLR(pLC);

  result = jtag_9056_validate(ev_dev);
  if (result < 0)
    goto out;
  
  /* Load cgf_in */
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, 0, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);
  jtag_9056_SDR(pLC, 0, 192, dinRDSTAT, 0, dout, 0);
  jtag_9056_EX1toRTI(pLC);

  /* Load cfg_out */
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGOUT, 0, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);
  jtag_9056_SDR(pLC, 0, 32, dinRDSTAT, 0, dout, 0);
  bitrev = ((dout[0] & 0x00ff) << 24) +
    ((dout[1] & 0x00ff) << 16) +
    ((dout[2] & 0x00ff) << 8) +
    (dout[3] & 0x00ff);
  jtag_9056_EX1toRTI(pLC);
  /* Reverse status register bits read out */
  fpgastat = 0;
  for (i = 0; i < 32; i++)
    {
      fpgastat <<= 1;
      fpgastat |=  bitrev & 1;
      bitrev >>= 1;
    }

  /* Load cgf_in */
#if 0
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, XCFXXP_IRLEN, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);
  jtag_9056_SDR(pLC, 0, 128, dinRCRC, 1, dout, 1);
#endif

  jtag_9056_TLR(pLC);

  result = fpgastat;

 out:
  jtag_9056_release(ev_dev);
  return result;
}

/* Configure FPGA from .bit file.
   Startup clock has to be set to JTAG in bitgen. */

int jtag_9056_init_load_fpga(struct mrf_dev *ev_dev)
{
  volatile struct Pci9056LocalConf *pLC = ev_dev->pLC;

  int tdo;
  /*
  char dout[40];
  char dinRCRC[] = {0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                    0xe0, 0x00, 0x00, 0x00,
                    0x80, 0x01, 0x00, 0x0c,
                    0x66, 0xaa, 0x99, 0x55,
                    0xff, 0xff, 0xff, 0xff};
  char dinAGHIGH[] = {0x10, 0x00, 0x00, 0x00,
                      0x80, 0x01, 0x00, 0x0c};
  */
  int i;
  int result;

#ifdef JTAG_DEBUG
      printk(KERN_INFO "jtag_9056_init_load_fpga\n");
#endif

  jtag_9056_open(ev_dev);

  jtag_9056_TLR(pLC);
  result = jtag_9056_validate(ev_dev);
  if (result < 0)
    {
      jtag_9056_release(ev_dev);
      return result;
    }

  /* Load jprogram */
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_JPROG_B, 0, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);

  for (i = 100; i; i--)
    {
      /* Load cfg_in */
      jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, 0, &tdo, 0);
      jtag_9056_EX1toRTI(pLC);
      
      jtag_9056_pulseTCK(pLC, 100000);
      
      /* Load cfg_in */
      jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, 0, &tdo, 0);
      jtag_9056_EX1toRTI(pLC);
#ifdef JTAG_DEBUG
	  printk(KERN_INFO "jtag_9056_init_load_fpga INIT_COMPLETE %04x\n", tdo);
#endif
      if ((tdo & 0x04000000) == 0x04000000)
	break;
    }

  if ((tdo & 0x04000000) != 0x04000000)
    {
#ifdef JTAG_DEBUG
      printk(KERN_INFO "jtag_9056_init_load_fpga INIT_COMPLETE %04x\n", tdo);
#endif
      jtag_9056_release(ev_dev);
      return result;
    }

  jtag_9056_TLR(pLC);

  /* Load cgf_in */
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, 0, &tdo, 0);
  jtag_9056_EX1toSDR(pLC);

  return 0;
}

int jtag_9056_end_load_fpga(struct mrf_dev *ev_dev)
{
  volatile struct Pci9056LocalConf *pLC = ev_dev->pLC;

  int tdo;

#ifdef JTAG_DEBUG
      printk(KERN_INFO "jtag_9056_end_load_fpga\n");
#endif

  /* Last bit */
  pLC->CNTRL |= __constant_cpu_to_le32(PLX9056_CNTRL_TMS);
  jtag_9056_pulseTCK(pLC, 1);
  jtag_9056_EX1toRTI(pLC);

  /* Load jstart */
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_JSTART, 0, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);
  jtag_9056_pulseTCK(pLC, 1200);

  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_BYPASS, 0, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);
  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_BYPASS, 0, &tdo, 0);
  jtag_9056_EX1toRTI(pLC);

  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_JSTART, 0, &tdo, 0);
  jtag_9056_TLRtoRTI(pLC);
  jtag_9056_pulseTCK(pLC, 1200);

  jtag_9056_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_BYPASS, 0, &tdo, 0);

  if (!(tdo & 0x20))
    {
#ifdef JTAG_DEBUG
      printk(KERN_INFO "jtag_9056_end_load_fpga DONE low %04x\n", tdo);
#endif
    }

  jtag_9056_TLR(pLC);
  jtag_9056_release(ev_dev);

  return 0;
}

int fpga_9056_sprint(struct mrf_dev *ev_dev, char *buf, int size)
{
  int fpgastat = 0, s;

  fpgastat = jtag_9056_read_fpga_status(ev_dev);
  if (fpgastat < 0)
    return fpgastat;

  s = 0;
  s += snprintf(&buf[s], size-s, "FPGA status\n");
  s += snprintf(&buf[s], size-s, "DONE      %d\n",
		fpgastat & 0x1000 ? 1 : 0);
  s += snprintf(&buf[s], size-s, "INIT      %d\n",
		fpgastat & 0x0800 ? 1 : 0);
  s += snprintf(&buf[s], size-s, "MODE      %d%d%d\n",
		fpgastat & 0x0400 ? 1 : 0,
         fpgastat & 0x0200 ? 1 : 0, fpgastat & 0x0100 ? 1 : 0);
  s += snprintf(&buf[s], size-s, "GHIGH_B   %s\n",
		fpgastat & 0x0080 ? "deasserted" : "asserted");
  s += snprintf(&buf[s], size-s, "GWE       %d\n",
		fpgastat & 0x0040 ? 1 : 0);
  s += snprintf(&buf[s], size-s, "DCI_MATCH %d\n",
		fpgastat & 0x0008 ? 1 : 0);
  s += snprintf(&buf[s], size-s, "DCM_LOCK  %d\n",
		fpgastat & 0x0004 ? 1 : 0);
  s += snprintf(&buf[s], size-s, "CRC_ERROR %d\n",
		fpgastat & 0x0001 ? 1 : 0);

  /* Return the number of characters written if successful,
     if buffer is too small return 'no space left on device'. */
  if (s < size)
    return s;
  else
    return -ENOSPC;
}
