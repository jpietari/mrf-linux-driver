/*
  jtag.c -- Micro-Research Event Receiver Linux 2.6 driver
            PCI9030 GPIO JTAG interface functions

  Author: Jukka Pietarinen (MRF)
  Date:   1.12.2006

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
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/sched.h>

#include <asm/uaccess.h>

#include "pci_mrfev.h"

/* jtag_9030_init has to be called one time when the driver is beeing
   initialized. Here we clear the reference count to the JTAG pins.
   We only want to drive the JTAG pins when they are really needed
   this allows using the on-board JTAG connector for FPGA debugging
   purposes. */

void jtag_9030_init(struct mrf_dev *ev_dev)
{
  ev_dev->jtag_refcount = 0;
}

int jtag_9030_open(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  printk(KERN_INFO "jtag_9030open: refcount %d\n", ev_dev->jtag_refcount+1);
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030open: refcount %d\n", ev_dev->jtag_refcount+1);
#endif
  /* Keep track of how many times open is called.
     If JTAG is already open just increase reference count. */
  if (!ev_dev->jtag_refcount)
    {
      /* Enable TMS, TDI and TCK direction out */
      pLC->GPIOC = __constant_cpu_to_le32(PLX9030_GPIOC_JTAG);
    }
  ev_dev->jtag_refcount++;

  return ev_dev->jtag_refcount;
}

int jtag_9030_release(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  printk(KERN_INFO "jtag_9030release: refcount %d\n", ev_dev->jtag_refcount-1);
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030release: refcount %d\n", ev_dev->jtag_refcount-1);
#endif
  if (ev_dev->jtag_refcount > 0)
    ev_dev->jtag_refcount--;
  
  if (!ev_dev->jtag_refcount)
    {
      /* Disable TMS, TDI and TCK outputs */
      pLC->GPIOC = __constant_cpu_to_le32(PLX9030_GPIOC_RESET);
    }

  return ev_dev->jtag_refcount;
}

void jtag_9030_pulseTCK(volatile struct Pci9030LocalConf *pLC, int n)
{
  int gpioc;

  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TCK);
  for (; n; n--)
    {
      pLC->GPIOC = gpioc;
      pLC->GPIOC = gpioc | __constant_cpu_to_le32(PLX9030_GPIOC_TCK);
    } 
  pLC->GPIOC = gpioc;
}

void jtag_9030_TLR(volatile struct Pci9030LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030TLR\n");
#endif
  
  gpioc = pLC->GPIOC | __constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  pLC->GPIOC = gpioc;

  jtag_9030_pulseTCK(pLC, 5);
}

void jtag_9030_TLRtoRTI(volatile struct Pci9030LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030TLRtoRTI\n");
#endif

  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  pLC->GPIOC = gpioc;

  jtag_9030_pulseTCK(pLC, 1);
}

void jtag_9030_TLRtoSIR(volatile struct Pci9030LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030TLRtoSIR\n");
#endif

  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  pLC->GPIOC = gpioc;
  jtag_9030_pulseTCK(pLC, 1);
  pLC->GPIOC = gpioc | __constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  jtag_9030_pulseTCK(pLC, 2);
  pLC->GPIOC = gpioc;
  jtag_9030_pulseTCK(pLC, 2);
}

void jtag_9030_TLRtoSDR(volatile struct Pci9030LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030TLRtoSDR\n");
#endif

  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  pLC->GPIOC = gpioc;
  jtag_9030_pulseTCK(pLC, 1);
  pLC->GPIOC = gpioc | __constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  jtag_9030_pulseTCK(pLC, 1);
  pLC->GPIOC = gpioc;
  jtag_9030_pulseTCK(pLC, 2);
}

void jtag_9030_EX1toSDR(volatile struct Pci9030LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030_EX1toSDR\n");
#endif

  jtag_9030_pulseTCK(pLC, 2);
  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  pLC->GPIOC = gpioc;
  jtag_9030_pulseTCK(pLC, 2);
}

void jtag_9030_EX1toRTI(volatile struct Pci9030LocalConf *pLC)
{
  int gpioc;
#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030_EX1toRTI\n");
#endif

  jtag_9030_pulseTCK(pLC, 1);
  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  pLC->GPIOC = gpioc;
  jtag_9030_pulseTCK(pLC, 1);
}

int jtag_9030_SDI(volatile struct Pci9030LocalConf *pLC, int n, int tdi,
	      int *tdo, int last)
{
  int i, gpioc, rdtdo = 0;

#ifdef JTAG_DEBUG
  char bits[33];

  for (i = 0; i < n; i++)
    bits[n-i-1] = '0' + ((tdi >> i) & 1);
  bits[i] = 0;
  
  printk(KERN_INFO "jtag_9030_SDI tdi %s, last %d\n", bits, last);
#endif

  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TDI);

  if (n < 1 || n > 32)
    return -EOVERFLOW;

  for (i = n; i; i--)
    {
      if (last && i == 1)
        gpioc |= __constant_cpu_to_le32(PLX9030_GPIOC_TMS);
      if (tdi & 1)
        pLC->GPIOC = gpioc | __constant_cpu_to_le32(PLX9030_GPIOC_TDI);
      else
        pLC->GPIOC = gpioc;
      rdtdo |= ((pLC->GPIOC & __constant_cpu_to_le32(PLX9030_GPIOC_TDO)) ?
                 1 : 0) << (32 - i);
      jtag_9030_pulseTCK(pLC, 1);
      tdi >>= 1;
    }

#ifdef JTAG_DEBUG
  for (i = 0; i < n; i++)
    bits[i] = '0' + (((rdtdo >> (31 - i)) & 1));
  printk(KERN_INFO "jtag_9030_SDI tdo %s\n", bits);
#endif

  *tdo = rdtdo;

  return 0;
}

void jtag_9030_SCFD(volatile struct Pci9030LocalConf *pLC, char tdi, int last)
{
  int gpioc, n;

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030_SCFD\n");
#endif
  gpioc = pLC->GPIOC & ~__constant_cpu_to_le32(PLX9030_GPIOC_TDI);
 
  for (n = 8; n; n--)
    {
      if (last && n == 1)
        gpioc |= __constant_cpu_to_le32(PLX9030_GPIOC_TMS);
      if (tdi & 0x80)
        pLC->GPIOC = gpioc | __constant_cpu_to_le32(PLX9030_GPIOC_TDI);
      else
        pLC->GPIOC = gpioc;
      jtag_9030_pulseTCK(pLC, 1);
      tdi <<= 1;
    }
}

int jtag_9030_validate(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int did[3] = {0, 0, 0};

  ev_dev->fpgairlen = 0;
  ev_dev->xcfirlen = 0;

  /* JTAG state Test Logic Reset (TLR) loads the 32 bit device ids into
     the devices' DR and we can recognize the devices based on the IDs */
  jtag_9030_TLR(pLC);
  jtag_9030_SDR(pLC, 0, 96, (char *) dr, 0, (char *) did, 1);

  if ((__cpu_to_be32(did[2]) & 0x0fffffff) == XC2VP4_DID)
    ev_dev->fpgairlen = XC2VP4_IRLEN;
  ev_dev->fpgaid = __cpu_to_be32(did[2]);

  if ((__cpu_to_be32(did[1]) & 0x0fffffff) == XCF08P_DID)
    ev_dev->xcfirlen = XCFXXP_IRLEN;
  ev_dev->xcfid = __cpu_to_be32(did[1]);

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030_");
#endif
  printk(KERN_INFO "jtag_9030_validate id_fpga: %08x, id_xcf: %08x\n",
	 ev_dev->fpgaid, ev_dev->xcfid);

  /* If the chain has more than two devices or the devices are not
     recognized return error. */
  if (did[0] != 0 || ev_dev->fpgairlen == 0 || ev_dev->xcfirlen == 0)
    {
      return -EIO;
    }

  return 0;
}

/*
  jtag_9030_SIR - JTAG Shift Instruction Register

  hir    - IR chain header length in bits for devices in chain to be
           bypassed
  irlen  - Instruction Register length
  tdi    - Instruction Code to be loaded
  tir    - IR chain trailer length in bits for devices in chain to be
           bypassed
  tdo    - pointer to TDO output or NULL if not required
  tlr    - set to true if end state is TLR, else EXIT1
*/

int jtag_9030_SIR(volatile struct Pci9030LocalConf *pLC, int hir, int irlen,
	     int tdi, int tir, int *tdo, int tlr)
{
  int irout;

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030_SIR\n");
#endif
  if (hir < 0 || hir > 32 || tir < 0 || tir > 32 || irlen < 1 || irlen > 32)
    return -EOVERFLOW;

  jtag_9030_TLRtoSIR(pLC);
  if (hir > 0)
    jtag_9030_SDI(pLC, hir, 0xffffffff, &irout, 0);

  jtag_9030_SDI(pLC, irlen, tdi, &irout, !tir);
  if (tdo != NULL)
    *tdo = irout;

  if (tir > 0)
    jtag_9030_SDI(pLC, tir, 0xffffffff, &irout, 1);
  
  if (tlr)
    jtag_9030_EX1toRTI(pLC);

  return 0;
}

int jtag_9030_SDR(volatile struct Pci9030LocalConf *pLC, int hdr, int drlen,
	      char *tdi, int tdr, char *tdo, int tlr)
{
  int drout;
  int bytes = (drlen + 7) >> 3;
  int i, dr, last;

#ifdef JTAG_DEBUG
  printk(KERN_INFO "jtag_9030_SDR\n");
#endif
  if (hdr < 0 || hdr > 32|| tdr < 0 || tdr > 32 || drlen < 0)
    return -EOVERFLOW;

  jtag_9030_TLRtoSDR(pLC);
  if (hdr > 0)
    jtag_9030_SDI(pLC, hdr, 0, &drout, 0);

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
      jtag_9030_SDI(pLC, dr, tdi[i], &drout, last);
      if (tdo != NULL)
        tdo[i] = (drout >> (32 - dr));
    }

  if (tdr > 0)
    jtag_9030_SDI(pLC, tdr, 0, &drout, 1);

  if (tlr)
    jtag_9030_EX1toRTI(pLC);

  return 0;
}

/* Read FPGA status register (see Configuration Details in Xilinx
   Virtex-II Pro Users Guide, ug012.pdf) */

int jtag_9030_read_fpga_status(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int result;
  char dout[40];
  int tdo, bitrev, fpgastat, i;
  char dinRDSTAT[] = {0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00,
                      0x40, 0x07, 0x00, 0x14,
                      0x66, 0xaa, 0x99, 0x55,
                      0xff, 0xff, 0xff, 0xff};
  char dinRCRC[] = {0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                    0xe0, 0x00, 0x00, 0x00,
                    0x80, 0x01, 0x00, 0x0c};
  jtag_9030_open(ev_dev);

  jtag_9030_TLR(pLC);

  result = jtag_9030_validate(ev_dev);
  if (result < 0)
    goto out;
  
  /* Load cgf_in */
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toRTI(pLC);
  jtag_9030_SDR(pLC, 0, 192, dinRDSTAT, 1, dout, 0);
  jtag_9030_EX1toRTI(pLC);

  /* Load cfg_out */
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGOUT, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toRTI(pLC);
  jtag_9030_SDR(pLC, 0, 32, dinRDSTAT, 1, dout, 0);
  bitrev = ((dout[0] & 0x00ff) << 24) +
    ((dout[1] & 0x00ff) << 16) +
    ((dout[2] & 0x00ff) << 8) +
    (dout[3] & 0x00ff);
  jtag_9030_EX1toRTI(pLC);
  /* Reverse status register bits read out */
  fpgastat = 0;
  for (i = 0; i < 32; i++)
    {
      fpgastat <<= 1;
      fpgastat |=  bitrev & 1;
      bitrev >>= 1;
    }

  /* Load cgf_in */
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toRTI(pLC);
  jtag_9030_SDR(pLC, 0, 128, dinRCRC, 1, dout, 1);

  jtag_9030_TLR(pLC);

  result = fpgastat;

 out:
  jtag_9030_release(ev_dev);
  return result;
}

/* Issue JTAG command XSC_CONFIG which forces PROG_B pin on Platform
   Flash low and initiates a new configuration from Flash */ 

int jtag_9030_XCF_conf(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int tdo;
  int result;

  jtag_9030_open(ev_dev);
  result = jtag_9030_validate(ev_dev);
  if (result < 0)
    goto out;

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_CONFIG, 0, &tdo, 1);
  jtag_9030_TLR(pLC);

  result = 0;

 out:
  jtag_9030_release(ev_dev);
  return result;
}

/* Read flash contents
   Note: size has to be multiple of 1024 */

int jtag_9030_XCF_readstart(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int tdo;
  int result;

  jtag_9030_open(ev_dev);

  jtag_9030_TLR(pLC);
  result = jtag_9030_validate(ev_dev);
  if (result < 0)
    {
      jtag_9030_release(ev_dev);
      return result;
    }

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ENABLE, 0, &tdo, 1);
  dr[0] =__cpu_to_be32(0x03);
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);

  return 0;
}
  
int jtag_9030_XCF_read(struct mrf_dev *ev_dev, char *data,
		  unsigned int addr, unsigned int size)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int tdo, i, j;
  char *din, *dout, bitnorm, bitrev;
  int block;

  din = kmalloc(XCF_BLOCK_SIZE, GFP_KERNEL);
  dout = kmalloc(XCF_BLOCK_SIZE, GFP_KERNEL);
  memset(din, 0, XCF_BLOCK_SIZE);
  memset(dout, 0, XCF_BLOCK_SIZE);

  for (block = 0; block < size / 1024; block++)
    {
      if ((addr/XCF_BLOCK_SIZE) + block == 1024)
        {
          jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_OP_STATUS,
		   0, &tdo, 1);
          dr[0] = 0;
          jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);
          jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_BYPASS,
		   0, &tdo, 1);
        }

      jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ADDRESS_SHIFT,
	       0, &tdo, 1);
      /* bytes have to be in big-endian order */
      dr[0] = __cpu_to_be32(((addr/XCF_BLOCK_SIZE) + block) << 18);
      jtag_9030_SDR(pLC, 1, 24, (char *) dr, 0, (char *) &tdo, 1);
      jtag_9030_pulseTCK(pLC, 15);

      jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_READ, 0, &tdo, 1);
      jtag_9030_pulseTCK(pLC, 15);
      jtag_9030_SDR(pLC, 1, XCF_BLOCK_SIZE*8, din, 0, dout, 1);
      for (i = XCF_BLOCK_SIZE-1; i >= 0; i--)
	{
	  bitnorm = dout[XCF_BLOCK_SIZE-1-i];
	  bitrev = 0;
	  for (j = 0; j < 8; j++)
	    {
	      bitrev <<= 1;
	      if (bitnorm & 1)
		bitrev |= 1;
	      bitnorm >>= 1;
	    }
	  data[XCF_BLOCK_SIZE*block+i] = bitrev;
	}
    }

  kfree(dout);
  kfree(din);

  return block * XCF_BLOCK_SIZE;
}

int jtag_9030_XCF_readend(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int tdo;

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_OP_STATUS,
	   0, &tdo, 1);
  dr[0] = 0;
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);
  
  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_DISABLE, 0, &tdo, 1);
  jtag_9030_pulseTCK(pLC, 15);

  jtag_9030_TLR(pLC);

  return 0;
}

/* Program data to Platform Flash,
   This function erases the flash prior to programming */

int jtag_9030_XCF_progstart(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int tdo, i;
  int result;

  jtag_9030_open(ev_dev);

  result = jtag_9030_validate(ev_dev);
  if (result < 0)
    return result;

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ENABLE, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0x03000000);
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_IDCODE, 0, &tdo, 1);
  dr[0] = 0;
  jtag_9030_SDR(pLC, 1, 32, (char *) dr, 0, (char *) &tdo, 1);

  printk(KERN_INFO "ID_CODE %08x\n", __be32_to_cpu(tdo));

  jtag_9030_TLR(pLC);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ENABLE, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0xd0000000);
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_UNLOCK, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0x00003f00);
  jtag_9030_SDR(pLC, 1, 24, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ERASE, 0, &tdo, 0);
  jtag_9030_EX1toSDR(pLC);
  jtag_9030_SDI(pLC, 1, 0, &tdo, 0);
  dr[0] = 0x0000003f;
  jtag_9030_SDI(pLC, 24, dr[0], &tdo, 1);
  jtag_9030_EX1toRTI(pLC);

  /* Here we define how often to call schedule during erasing the
     flash. This takes quite some time and we don't want to block. */
#define XCF_ERASE_SCHEDULE_COUNT 1000

  for (i = 0; i < XCF_ERASE_SCHEDULE_COUNT; i++)
    {
      jtag_9030_pulseTCK(pLC, XCF_ERASE_TCKS/XCF_ERASE_SCHEDULE_COUNT);
      schedule();
    }

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_IDCODE, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0x00000000);
  jtag_9030_SDR(pLC, 1, 32, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ENABLE, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0x03000000);
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_DATA_BTC, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0xffffffe0);
  jtag_9030_SDR(pLC, 1, 32, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_PROGRAM, 0, &tdo, 1);

  jtag_9030_pulseTCK(pLC, 120);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_BYPASS, 0, &tdo, 1);

  return 0;
}

/* Program bytes to Platform Flash
   size has to be a multiple of 32 */

int jtag_9030_XCF_write(struct mrf_dev *ev_dev, char *data,
		   int addr, int size)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int tdo, i;
  char *din, *dout, bits, bitswap;
  int block;

  din = kmalloc(XCF_BLOCK_SIZE, GFP_KERNEL);
  dout = kmalloc(XCF_BLOCK_SIZE, GFP_KERNEL);
  memset(din, 0, XCF_BLOCK_SIZE);
  memset(dout, 0, XCF_BLOCK_SIZE);

  for (block = 0; block < size / 32; block++)
    {
      if ((addr/32) + block == 32768)
        {
          jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_OP_STATUS,
                        0, &tdo, 1);
          dr[0] = __constant_cpu_to_be32(0);
          jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);
          jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_BYPASS,
                        0, &tdo, 1);
        }

      jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_DATA_SHIFT,
                    0, &tdo, 1);
      /* The bit ordering inside the flash differs from the .bit file.
	 We need to reverse the bits here. */
      for (i = 31; i >= 0; i--)
	{
	  bits = data[32*block+i];
	  bitswap = ((bits & 0x01) << 7) |
	    ((bits & 0x02) << 5) |
	    ((bits & 0x04) << 3) |
	    ((bits & 0x08) << 1) |
	    ((bits & 0x10) >> 1) |
	    ((bits & 0x20) >> 3) |
	    ((bits & 0x40) >> 5) |
	    ((bits & 0x80) >> 7);	    
	  din[31-i] = bitswap;
	}
      jtag_9030_SDR(pLC, 1, 256, din, 0, dout, 1);

      if (block == 0 || block == 32768)
        {
          jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_ADDRESS_SHIFT,
                        0, &tdo, 1);
          dr[0] = __cpu_to_be32(((addr/32) + block) << 13);
          jtag_9030_SDR(pLC, 1, 24, (char *) dr, 0, (char *) &tdo, 1);
        }

      jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_PROGRAM, 
                    0, &tdo, 1);

      jtag_9030_pulseTCK(pLC, 1000);

    }

  kfree(dout);
  kfree(din);

  return 0;
}

int jtag_9030_XCF_progend(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int dr[3] = {0, 0, 0};
  int tdo;

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_OP_STATUS,
                0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0);
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_DATA_SUCR,
                0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0xfffc0000);
  jtag_9030_SDR(pLC, 1, 16, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_PROGRAM, 0, &tdo, 1);
  jtag_9030_pulseTCK(pLC, 60);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_DATA_CCB, 0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0xffff0000);
  jtag_9030_SDR(pLC, 1, 16, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_PROGRAM, 0, &tdo, 1);
  jtag_9030_pulseTCK(pLC, 60);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_XSC_DATA_DONE,
                0, &tdo, 1);
  dr[0] = __constant_cpu_to_be32(0xce000000);
  jtag_9030_SDR(pLC, 1, 8, (char *) dr, 0, (char *) &tdo, 1);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_PROGRAM, 0, &tdo, 1);
  jtag_9030_pulseTCK(pLC, 60);

  jtag_9030_SIR(pLC, ev_dev->fpgairlen, XCFXXP_IRLEN, XCFXXP_ISC_DISABLE, 0, &tdo, 1);
  jtag_9030_pulseTCK(pLC, 50);

  jtag_9030_TLR(pLC);

  jtag_9030_release(ev_dev);

  return 0;
}

/* Configure FPGA from .bit file.
   Startup clock has to be set to JTAG in bitgen. */

int jtag_9030_init_load_fpga(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  char dout[40];
  int tdo;
  char dinRCRC[] = {0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                    0xe0, 0x00, 0x00, 0x00,
                    0x80, 0x01, 0x00, 0x0c,
                    0x66, 0xaa, 0x99, 0x55,
                    0xff, 0xff, 0xff, 0xff};
  char dinAGHIGH[] = {0x10, 0x00, 0x00, 0x00,
                      0x80, 0x01, 0x00, 0x0c};
  int i;
  int result;

  jtag_9030_open(ev_dev);

  jtag_9030_TLR(pLC);
  result = jtag_9030_validate(ev_dev);
  if (result < 0)
    {
      jtag_9030_release(ev_dev);
      return result;
    }

    /* Load cgf_in */
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toRTI(pLC);
  jtag_9030_SDR(pLC, 0, 192, dinRCRC, 1, dout, 0);
  jtag_9030_EX1toRTI(pLC);

  /* Load jshutdown */
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_JSHUTDOWN, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toRTI(pLC);
  jtag_9030_pulseTCK(pLC, 12);

  jtag_9030_TLR(pLC);

  /* Load cgf_in */
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_CFGIN, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toSDR(pLC);
  for (i = 7; i >= 0; i--)
    jtag_9030_SDI(pLC, 8, dinAGHIGH[i], &tdo, 0);

  return 0;
}

int jtag_9030_end_load_fpga(struct mrf_dev *ev_dev)
{
  volatile struct Pci9030LocalConf *pLC = ev_dev->BAR_mmapped[0];

  int tdo, result;

  /* Last bit */
  pLC->GPIOC |= __constant_cpu_to_le32(PLX9030_GPIOC_TMS);
  jtag_9030_pulseTCK(pLC, 1);

  jtag_9030_TLR(pLC);
  result = jtag_9030_validate(ev_dev);
  if (result < 0)
    {
      jtag_9030_release(ev_dev);
      return result;
    }

  /* Issue JSTART */

  jtag_9030_TLR(pLC);
  jtag_9030_TLRtoRTI(pLC);
  jtag_9030_TLR(pLC);
  jtag_9030_TLRtoRTI(pLC);

  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_JSTART, XCFXXP_IRLEN, &tdo, 0);
  jtag_9030_EX1toRTI(pLC);
  jtag_9030_pulseTCK(pLC, 12);

  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_BYPASS, XCFXXP_IRLEN, &tdo, 1);
  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_BYPASS, XCFXXP_IRLEN, &tdo, 1);

  jtag_9030_TLR(pLC);

  jtag_9030_TLR(pLC);
  jtag_9030_TLRtoRTI(pLC);
  jtag_9030_TLR(pLC);
  jtag_9030_TLRtoRTI(pLC);

  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_JSTART, XCFXXP_IRLEN, &tdo, 1);
  jtag_9030_TLRtoRTI(pLC);
  jtag_9030_pulseTCK(pLC, 12);

  jtag_9030_SIR(pLC, 0, ev_dev->fpgairlen, XC2VP_BYPASS, XCFXXP_IRLEN, &tdo, 1);

  jtag_9030_TLR(pLC);
  jtag_9030_release(ev_dev);

  return 0;
}

int fpga_9030_sprint(struct mrf_dev *ev_dev, char *buf, int size)
{
  int fpgastat = 0, s;

  fpgastat = jtag_9030_read_fpga_status(ev_dev);
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
