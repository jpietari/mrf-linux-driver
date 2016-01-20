/*
  mrfevr.c -- Micro-Research Event Receiver
              Linux 2.6 driver

	      This driver handles multiple instances

  Author: Jukka Pietarinen (MRF)
  Date:   30.1.2014

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/version.h>

#include <asm/page.h>
#include <asm/uaccess.h>

#include "pci_mrfev.h"
MODULE_LICENSE("GPL");

#define DEVICE_NAME        "mrfevr"

/*
#define DETECT_UNCONFIGURED_BOARD 1
*/

extern struct mrf_dev mrf_devices[MAX_MRF_DEVICES];

/*
  File operations for Event Boards
*/
static struct file_operations evr_fops = {
  .owner = THIS_MODULE,
  .read = ev_read,
  .write = ev_write,
#ifdef HAVE_UNLOCKED_IOCTL
  .unlocked_ioctl = ev_unlocked_ioctl,
#else
  .ioctl = ev_ioctl,
#endif
  .open = ev_open,
  .release = ev_release,
  .fasync = ev_fasync,
  .mmap = ev_remap_mmap,
};

/* The first prototypes shipped had both the vendor ID and subsystem
   vendor ID set to MRF's vendor ID. Now, boards are shipped with
   vendor ID and device ID set to PLX's 9030 default values and
   subsystem field set to MRF's codes. This driver supports both
   conventions. */

static struct pci_device_id evr_ids[] = {
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PMCEVR200, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PMCEVR230, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PXIEVR220, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PXIEVR230, },
  { PCI_DEVICE(PCI_VENDOR_ID_MRF, PCI_DEVICE_ID_MRF_PMCEVR200), },
  { PCI_DEVICE(PCI_VENDOR_ID_MRF, PCI_DEVICE_ID_MRF_PXIEVR220), },
#ifdef DETECT_UNCONFIGURED_BOARD
  { .vendor = MODULE_VENDOR_ID_NOCONF,
    .device = MODULE_DEVICE_ID_NOCONF,
    .subvendor = MODULE_SUBVENDOR_ID_NOCONF,
    .subdevice = MODULE_SUBDEVICE_ID_NOCONF, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9056,
    .subvendor = PCI_VENDOR_ID_PLX,
    .subdevice = PCI_DEVICE_ID_PLX_9056, },
#endif
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9056,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_CPCIEVRTG, },
  { .vendor = PCI_VENDOR_ID_MRF,
    .device = PCI_DEVICE_ID_MRF_CPCIEVR300,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_CPCIEVR300},
  { .vendor = PCI_VENDOR_ID_LATTICE,
    .device = PCI_DEVICE_ID_LATTICE_ECP3,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_CPCIEVR300},
  { .vendor = PCI_VENDOR_ID_LATTICE,
    .device = PCI_DEVICE_ID_LATTICE_ECP3,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PCIEEVR300},
  { .vendor = PCI_VENDOR_ID_XILINX,
    .device = PCI_DEVICE_ID_ZOMOJO_Z1,
    .subvendor = PCI_VENDOR_ID_XILINX,
    .subdevice = PCI_DEVICE_ID_ZOMOJO_Z1},
  { .vendor = PCI_VENDOR_ID_XILINX,
    .device = PCI_DEVICE_ID_0505,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PXIEEVR300},
  { .vendor = PCI_VENDOR_ID_XILINX,
    .device = PCI_DEVICE_ID_KINTEX7,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_MTCAEVR300},
  { 0, }};
MODULE_DEVICE_TABLE(pci, evr_ids);

static int pci_evr_probe(struct pci_dev *pcidev, const struct pci_device_id *dev_id)
{
  int i, res;
  dev_t chrdev = 0;
  struct mrf_dev *ev_device;
  struct pci_device_id *id = (struct pci_device_id *) dev_id;
  volatile u32 *evr_fw_version;

  /* We keep device instance number in id->driver_data */
  id->driver_data = -1;

  /* Find empty mrf_dev structure - an empty subsys_id field shows this */
  for (i = 0; i < MAX_MRF_DEVICES; i++)
    if (mrf_devices[i].subsys_id == 0)
      {
	id->driver_data = i;
	break;
      }

  if (id->driver_data < 0)
    {
      printk(KERN_WARNING DEVICE_NAME ": too many devices.\n");
      return -EMFILE;
    }

  ev_device = &mrf_devices[id->driver_data];

  /* Allocate device numbers for character device. */
  res = alloc_chrdev_region(&chrdev, 0, DEVICE_MINOR_NUMBERS, DEVICE_NAME);
  if (res < 0)
    {
      printk(KERN_WARNING DEVICE_NAME ": cannot register char device.\n");
      return res;
    }

  /* Initialize device structure */
  ev_device->major = MAJOR(chrdev);
  cdev_init(&ev_device->cdev, &evr_fops);
  ev_device->cdev.owner = THIS_MODULE;
  ev_device->cdev.ops = &evr_fops;
  ev_device->access_mode = 0;
  ev_device->access_device = 0;
  ev_device->eeprom_data = NULL;
  ev_device->eeprom_data_size = 0;
  ev_device->xcf_data = NULL;
  ev_device->xcf_data_pos = -1;
  ev_device->fpga_conf_data = NULL;
  ev_device->fpga_conf_size = 0;
#ifndef init_MUTEX
  sema_init(&ev_device->sem, 1);
#else
  init_MUTEX(&ev_device->sem);
#endif
  res = cdev_add(&ev_device->cdev, chrdev, DEVICE_MINOR_NUMBERS);
  ev_device->jtag_refcount = 0;
  ev_device->fpgaid = 0;
  ev_device->xcfid = 0;
  ev_device->fpgairlen = 0;
  ev_device->xcfirlen = 0;

  if (res)
    {
      printk(KERN_WARNING DEVICE_NAME ": error adding " DEVICE_NAME "0-%d\n.",
	     DEVICE_MINOR_NUMBERS);
    }

  /* Here we enable device before we can do any accesses to device. */
  res = pci_enable_device(pcidev);
  if (res)
    {
      printk(KERN_WARNING DEVICE_NAME ": error enabling device\n.");
      return res;
    }

  for (i = 0; i < MAX_MRF_BARS; i++)
    {
      ev_device->BAR_start[i] = pci_resource_start(pcidev, i);
      ev_device->BAR_end[i] = pci_resource_end(pcidev, i);
      ev_device->BAR_flags[i] = pci_resource_flags(pcidev, i);
      if (ev_device->BAR_start[i] != 0x00000000)
	{
	  if (request_mem_region(ev_device->BAR_start[i], ev_device->BAR_end[i]
				 - ev_device->BAR_start[i] + 1,
				 DEVICE_NAME) != NULL)
	    ev_device->BAR_mmapped[i] = ioremap_nocache(ev_device->BAR_start[i],
							ev_device->BAR_end[i] -
							ev_device->BAR_start[i] + 1);
	  printk(KERN_WARNING DEVICE_NAME ":BAR%d start %08x end %08x, mmap %08x\n", i,
		 (unsigned int) ev_device->BAR_start[i],
		 (unsigned int) ev_device->BAR_end[i],
		 (unsigned int) ev_device->BAR_mmapped[i]);
	  
#ifdef DEBUG_BARS
	  {
	    int j;
	    unsigned char *pEvrbyte = (unsigned char *) ev_device->BAR_mmapped[i];
	    for (j = 0; j < 16; j++)
	      printk(KERN_WARNING "%02x\n", pEvrbyte[j]);
	  }
#endif
	}
      else
	ev_device->BAR_mmapped[i] = NULL;
    }

  /* Read subsystem device ID to identify board */
  pci_read_config_word(pcidev, PCI_SUBSYSTEM_ID, &ev_device->subsys_id);

  switch (ev_device->subsys_id)
    {
    case PCI_DEVICE_ID_MRF_PMCEVR200:
    case PCI_DEVICE_ID_MRF_PMCEVR230:
    case PCI_DEVICE_ID_MRF_PXIEVR220:
    case PCI_DEVICE_ID_MRF_PXIEVR230:
    case MODULE_SUBDEVICE_ID_NOCONF:
      ev_device->devtype = MRF_DEVTYPE_V2P_9030;
      ev_device->pLC = ev_device->BAR_mmapped[0];
      ev_device->pEv = ev_device->BAR_mmapped[2];
      break;

    case PCI_DEVICE_ID_MRF_CPCIEVR300:
      ev_device->devtype = MRF_DEVTYPE_ECP3_PCI;   
      ev_device->pEv = ev_device->BAR_mmapped[0];
      break;

    case PCI_DEVICE_ID_MRF_PCIEEVR300: 
      ev_device->devtype = MRF_DEVTYPE_ECP3_PCIE;
      ev_device->pEv = ev_device->BAR_mmapped[0];
      break;

    case PCI_DEVICE_ID_MRF_CPCIEVRTG:
    case PCI_DEVICE_ID_MRF_CPCIFCT:
    case PCI_DEVICE_ID_PLX_9056: /* This is to identify unconfigured board */
      ev_device->devtype = MRF_DEVTYPE_V5_9056;
      ev_device->pLC = ev_device->BAR_mmapped[0];
      ev_device->pEv = ev_device->BAR_mmapped[2];
      break;

    case PCI_DEVICE_ID_ZOMOJO_Z1:
    case PCI_DEVICE_ID_MRF_PXIEEVR300:
      ev_device->devtype = MRF_DEVTYPE_V5_PCIE;
      ev_device->pEv = ev_device->BAR_mmapped[0];
      break;

    case PCI_DEVICE_ID_KINTEX7:
    case PCI_DEVICE_ID_MRF_MTCAEVR300:
      ev_device->devtype = MRF_DEVTYPE_K7_PCIE;
      ev_device->pEv = ev_device->BAR_mmapped[0];
      break;

    default:
      printk(KERN_WARNING DEVICE_NAME ": Unknown subsystem device id 0x%04X\n.",
	     ev_device->subsys_id);
    }    
  
  /* Check the interrupt line */
  ev_device->irq = pcidev->irq;

  /* Check firmware version */
  evr_fw_version = ((ev_device->pEv) + EV_FW_VERSION_OFFSET);
  ev_device->fw_version = be32_to_cpu(*evr_fw_version);

  return 0;
}

static void pci_evr_remove(struct pci_dev *pcidev)
{
  int i;
  struct mrf_dev *ev_device = NULL;

  for (i = 0; i < MAX_MRF_DEVICES; i++)
    if (mrf_devices[i].BAR_start[0] == pci_resource_start(pcidev, 0))
      {
	ev_device = &mrf_devices[i];
	break;
      }

  if (ev_device == NULL)
    {
      printk(KERN_ALERT "Trying to remove uninstalled driver for bus\n");
    }
  else
    {
      /* Unmap BARs */
      for (i = 0; i < MAX_MRF_BARS; i++)
	{
	  if (ev_device->BAR_mmapped[i] != NULL)
	    {
	      iounmap(ev_device->BAR_mmapped[i]);
	      release_mem_region(ev_device->BAR_start[i], ev_device->BAR_end[i]
				 - ev_device->BAR_start[i] + 1);
	    }
	  ev_device->BAR_mmapped[i] = NULL;
	}

      pci_disable_device(pcidev);
      cdev_del(&ev_device->cdev);
      unregister_chrdev_region(MKDEV(ev_device->major, 0), DEVICE_MINOR_NUMBERS);
      /* Clear out subsys_id field when device is removed */
      ev_device->subsys_id = 0;
    }
}

static struct pci_driver evr_driver = {
  .name = "pci_mrfevr",
  .id_table = evr_ids,
  .probe = pci_evr_probe,
  .remove = pci_evr_remove,
};

int ev_assign_irq(struct mrf_dev *ev_device)
{
  int result;

  result = request_irq(ev_device->irq,
		       &ev_interrupt,
		       /* Is there need for SA_INTERRUPT? */
#if LINUX_VERSION_CODE > (0x020619)
		       IRQF_SHARED,
#else
		       SA_SHIRQ,
#endif
		       DEVICE_NAME,
		       (void *) ev_device);
  if (result)
    printk(KERN_INFO DEVICE_NAME ": cannot get interrupt %d\n",
	   ev_device->irq);
  return result;
}

static int __init pci_evr_init(void)
{
  /* Allocate and clear memory for all devices. */
  memset(mrf_devices, 0, sizeof(struct mrf_dev)*MAX_MRF_DEVICES);

  printk(KERN_ALERT "Event Receiver PCI/PCIe driver init.\n");
  return pci_register_driver(&evr_driver);    
}

static void __exit pci_evr_exit(void)
{
  printk(KERN_ALERT "Event Receiver PCI/PCIe driver exiting.\n");
  pci_unregister_driver(&evr_driver);
}  

module_init(pci_evr_init);
module_exit(pci_evr_exit);
