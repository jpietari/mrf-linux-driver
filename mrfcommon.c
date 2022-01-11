/*
  mrfcommon.c -- Micro-Research Event Generator/Event Receiver
                 Linux 2.6 driver common functions

		 This driver handles multiple boards.

  Author: Jukka Pietarinen (MRF)
  Date:   30.01.2014

  22.09.2014     Fixed driver code for cPCI-EVRTG-300

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
#include <linux/sched.h>
#include <linux/uaccess.h>

#include <asm/page.h>
#include <asm/uaccess.h>

#include "pci_mrfev.h"
MODULE_LICENSE("GPL");

#ifndef DEVICE_NAME
#define DEVICE_NAME "pci_mrf"
#endif

#define LATTICE_IRQ 1

struct mrf_dev mrf_devices[MAX_MRF_DEVICES];

int ev_assign_irq(struct mrf_dev *ev_device);
int ev_plx_irq_enable(struct mrf_dev *ev_device);
int ev_plx_irq_disable(struct mrf_dev *ev_device);
int ev_plx_9056_irq_enable(struct mrf_dev *ev_device);
int ev_plx_9056_irq_disable(struct mrf_dev *ev_device);
int ev_irq_enable(struct mrf_dev *ev_device);
int ev_irq_disable(struct mrf_dev *ev_device);

/*
  Open and close
*/

int ev_open(struct inode *inode, struct file *filp)
{
  int result = 0;
  struct mrf_dev *ev_device;
  int major, minor;

  ev_device = container_of(inode->i_cdev, struct mrf_dev, cdev);
  filp->private_data = ev_device;

  major = MAJOR(inode->i_rdev);
  minor = MINOR(inode->i_rdev);

  printk(KERN_WARNING DEVICE_NAME ": open device major %d minor %d\n",
	 major, minor);

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  /* Check that none of the first three devices is open.
     We allow several simultaneous connections on the 4th minor device. */
  if ((ev_device->access_device > 0 &&
       ev_device->access_device < DEVICE_EV) ||
      (ev_device->access_device == DEVICE_EV && minor != DEVICE_EV))
    {
      printk(KERN_WARNING DEVICE_NAME ": open device major %d minor %d"
	     "already open. Only one open minor device allowed.\n",
	     major, ev_device->access_device);
      result = -EPERM;
      goto out;
    }

  if (minor >= DEVICE_FIRST && minor <= DEVICE_LAST)
    {
      ev_device->access_mode = filp->f_flags & O_ACCMODE;
      ev_device->access_device = minor;

      switch (ev_device->access_device)
	{
	case DEVICE_EEPROM:
	  if (ev_device->devtype &
	      (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	    {
	      /* Allocate and clear memory for EEPROM data */
	      ev_device->eeprom_data = kmalloc(EEPROM_DATA_ALLOC_SIZE,
					       GFP_KERNEL);
	      memset(ev_device->eeprom_data, 0, EEPROM_DATA_ALLOC_SIZE);
	      
	      /* If the EEPROM device was opened in read-only mode we need
		 to read in the data from the EEPROM. */
	      if (ev_device->access_mode == O_RDONLY)
		{
		  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
		    result = eeprom_9030_sprint(ev_device->pLC,
					   ev_device->eeprom_data,
					   EEPROM_DATA_ALLOC_SIZE);
		  if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
		    result = eeprom_9056_sprint(ev_device->pLC,
					   ev_device->eeprom_data,
					   EEPROM_DATA_ALLOC_SIZE);
		  if (result < 0)
		    {
		      /* If there was an error, clean up */
		      kfree(ev_device->eeprom_data);
		      ev_device->eeprom_data = NULL;
		      ev_device->access_mode = 0;
		      ev_device->access_device = 0;
		      goto out;
		    }
		  
		  ev_device->eeprom_data_size = result;
		}
	    }
	  else
	    {
	      result = -ENXIO;
	      goto out;
	    }
	  break;
	case DEVICE_FLASH:
	  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	    {
	      ev_device->xcf_data = kmalloc(XCF_DATA_ALLOC_SIZE,
					    GFP_KERNEL);
	      memset(ev_device->xcf_data, 0xffffffff, XCF_DATA_ALLOC_SIZE);
	      /* If the XCF device was opened in write-only mode we
		 erase the Platform Flash and start the programming
		 sequence */
	      if (ev_device->access_mode == O_RDONLY)
		{
		  ev_device->xcf_data_pos = -1;
		  result = jtag_9030_XCF_readstart(ev_device);
		  if (result < 0)
		    {
		      /* If there was an error, clean up */
		      kfree(ev_device->xcf_data);
		      ev_device->xcf_data = NULL;
		      ev_device->access_mode = 0;
		      ev_device->xcf_data_pos = 0;
		      ev_device->access_device = 0;
		      goto out;
		    }
		}
	      if (ev_device->access_mode == O_WRONLY)
		{
		  ev_device->xcf_data_pos = 0;
		  result = jtag_9030_XCF_progstart(ev_device);
		  if (result < 0)
		    {
		      /* If there was an error, clean up */
		      kfree(ev_device->xcf_data);
		      ev_device->xcf_data = NULL;
		      ev_device->access_mode = 0;
		      ev_device->access_device = 0;
		      goto out;
		    }
		}
	    }
	  else if (ev_device->devtype &
		   (MRF_DEVTYPE_V5_9056))
	    {
	      /* Allocate and clear memory for FPGA data */
	      ev_device->xcf_data = kmalloc(FLASH_DATA_ALLOC_SIZE,
					    GFP_KERNEL);
	      memset(ev_device->xcf_data, 0, FLASH_DATA_ALLOC_SIZE);
	      /* If the Flash device was opened in write-only mode we
		 erase the Flash and start the programming
		 sequence */
	      if (ev_device->access_mode == O_RDONLY)
		{
		}
	      if (ev_device->access_mode == O_WRONLY)
		{
		  ev_device->xcf_data_pos = 0;
		  result = flash_bulkerase(ev_device);
		  if (result < 0)
		    {
		      /* If there was an error, clean up */
		      kfree(ev_device->xcf_data);
		      ev_device->xcf_data = NULL;
		      ev_device->access_mode = 0;
		      ev_device->access_device = 0;
		      goto out;
		    }
		}
	    }
	  else if (ev_device->devtype &
		 (MRF_DEVTYPE_ECP3_PCI | MRF_DEVTYPE_ECP3_PCIE))
	    {
	      /* Allocate and clear memory for FPGA data */
	      ev_device->xcf_data = kmalloc(FLASH_DATA_ALLOC_SIZE,
					    GFP_KERNEL);
	      memset(ev_device->xcf_data, 0, FLASH_DATA_ALLOC_SIZE);
	      /* If the Flash device was opened in write-only mode we
		 erase the Flash and start the programming
		 sequence */
	      if (ev_device->access_mode == O_RDONLY)
		{
		}
	      if (ev_device->access_mode == O_WRONLY)
		{
		  ev_device->xcf_data_pos = 0;
		  result = flash_primaryerase(ev_device);
		  if (result < 0)
		    {
		      /* If there was an error, clean up */
		      kfree(ev_device->xcf_data);
		      ev_device->xcf_data = NULL;
		      ev_device->access_mode = 0;
		      ev_device->access_device = 0;
		      goto out;
		    }
		}
	      break;
	    }
	  else if (ev_device->devtype & MRF_DEVTYPE_V5_PCIE)
	    {
	      result = 0;
	      break;
	    }
	  else if (ev_device->devtype & MRF_DEVTYPE_K7_PCIE)
	    {
	      /* Allocate and clear memory for FPGA data */
	      ev_device->xcf_data = kmalloc(FLASH_DATA_ALLOC_SIZE,
					    GFP_KERNEL);
	      memset(ev_device->xcf_data, 0, FLASH_DATA_ALLOC_SIZE);
	      /* If the Flash device was opened in write-only mode we
		 erase the Flash and start the programming
		 sequence */
	      if (ev_device->access_mode == O_RDONLY)
		{
		}
	      if (ev_device->access_mode == O_WRONLY)
		{
		  ev_device->xcf_data_pos = 0;
		  result = flash_bulkerase(ev_device);
		  if (result < 0)
		    {
		      /* If there was an error, clean up */
		      kfree(ev_device->xcf_data);
		      ev_device->xcf_data = NULL;
		      ev_device->access_mode = 0;
		      ev_device->access_device = 0;
		      goto out;
		    }
		}
	      break;
	    }
	  else
	    {
	      result = -ENXIO;
	      goto out;
	    }
	  break;
	case DEVICE_FPGA:
	  if (ev_device->devtype &
	      (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	    {
	      /* Allocate and clear memory for FPGA data */
	      ev_device->fpga_conf_data = kmalloc(FPGA_DATA_ALLOC_SIZE,
						  GFP_KERNEL);
	      memset(ev_device->fpga_conf_data, 0, FPGA_DATA_ALLOC_SIZE);
	      /* If the FPGA device was opened in read-only mode we read
		 in the FPGA status - reading back configuration data is
		 not supported */
	      result = -1;
	      if (ev_device->access_mode == O_RDONLY)
		{
		  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
		    result = fpga_9030_sprint(ev_device, ev_device->fpga_conf_data,
					 FPGA_DATA_ALLOC_SIZE);
		  else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
		    result = fpga_9056_sprint(ev_device,
					      ev_device->fpga_conf_data,
					      FPGA_DATA_ALLOC_SIZE);
		  ev_device->fpga_conf_size = result;
		}
	      if (ev_device->access_mode == O_WRONLY)
		{
		  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
		    result = jtag_9030_init_load_fpga(ev_device);
		  else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
		    result = jtag_9056_init_load_fpga(ev_device);
		  ev_device->fpga_conf_size = 0;	      
		}	  
	      if (result < 0)
		{
		  /* If there was an error, clean up */
		  kfree(ev_device->fpga_conf_data);
		  ev_device->fpga_conf_data = NULL;
		  ev_device->access_mode = 0;
		  ev_device->access_device = 0;
		  goto out;
		}
	    }
	  else
	    {
	      result = -ENXIO;
	      goto out;
	    }
	  break;

	case DEVICE_EV:
	  result = 0;
	  /* Assign interrupt handler when first instance opened. */
	  if (!(ev_device->refcount_ev))
	    {
	      if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
		{
		  result = ev_assign_irq(ev_device);
		  ev_plx_irq_enable(ev_device);
		}
	      else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
		{
		  result = ev_assign_irq(ev_device);
		  ev_plx_9056_irq_enable(ev_device);
		}
#ifdef LATTICE_IRQ
	      else
		{
		  result = ev_assign_irq(ev_device);		  
		  ev_irq_enable(ev_device);
		}
#endif
	    }
	  /* Increase device reference count. */
	  ev_device->refcount_ev++;
	  break;
	default:
	  result = -ENXIO;
	  goto out;
	}
    }
  else
    {
      result = -ENXIO;
      goto out;
    }

  result = 0;

 out:
  up(&ev_device->sem);
  return result;
}

int ev_release(struct inode *inode, struct file *filp)
{
  int result = 0;
  struct mrf_dev *ev_device = (struct mrf_dev *) filp->private_data;

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  switch (ev_device->access_device)
    {
    case DEVICE_EEPROM:
      if (ev_device->devtype & 
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
		result = eeprom_9030_sscan(ev_device->pLC,
					   ev_device->eeprom_data);
	      if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
		result = eeprom_9056_sscan(ev_device->pLC,
					   ev_device->eeprom_data);
	    }
	  /* Release memory allocated for EEPROM data */
	  kfree(ev_device->eeprom_data);
	  ev_device->eeprom_data = NULL;
	  ev_device->access_mode = 0;
	  ev_device->access_device = 0;
	}
      else
	result = -ENXIO;
      break;
    case DEVICE_FLASH:
      if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	{
	  if (ev_device->access_mode == O_RDONLY)
	    {
	      result = jtag_9030_XCF_readend(ev_device);
	    }
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      /* Flush buffer */
	      jtag_9030_XCF_write(ev_device, ev_device->xcf_data,
			     ev_device->xcf_data_pos, XCF_DATA_ALLOC_SIZE);
	      result = jtag_9030_XCF_progend(ev_device);
	      
	      /* Issue load FPGA from flash (cycle nPROG) */
	      jtag_9030_XCF_conf(ev_device);
	    }
	  /* Release memory allocated for XCF data */
	  kfree(ev_device->xcf_data);
	  ev_device->xcf_data = NULL;
	}
      if (ev_device->devtype &
	  (MRF_DEVTYPE_V5_9056))
	{
	  if (ev_device->access_mode == O_RDONLY)
	    {
	    }
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      /* Flush buffer */
	      flash_pageprogram(ev_device, ev_device->xcf_data,
				ev_device->xcf_data_pos, FLASH_DATA_ALLOC_SIZE);
	      
	      /* Issue load FPGA from flash (cycle nPROG) */
	      /*      jtag_XCF_conf(ev_device); */
	    }
	  /* Release memory allocated for XCF data */
	  kfree(ev_device->xcf_data);
	  ev_device->xcf_data = NULL;
	}
      if (ev_device->devtype &
	  (MRF_DEVTYPE_ECP3_PCI | MRF_DEVTYPE_ECP3_PCIE))
	{
	  if (ev_device->access_mode == O_RDONLY)
	    {
	    }
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      /* Flush buffer */
	      flash_pageprogram(ev_device, ev_device->xcf_data,
				ev_device->xcf_data_pos + FLASH_SECTOR_PRIMARY_START,
				FLASH_DATA_ALLOC_SIZE);
	    }
	  /* Release memory allocated for XCF/Flash data */
	  kfree(ev_device->xcf_data);
	  ev_device->xcf_data = NULL;
	}
      if (ev_device->devtype &
	  (MRF_DEVTYPE_K7_PCIE))
	{
	  if (ev_device->access_mode == O_RDONLY)
	    {
	    }
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      /* Flush buffer */
	      flash_pageprogram(ev_device, ev_device->xcf_data,
				ev_device->xcf_data_pos,
				FLASH_DATA_ALLOC_SIZE);
	    }
	  /* Release memory allocated for XCF/Flash data */
	  kfree(ev_device->xcf_data);
	  ev_device->xcf_data = NULL;
	}
      ev_device->access_mode = 0;
      ev_device->access_device = 0;
      break;
    case DEVICE_FPGA:
      if (ev_device->devtype &
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
		result = jtag_9030_end_load_fpga(ev_device);
	      else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
		result = jtag_9056_end_load_fpga(ev_device);
	    }
	  /* Release memory allocated for FPGA data */
	  kfree(ev_device->fpga_conf_data);
	  ev_device->fpga_conf_data = NULL;
	  ev_device->access_mode = 0;
	  ev_device->access_device = 0;
	}
      else
	{
	  result = -ENXIO;
	}
      break;	  
    case DEVICE_EV:
      /* Remove from list of asynchronously notified filps */ 
      ev_fasync(-1, filp, 0);
      if (ev_device->refcount_ev)
	ev_device->refcount_ev--;
      if (!ev_device->refcount_ev)
	{
	  /* Release interrupt handler when device is closed for the last
	     time. */
	  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	    {
	      ev_plx_irq_disable(ev_device);
	      free_irq(ev_device->irq, (void *) ev_device);
	    }
	  else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
	    {
	      ev_plx_9056_irq_disable(ev_device);
	      free_irq(ev_device->irq, (void *) ev_device);
	    }
#ifdef LATTICE_IRQ
	  else
	    {
	      ev_irq_disable(ev_device);
	      free_irq(ev_device->irq, (void *) ev_device);
	    }
#endif
	  ev_device->access_mode = 0;
	  ev_device->access_device = 0;
	}
      break;
    default:
      printk(KERN_WARNING DEVICE_NAME ": Trying to release unknown device major %d minor %d\n",
	     MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
      result = -ENXIO;
    }

  up(&ev_device->sem);
  return result;
}

/*
  Read and write
*/

ssize_t ev_read(struct file *filp, char __user *buf, size_t count,
		 loff_t *f_pos)
{
  ssize_t retval = 0;
  struct mrf_dev *ev_device = (struct mrf_dev *) filp->private_data;

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  switch (ev_device->access_device)
    {
    case DEVICE_EEPROM:
      if (ev_device->devtype & 
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  if (*f_pos > ev_device->eeprom_data_size)
	    goto read_out;
	  
	  if (*f_pos + count > ev_device->eeprom_data_size)
	    count = ev_device->eeprom_data_size - *f_pos;
	  
	  if (copy_to_user(buf, &ev_device->eeprom_data[*f_pos], count))
	    {
	      retval = -EFAULT;
	      goto read_out;
	    }
	  
	  *f_pos += count;
	  
	  retval = count;
	}
      break;

    case DEVICE_FLASH:
      {
	if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	  {
	    if ((*f_pos & ~(XCF_DATA_ALLOC_SIZE - 1)) !=
		ev_device->xcf_data_pos)
	      {
		jtag_9030_XCF_read(ev_device, ev_device->xcf_data,
			      (unsigned int) (*f_pos & ~(XCF_DATA_ALLOC_SIZE - 1)),
			      XCF_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos & ~(XCF_DATA_ALLOC_SIZE - 1);
	      }
	    
	    if (*f_pos + count > ev_device->xcf_data_pos +
		XCF_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + XCF_DATA_ALLOC_SIZE - *f_pos;
	    
	    if (copy_to_user(buf, &ev_device->xcf_data[(*f_pos & (XCF_DATA_ALLOC_SIZE - 1))], count))
	      {
		retval = -EFAULT;
		goto read_out;
	      }

	    *f_pos += count;

	    retval = count;
	    break;
	  }
	if (ev_device->devtype &
	    (MRF_DEVTYPE_V5_9056))
	  {
	    if ((*f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1)) !=
		ev_device->xcf_data_pos)
	      {
		flash_fastread(ev_device, ev_device->xcf_data,
			       (unsigned int) (*f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1)),
			       FLASH_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1);
	      }
	    
	    if (*f_pos + count > ev_device->xcf_data_pos +
		FLASH_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + FLASH_DATA_ALLOC_SIZE - *f_pos;
	    
	    if (copy_to_user(buf, &ev_device->xcf_data[(*f_pos & (FLASH_DATA_ALLOC_SIZE - 1))], count))
	      {
		retval = -EFAULT;
		goto read_out;
	      }
	    
	    *f_pos += count;
	    
	    retval = count;
	    break;  
	  }
	if (ev_device->devtype &
	    (MRF_DEVTYPE_ECP3_PCI | MRF_DEVTYPE_ECP3_PCIE))
	  {
	    if ((*f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1)) !=
		ev_device->xcf_data_pos)
	      {
		flash_fastread(ev_device, ev_device->xcf_data,
			       (unsigned int) (*f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1)) +
			       FLASH_SECTOR_PRIMARY_START,
			       FLASH_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1);
	      }
	    
	    if (*f_pos + count > ev_device->xcf_data_pos +
		FLASH_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + FLASH_DATA_ALLOC_SIZE - *f_pos;
	    
	    if (copy_to_user(buf, &ev_device->xcf_data[(*f_pos & (FLASH_DATA_ALLOC_SIZE - 1))], count))
	      {
		retval = -EFAULT;
		goto read_out;
	      }

	    *f_pos += count;

	    retval = count;
	    break;
	  }
	if (ev_device->devtype &
	    (MRF_DEVTYPE_K7_PCIE))
	  {
	    if ((*f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1)) !=
		ev_device->xcf_data_pos)
	      {
		flash_fastread(ev_device, ev_device->xcf_data,
			       (unsigned int) (*f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1)),
			       FLASH_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos & ~(FLASH_DATA_ALLOC_SIZE - 1);
	      }
	    
	    if (*f_pos + count > ev_device->xcf_data_pos +
		FLASH_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + FLASH_DATA_ALLOC_SIZE - *f_pos;
	    
	    if (copy_to_user(buf, &ev_device->xcf_data[(*f_pos & (FLASH_DATA_ALLOC_SIZE - 1))], count))
	      {
		retval = -EFAULT;
		goto read_out;
	      }

	    *f_pos += count;

	    retval = count;
	    break;
	  }
      }
      
    case DEVICE_FPGA:
      if (ev_device->devtype &
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  if (*f_pos > ev_device->fpga_conf_size)
	    goto read_out;
	  
	  if (*f_pos + count > ev_device->fpga_conf_size)
	    count = ev_device->fpga_conf_size - *f_pos;
	  
	  if (copy_to_user(buf, &ev_device->fpga_conf_data[*f_pos], count))
	    {
	      retval = -EFAULT;
	      goto read_out;
	    }
	  
	  *f_pos += count;
	  
	  retval = count;
	  break;
	}
    }

 read_out:
  up(&ev_device->sem);
  return retval;
}

ssize_t ev_write(struct file *filp, const char __user *buf, size_t count,
		 loff_t *f_pos)
{
  ssize_t retval = -ENOMEM;
  struct mrf_dev *ev_device = (struct mrf_dev *) filp->private_data;

#ifdef DEBUG
  printk(KERN_INFO DEVICE_NAME ": write %d bytes.\n", (int) count);
#endif

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  switch (ev_device->access_device)
    {
    case DEVICE_EEPROM:
      if (ev_device->devtype & 
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  if (*f_pos > EEPROM_DATA_ALLOC_SIZE - 1)
	    goto write_out;

	  if (*f_pos + count > EEPROM_DATA_ALLOC_SIZE - 1)
	    count = EEPROM_DATA_ALLOC_SIZE - 1 - *f_pos;

	  if (copy_from_user(&ev_device->eeprom_data[*f_pos], buf, count))
	    {
	      retval = -EFAULT;
	      goto write_out;
	    }

	  *f_pos += count;
	  ev_device->eeprom_data_size = *f_pos;

	  retval = count;
	}
      break;

      case DEVICE_FLASH:
	if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	  {
	    if (*f_pos < ev_device->xcf_data_pos)
	      goto write_out;

	    if (*f_pos + count > ev_device->xcf_data_pos +
		XCF_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + XCF_DATA_ALLOC_SIZE - *f_pos;
	    
	    if (copy_from_user(&ev_device->xcf_data[*f_pos & (XCF_DATA_ALLOC_SIZE - 1)], buf, count))
	      {
		retval = -EFAULT;
		goto write_out;
	      }

	    *f_pos += count;
	    if (*f_pos - ev_device->xcf_data_pos == XCF_DATA_ALLOC_SIZE)
	      {
		jtag_9030_XCF_write(ev_device, ev_device->xcf_data, 
			       ev_device->xcf_data_pos, XCF_DATA_ALLOC_SIZE);
		memset(ev_device->xcf_data, 0xffffffff, XCF_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos;
	      }
	    
	    retval = count;
	    break;
	  }
	if (ev_device->devtype &
	    (MRF_DEVTYPE_V5_9056))
	  {
	    if (*f_pos < ev_device->xcf_data_pos)
	      goto write_out;

	    if (*f_pos + count > ev_device->xcf_data_pos +
		FLASH_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + FLASH_DATA_ALLOC_SIZE - *f_pos;
	    
	    if (copy_from_user(&ev_device->xcf_data[*f_pos & (FLASH_DATA_ALLOC_SIZE - 1)], buf, count))
	      {
		retval = -EFAULT;
		goto write_out;
	      }
	    
	    *f_pos += count;
	    if (*f_pos - ev_device->xcf_data_pos == FLASH_DATA_ALLOC_SIZE)
	      {
		flash_pageprogram(ev_device, ev_device->xcf_data, 
				  ev_device->xcf_data_pos, FLASH_DATA_ALLOC_SIZE);
		memset(ev_device->xcf_data, 0xffffffff, FLASH_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos;
	      }
	    
	    retval = count;
	    break;	    
	  }
	if (ev_device->devtype &
	    (MRF_DEVTYPE_ECP3_PCI | MRF_DEVTYPE_ECP3_PCIE))
	  {
	    if (*f_pos < ev_device->xcf_data_pos)
	      goto write_out;

	    if (*f_pos + count > ev_device->xcf_data_pos +
		FLASH_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + FLASH_DATA_ALLOC_SIZE - *f_pos;

	    if (copy_from_user(&ev_device->xcf_data[*f_pos & (FLASH_DATA_ALLOC_SIZE - 1)], buf, count))
	      {
		retval = -EFAULT;
		goto write_out;
	      }

	    *f_pos += count;
	    if (*f_pos - ev_device->xcf_data_pos == FLASH_DATA_ALLOC_SIZE)
	      {
		flash_pageprogram(ev_device, ev_device->xcf_data, 
				  ev_device->xcf_data_pos + FLASH_SECTOR_PRIMARY_START,
				  FLASH_DATA_ALLOC_SIZE);
		memset(ev_device->xcf_data, 0xffffffff, FLASH_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos;
	      }
	    
	    retval = count;
	    break;
	  }
	if (ev_device->devtype &
	    (MRF_DEVTYPE_K7_PCIE))
	  {
	    if (*f_pos < ev_device->xcf_data_pos)
	      goto write_out;

	    if (*f_pos + count > ev_device->xcf_data_pos +
		FLASH_DATA_ALLOC_SIZE)
	      count = ev_device->xcf_data_pos + FLASH_DATA_ALLOC_SIZE - *f_pos;

	    if (copy_from_user(&ev_device->xcf_data[*f_pos & (FLASH_DATA_ALLOC_SIZE - 1)], buf, count))
	      {
		retval = -EFAULT;
		goto write_out;
	      }

	    *f_pos += count;
	    if (*f_pos - ev_device->xcf_data_pos == FLASH_DATA_ALLOC_SIZE)
	      {
		flash_pageprogram(ev_device, ev_device->xcf_data, 
				  ev_device->xcf_data_pos,
				  FLASH_DATA_ALLOC_SIZE);
		memset(ev_device->xcf_data, 0xffffffff, FLASH_DATA_ALLOC_SIZE);
		ev_device->xcf_data_pos = *f_pos;
	      }
	    
	    retval = count;
	    break;
	  }
    case DEVICE_FPGA:
      if (ev_device->devtype &
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  int i;

	  if (count > FPGA_DATA_ALLOC_SIZE - 1)
	    count = FPGA_DATA_ALLOC_SIZE - 1;

	  if (copy_from_user(ev_device->fpga_conf_data, buf, count))
	    {
	      retval = -EFAULT;
	      goto write_out;
	    }

	  *f_pos += count;
	  ev_device->fpga_conf_size = *f_pos;

	  for (i = 0; i < count; i++)
	    if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	      jtag_9030_load_fpga_byte(ev_device->pLC,
				       ev_device->fpga_conf_data[i]);
	    else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
	      jtag_9056_load_fpga_byte(ev_device->pLC,
				       ev_device->fpga_conf_data[i]);

	  retval = count;
	  break;
	}
    }
  
 write_out:
  up(&ev_device->sem);
  return retval;
}

long ev_unlocked_ioctl(struct file *filp,
	     unsigned int cmd, unsigned long arg)
{
  struct mrf_dev *ev_device = (struct mrf_dev *) filp->private_data;
  int ret = 0;

  /* Check that cmd is valid */
  if (_IOC_TYPE(cmd) != EV_IOC_MAGIC)
    return -ENOTTY;
  if (_IOC_NR(cmd) > EV_IOC_MAX)
    return -ENOTTY;

  /* Check access */
  if (_IOC_DIR(cmd) & _IOC_READ)
    ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    ret = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if (ret)
    return -EFAULT;

  switch (cmd)
    {
    case EV_IOCRESET:
      ret = 0;
      break;

    case EV_IOCIRQEN:
      if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
	ev_plx_irq_enable(ev_device);
      else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
	ev_plx_9056_irq_enable(ev_device);
      else
	ev_irq_enable(ev_device);
      ret = 0;
      break;

    case EV_IOCIRQDIS:
      ret = 0;
      break;

    default:
      return -ENOTTY;
    }

  return ret;
}

/*
  Virtual Memory Area operations
*/

void ev_vma_open(struct vm_area_struct *vma)
{
}

void ev_vma_close(struct vm_area_struct *vma)
{
}

static struct vm_operations_struct ev_remap_vm_ops = {
  .open = ev_vma_open,
  .close = ev_vma_close,
};

int ev_remap_mmap(struct file *filp, struct vm_area_struct *vma)
{
  struct mrf_dev *ev_device = (struct mrf_dev *) filp->private_data;
  unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
  unsigned long physical;
  unsigned long vsize = vma->vm_end - vma->vm_start;
  unsigned long barlen;
  int result;

  printk(KERN_NOTICE DEVICE_NAME ": mmap %d\n", ev_device->access_device);

  switch (ev_device->access_device)
    {
    case DEVICE_EV:
      if (ev_device->devtype &
	  (MRF_DEVTYPE_V2P_9030 | MRF_DEVTYPE_V5_9056))
	{
	  physical = ((unsigned long) ev_device->BAR_start[2]) + offset;
	  barlen = ev_device->BAR_end[2] - ev_device->BAR_start[2] + 1;
	}
      else
	{
	  physical = ((unsigned long) ev_device->BAR_start[0]) + offset;
	  barlen = ev_device->BAR_end[0] - ev_device->BAR_start[0] + 1;
	}
      break;
    case DEVICE_FLASH:
      printk(KERN_NOTICE DEVICE_NAME ": mmap DEVICE_FLASH %d\n", ev_device->access_device);
      if (!(ev_device->devtype & (MRF_DEVTYPE_V5_PCIE)))
	{
	  return -EPERM;
	}
      physical = ((unsigned long) ev_device->BAR_start[1]) + offset;
      barlen = ev_device->BAR_end[1] - ev_device->BAR_start[1] + 1;
      break;
    default:
      printk(KERN_NOTICE DEVICE_NAME ": mmap not allowed for access_device %d\n",
	     ev_device->access_device);
      return -EPERM;
    }

  if (vsize > barlen)
    {
      printk(KERN_NOTICE DEVICE_NAME ": mmap requested window %08x, device %d window %08x\n",
	     (unsigned int) vsize, (unsigned int) ev_device->access_device,
	     (unsigned int) barlen);
      return -EINVAL;
    }

  printk(KERN_NOTICE DEVICE_NAME ": mmap physical %08lx, vsize %08lx\n",
	 physical, vsize);

#if LINUX_VERSION_CODE > (0x020609)
  result = io_remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT,
			      vsize, vma->vm_page_prot);
#else
  result = io_remap_page_range(vma, vma->vm_start, physical, vsize, 
			       vma->vm_page_prot);
#endif

  if (result)
    return -EAGAIN;
  
  vma->vm_ops = &ev_remap_vm_ops;
  ev_vma_open(vma);
  
  return 0;  
}

int ev_fasync(int fd, struct file *filp, int mode)
{
  struct mrf_dev *ev_device = (struct mrf_dev *) filp->private_data;

  return fasync_helper(fd, filp, mode, &ev_device->async_queue);
}

#if LINUX_VERSION_CODE > (0x020619)
irqreturn_t ev_interrupt(int irq, void *dev_id)
#else
irqreturn_t ev_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
  struct mrf_dev *ev_device = (struct mrf_dev *) dev_id;
  volatile struct Pci9030LocalConf *pLC = ev_device->pLC;
  volatile struct Pci9056LocalConf *pLC9056 = (struct Pci9056LocalConf *) ev_device->pLC;
  volatile u32 *evr_irq_enable = ((ev_device->pEv) + EV_IRQ_ENABLE_OFFSET);
  volatile u32 *evr_irq_flag   = ((ev_device->pEv) + EV_IRQ_FLAG_OFFSET);

  printk(KERN_WARNING DEVICE_NAME ": EV irq addr %p, %08x\n",
	 evr_irq_flag, (int) be32_to_cpu(*evr_irq_flag));
  printk(KERN_WARNING DEVICE_NAME ": EV irq addr %p, %08x\n",
	 evr_irq_enable, (int) be32_to_cpu(*evr_irq_enable));

  /* We use shared interrupts: return immediately if irq is not from our
     device. */
  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
    {
      if (!(le16_to_cpu(pLC->INTCSR) & PLX9030_INTCSR_LINTI1_STAT))
      return IRQ_NONE;
    }
  else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
    {
      printk(KERN_WARNING DEVICE_NAME ": V5_9056 INTCSR %08x\n",
	     pLC9056->INTCSR);
      if (!(le32_to_cpu(pLC9056->INTCSR) & PLX9056_INTCSR_LINTI))
	return IRQ_NONE;
    }
  else
    if (!(be32_to_cpu(*evr_irq_flag) & EV_IRQ_PCI_DRIVER_ENA))
      return IRQ_NONE;

  /* We turn off the interrupt line here to signal the kernel that the interrupt is
     handled and we notify the user space application about a pending irq. */
  if (ev_device->devtype & MRF_DEVTYPE_V2P_9030)
    ev_plx_irq_disable(ev_device);
  else if (ev_device->devtype & MRF_DEVTYPE_V5_9056)
    ev_plx_9056_irq_disable(ev_device);
  else
    ev_irq_disable(ev_device);

  printk(KERN_WARNING DEVICE_NAME ": EVx irq sending signal\n");

  kill_fasync(&ev_device->async_queue, SIGIO, POLL_IN);
  return IRQ_HANDLED;
}

int ev_irq_enable(struct mrf_dev *ev_device)
{
  volatile int *evr_irq_enable = ((ev_device->pEv) + EV_IRQ_ENABLE_OFFSET);
  volatile int *evr_mirq_enable = ((ev_device->pEv) + EV_MIRQ_ENABLE_OFFSET);

  /* PCI master interrupt enable bit moved starting from FW version 0x000A */
  if ((((ev_device->fw_version & 0xF0000000) == 0x10000000)
       && ((ev_device->fw_version & 0x0FEFF) >= 0x000A)) ||
      (((ev_device->fw_version & 0xF0000000) == 0x20000000)
       && ((ev_device->fw_version & 0x0FEFF) >= 0x0008)))
    {
      *evr_mirq_enable |= __constant_cpu_to_be32(EV_IRQ_PCI_DRIVER_ENA);
      printk(KERN_WARNING DEVICE_NAME ": EVR %08x irq addr %08x, %08x enable\n",
	     (int) ev_device->fw_version,
	     (int) evr_mirq_enable, (int) be32_to_cpu(*evr_mirq_enable));
    }
  else
    {
      *evr_irq_enable |= __constant_cpu_to_be32(EV_IRQ_PCI_DRIVER_ENA);
      printk(KERN_WARNING DEVICE_NAME ": EVR %08x irq addr %08x, %08x enable\n",
	     (int) ev_device->fw_version,
	     (int) evr_irq_enable, (int) be32_to_cpu(*evr_irq_enable));
    }

  barrier();
  return be32_to_cpu(*evr_irq_enable);
}

int ev_irq_disable(struct mrf_dev *ev_device)
{
  volatile int *evr_irq_enable = ((ev_device->pEv) + EV_IRQ_ENABLE_OFFSET);
  volatile int *evr_mirq_enable = ((ev_device->pEv) + EV_MIRQ_ENABLE_OFFSET);

  /* PCI master interrupt enable bit moved starting from FW version 0x000A */
  if ((((ev_device->fw_version & 0xF0000000) == 0x10000000)
       && ((ev_device->fw_version & 0x0FEFF) >= 0x000A)) ||
      (((ev_device->fw_version & 0xF0000000) == 0x20000000)
       && ((ev_device->fw_version & 0x0FEFF) >= 0x0008)))
    {
      *evr_mirq_enable &= __constant_cpu_to_be32(~EV_IRQ_PCI_DRIVER_ENA);
      printk(KERN_WARNING DEVICE_NAME ": EVR %08x irq addr %08x, %08x disable\n",
	     (int) ev_device->fw_version,
	     (int) evr_mirq_enable, (int) be32_to_cpu(*evr_mirq_enable));
    }
  else
    {
      *evr_irq_enable &= __constant_cpu_to_be32(~EV_IRQ_PCI_DRIVER_ENA);
      printk(KERN_WARNING DEVICE_NAME ": EVR %08x irq addr %08x, %08x disable\n",
	     (int) ev_device->fw_version,
	     (int) evr_irq_enable, (int) be32_to_cpu(*evr_irq_enable));
    }

  barrier();
  return be32_to_cpu(*evr_irq_enable);
}

int ev_plx_irq_enable(struct mrf_dev *ev_device)
{
  volatile struct Pci9030LocalConf *pLC = ev_device->pLC;

  pLC->INTCSR = __constant_cpu_to_le16(PLX9030_INTCSR_LINTI1_ENA |
				       PLX9030_INTCSR_LINTI1_POL |
				       PLX9030_INTCSR_PCI_IRQENA);
  barrier();
  return le16_to_cpu(pLC->INTCSR);
}

int ev_plx_irq_disable(struct mrf_dev *ev_device)
{
  volatile struct Pci9030LocalConf *pLC = ev_device->pLC;

  pLC->INTCSR = __constant_cpu_to_le16(PLX9030_INTCSR_LINTI1_ENA |
				       PLX9030_INTCSR_LINTI1_POL);
  barrier();
  return le16_to_cpu(pLC->INTCSR);
}

int ev_plx_9056_irq_enable(struct mrf_dev *ev_device)
{
  volatile struct Pci9056LocalConf *pLC = ev_device->pLC;

  printk("ev_plx_9056_irq_enable\n");
  pLC->INTCSR = __constant_cpu_to_le32(PLX9056_INTCSR_LINTI_ENA |
                                       PLX9056_INTCSR_PCI_IRQENA);
  barrier();
  return le32_to_cpu(pLC->INTCSR);
}

int ev_plx_9056_irq_disable(struct mrf_dev *ev_device)
{
  volatile struct Pci9056LocalConf *pLC = ev_device->pLC;

  printk("ev_plx_9056_irq_disable\n");
  pLC->INTCSR = __constant_cpu_to_le32(PLX9056_INTCSR_LINTI_ENA);
  barrier();
  return le32_to_cpu(pLC->INTCSR);
}
