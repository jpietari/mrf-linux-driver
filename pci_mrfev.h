/*
  pci_mrfev.h -- Definitions for Micro-Research Event Generator /
                 Event Receiver Linux 2.6 driver

  Author: Jukka Pietarinen (MRF)
  Date:   29.11.2006

*/

#define DEVICE_MINOR_NUMBERS      4
#define MAX_MRF_DEVICES           8
#define MAX_MRF_BARS              3

#define PCI_VENDOR_ID_LATTICE       0x1204
#define PCI_DEVICE_ID_LATTICE_ECP3  0xec30
#define PCI_VENDOR_ID_XILINX        0x10ee
#define PCI_DEVICE_ID_ZOMOJO_Z1     0x0007
#define PCI_DEVICE_ID_0505          0x0505
#define PCI_DEVICE_ID_KINTEX7       0x7011
#define PCI_DEVICE_ID_PLX_9056      0x9056

#define PCI_VENDOR_ID_MRF           0x1A3E
#define PCI_DEVICE_ID_MRF_PMCEVR200 0x10C8
#define PCI_DEVICE_ID_MRF_PMCEVR230 0x11E6
#define PCI_DEVICE_ID_MRF_PXIEVR220 0x10DC
#define PCI_DEVICE_ID_MRF_PXIEVG220 0x20DC
#define PCI_DEVICE_ID_MRF_PXIEVR230 0x10E6
#define PCI_DEVICE_ID_MRF_PXIEVG230 0x20E6
#define PCI_DEVICE_ID_MRF_CPCIEVR300 0x152C
#define PCI_DEVICE_ID_MRF_PCIEEVR300 0x172C
#define PCI_DEVICE_ID_MRF_CPCIEVG300 0x252C
#define PCI_DEVICE_ID_MRF_PXIEEVR300 0x112C
#define PCI_DEVICE_ID_MRF_PXIEEVG300 0x212C
#define PCI_DEVICE_ID_MRF_CPCIEVRTG 0x192C
#define PCI_DEVICE_ID_MRF_CPCIFCT   0x30E6
#define PCI_DEVICE_ID_MRF_MTCAEVR300 0x132C
#define PCI_DEVICE_ID_MRF_MTCAEVG300 0x232C

#define MODULE_VENDOR_ID_NOCONF     PCI_VENDOR_ID_PLX
#define MODULE_DEVICE_ID_NOCONF     PCI_DEVICE_ID_PLX_9030
#define MODULE_SUBVENDOR_ID_NOCONF  0
#define MODULE_SUBDEVICE_ID_NOCONF  0

#define DEVICE_MINORS 4
#define DEVICE_FIRST  0
#define DEVICE_EEPROM 0
#define DEVICE_FLASH  1
#define DEVICE_FPGA   2
#define DEVICE_EV     3
#define DEVICE_LAST   3

/* Define the maximum number of words in EEPROM */
#define EEPROM_MAX_WORDS            256

/* Define space needed for data */
#define EEPROM_DATA_ALLOC_SIZE      0x00002000
/*
#define XCF_DATA_ALLOC_SIZE         0x00100000
#define FPGA_DATA_ALLOC_SIZE        0x00100000
*/
#define XCF_DATA_ALLOC_SIZE         0x000400
#define FPGA_DATA_ALLOC_SIZE        0x001000
#define FLASH_DATA_ALLOC_SIZE       (256)
#define FLASH_SECTOR_SIZE           (0x00010000)
#define FLASH_SECTOR_PRIMARY_START  (0x00010000)
#define FLASH_SECTOR_PRIMARY_END    (0x00260000)

#define XCF_BLOCK_SIZE              1024
#define XCF_ERASE_TCKS              140000000

#define EV_IRQ_FLAG_OFFSET          (0x0008)
#define EV_IRQ_ENABLE_OFFSET        (0x000C)
#define EV_MIRQ_ENABLE_OFFSET       (0x001C)
#define EV_FW_VERSION_OFFSET        (0x002C)
#define EV_IRQ_PCI_DRIVER_ENA       (0x40000000)

#define EV_SPI_DATA_OFFSET          (0x00A0)
#define EV_SPI_CONTROL_OFFSET       (0x00A4)
#define EV_SPI_CONTROL_RRDY         (0x0040)
#define EV_SPI_CONTROL_TRDY         (0x0020)
#define EV_SPI_CONTROL_TMT          (0x0010)
#define EV_SPI_CONTROL_OE           (0x0002)
#define EV_SPI_CONTROL_SELECT       (0x0001)
#define SPI_RETRY_COUNT             (10000)
#define SPI_BE_COUNT                (0x10000000)

#define M25P_FAST_READ              (0x0B)
#define M25P_RDID                   (0x9F)
#define M25P_WREN                   (0x06)
#define M25P_WRDI                   (0x04)
#define M25P_RDSR                   (0x05)
#define M25P_SE                     (0xD8)
#define M25P_BE                     (0xC7)
#define M25P_PP                     (0x02)
#define M25P_STATUS_WIP             (0x01)

#define MRF_DEVTYPE_V2P_9030        (0x00000001)
#define MRF_DEVTYPE_V5_9056         (0x00000002)
#define MRF_DEVTYPE_V5_PCIE         (0x00000004)
#define MRF_DEVTYPE_ECP3_PCI        (0x00000008)
#define MRF_DEVTYPE_ECP3_PCIE       (0x00000010)
#define MRF_DEVTYPE_K7_PCIE         (0x00000020)

struct mrf_dev {
  int    access_mode;        /* Only one minor device is allowed open
                                at a time. access_mode hold either 0 for
                                no devices open or O_WRONLY or O_RDONLY. */
  int    access_device;      /* access_device holds the minor number of
                                the device open. */
  int    refcount_ev;        /* FPGA memory mapped register map reference
				count. We allow several simultaneos connections
				on this minor device and keep track on the
				number of these connections. */
  char             *eeprom_data;
  int              eeprom_data_size;
  char             *xcf_data;
  int              xcf_data_pos;
  char             *fpga_conf_data;
  int              fpga_conf_size;
  int              major;          /* Hold major number of device */
  u16              subsys_id;      /* Hold subsystem ID of device */
  u32              devtype;        /* Hold device type id bits */
  struct cdev      cdev;
  struct semaphore sem;
  unsigned long    BAR_start[MAX_MRF_BARS];
  unsigned long    BAR_end[MAX_MRF_BARS];
  unsigned long    BAR_flags[MAX_MRF_BARS];
  void             *BAR_mmapped[MAX_MRF_BARS];
  void             *pLC;
  void             *pEv;
  unsigned int     jtag_refcount;
  int              fpgaid;
  int              xcfid;
  int              fpgairlen;
  int              xcfirlen;
  int              irq;            /* Interrupt line */
  unsigned long    fw_version;
  struct fasync_struct *async_queue;
};

/* fops prototypes */

int ev_open(struct inode *inode, struct file *filp);
int ev_release(struct inode *inode, struct file *filp);
ssize_t ev_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos);
ssize_t ev_write(struct file *filp, const char __user *buf, size_t count,
		 loff_t *f_pos);
#ifdef HAVE_UNLOCKED_IOCTL
long ev_unlocked_ioctl(struct file *filp,
		       unsigned int cmd, unsigned long arg);
#else
int ev_ioctl(struct inode *inode, struct file *filp,
	     unsigned int cmd, unsigned long arg);
#endif
int ev_fasync(int fd, struct file *filp, int mode);
int ev_remap_mmap(struct file *filp, struct vm_area_struct *vma);
#if LINUX_VERSION_CODE > (0x020619)
irqreturn_t ev_interrupt(int irq, void *dev_id);
#else
irqreturn_t ev_interrupt(int irq, void *dev_id, struct pt_regs *regs);
#endif

#define EV_IOC_MAGIC 220
#define EV_IOCRESET  _IO(EV_IOC_MAGIC, 0)
#define EV_IOCIRQEN  _IO(EV_IOC_MAGIC, 1)
#define EV_IOCIRQDIS _IO(EV_IOC_MAGIC, 2)

#define EV_IOC_MAX   3

#define PLX9030_INTCSR_LINTI1_ENA  0x0001 /* LINTi1 enable */
#define PLX9030_INTCSR_LINTI1_POL  0x0002 /* LINTi1 polarity, 1 = active high */
#define PLX9030_INTCSR_LINTI1_STAT 0x0004 /* LINTi1 status, 1 = interrupt active */
#define PLX9030_INTCSR_LINTI2_ENA  0x0008 /* LINTi2 enable */
#define PLX9030_INTCSR_LINTI2_POL  0x0010 /* LINTi2 polarity, 1 = active high */
#define PLX9030_INTCSR_LINTI2_STAT 0x0020 /* LINTi2 status, 1 = interrupt active */
#define PLX9030_INTCSR_PCI_IRQENA  0x0040 /* PCI interrupt enable, 1 = enabled */
#define PLX9030_INTCSR_SWINT       0x0080 /* Software interrupt, 1 = generate PCI IRQ */
#define PLX9030_INTCSR_LINTI1_SENA 0x0100 /* LINTi1 select enable,
					     0 = level, 1 = edge triggerable */
#define PLX9030_INTCSR_LINTI2_SENA 0x0200 /* LINTi1 select enable,
					     0 = level, 1 = edge triggerable */
#define PLX9030_INTCSR_LINTI1_ICLR 0x0400 /* LINTi1 edge triggerable IRQ clear,
					     writing 1 clears irq */
#define PLX9030_INTCSR_LINTI2_ICLR 0x0800 /* LINTi2 edge triggerable IRQ clear,
					     writing 1 clears irq */

#define PLX9030_CNTRL_EESK         0x01000000
#define PLX9030_CNTRL_EECS         0x02000000
#define PLX9030_CNTRL_EEDI         0x04000000
#define PLX9030_CNTRL_EEDO         0x08000000
#define PLX9030_CNTRL_RESET        0x00780000

#define PLX9030_GPIOC_GPIO0_nWAIT  0x00000001
#define PLX9030_GPIOC_GPIO0_DIR_O  0x00000002
#define PLX9030_GPIOC_GPIO0_DATA   0x00000004
#define PLX9030_GPIOC_GPIO1_nLLOCK 0x00000008
#define PLX9030_GPIOC_GPIO1_DIR_O  0x00000010
#define PLX9030_GPIOC_GPIO1_DATA   0x00000020
#define PLX9030_GPIOC_GPIO2_nCS2   0x00000040
#define PLX9030_GPIOC_GPIO2_DIR_O  0x00000080
#define PLX9030_GPIOC_GPIO2_DATA   0x00000100
#define PLX9030_GPIOC_GPIO3_nCS3   0x00000200
#define PLX9030_GPIOC_GPIO3_DIR_O  0x00000400
#define PLX9030_GPIOC_GPIO3_DATA   0x00000800

#define PLX9030_GPIOC_TMS        PLX9030_GPIOC_GPIO1_DATA
#define PLX9030_GPIOC_TDI        PLX9030_GPIOC_GPIO3_DATA
#define PLX9030_GPIOC_TCK        PLX9030_GPIOC_GPIO0_DATA
#define PLX9030_GPIOC_TDO        PLX9030_GPIOC_GPIO2_DATA
#define PLX9030_GPIOC_RESET      0x00249000
#define PLX9030_GPIOC_INIT       0x00249C36
#define PLX9030_GPIOC_JTAG       0x00249412

#define XC2VP_CFGOUT    0xffc4
#define XC2VP_CFGIN     0xffc5
#define XC2VP_IDCODE    0xffc9
#define XC2VP_JPROG_B   0xffcb
#define XC2VP_JSTART    0xffcc
#define XC2VP_JSHUTDOWN 0xffcd
#define XC2VP_BYPASS    0xffff

#define XC2VP4_DID    0x0123E093L
#define XC2VP7_DID    0x0124A093L
#define XC2VP20_DID   0x01266093L
#define XC2VP30_DID   0x0127e093L
#define XC5FX70T_DID  0x032c6093L
#define XCF08P_DID    0x05057093L
#define XCF16P_DID    0x05058093L
#define XC2VP4_IRLEN  10
#define XC2VP7_IRLEN  10
#define XC2VP20_IRLEN 14
#define XC2VP30_IRLEN 14
#define XCFXXP_IRLEN  16
#define XC5FXT_IRLEN  10

#define XCFXXP_BYPASS            0xffff
#define XCFXXP_SAMPLE            0x0001
#define XCFXXP_EXTEST            0x0000
#define XCFXXP_IDCODE            0x00fe
#define XCFXXP_USERCODE          0x00fd
#define XCFXXP_HIGHZ             0x00fc
#define XCFXXP_CLAMP             0x00fa
#define XCFXXP_ISC_ENABLE        0x00e8
#define XCFXXP_XSC_ENABLEC       0x00e9
#define XCFXXP_ISC_PROGRAM       0x00ea
#define XCFXXP_ISC_ADDRESS_SHIFT 0x00eb
#define XCFXXP_ISC_READ          0x00f8
#define XCFXXP_ISC_ERASE         0x00ec
#define XCFXXP_ISC_DATA_SHIFT    0x00ed
#define XCFXXP_XSC_READ          0x00ef
#define XCFXXP_XSC_BLANK_CHECK   0x000d
#define XCFXXP_ISC_DISABLE       0x00f0
#define XCFXXP_ISC_NOOP          0x00e0
#define XCFXXP_XSC_CONFIG        0x00ee
#define XCFXXP_XSC_CLR_STATUS    0x00f4
#define XCFXXP_XSC_OP_STATUS     0x00e3
#define XCFXXP_XSC_MFG_READ      0x00f1
#define XCFXXP_XSC_CONFIG_PLUS   0x00f6
#define XCFXXP_XSC_UNLOCK        0xaa55
#define XCFXXP_XSC_DATA_BTC      0x00f2
#define XCFXXP_XSC_DATA_WRPT     0x00f7
#define XCFXXP_XSC_DATA_RDPT     0x0004
#define XCFXXP_XSC_DATA_UC       0x0006
#define XCFXXP_XSC_DATA_CC       0x0007
#define XCFXXP_XSC_DATA_DONE     0x0009
#define XCFXXP_XSC_DATA_CCB      0x000c
#define XCFXXP_XSC_DATA_BLANK    0x00f5
#define XCFXXP_XSC_DATA_SUCR     0x000e
#define XCFXXP_XSC_ADDRESS_DUMP  0x00e6

struct Pci9030LocalConf
{
  u32 LAS0RR;    /* 0x00 Local Address Space 0 Range */
  u32 LAS1RR;    /* 0x04 Local Address Space 1 Range */
  u32 LAS2RR;    /* 0x08 Local Address Space 2 Range */
  u32 LAS3RR;    /* 0x0C Local Address Space 3 Range */
  u32 EROMRR;    /* 0x10 Expansion ROM Range */
  u32 LAS0BA;    /* 0x14 Local Address Space 0 Local Base Address */
  u32 LAS1BA;    /* 0x18 Local Address Space 1 Local Base Address */
  u32 LAS2BA;    /* 0x1C Local Address Space 2 Local Base Address */
  u32 LAS3BA;    /* 0x20 Local Address Space 3 Local Base Address */
  u32 EROMBA;    /* 0x24 Expansion ROM Local Base Address */
  u32 LAS0BRD;   /* 0x28 Local Address Space 0 Bus Region Descriptor */
  u32 LAS1BRD;   /* 0x2C Local Address Space 1 Bus Region Descriptor */
  u32 LAS2BRD;   /* 0x30 Local Address Space 2 Bus Region Descriptor */
  u32 LAS3BRD;   /* 0x34 Local Address Space 3 Bus Region Descriptor */
  u32 EROMBRD;   /* 0x38 Expansion ROM Bus Region Descriptor */
  u32 CS0BASE;   /* 0x3C Chip Select 0 Base Address */
  u32 CS1BASE;   /* 0x40 Chip Select 1 Base Address */
  u32 CS2BASE;   /* 0x44 Chip Select 2 Base Address */
  u32 CS3BASE;   /* 0x48 Chip Select 3 Base Address */
  u16 INTCSR;    /* 0x4C Interrupt Control/Status */
  u16 PROT_AREA; /* 0x4E Serial EEPROM Write-Protected Address Boundary */
  u32 CNTRL;     /* 0x50 PCI Target Response, Serial EEPROM, and
                       Initialization Control */
  u32 GPIOC;     /* 0x54 General Purpose I/O Control */
  u32 reserved1; /* 0x58 */
  u32 reserved2; /* 0x5C */
  u32 reserved3; /* 0x60 */
  u32 reserved4; /* 0x64 */
  u32 reserved5; /* 0x68 */
  u32 reserved6; /* 0x6C */
  u32 PMDATASEL; /* 0x70 Hidden 1 Power Management Data Select */
  u32 PMDATASCALE; /* 0x74 Hidden 2 Power Management Data Scale */
};

#define PLX9056_CNTRL_EESK         (0x01000000)
#define PLX9056_CNTRL_EECS         (0x02000000)
#define PLX9056_CNTRL_EEDI         (0x04000000)
#define PLX9056_CNTRL_EEDO         (0x08000000)
#define PLX9056_CNTRL_EEDO_IE      (0x80000000)
#define PLX9056_CNTRL_RESET        (0x000d767e)

#define PLX9056_CNTRL_TMS          (0x00010000)
#define PLX9056_CNTRL_TDO          (0x00020000)
#define PLX9056_CNTRL_TDI          (PLX9056_CNTRL_EEDI)
#define PLX9056_CNTRL_TCK          (PLX9056_CNTRL_EESK)
#define PLX9056_CNTRL_JTAG         (PLX9056_CNTRL_RESET & \
				    ~(PLX9056_CNTRL_TMS | PLX9056_CNTRL_TDI \
				      | PLX9056_CNTRL_TCK))

#define PLX9056_INTCSR_PCI_IRQENA  (0x00000100)
#define PLX9056_INTCSR_LINTI_ENA   (0x00000800)
#define PLX9056_INTCSR_LINTI       (0x00008000)

struct Pci9056LocalConf
{
  u32 LAS0RR;     /* 0x00 Local Address Space 0 Range */
  u32 LAS0BA;     /* 0x04 Local Address Space 0 Local Base Address */
  u32 MARBR;      /* 0x08 Mode/DMA Arbitration */
  u8 BIGEND;    /* 0x0C Big/Little Endian Descriptor */
  u8 LMISC1;    /* 0x0D Local Miscellanous Control 1 */
  u8 PROT_AREA; /* 0x0E Serial EEPROM Write-Protected Address Boundary */
  u8 LMISC2;    /* 0x0F Local Miscellanous Control 2 */
  u32 EROMRR;     /* 0x10 Direct Slave Expansion ROM Range */
  u32 EROMBA;     /* 0x14 Direct Slave Expansion ROM Local Base Address (Remap) */
  u32 LBRD0;      /* 0x18 Local Address Space 0/Expansion ROM Bus Region Descriptor */
  u32 DMRR;       /* 0x1C Local Range for Direct Master-to-PCI */
  u32 DMLBAM;     /* 0x20 Local Base Address for Direct Master-to-PCI Memory */
  u32 DMLBAI;     /* 0x24 Local Base Address for Direct Master-to-PCI I/O Configuration */
  u32 DMPBAM;     /* 0x28 PCI Base Address (Remap) for Direct Master-to-PCI Memory */
  u32 DMCFGA;     /* 0x2C PCI Configuration Address for Direct Master-to-PCI I/O Configuration */
  u32 OPQIS;     /* 0x30 Outbound Post Queue Interrupt Status */
  u32 OPQIM;     /* 0x34 Outbound Post Queue Interrupt Mask */
  u32 res_0x38;  /* 0x38 */
  u32 res_0x3C;  /* 0x3C */
  u32 IQP;       /* 0x40 Inbound Queue Port */
  u32 OQP;       /* 0x44 Outbound Queue Port */
  u32 MBOX2;     /* 0x48 Mailbox 2 */
  u32 MBOX3;     /* 0x4C Mailbox 3 */
  u32 MBOX4;     /* 0x50 Mailbox 4 */
  u32 MBOX5;     /* 0x54 Mailbox 5 */
  u32 MBOX6;     /* 0x58 Mailbox 6 */
  u32 MBOX7;     /* 0x5C Mailbox 7 */
  u32 P2LDBELL;  /* 0x60 PCI-to-Local Doorbell */
  u32 L2PDBELL;  /* 0x64 Local-to-PCI Doorbell */
  u32 INTCSR;    /* 0x68 Interrupt Control/Status */
  u32 CNTRL;     /* 0x6C PCI Target Response, Serial EEPROM, and Initialization Control */
  u32 PCIHIDR;   /* 0x70 PCI Hardwired Configuration ID */
  u32 PCIHREV;   /* 0x74 PCI Hardwired Revision ID */
  u32 MBOX0;     /* 0x78 Mailbox 0 */
  u32 MBOX1;     /* 0x7C Mailbox 1 */
  u32 DMAMODE0;  /* 0x80 DMA Channel 0 Mode */
  u32 DMAPADR0;  /* 0x84 DMA Channel 0 PCI Address */
  u32 DMALADR0;  /* 0x88 DMA Channel 0 Local Address */
  u32 DMASIZ0;   /* 0x8C DMA Channel 0 Transfer Size (Bytes) */
  u32 DMADPR0;   /* 0x90 DMA Channel 0 Descriptor Pointer */
  u32 DMAMODE1;  /* 0x94 DMA Channel 1 Mode */
  u32 DMAPADR1;  /* 0x98 DMA Channel 1 PCI Address */
  u32 DMALADR1;  /* 0x9C DMA Channel 1 Local Address */
  u32 DMASIZ1;   /* 0xA0 DMA Channel 1 Transfer Size (Bytes) */
  u32 DMADPR1;   /* 0xA4 DMA Channel 1 Descriptor Pointer */
  u8 DMACSR0;  /* 0xA8 DMA Channel 0 Command/Status */
  u8 DMACSR1;  /* 0xA9 DMA Channel 1 Command/Status */
  u16 res_0xAA;
  u32 DMAARB;    /* 0xAC Mode/DMA Arbitration (copy of 0x08) */
  u32 DMATHR;    /* 0xB0 DMA Threshold */
  u32 DMADAC0;   /* 0xB4 DMA Channel 0 PCI Dual Address Cycles Upper Address */
  u32 DMADAC1;   /* 0xB8 DMA Channel 1 PCI Dual Address Cycles Upper Address */
  u32 res_0xBC;
  u32 MQCR;      /* 0xC0 Messaging Queue Configuration */
  u32 QBAR;      /* 0xC4 Queue Base Address */
  u32 IFHPR;     /* 0xC8 Inbound Free Head Pointer */
  u32 IFTPR;     /* 0xCC Inbound Free Tail Pointer */
  u32 IPHPR;     /* 0xD0 Inbound Post Head Pointer */
  u32 IPTPR;     /* 0xD4 Inbound Post Tail Pointer */
  u32 OFHPR;     /* 0xD8 Outbound Free Head Pointer */
  u32 OFTPR;     /* 0xDC Outbound Free Tail Pointer */
  u32 OPHPR;     /* 0xE0 Outbound Post Head Pointer */
  u32 OPTPR;     /* 0xE4 Outbound Post Tail Pointer */
  u32 QSR;       /* 0xE8 Queue Status/Control */
  u32 res_0xEC;
  u32 LAS1RR;    /* 0xF0 Direct Slave Local Address Space 1 Range */
  u32 LAS1BA;    /* 0xF4 Direct Slave Local Address Space 1 Local Base Address (Remap) */
  u32 LAS1BRD;   /* 0xF8 Local Address Space 1 Bus Region Descriptor */
  u32 DMDAC;     /* 0xFC Direct Master PCI Dual Address Cycles Upper Address */
  u32 PCIARB;    /* 0x100 PCI Arbiter Control */
  u32 PABTADR;   /* 0x104 PCI Abort Address */
};

void evr_dumpLC(struct Pci9030LocalConf *pLC);
int eeprom_9030_sprint(struct Pci9030LocalConf *pLC, char *buf, int size);
int eeprom_9056_sprint(struct Pci9056LocalConf *pLC, char *buf, int size);
int eeprom_9030_sscan(struct Pci9030LocalConf *pLC, char *buf);
int eeprom_9056_sscan(struct Pci9056LocalConf *pLC, char *buf);

void jtag_9030_init(struct mrf_dev *ev_dev);
int jtag_9030_open(struct mrf_dev *ev_dev);
int jtag_9030_release(struct mrf_dev *ev_dev);
void jtag_9030_pulseTCK(volatile struct Pci9030LocalConf *pLC, int n);
void jtag_9030_TLR(volatile struct Pci9030LocalConf *pLC);
void jtag_9030_TLRtoRTI(volatile struct Pci9030LocalConf *pLC);
void jtag_9030_TLRtoSIR(volatile struct Pci9030LocalConf *pLC);
void jtag_9030_TLRtoSDR(volatile struct Pci9030LocalConf *pLC);
void jtag_9030_EX1toSDR(volatile struct Pci9030LocalConf *pLC);
void jtag_9030_EX1toRTI(volatile struct Pci9030LocalConf *pLC);
int jtag_9030_SDI(volatile struct Pci9030LocalConf *pLC, int n, int tdi,
	     int *tdo, int last);
void jtag_9030_SCFD(volatile struct Pci9030LocalConf *pLC, char cb, int last);
int jtag_9030_validate(struct mrf_dev *ev_dev);
int jtag_9030_SIR(volatile struct Pci9030LocalConf *pLC, int hir, int irlen,
	     int tdi, int tir, int *tdo, int tlr);
int jtag_9030_SDR(volatile struct Pci9030LocalConf *pLC, int hdr, int drlen,
	     char *tdi, int tdr, char *tdo, int tlr);
/* Read FPGA status register (see Configuration Details in Xilinx
   Virtex-II Pro Users Guide, ug012.pdf) */

int jtag_9030_read_fpga_status(struct mrf_dev *ev_dev);
int fpga_9030_sprint(struct mrf_dev *ev_dev, char *buf, int size);

/* Issue JTAG command XSC_CONFIG which forces PROG_B pin on Platform
   Flash low and initiates a new configuration from Flash */ 

int jtag_9030_XCF_conf(struct mrf_dev *ev_dev);

/* Read Platform Flash contents.
   addr and size have to be multiples of 1024 */

int jtag_9030_XCF_readstart(struct mrf_dev *ev_dev);
int jtag_9030_XCF_read(struct mrf_dev *ev_dev,
		  char *data, unsigned int addr, unsigned int size);
int jtag_9030_XCF_readend(struct mrf_dev *ev_dev);

/* Program data to Platform Flash 
   These functions erase the flash prior to programming */

int jtag_9030_XCF_progstart(struct mrf_dev *ev_dev);
int jtag_9030_XCF_write(struct mrf_dev *ev_dev, char *data,
		   int addr, int size);
int jtag_9030_XCF_progend(struct mrf_dev *ev_dev);

/* Configure FPGA from .bit file.
   Startup clock has to be set to JTAG in bitgen. */

int jtag_9030_init_load_fpga(struct mrf_dev *ev_dev);
#define jtag_9030_load_fpga_byte(x, y) jtag_9030_SCFD(x, y, 0)
int jtag_9030_end_load_fpga(struct mrf_dev *ev_dev);

void jtag_9056_init(struct mrf_dev *ev_dev);
int jtag_9056_open(struct mrf_dev *ev_dev);
int jtag_9056_release(struct mrf_dev *ev_dev);
void jtag_9056_pulseTCK(volatile struct Pci9056LocalConf *pLC, int n);
void jtag_9056_TLR(volatile struct Pci9056LocalConf *pLC);
void jtag_9056_TLRtoRTI(volatile struct Pci9056LocalConf *pLC);
void jtag_9056_TLRtoSIR(volatile struct Pci9056LocalConf *pLC);
void jtag_9056_TLRtoSDR(volatile struct Pci9056LocalConf *pLC);
void jtag_9056_EX1toSDR(volatile struct Pci9056LocalConf *pLC);
void jtag_9056_EX1toRTI(volatile struct Pci9056LocalConf *pLC);
int jtag_9056_SDI(volatile struct Pci9056LocalConf *pLC, int n, int tdi,
	     int *tdo, int last);
void jtag_9056_SCFD(volatile struct Pci9056LocalConf *pLC, char cb, int last);
int jtag_9056_validate(struct mrf_dev *ev_dev);
int jtag_9056_SIR(volatile struct Pci9056LocalConf *pLC, int hir, int irlen,
	     int tdi, int tir, int *tdo, int tlr);
int jtag_9056_SDR(volatile struct Pci9056LocalConf *pLC, int hdr, int drlen,
	     char *tdi, int tdr, char *tdo, int tlr);
/* Read FPGA status register (see Configuration Details in Xilinx
   Virtex-II Pro Users Guide, ug012.pdf) */

int jtag_9056_read_fpga_status(struct mrf_dev *ev_dev);
int fpga_9056_sprint(struct mrf_dev *ev_dev, char *buf, int size);

/* Issue JTAG command XSC_CONFIG which forces PROG_B pin on Platform
   Flash low and initiates a new configuration from Flash */ 

int jtag_9056_XCF_conf(struct mrf_dev *ev_dev);

/* Read Platform Flash contents.
   addr and size have to be multiples of 1024 */

int jtag_9056_XCF_readstart(struct mrf_dev *ev_dev);
int jtag_9056_XCF_read(struct mrf_dev *ev_dev,
		  char *data, unsigned int addr, unsigned int size);
int jtag_9056_XCF_readend(struct mrf_dev *ev_dev);

/* Program data to Platform Flash 
   These functions erase the flash prior to programming */

int jtag_9056_XCF_progstart(struct mrf_dev *ev_dev);
int jtag_9056_XCF_write(struct mrf_dev *ev_dev, char *data,
		   int addr, int size);
int jtag_9056_XCF_progend(struct mrf_dev *ev_dev);

/* Configure FPGA from .bit file.
   Startup clock has to be set to JTAG in bitgen. */

int jtag_9056_init_load_fpga(struct mrf_dev *ev_dev);
#define jtag_9056_load_fpga_byte(x, y) jtag_9056_SCFD(x, y, 0)
int jtag_9056_end_load_fpga(struct mrf_dev *ev_dev);

int flash_fastread(struct mrf_dev *ev_dev,
                   char *data, unsigned int addr, unsigned int size);
int flash_bulkerase(struct mrf_dev *ev_dev);
int flash_primaryerase(struct mrf_dev *ev_dev);
int flash_pageprogram(struct mrf_dev *ev_dev, char *data,
                      int addr, int size);
