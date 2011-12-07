#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/pci.h>	        /* Needed for pci functions */
#include <linux/init.h>	        /* Needed for __init and __exit macros */

#define DRIVER_AUTHOR "Moises Silva <moy@sangoma.com>"
#define DRIVER_DESC "W400 test driver"

#define SANGOMA_PCI_VENDOR 0x1923
#define AFT_W400_SUBSYS_VENDOR 0xF400

#define AFT_CORE_ID(id) (id & 0x00FF)
#define AFT_CORE_REV_SHIFT 8
#define AFT_CORE_REV(id) ((id >> AFT_CORE_REV_SHIFT) & 0xFF)

#define W400_PCI_MEM_SIZE 0x0FFFF

#define AFT_W400_GLOBAL_REG 0x1400
#define AFT_W400_PLL_RESET_BIT 0
#define AFT_W400_PLL_PHASE_SHIFT_OVERFLOW_BIT 1
#define AFT_W400_PLL_INPUT_CLOCK_LOST_BIT 2
#define AFT_W400_PLL_OUTPUT_CLOCK_STOPPED_BIT 3
#define AFT_W400_PLL_LOCKED_BIT 4
#define AFT_W400_SIM_MUXING_ERROR_BIT 5
#define AFT_W400_GLOBAL_SHUTDOWN_BIT 31

#define aft_test_bit(a,b) test_bit((a), (unsigned long *)(b))

typedef struct w400_dev {
	struct pci_dev *pci_dev;
	unsigned long pci_base_addr;
	void *mapped_memory;
} w400_dev_t;

#define MAX_DEVICES 10 
static w400_dev_t w400_devices[MAX_DEVICES];
static int w400_count = 0;

static void dump_w400_status(w400_dev_t *dev)
{
	u32 reg = 0;
	/* try to read the global W400 reg */
	reg = readl((dev->mapped_memory + AFT_W400_GLOBAL_REG));
	printk(KERN_INFO "W400 global reg: %X\n", reg);
	printk(KERN_INFO "W400 PLL reset: %d\n", aft_test_bit(AFT_W400_PLL_RESET_BIT, &reg));
	printk(KERN_INFO "W400 PLL Phase Shift Overflow: %d\n", aft_test_bit(AFT_W400_PLL_PHASE_SHIFT_OVERFLOW_BIT, &reg));
	printk(KERN_INFO "W400 PLL Input Clock Lost Bit: %d\n", aft_test_bit(AFT_W400_PLL_INPUT_CLOCK_LOST_BIT, &reg));
	printk(KERN_INFO "W400 PLL Output Clock Stopped: %d\n", aft_test_bit(AFT_W400_PLL_OUTPUT_CLOCK_STOPPED_BIT, &reg));
	printk(KERN_INFO "W400 PLL Locked: %d\n", aft_test_bit(AFT_W400_PLL_LOCKED_BIT, &reg));
	printk(KERN_INFO "W400 SIM Muxing Errror: %d\n", aft_test_bit(AFT_W400_SIM_MUXING_ERROR_BIT, &reg));
	printk(KERN_INFO "W400 Global Shutdown: %d\n", aft_test_bit(AFT_W400_GLOBAL_SHUTDOWN_BIT, &reg));
}

int init_module(void)
{
	struct pci_dev *pci_dev = NULL;
	u16 pci_device_id = 0;
	u16 pci_subsys_id = 0;
	u16 pci_subsys_vendor = 0;
	u16 core_rev = 0;
	u16 core_id = 0;
	unsigned long pci_base_addr = 0;
	void *mapped_memory = 0;
	int rc = 0;
	int i = 0;

	memset(w400_devices, 0, sizeof(w400_devices));
	while ((pci_dev = pci_get_device(SANGOMA_PCI_VENDOR, PCI_ANY_ID, pci_dev))) {

		pci_read_config_word(pci_dev, PCI_DEVICE_ID, &pci_device_id);
		pci_read_config_word(pci_dev, PCI_SUBSYSTEM_VENDOR_ID, &pci_subsys_vendor);
		pci_read_config_word(pci_dev, PCI_SUBSYSTEM_ID, &pci_subsys_id);
		
		if (pci_subsys_vendor == AFT_W400_SUBSYS_VENDOR) {
			printk(KERN_INFO "Found Sangoma PCI W400 with IRQ %d at bus %d\n", pci_dev->bus->number, pci_dev->irq);
			printk(KERN_INFO "Device Id: %X, Vendor: %X, Subsys ID: %X\n", pci_device_id, pci_subsys_vendor, pci_subsys_id);

			core_id = AFT_CORE_ID(pci_subsys_id);
			core_rev = AFT_CORE_REV(pci_subsys_id);
			printk(KERN_INFO "Core ID = %d, Core Rev = %d\n", core_id, core_rev);

			/* find out where the memory base is (from sdladrv.h sdla_get_pci_base_resource_addr() */
			pci_base_addr = pci_resource_start(pci_dev, 0);
			printk(KERN_INFO "base addr at %p\n", (void *)pci_base_addr);
			
			/* memory map the addr ... see sdla_memory_map() */	
			rc = pci_enable_device(pci_dev);	
			printk(KERN_INFO "enable device ret = %d\n", rc);

			rc = pci_request_region(pci_dev, 0, "Moy W400 AFT");
			printk(KERN_INFO "request device region ret = %d\n", rc);

			rc = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));
			printk(KERN_INFO "set dma mask rc = %d\n", rc);

			mapped_memory = ioremap(pci_base_addr, W400_PCI_MEM_SIZE);
			printk(KERN_INFO "physical memory %p mapped into virtual memory %p\n", (void *)pci_base_addr, mapped_memory);

			pci_set_master(pci_dev);

			w400_devices[i].pci_dev = pci_dev;
			w400_devices[i].pci_base_addr = pci_base_addr;
			w400_devices[i].mapped_memory = mapped_memory;
			printk(KERN_INFO "Created W400 %p\n", w400_devices[i].pci_dev);

			dump_w400_status(&w400_devices[i]);

			/* try to reset the uart */
			udelay(10);
			i++;
		}
	}
	w400_count = i;
	printk(KERN_INFO "Found %d W400 devices\n", w400_count);
	return 0;
}

void cleanup_module(void)
{
	int i = 0;
	for (i = 0; i < w400_count; i++) {
		if (!w400_devices[i].pci_dev) {
			break;
		}
		printk(KERN_INFO "Destroying W400 %p\n", w400_devices[i].pci_dev);
		if (w400_devices[i].mapped_memory) {
			iounmap(w400_devices[i].mapped_memory);
		}
		pci_release_region(w400_devices[i].pci_dev, 0);
	}
	memset(w400_devices, 0, sizeof(w400_devices));
	w400_count = 0;
	return;
}

//module_init(w400_init);
//modle_exit(w400_end);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

