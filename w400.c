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

#define AFT_W400_FPGA_GLOBAL_REG 0x1040
#define AFT_W400_FPGA_SFR_EX_BIT 1
#define AFT_W400_FPGA_SFR_IN_BIT 2

#define W400_PCI_MEM_SIZE 0x0FFFF

/* Starting configuration register (incremented by AFT_GSM_REG_OFFSET for each module) */
#define AFT_W400_CONFIG_REG 0x1200

/* Per module register offset */
#define AFT_W400_REG_OFFSET 0x0004

/*!< Retrieve the register address for a module */
#define AFT_W400_MOD_REG(mod_no, reg) (reg + ((mod_no-1) * AFT_W400_REG_OFFSET))

#define AFT_W400_GLOBAL_REG 0x1400
#define AFT_W400_PLL_RESET_BIT 0
#define AFT_W400_PLL_PHASE_SHIFT_OVERFLOW_BIT 1
#define AFT_W400_PLL_INPUT_CLOCK_LOST_BIT 2
#define AFT_W400_PLL_OUTPUT_CLOCK_STOPPED_BIT 3
#define AFT_W400_PLL_LOCKED_BIT 4
#define AFT_W400_SIM_MUXING_ERROR_BIT 5
#define AFT_W400_GLOBAL_SHUTDOWN_BIT 31

#define aft_test_bit(a,b) test_bit((a), (unsigned long *)(b))
#define aft_set_bit(a,b) set_bit((a), (unsigned long *)(b))
#define aft_clear_bit(a,b) clear_bit((a), (unsigned long *)(b))

#define MAX_DEVICES 10 
#define MAX_W400_MODULES 4
/* All modules masked (4 modules, 1111) */
#define W400_ALL_MODULES_MASK 0x000F
#define MOD_BIT(mod_no) (mod_no - 1)

#define w400_read_reg(dev, reg) readl(((dev)->mapped_memory + reg));
#define w400_write_reg(dev, reg, data) writel(data, ((dev)->mapped_memory + reg));

/*!< W400 module configuration (bit numbers are zero-based) */
#define AFT_W400_MOD_POWER_BIT 0x0
#define AFT_W400_MOD_RESET_BIT 0x1
#define AFT_W400_MOD_POWER_MONITOR_BIT 0x2
#define AFT_W400_MOD_TX_MONITOR_BIT 0x3
#define AFT_W400_MOD_SIM_INSERTED_BIT 0x4

/*!< The high order last 3 bits of first byte of the module configuration register select the SIM */
#define AFT_W400_MOD_SIM_BIT_OFFSET 5
#define AFT_W400_MOD_SIM_MASK (0x7 << AFT_W400_MOD_SIM_BIT_OFFSET)

/*!< The first 3 bits of the second byte select the UART baud rate */
#define AFT_W400_MOD_UART_BAUD_RATE_BIT_OFFSET 0x8
#define AFT_W400_MOD_UART_BAUD_RATE_MASK (0x7 << AFT_W400_MOD_UART_BAUD_RATE_BIT_OFFSET)

/*!< The fourth bit of the second byte report if there is service or not */
#define AFT_W400_MOD_SERVICE_BIT 0xB

/*!< The fifth bit of the second byte report if there is a module (cell phone) present or not */
#define AFT_W400_MOD_PRESENT_BIT 0xC

/*! Absolute timeout to power on/off a module 
 *  Telit data sheet suggests 2 seconds, but once in a while
 *  the fourth module in W400 takes ~4 seconds to stop ... lets be safe and do 8 seconds
 */
#define AFT_W400_MODULE_POWER_TOGGLE_TIMEOUT_MS 8000

/*!< How often to check if the module is already on/off during startup/shutdown */
#define AFT_W400_MODULE_POWER_TOGGLE_CHECK_INTERVAL_MS 10

/*! Absolute timeout to enable PLL */
#define AFT_W400_PLL_ENABLE_TIMEOUT_MS 2000

/*!< How often to check if the PLL is already enabled on startup */
#define AFT_W400_PLL_ENABLE_CHECK_INTERVAL_MS 100

/*! How much to wait between power registry writes 
 *  Telit data sheet says input command for switch off/on must be equal or bigger to 1 second let's use 2 seconds to be safe
 */
#define AFT_W400_MODULE_POWER_TOGGLE_DELAY_MS 2000

typedef struct w400_dev {
	struct pci_dev *pci_dev;
	unsigned long pci_base_addr;
	void *mapped_memory;
	int powered_modules_map;
} w400_dev_t;

static w400_dev_t w400_devices[MAX_DEVICES];
static int w400_count = 0;

static void w400_dump_status(w400_dev_t *dev)
{
	u32 reg = 0;
	/* try to read the global W400 reg */
	reg = w400_read_reg(dev, AFT_W400_GLOBAL_REG);
	printk(KERN_INFO "W400 global reg: %X\n", reg);
	printk(KERN_INFO "W400 PLL reset: %d\n", aft_test_bit(AFT_W400_PLL_RESET_BIT, &reg));
	printk(KERN_INFO "W400 PLL Phase Shift Overflow: %d\n", aft_test_bit(AFT_W400_PLL_PHASE_SHIFT_OVERFLOW_BIT, &reg));
	printk(KERN_INFO "W400 PLL Input Clock Lost Bit: %d\n", aft_test_bit(AFT_W400_PLL_INPUT_CLOCK_LOST_BIT, &reg));
	printk(KERN_INFO "W400 PLL Output Clock Stopped: %d\n", aft_test_bit(AFT_W400_PLL_OUTPUT_CLOCK_STOPPED_BIT, &reg));
	printk(KERN_INFO "W400 PLL Locked: %d\n", aft_test_bit(AFT_W400_PLL_LOCKED_BIT, &reg));
	printk(KERN_INFO "W400 SIM Muxing Errror: %d\n", aft_test_bit(AFT_W400_SIM_MUXING_ERROR_BIT, &reg));
	printk(KERN_INFO "W400 Global Shutdown: %d\n", aft_test_bit(AFT_W400_GLOBAL_SHUTDOWN_BIT, &reg));
}

static void w400_reset_fpga(w400_dev_t *dev)
{
	u32 reg = 0;
	//reg = w400_read_reg(dev, AFT_W400_GLOBAL_REG);
	aft_set_bit(AFT_W400_FPGA_SFR_EX_BIT, &reg);
	aft_set_bit(AFT_W400_FPGA_SFR_IN_BIT, &reg);
	w400_write_reg(dev, AFT_W400_FPGA_GLOBAL_REG, reg);
	udelay(10);

	aft_clear_bit(AFT_W400_FPGA_SFR_EX_BIT, &reg);
	aft_clear_bit(AFT_W400_FPGA_SFR_IN_BIT, &reg);
	w400_write_reg(dev, AFT_W400_FPGA_GLOBAL_REG, reg);
	udelay(10);
}

static void w400_reset_uart(w400_dev_t *dev)
{
	u32 reg = 0;
	reg = w400_read_reg(dev, AFT_W400_GLOBAL_REG);
	aft_set_bit(AFT_W400_PLL_RESET_BIT, &reg);
	w400_write_reg(dev, AFT_W400_GLOBAL_REG, reg);
	udelay(10);
	aft_clear_bit(AFT_W400_PLL_RESET_BIT, &reg);
	w400_write_reg(dev, AFT_W400_GLOBAL_REG, reg);
	udelay(10);
}

static void w400_restore_uart(w400_dev_t *dev)
{
	u32 reg = 0;
	reg = w400_read_reg(dev, AFT_W400_GLOBAL_REG);
	aft_set_bit(AFT_W400_PLL_RESET_BIT, &reg);
	w400_write_reg(dev, AFT_W400_GLOBAL_REG, reg);
	udelay(10);
}

int w400_toggle_power(w400_dev_t *dev, int mod_map, int turn_on)
{
	u32 reg[MAX_W400_MODULES+1]; /* mod_no is not zero-based */
	int timeout_loops = 0;
	int mod_no = 1;

	/* 
	 * Power toggle sequence as described by the Telit documentation
	 * We have a power monitor pin that tells us whether the module is on/off
	 * We have a power pin that is equivalent to the power on/off button on your cell phone
	 * In order to turn on/off we hold high the power pin for at least one second and then
	 * set it low. Then we monitor the power monitor pin until goes high/low depending on
	 * whether we're turning on or off the module
	 * 
	 * Note that in the Telit documentation you will see the high/low order inversed, there
	 * is an inversor in our hardware doing that, ask our hw engineers why? :-)
	 */

	for (mod_no = 1; mod_no <= MAX_W400_MODULES; mod_no++) {
		if (!aft_test_bit(MOD_BIT(mod_no), &mod_map)) {
			continue;
		}
		printk(KERN_INFO "Turning W400 module %d %s ...\n", mod_no, turn_on ? "on" : "off");
		reg[mod_no] = w400_read_reg(dev, AFT_W400_MOD_REG(mod_no, AFT_W400_CONFIG_REG));
		if (turn_on && aft_test_bit(AFT_W400_MOD_POWER_MONITOR_BIT, &reg[mod_no])) {
			printk(KERN_INFO "W400 module %d is already %s ...\n", mod_no, "on");
			aft_clear_bit(MOD_BIT(mod_no), &mod_map);
			continue;
		}
		if (!turn_on && !aft_test_bit(AFT_W400_MOD_POWER_MONITOR_BIT, &reg[mod_no])) {
			printk(KERN_INFO "W400 module %d is already %s ...\n", mod_no, "off");
			aft_clear_bit(MOD_BIT(mod_no), &mod_map);
			continue;
		}
		aft_set_bit(AFT_W400_MOD_POWER_BIT, &reg[mod_no]);
		w400_write_reg(dev, AFT_W400_MOD_REG(mod_no, AFT_W400_CONFIG_REG), reg[mod_no]);
	}

	mdelay(AFT_W400_MODULE_POWER_TOGGLE_DELAY_MS);

	for (mod_no = 1; mod_no <= MAX_W400_MODULES; mod_no++) {
		if (!aft_test_bit(MOD_BIT(mod_no), &mod_map)) {
			continue;
		}
		aft_clear_bit(AFT_W400_MOD_POWER_BIT, &reg[mod_no]);
		w400_write_reg(dev, AFT_W400_MOD_REG(mod_no, AFT_W400_CONFIG_REG), reg[mod_no]);
	}

	for (timeout_loops = (AFT_W400_MODULE_POWER_TOGGLE_TIMEOUT_MS / AFT_W400_MODULE_POWER_TOGGLE_CHECK_INTERVAL_MS); 
	     (timeout_loops && mod_map); 
	     timeout_loops--) {
		mdelay(AFT_W400_MODULE_POWER_TOGGLE_CHECK_INTERVAL_MS);
		for (mod_no = 1; mod_no <= MAX_W400_MODULES; mod_no++) {
			if (!aft_test_bit(MOD_BIT(mod_no), &mod_map)) {
				continue;
			}
			reg[mod_no] = w400_read_reg(dev, AFT_W400_MOD_REG(mod_no, AFT_W400_CONFIG_REG));
			/* if we were asked to turn the module on and is on, we're done */
			if (turn_on && aft_test_bit(AFT_W400_MOD_POWER_MONITOR_BIT, &reg[mod_no])) {
				printk(KERN_INFO "W400 module %d is now %s ...\n", mod_no, "on");
				aft_clear_bit(MOD_BIT(mod_no), &mod_map);
				aft_set_bit(MOD_BIT(mod_no), &dev->powered_modules_map);
			}
			/* if we were asked to turn the module off and is off, we're done */
			if (!turn_on && !aft_test_bit(AFT_W400_MOD_POWER_MONITOR_BIT, &reg[mod_no])) {
				printk(KERN_INFO "W400 module %d is now %s ...\n", mod_no, "off");
				aft_clear_bit(MOD_BIT(mod_no), &mod_map);
				aft_clear_bit(MOD_BIT(mod_no), &dev->powered_modules_map);
			}
		}
	}
	return mod_map;
}
static void w400_power_on_modules(w400_dev_t *dev)
{
	w400_toggle_power(dev, W400_ALL_MODULES_MASK, 1);
}

static void w400_power_off_modules(w400_dev_t *dev)
{
	w400_toggle_power(dev, W400_ALL_MODULES_MASK, 0);
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

			if (mapped_memory) {
				pci_set_master(pci_dev);

				w400_devices[i].pci_dev = pci_dev;
				w400_devices[i].pci_base_addr = pci_base_addr;
				w400_devices[i].mapped_memory = mapped_memory;

				w400_reset_fpga(&w400_devices[i]);

				w400_dump_status(&w400_devices[i]);

				mdelay(1000);

				/* try to reset the uart */
				w400_reset_uart(&w400_devices[i]);

				mdelay(1000);

				/* try to power on all the modules */
				w400_power_on_modules(&w400_devices[i]);

				w400_dump_status(&w400_devices[i]);
			}

			printk(KERN_INFO "Created W400 %p\n", w400_devices[i].pci_dev);
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
			w400_power_off_modules(&w400_devices[i]);
			w400_restore_uart(&w400_devices[i]);
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

