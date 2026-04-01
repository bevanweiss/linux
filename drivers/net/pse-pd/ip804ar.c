// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the IC Plus IP804AR 4-Port PoE PSE Controller
 *
 * Copyright (c) 2026
 *
 * The IP804AR is a 4-port PSE controller supporting IEEE 802.3af/at.
 * It uses an I2C interface with a paged register scheme (3 pages).
 * Page selection is via register 0x00 which exists on all pages.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pse-pd/pse.h>

#define IP804AR_NUM_PORTS		4
#define IP804AR_MAX_IRQ_RETRIES		10

/* ============================================================
 * Register definitions - sorted by page then by address
 * ============================================================
 */

/* Page selection register (exists on all pages) */
#define IP804AR_REG_PAGE			0x00
#define   IP804AR_PAGE_MASK			GENMASK(7, 6)
#define   IP804AR_PAGE_0			0
#define   IP804AR_PAGE_1			1
#define   IP804AR_PAGE_2			2
#define   IP804AR_I2C_TIMEOUT_5MS		BIT(4)
#define   IP804AR_I2C_ADDR_MASK			GENMASK(2, 0)

/* ==================== Page 0 Register Definitions ==================== */

/* Device address register (0x00 when page is set) */
#define IP804AR_REG_DEV_ADDR			0x00

/* AF/AT Mode Register */
#define IP804AR_REG_PAGE0_AF_AT_MODE		0x25
#define   IP804AR_AF_AT_MODE_MASK		GENMASK(3, 0)
#define   IP804AR_MODE_AF			0
#define   IP804AR_MODE_AT			1

/* Inrush Time Register */
#define IP804AR_REG_PAGE0_INRUSH_TIME		0x5F
#define   IP804AR_INRUSH_TIME_DEFAULT		0x7E

/* Detection Result Registers */
#define IP804AR_REG_PAGE0_DET_SIG(port)		(0x68 + (port))
#define   IP804AR_DET_SIG_MASK			GENMASK(2, 0)
#define   IP804AR_DET_SIG_R_BAD			0
#define   IP804AR_DET_SIG_R_GOOD		1
#define   IP804AR_DET_SIG_R_OPEN		2
#define   IP804AR_DET_SIG_C_LARGE		4
#define   IP804AR_DET_SIG_R_LOW			5
#define   IP804AR_DET_SIG_R_HIGH		6

/* Classification Current Registers (14-bit: 10 int + 4 frac, mA) */
#define IP804AR_REG_PAGE0_ICLASS_MSB(port)	(0x78 + (port) * 2)
#define IP804AR_REG_PAGE0_ICLASS_LSB(port)	(0x79 + (port) * 2)

/* Detected PD Class Registers */
#define IP804AR_REG_PAGE0_PD_CLASS_0_1		0x88
#define IP804AR_REG_PAGE0_PD_CLASS_2_3		0x89
#define   IP804AR_PD_CLASS_MASK			GENMASK(2, 0)
#define   IP804AR_PD_CLASS_0			0
#define   IP804AR_PD_CLASS_1			1
#define   IP804AR_PD_CLASS_2			2
#define   IP804AR_PD_CLASS_3			3
#define   IP804AR_PD_CLASS_4			4
#define   IP804AR_PD_CLASS_UNKNOWN		5

/* PD Requested Power Registers (8-bit: 6 int + 2 frac, W) */
#define IP804AR_REG_PAGE0_PD_REQ_PW(port)	(0x90 + (port))

/* Port Current Registers (12-bit: 10 int + 2 frac, mA) */
#define IP804AR_REG_PAGE0_PORT_CUR_MSB(port)	(0xA0 + (port) * 2)
#define IP804AR_REG_PAGE0_PORT_CUR_LSB(port)	(0xA1 + (port) * 2)

/* Port Voltage Registers (12-bit: 8 int + 4 frac, V) */
#define IP804AR_REG_PAGE0_PORT_VOL_MSB(port)	(0xB0 + (port) * 2)
#define IP804AR_REG_PAGE0_PORT_VOL_LSB(port)	(0xB1 + (port) * 2)

/* Port Temperature Registers (13-bit: 9 int + 4 frac, C) */
#define IP804AR_REG_PAGE0_PORT_TEMP_MSB(port)	(0xC0 + (port) * 2)
#define IP804AR_REG_PAGE0_PORT_TEMP_LSB(port)	(0xC1 + (port) * 2)

/* Supply Voltage Registers (12-bit: 8 int + 4 frac, V) */
#define IP804AR_REG_PAGE0_SUPPLY_VOL_MSB	0xE0
#define IP804AR_REG_PAGE0_SUPPLY_VOL_LSB	0xE1

/* Force Poll Register */
#define IP804AR_REG_PAGE0_FORCE_POLL		0xE2

/* IVT Poll Control Register */
#define IP804AR_REG_PAGE0_IVT_POLL		0xE3
#define   IP804AR_POLL_IN_PROGRESS		BIT(7)
#define   IP804AR_POLL_AUTO_ENABLE		BIT(5)
#define   IP804AR_POLL_INTERVAL_MASK		GENMASK(3, 0)

/* Error Delay Register (unit: 100ms) */
#define IP804AR_REG_PAGE0_ERROR_DELAY		0xE4
#define   IP804AR_ERROR_DELAY_DEFAULT		0x10

/* ==================== Page 1 Register Definitions ==================== */

/* System Configuration Register */
#define IP804AR_REG_PAGE1_SYS_CONFIG		0x01
#define   IP804AR_MODE_MASK			GENMASK(7, 6)
#define   IP804AR_OPMODE_AUTO			0
#define   IP804AR_OPMODE_MANUAL			1
#define   IP804AR_OPMODE_DIAG			2
#define   IP804AR_OPMODE_SCAN			3
#define   IP804AR_SUSPEND_CLASS			BIT(2)
#define   IP804AR_SUSPEND_POWER_UP		BIT(1)

/* System Control Register */
#define IP804AR_REG_PAGE1_SYS_CTRL		0x02
#define   IP804AR_SOFT_RESET			BIT(0)

/* Hardware Revision Registers */
#define IP804AR_REG_PAGE1_HW_REV_MSB		0x03
#define   IP804AR_HW_REV_MSB_DEFAULT		0x04
#define IP804AR_REG_PAGE1_HW_REV_LSB		0x04
#define   IP804AR_HW_REV_LSB_DEFAULT		0xA2

/* Watchdog Timer Register */
#define IP804AR_REG_PAGE1_WDT			0x05

/* Scratch Register */
#define IP804AR_REG_PAGE1_SCRATCH		0x06

/* Alternative A/B Register */
#define IP804AR_REG_PAGE1_ALT_AB		0x07

/* LED Configuration Registers */
#define IP804AR_REG_PAGE1_LED_CONFIG		0x08
#define   IP804AR_LED_SERIAL_EN			BIT(7)
#define   IP804AR_LED_DIRECT_EN			BIT(6)
#define   IP804AR_LED_ORDER			BIT(4)
#define   IP804AR_LED_ACTIVE_LOW		BIT(3)
#define   IP804AR_LED_INIT_HIGH			BIT(2)
#define   IP804AR_LED_CLK_RATE			BIT(1)
#define   IP804AR_LED_MASTER			BIT(0)

/* LED Flash Control Register */
#define IP804AR_REG_PAGE1_LED_FLASH		0x09

/* Warning LED Control Register */
#define IP804AR_REG_PAGE1_WARN_LED_CTRL		0x0A

/* LED Start Index Register */
#define IP804AR_REG_PAGE1_LED_START_IDX		0x0B

/* Warning LED Index Register */
#define IP804AR_REG_PAGE1_WARN_LED_IDX		0x0C

/* LED Control Register */
#define IP804AR_REG_PAGE1_LED_CTRL		0x0D

/* Warning LED Display Register */
#define IP804AR_REG_PAGE1_WARN_LED_DISP		0x0E

/* Warning LED Current/Power Threshold Registers */
#define IP804AR_REG_PAGE1_WARN_THR_MSB		0xDD
#define IP804AR_REG_PAGE1_WARN_THR_LSB		0xDE

/* Warning LED Temperature Threshold Registers */
#define IP804AR_REG_PAGE1_WARN_TEMP_THR_MSB	0xDF
#define IP804AR_REG_PAGE1_WARN_TEMP_THR_LSB	0xE0

/* Power Configuration Mode Register */
#define IP804AR_REG_PAGE1_PW_CFG_MODE		0x10
#define   IP804AR_RPP_MASK			GENMASK(4, 3)
#define   IP804AR_RPP_HOST_DEFINED		0
#define   IP804AR_RPP_CLASS_DEFINED		1
#define   IP804AR_RPP_HIGHEST			2
#define   IP804AR_PSE_PWR_TYPE_MASK		GENMASK(1, 0)

/* Class Defined Power Limit Registers */
#define IP804AR_REG_PAGE1_CDPL_CLASS0		0x12
#define IP804AR_REG_PAGE1_CDPL_CLASS1		0x13
#define IP804AR_REG_PAGE1_CDPL_CLASS2		0x14
#define IP804AR_REG_PAGE1_CDPL_CLASS3		0x15
#define IP804AR_REG_PAGE1_CDPL_CLASS4_T1	0x16
#define IP804AR_REG_PAGE1_CDPL_CLASS4_T2	0x17

/* Host Defined Power Limit Registers */
#define IP804AR_REG_PAGE1_HDPL_PORT0		0x18
#define IP804AR_REG_PAGE1_HDPL_PORT1		0x19
#define IP804AR_REG_PAGE1_HDPL_PORT2		0x1A
#define IP804AR_REG_PAGE1_HDPL_PORT3		0x1B

/* Port Temperature Limit Registers */
#define IP804AR_REG_PAGE1_TEMP_LIM_MSB		0x24
#define IP804AR_REG_PAGE1_TEMP_LIM_LSB		0x25
#define   IP804AR_TEMP_LIM_DEFAULT_MSB		0x09
#define   IP804AR_TEMP_LIM_DEFAULT_LSB		0x60

/* Port Current Limit Registers */
#define IP804AR_REG_PAGE1_PORT_CUR_LIM_MSB(port)	(0x30 + (port) * 2)
#define IP804AR_REG_PAGE1_PORT_CUR_LIM_LSB(port)	(0x31 + (port) * 2)

/* Trunk Power Limit Registers */
#define IP804AR_REG_PAGE1_TRUNK0_PW_LIM_MSB	0x40
#define IP804AR_REG_PAGE1_TRUNK0_PW_LIM_LSB	0x41
#define IP804AR_REG_PAGE1_TRUNK1_PW_LIM_MSB	0x42
#define IP804AR_REG_PAGE1_TRUNK1_PW_LIM_LSB	0x43

/* Trunk Voltage Limit Registers */
#define IP804AR_REG_PAGE1_TRUNK0_VOL_UP_MSB	0x4C
#define IP804AR_REG_PAGE1_TRUNK0_VOL_UP_LSB	0x4D
#define IP804AR_REG_PAGE1_TRUNK0_VOL_LO_MSB	0x4E
#define IP804AR_REG_PAGE1_TRUNK0_VOL_LO_LSB	0x4F

/* PSE Available Current Registers */
#define IP804AR_REG_PAGE1_PSE_AVAIL_CUR_MSB	0x54
#define IP804AR_REG_PAGE1_PSE_AVAIL_CUR_LSB	0x55

/* PSE Consumed Current Registers */
#define IP804AR_REG_PAGE1_PSE_CONS_CUR_MSB	0x56
#define IP804AR_REG_PAGE1_PSE_CONS_CUR_LSB	0x57

/* Port Priority Registers */
#define IP804AR_REG_PAGE1_PORT_PRIO(port)	(0x58 + (port))

/* Total Current & Power Limiter Power Off Event Register */
#define IP804AR_REG_PAGE1_CRIT_PW_OFF_EVT	0x60

/* Trunk Select Register */
#define IP804AR_REG_PAGE1_TRUNK_SEL		0x69
#define   IP804AR_TRUNK_SEL_MASK		GENMASK(1, 0)

/* PSE Consumed Power Registers */
#define IP804AR_REG_PAGE1_PSE_CONS_PW_MSB	0x6A
#define IP804AR_REG_PAGE1_PSE_CONS_PW_LSB	0x6B

/* Trunk Available/Allocated Power Registers (read-only) */
#define IP804AR_REG_PAGE1_TRUNK_AVAIL_PW_MSB	0x6C
#define IP804AR_REG_PAGE1_TRUNK_AVAIL_PW_LSB	0x6D
#define IP804AR_REG_PAGE1_TRUNK_ALLOC_PW_MSB	0x6E
#define IP804AR_REG_PAGE1_TRUNK_ALLOC_PW_LSB	0x6F

/* Port Power Event Registers (W1C) */
#define IP804AR_REG_PAGE1_PORT_EVT(port)	(0x70 + (port))
#define   IP804AR_EVT_TEMP_LIM			BIT(7)
#define   IP804AR_EVT_VOL_LIM			BIT(6)
#define   IP804AR_EVT_CUR_LIM			BIT(5)
#define   IP804AR_EVT_THERM_SHUTDOWN		BIT(4)
#define   IP804AR_EVT_VOL_BAD			BIT(3)
#define   IP804AR_EVT_MPS_ERR			BIT(2)
#define   IP804AR_EVT_SHORT_CKT			BIT(1)
#define   IP804AR_EVT_OVERLOAD			BIT(0)

/* Port Power Event Mask Registers */
#define IP804AR_REG_PAGE1_PORT_EVT_MASK(port)	(0x78 + (port))

/* Port Interrupt Register */
#define IP804AR_REG_PAGE1_PORT_INT		0x80

/* Port Power Event Handle Register */
#define IP804AR_REG_PAGE1_EVT_HANDLE		0x81
#define   IP804AR_HANDLE_TEMP_LIM		BIT(7)
#define   IP804AR_HANDLE_VOL_LIM		BIT(6)
#define   IP804AR_HANDLE_CUR_LIM		BIT(5)
#define   IP804AR_HANDLE_VOL_BAD		BIT(3)

/* Port Power Status Register */
#define IP804AR_REG_PAGE1_PORT_PW_STATUS	0x82

/* MPS Present Status Register */
#define IP804AR_REG_PAGE1_MPS_STATUS		0x83

/* System Init & Current Overload Event Register */
#define IP804AR_REG_PAGE1_SYS_EVT		0x84
#define   IP804AR_SYS_EVT_PSE_OVERLOAD		BIT(5)
#define   IP804AR_SYS_EVT_NO_EEPROM		BIT(2)
#define   IP804AR_SYS_EVT_EEPROM_ERR		BIT(1)
#define   IP804AR_SYS_EVT_INIT_DONE		BIT(0)

/* System Init & Current Overload Event Mask Register */
#define IP804AR_REG_PAGE1_SYS_EVT_MASK		0x85

/* Port Voltage Bad Event Register */
#define IP804AR_REG_PAGE1_VOL_BAD_EVT		0x86

/* Severe Short Circuit Event Register */
#define IP804AR_REG_PAGE1_SEV_SHORT_EVT		0x87

/* Port State Machine Control Registers */
#define IP804AR_REG_PAGE1_SM_CTRL(port)		(0x90 + (port))
#define   IP804AR_SM_SUSPENDED			BIT(7)
#define   IP804AR_SM_STEP			BIT(6)
#define   IP804AR_SM_START_PWUP			BIT(5)
#define   IP804AR_SM_STATE_MASK			GENMASK(4, 0)
#define   IP804AR_SM_STATE_DISABLED		0
#define   IP804AR_SM_STATE_IDLE			3
#define   IP804AR_SM_STATE_POWER_ON		16

/* Port Power Control Registers */
#define IP804AR_REG_PAGE1_PORT_PW_CTRL(port)	(0x98 + (port))
#define   IP804AR_PW_CTRL_EN_AUTO_BOFF		BIT(5)
#define   IP804AR_PW_CTRL_EN_CLS_MOD		BIT(4)
#define   IP804AR_PW_CTRL_INIT_STATE_MASK	GENMASK(3, 2)
#define   IP804AR_PW_CTRL_INIT_ENABLED		0x01
#define   IP804AR_PW_CTRL_FORCE_ON		0x02
#define   IP804AR_PW_CTRL_SKIP_DETECT		0x03
#define   IP804AR_PW_CTRL_ENABLE		BIT(0)

/* Classification Event Number Register */
#define IP804AR_REG_PAGE1_CLS_EVT_NUM		0xA0

/* Skip Event 2 Register */
#define IP804AR_REG_PAGE1_SKIP_EVT2		0xA2

/* Total Current & Power Limit Control Register */
#define IP804AR_REG_PAGE1_TOT_PW_CTRL		0xC0
#define   IP804AR_TOT_CUR_EN			BIT(7)
#define   IP804AR_TOT_PW_EN			BIT(6)
#define   IP804AR_VICTIM_STRAT_MASK		GENMASK(2, 0)

/* Power Denied Event Register */
#define IP804AR_REG_PAGE1_PW_DENIED_EVT		0xDA

/* Invalid Signature Event Register */
#define IP804AR_REG_PAGE1_INV_SIG_EVT		0xDC

/* 4-Pair High Power Control Register */
#define IP804AR_REG_PAGE1_4PAIR_CTRL		0xF3
#define   IP804AR_4PAIR_AUTO_SWAP		BIT(5)
#define   IP804AR_4PAIR_DISC_MODE		BIT(4)
#define   IP804AR_4PAIR_PORT23_EN		BIT(1)
#define   IP804AR_4PAIR_PORT01_EN		BIT(0)

/* 4-Pair High Power Timing Register */
#define IP804AR_REG_PAGE1_4PAIR_TIMING		0xF4

/* ==================== Page 2 Register Definitions ==================== */

/* Power Up Sequence Registers */
#define IP804AR_REG_PAGE2_PWUP_SEQ_1		0x80
#define IP804AR_REG_PAGE2_PWUP_SEQ_2		0x81
#define   IP804AR_PWUP_ORDER_MASK		(GENMASK(5, 4) | GENMASK(1, 0))

/* PSE Available Power Registers */
#define IP804AR_REG_PAGE2_PSE_AVAIL_PW_MSB	0x2E
#define IP804AR_REG_PAGE2_PSE_AVAIL_PW_LSB	0x2F

/* Per-port Consumed Power Registers */
#define IP804AR_REG_PAGE2_PORT_CONS_PW_MSB(port)	(0x30 + (port) * 2)
#define IP804AR_REG_PAGE2_PORT_CONS_PW_LSB(port)	(0x31 + (port) * 2)

/* Per-port Max Consumed Power Registers */
#define IP804AR_REG_PAGE2_PORT_MAX_PW_MSB(port)		(0x40 + (port) * 2)
#define IP804AR_REG_PAGE2_PORT_MAX_PW_LSB(port)		(0x41 + (port) * 2)

/* Per-port Max Consumed Current Registers */
#define IP804AR_REG_PAGE2_PORT_MAX_CUR_MSB(port)	(0x50 + (port) * 2)
#define IP804AR_REG_PAGE2_PORT_MAX_CUR_LSB(port)	(0x51 + (port) * 2)

/* Max PSE Consumed Power Registers */
#define IP804AR_REG_PAGE2_MAX_PSE_CONS_PW_MSB	0x60
#define IP804AR_REG_PAGE2_MAX_PSE_CONS_PW_LSB	0x61

/* Max PSE Consumed Current Registers */
#define IP804AR_REG_PAGE2_MAX_PSE_CONS_CUR_MSB	0x62
#define IP804AR_REG_PAGE2_MAX_PSE_CONS_CUR_LSB	0x63

/* ============================================================
 * Conversion macros
 * ============================================================
 */

/* Supply voltage: 12-bit (8 int + 4 frac), unit = V, return in uV */
#define IP804AR_SUPPLY_VOL_UV(val)	(((val) * 1000 * 1000) / 16)

/* Port voltage: 12-bit (8 int + 4 frac), unit = V, return in uV */
#define IP804AR_PORT_VOL_UV(val)	(((val) * 1000 * 1000) / 16)

/* Port current: 12-bit (10 int + 2 frac), unit = mA, return in uA */
#define IP804AR_PORT_CUR_UA(val)	(((val) * 1000 * 1000) / 4)

/* Port power (consumed): 16-bit (8 int + 8 frac), unit = W */
#define IP804AR_PORT_PW_MW(msb, lsb)	(((msb) << 8 | (lsb)) * 1000 / 256)

/* Temperature: 13-bit (9 int + 4 frac), unit = C */
#define IP804AR_TEMP_mC(val)		(((val) * 1000) / 16)

/* PD requested power: 8-bit (6 int + 2 frac), unit = W, return in mW */
#define IP804AR_PD_REQ_PW_MW(val)	(((val) * 1000) / 4)

/* Port current limit: 12-bit (10 int + 2 frac), unit = mA */
#define IP804AR_CUR_LIM_UA(val)		(((val) * 1000 * 1000) / 4)

/* Class defined power limit: 8-bit (6 int + 2 frac), unit = W, return in mW */
#define IP804AR_CDPL_MW(val)		(((val) * 1000) / 4)

/* Host defined power limit: 8-bit (6 int + 2 frac), unit = W, return in mW */
#define IP804AR_HDPL_MW(val)		(((val) * 1000) / 4)

/* ============================================================
 * Data structures
 * ============================================================
 */

/**
 * struct ip804ar_port - per-port state
 * @present: port is described in device tree
 * @is_4pair: port is paired for 4-pair operation
 * @pair_port: the port this port is paired with (or -1)
 * @admin_enabled: software tracking of admin enabled state
 * @pd_detected: PD detection status for IRQ handling
 */
struct ip804ar_port {
	bool present;
	bool is_4pair;
	int pair_port;
	bool admin_enabled;
	bool pd_detected;
};

/**
 * struct ip804ar_priv - driver private data
 * @client: I2C client device
 * @pcdev: PSE controller device
 * @port: per-port state array
 * @page_lock: mutex protecting paged register access
 * @irq_notifs: notification bitmask for IRQ handler
 */
struct ip804ar_priv {
	struct i2c_client *client;
	struct pse_controller_dev pcdev;
	struct ip804ar_port port[IP804AR_NUM_PORTS];
	struct mutex page_lock; /* protect paged register access */
	unsigned long *irq_notifs;
	unsigned long irq_notifs_mask;
};

/* ============================================================
 * I2C page access helpers
 * ============================================================
 */

static int ip804ar_set_page(struct i2c_client *client, int page)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, IP804AR_REG_PAGE,
					FIELD_PREP(IP804AR_PAGE_MASK, page));
	if (ret < 0)
		return ret;

	return 0;
}

static int ip804ar_read_page_reg(struct ip804ar_priv *priv, int page, int reg)
{
	struct i2c_client *client = priv->client;
	int ret;

	ret = ip804ar_set_page(client, page);
	if (ret < 0)
		return ret;

	return i2c_smbus_read_byte_data(client, reg);
}

static int ip804ar_write_page_reg(struct ip804ar_priv *priv, int page,
				  int reg, u8 val)
{
	struct i2c_client *client = priv->client;
	int ret;

	ret = ip804ar_set_page(client, page);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client, reg, val);
}

static int ip804ar_read_page_word(struct ip804ar_priv *priv, int page, int reg)
{
	struct i2c_client *client = priv->client;
	int ret;

	ret = ip804ar_set_page(client, page);
	if (ret < 0)
		return ret;

	return i2c_smbus_read_word_data(client, reg);
}

static struct ip804ar_priv *to_ip804ar_priv(struct pse_controller_dev *pcdev)
{
	return container_of(pcdev, struct ip804ar_priv, pcdev);
}

/* ============================================================
 * PSE controller operations
 * ============================================================
 */

static int ip804ar_pi_enable(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int reg = IP804AR_REG_PAGE1_PORT_PW_CTRL(id);
	struct ip804ar_port *port = &priv->port[id];
	int ret;

	if (id < 0 || id >= IP804AR_NUM_PORTS)
		return -EINVAL;

	guard(mutex)(&priv->page_lock);

	/* For 4-pair ports also enable the paired port */
	if (port->is_4pair && port->pair_port >= 0) {
		ret = ip804ar_read_page_reg(priv, 1,
					    IP804AR_REG_PAGE1_PORT_PW_CTRL(port->pair_port));
		if (ret < 0)
			return ret;

		ret = i2c_smbus_write_byte_data(priv->client, reg,
						ret | IP804AR_PW_CTRL_ENABLE);
		if (ret < 0)
			return ret;
	}

	ret = ip804ar_read_page_reg(priv, 1, reg);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(priv->client, reg,
					ret | IP804AR_PW_CTRL_ENABLE);
	if (ret < 0)
		return ret;

	port->admin_enabled = true;
	return ret;
}

static int ip804ar_pi_disable(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int reg = IP804AR_REG_PAGE1_PORT_PW_CTRL(id);
	struct ip804ar_port *port = &priv->port[id];
	int ret;

	if (id < 0 || id >= IP804AR_NUM_PORTS)
		return -EINVAL;

	guard(mutex)(&priv->page_lock);

	ret = ip804ar_read_page_reg(priv, 1, reg);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(priv->client, reg,
					ret & ~IP804AR_PW_CTRL_ENABLE);
	if (ret < 0)
		return ret;

	port->admin_enabled = false;
	port->pd_detected = false;
	return ret;
}

static int ip804ar_pi_get_admin_state(struct pse_controller_dev *pcdev, int id,
				      struct pse_admin_state *admin_state)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int ret;

	guard(mutex)(&priv->page_lock);

	ret = ip804ar_read_page_reg(priv, 1, IP804AR_REG_PAGE1_SM_CTRL(id));
	if (ret < 0) {
		admin_state->c33_admin_state = ETHTOOL_C33_PSE_ADMIN_STATE_UNKNOWN;
		return ret;
	}

	if ((ret & IP804AR_SM_STATE_MASK) == IP804AR_SM_STATE_POWER_ON ||
	    (ret & IP804AR_SM_STATE_MASK) == IP804AR_SM_STATE_IDLE)
		admin_state->c33_admin_state = ETHTOOL_C33_PSE_ADMIN_STATE_ENABLED;
	else
		admin_state->c33_admin_state = ETHTOOL_C33_PSE_ADMIN_STATE_DISABLED;
	return 0;
}

static int ip804ar_pi_get_pw_status(struct pse_controller_dev *pcdev, int id,
				    struct pse_pw_status *pw_status)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int ret;

	guard(mutex)(&priv->page_lock);

	ret = ip804ar_read_page_reg(priv, 1, IP804AR_REG_PAGE1_PORT_PW_STATUS);
	if (ret < 0) {
		pw_status->c33_pw_status = ETHTOOL_C33_PSE_PW_D_STATUS_UNKNOWN;
		return ret;
	}

	if (ret & BIT(id))
		pw_status->c33_pw_status = ETHTOOL_C33_PSE_PW_D_STATUS_DELIVERING;
	else
		pw_status->c33_pw_status = ETHTOOL_C33_PSE_PW_D_STATUS_DISABLED;
	return 0;
}

static int ip804ar_pi_get_voltage(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int ret;

	guard(mutex)(&priv->page_lock);

	ret = ip804ar_read_page_word(priv, 0, IP804AR_REG_PAGE0_SUPPLY_VOL_MSB);
	if (ret < 0)
		return ret;

	return IP804AR_SUPPLY_VOL_UV(ret);
}

static int ip804ar_pi_get_chan_current(struct ip804ar_priv *priv, int id)
{
	int ret;

	ret = ip804ar_read_page_word(priv, 0,
				     IP804AR_REG_PAGE0_PORT_CUR_MSB(id));
	if (ret < 0)
		return ret;

	return IP804AR_PORT_CUR_UA(ret);
}

static int ip804ar_pi_get_actual_pw(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	struct ip804ar_port *port = &priv->port[id];
	int ret, uV, uA;
	u64 tmp_64;

	ret = ip804ar_pi_get_voltage(pcdev, id);
	if (ret <= 0)
		return ret;
	uV = ret;

	guard(mutex)(&priv->page_lock);

	ret = ip804ar_pi_get_chan_current(priv, id);
	if (ret < 0)
		return ret;
	uA = ret;

	/* For 4-pair mode add current from paired port */
	if (port->is_4pair && port->pair_port >= 0) {
		ret = ip804ar_pi_get_chan_current(priv, port->pair_port);
		if (ret < 0)
			return ret;
		uA += ret;
	}

	tmp_64 = uV;
	tmp_64 *= uA;
	return DIV_ROUND_CLOSEST_ULL(tmp_64, 1000000000);
}

static int ip804ar_pi_get_pw_class(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int ret, reg;

	guard(mutex)(&priv->page_lock);

	/* Class info for port 0,1 in register 0x88, port 2,3 in 0x89 */
	reg = (id < 2) ? IP804AR_REG_PAGE0_PD_CLASS_0_1 :
			 IP804AR_REG_PAGE0_PD_CLASS_2_3;

	ret = ip804ar_read_page_reg(priv, 0, reg);
	if (ret < 0)
		return ret;

	/* port 0,2 are bits[2:0], port 1,3 are bits[6:4] */
	if (id % 2 == 0)
		ret = FIELD_GET(IP804AR_PD_CLASS_MASK, ret);
	else
		ret = FIELD_GET(IP804AR_PD_CLASS_MASK, ret >> 4);

	/* Convert unknown to valid class */
	if (ret == IP804AR_PD_CLASS_UNKNOWN)
		ret = 0;
	return ret;
}

static int ip804ar_pi_get_pw_req(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	int ret;

	guard(mutex)(&priv->page_lock);

	/* Read PD requested power register */
	ret = ip804ar_read_page_reg(priv, 0,
				    IP804AR_REG_PAGE0_PD_REQ_PW(id));
	if (ret < 0)
		return ret;

	return IP804AR_PD_REQ_PW_MW(ret);
}

static int ip804ar_pi_get_pw_limit(struct pse_controller_dev *pcdev, int id)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	struct ip804ar_port *port = &priv->port[id];
	int ret, mw;

	guard(mutex)(&priv->page_lock);

	ret = ip804ar_read_page_reg(priv, 1,
				    IP804AR_REG_PAGE1_HDPL_PORT0 + id);
	if (ret < 0)
		return ret;

	mw = IP804AR_HDPL_MW(ret);

	/* For 4-pair mode read paired port too */
	if (port->is_4pair && port->pair_port >= 0) {
		ret = ip804ar_read_page_reg(priv, 1,
					    IP804AR_REG_PAGE1_HDPL_PORT0 +
					    port->pair_port);
		if (ret < 0)
			return ret;
		mw += IP804AR_HDPL_MW(ret);
	}

	return mw;
}

static int ip804ar_pi_set_pw_limit(struct pse_controller_dev *pcdev, int id,
				   int max_mW)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	struct ip804ar_port *port = &priv->port[id];
	u8 val;
	int ret;

	if (max_mW < 0 || max_mW > MAX_PI_PW)
		return -ERANGE;

	guard(mutex)(&priv->page_lock);

	/* Convert mW to register value (6 int + 2 frac, in W) */
	val = (max_mW * 4) / 1000;

	ret = ip804ar_write_page_reg(priv, 1,
				     IP804AR_REG_PAGE1_HDPL_PORT0 + id, val);
	if (ret < 0)
		return ret;

	/* For 4-pair mode also set limit on paired port */
	if (port->is_4pair && port->pair_port >= 0) {
		ret = ip804ar_write_page_reg(priv, 1,
					     IP804AR_REG_PAGE1_HDPL_PORT0 +
					     port->pair_port, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* ============================================================
 * PI matrix setup for 4-pair support
 * ============================================================
 */

static int ip804ar_match_channel(const struct pse_pi_pairset *pairset,
				 struct pse_pi *all_pi, int num_pi)
{
	int i;

	for (i = 0; i < num_pi; i++) {
		if (pairset->np == all_pi[i].pairset[0].np ||
		    pairset->np == all_pi[i].pairset[1].np)
			return i;
	}

	return -ENODEV;
}

static int
ip804ar_setup_pi_matrix(struct pse_controller_dev *pcdev)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	struct pse_pi *pi;
	int i;
	u8 high_power = 0;

	/* Check each PI for 4-pair configuration */
	for (i = 0; i < pcdev->nr_lines; i++) {
		pi = &pcdev->pi[i];

		/* Check if both pairsets are present (4-pair) */
		if (pi->pairset[0].np && pi->pairset[1].np) {
			int pair0_id, pair1_id;

			pair0_id = ip804ar_match_channel(&pi->pairset[0], pi,
							 pcdev->nr_lines);
			pair1_id = ip804ar_match_channel(&pi->pairset[1], pi,
							 pcdev->nr_lines);

			if (pair0_id < 0 && pair1_id < 0)
				continue;

			/* Valid 4-pair configuration: must be ports 0+1
			 * or ports 2+3
			 */
			if (pair0_id == 0 && pair1_id == 1) {
				priv->port[0].is_4pair = true;
				priv->port[0].pair_port = 1;
				priv->port[1].is_4pair = true;
				priv->port[1].pair_port = 0;
				high_power |= IP804AR_4PAIR_PORT01_EN;
			} else if (pair0_id == 1 && pair1_id == 0) {
				priv->port[0].is_4pair = true;
				priv->port[0].pair_port = 1;
				priv->port[1].is_4pair = true;
				priv->port[1].pair_port = 0;
				high_power |= IP804AR_4PAIR_PORT01_EN;
			} else if (pair0_id == 2 && pair1_id == 3) {
				priv->port[2].is_4pair = true;
				priv->port[2].pair_port = 3;
				priv->port[3].is_4pair = true;
				priv->port[3].pair_port = 2;
				high_power |= IP804AR_4PAIR_PORT23_EN;
			} else if (pair0_id == 3 && pair1_id == 2) {
				priv->port[2].is_4pair = true;
				priv->port[2].pair_port = 3;
				priv->port[3].is_4pair = true;
				priv->port[3].pair_port = 2;
				high_power |= IP804AR_4PAIR_PORT23_EN;
			} else {
				dev_err(pcdev->dev,
					"Invalid 4-pair pairing: ports %d and %d\n",
					pair0_id, pair1_id);
				return -EINVAL;
			}
		}
	}

	if (!high_power)
		return 0;

	/* Configure 4-pair mode */
	guard(mutex)(&priv->page_lock);
	return ip804ar_write_page_reg(priv, 1, IP804AR_REG_PAGE1_4PAIR_CTRL,
				      high_power | IP804AR_4PAIR_AUTO_SWAP);
}

/* ============================================================
 * IRQ handler and event mapping
 * ============================================================
 */

static int ip804ar_irq_event_handler(struct ip804ar_priv *priv,
				     unsigned long *notifs,
				     unsigned long *notifs_mask)
{
	int i, ret;
	u8 events;

	/* Read port power event registers to check for power events */
	for (i = 0; i < IP804AR_NUM_PORTS; i++) {
		if (!priv->port[i].present)
			continue;

		scoped_guard(mutex, &priv->page_lock) {
			ret = ip804ar_read_page_reg(priv, 1,
						    IP804AR_REG_PAGE1_PORT_EVT(i));
			if (ret < 0)
				return ret;

			events = ret;

			/* Clear events by writing 1 (W1C) */
			if (events) {
				ret = i2c_smbus_write_byte_data(priv->client,
								IP804AR_REG_PAGE1_PORT_EVT(i),
								events);
				if (ret < 0)
					return ret;
			}
		}

		if (!events)
			continue;

		*notifs_mask |= BIT(i);

		/* Map IP804AR events to PSE notifications */
		if (events & IP804AR_EVT_THERM_SHUTDOWN)
			notifs[i] |= ETHTOOL_PSE_EVENT_OVER_TEMP |
				     ETHTOOL_C33_PSE_EVENT_DISCONNECTION;
		if (events & IP804AR_EVT_TEMP_LIM)
			notifs[i] |= ETHTOOL_PSE_EVENT_OVER_TEMP;
		if (events & IP804AR_EVT_OVERLOAD)
			notifs[i] |= ETHTOOL_PSE_EVENT_OVER_CURRENT |
				     ETHTOOL_C33_PSE_EVENT_DISCONNECTION;
		if (events & IP804AR_EVT_SHORT_CKT)
			notifs[i] |= ETHTOOL_PSE_EVENT_OVER_CURRENT |
				     ETHTOOL_C33_PSE_EVENT_DISCONNECTION;
		if (events & IP804AR_EVT_MPS_ERR)
			notifs[i] |= ETHTOOL_C33_PSE_EVENT_DISCONNECTION;
	}

	/* Read severe short circuit event register */
	scoped_guard(mutex, &priv->page_lock) {
		ret = ip804ar_read_page_reg(priv, 1,
					    IP804AR_REG_PAGE1_SEV_SHORT_EVT);
		if (ret < 0)
			return ret;

		events = ret;
		if (events) {
			/* Clear severe short circuit events (W1C) */
			ret = i2c_smbus_write_byte_data(priv->client,
							IP804AR_REG_PAGE1_SEV_SHORT_EVT,
						events);
			if (ret < 0)
				return ret;

			for_each_set_bit(i, (unsigned long *)&events,
					 IP804AR_NUM_PORTS) {
				*notifs_mask |= BIT(i);
				notifs[i] |= ETHTOOL_PSE_EVENT_OVER_CURRENT |
					     ETHTOOL_C33_PSE_EVENT_DISCONNECTION;
			}
		}
	}

	return 0;
}

static int ip804ar_irq_map_event(int irq, struct pse_controller_dev *pcdev,
				 unsigned long *notifs,
				 unsigned long *notifs_mask)
{
	struct ip804ar_priv *priv = to_ip804ar_priv(pcdev);
	struct ip804ar_port *port;
	int ret, i;
	u8 sys_evt;

	/* Read initialization and current overload status register */
	scoped_guard(mutex, &priv->page_lock) {
		ret = ip804ar_read_page_reg(priv, 1,
					    IP804AR_REG_PAGE1_SYS_EVT);
		if (ret < 0)
			return ret;

		sys_evt = ret;

		/* Clear system event (W1C) */
		if (sys_evt & IP804AR_SYS_EVT_PSE_OVERLOAD) {
			ret = ip804ar_write_page_reg(priv, 1,
						     IP804AR_REG_PAGE1_SYS_EVT,
					     IP804AR_SYS_EVT_PSE_OVERLOAD);
			if (ret < 0)
				return ret;
		}
	}

	/* Now read the actual port events and map to notifications */
	ret = ip804ar_irq_event_handler(priv, notifs, notifs_mask);
	if (ret < 0)
		return ret;

	/* Also check for detection/classification events by reading
	 * state machine status for all ports
	 */
	scoped_guard(mutex, &priv->page_lock) {
		for (i = 0; i < IP804AR_NUM_PORTS; i++) {
			port = &priv->port[i];
			if (!port->present)
				continue;

			ret = ip804ar_read_page_reg(priv, 1,
						    IP804AR_REG_PAGE1_SM_CTRL(i));
			if (ret < 0)
				return ret;

			u8 state = ret & IP804AR_SM_STATE_MASK;

			/* Check for PD detection event */
			if (state == IP804AR_SM_STATE_POWER_ON &&
			    !port->pd_detected) {
				port->pd_detected = true;
				*notifs_mask |= BIT(i);
				notifs[i] |= ETHTOOL_C33_PSE_EVENT_DETECTION |
					     ETHTOOL_C33_PSE_EVENT_CLASSIFICATION;
			}

			/* Check for PD disconnection */
			if (state == IP804AR_SM_STATE_IDLE &&
			    port->pd_detected) {
				port->pd_detected = false;
				*notifs_mask |= BIT(i);
				notifs[i] |=
					ETHTOOL_C33_PSE_EVENT_DISCONNECTION;
			}
		}
	}

	return 0;
}

/* ============================================================
 * PSE controller operations table
 * ============================================================
 */

static const struct pse_controller_ops ip804ar_ops = {
	.setup_pi_matrix	= ip804ar_setup_pi_matrix,
	.pi_enable		= ip804ar_pi_enable,
	.pi_disable		= ip804ar_pi_disable,
	.pi_get_admin_state	= ip804ar_pi_get_admin_state,
	.pi_get_pw_status	= ip804ar_pi_get_pw_status,
	.pi_get_voltage		= ip804ar_pi_get_voltage,
	.pi_get_actual_pw	= ip804ar_pi_get_actual_pw,
	.pi_get_pw_class	= ip804ar_pi_get_pw_class,
	.pi_get_pw_req		= ip804ar_pi_get_pw_req,
	.pi_get_pw_limit	= ip804ar_pi_get_pw_limit,
	.pi_set_pw_limit	= ip804ar_pi_set_pw_limit,
};

/* ============================================================
 * I2C probe and IRQ setup
 * ============================================================
 */

static int ip804ar_setup_irq(struct ip804ar_priv *priv, int irq)
{
	struct pse_irq_desc irq_desc = {
		.name = "ip804ar-irq",
		.map_event = ip804ar_irq_map_event,
	};

	if (!irq) {
		dev_dbg(&priv->client->dev, "Interrupt is missing\n");
		return 0;
	}

	priv->irq_notifs = devm_kcalloc(&priv->client->dev,
					IP804AR_NUM_PORTS,
					sizeof(*priv->irq_notifs),
					GFP_KERNEL);
	if (!priv->irq_notifs)
		return -ENOMEM;

	return devm_pse_irq_helper(&priv->pcdev, irq, 0, &irq_desc);
}

static int ip804ar_init_chip(struct ip804ar_priv *priv)
{
	int ret, i;
	u8 cfg;

	guard(mutex)(&priv->page_lock);

	/* Set to manual mode */
	ret = ip804ar_read_page_reg(priv, 1, IP804AR_REG_PAGE1_SYS_CONFIG);
	if (ret < 0)
		return ret;

	cfg = (ret & ~IP804AR_MODE_MASK) |
	      FIELD_PREP(IP804AR_MODE_MASK, IP804AR_OPMODE_MANUAL);

	ret = i2c_smbus_write_byte_data(priv->client,
					IP804AR_REG_PAGE1_SYS_CONFIG, cfg);
	if (ret < 0)
		return ret;

	/* Read I2C address pins to determine device ID */
	ret = ip804ar_read_page_reg(priv, 0, IP804AR_REG_DEV_ADDR);
	if (ret < 0)
		return ret;

	/* Configure I2C timeout */
	ret = i2c_smbus_write_byte_data(priv->client, IP804AR_REG_DEV_ADDR,
					ret | IP804AR_I2C_TIMEOUT_5MS);
	if (ret < 0)
		return ret;

	/* Set power event handling: enable auto back-off */
	for (i = 0; i < IP804AR_NUM_PORTS; i++) {
		ret = ip804ar_read_page_reg(priv, 1,
					    IP804AR_REG_PAGE1_PORT_PW_CTRL(i));
		if (ret < 0)
			return ret;

		/* Enable auto back-off on error */
		cfg = ret | IP804AR_PW_CTRL_EN_AUTO_BOFF;
		/* Set initial state to disabled in manual mode */
		cfg &= ~IP804AR_PW_CTRL_INIT_STATE_MASK;
		cfg |= IP804AR_PW_CTRL_INIT_ENABLED;

		ret = i2c_smbus_write_byte_data(priv->client,
						IP804AR_REG_PAGE1_PORT_PW_CTRL(i),
						cfg);
		if (ret < 0)
			return ret;
	}

	/* Set power event handling options */
	ret = ip804ar_write_page_reg(priv, 1, IP804AR_REG_PAGE1_EVT_HANDLE,
				     IP804AR_HANDLE_VOL_LIM);
	if (ret < 0)
		return ret;

	/* Set default error delay (1.6s as per IEEE) */
	ret = ip804ar_write_page_reg(priv, 0, IP804AR_REG_PAGE0_ERROR_DELAY,
				     IP804AR_ERROR_DELAY_DEFAULT);
	if (ret < 0)
		return ret;

	/* Set AF mode for all ports (can be adjusted per port) */
	ret = ip804ar_write_page_reg(priv, 0, IP804AR_REG_PAGE0_AF_AT_MODE,
				     IP804AR_MODE_AF);
	if (ret < 0)
		return ret;

	/* Enable auto IVT polling */
	ret = ip804ar_write_page_reg(priv, 0, IP804AR_REG_PAGE0_IVT_POLL,
				     IP804AR_POLL_AUTO_ENABLE);
	if (ret < 0)
		return ret;

	return 0;
}

static struct ip804ar_priv *ip804ar_alloc_priv(struct device *dev)
{
	struct ip804ar_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	mutex_init(&priv->page_lock);

	priv->pcdev.owner = THIS_MODULE;
	priv->pcdev.ops = &ip804ar_ops;
	priv->pcdev.dev = dev;
	priv->pcdev.types = ETHTOOL_PSE_C33;
	priv->pcdev.nr_lines = IP804AR_NUM_PORTS;

	return priv;
}

static int ip804ar_i2c_probe(struct i2c_client *client)
{
	struct ip804ar_priv *priv;
	int ret, i;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "Required I2C functionality not supported\n");
		return -ENXIO;
	}

	priv = ip804ar_alloc_priv(&client->dev);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(client, priv);

	/* Verify chip by reading hardware revision */
	scoped_guard(mutex, &priv->page_lock) {
		ret = ip804ar_read_page_reg(priv, 1, IP804AR_REG_PAGE1_HW_REV_MSB);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to read hardware revision\n");
			return ret;
		}

		if (ret != IP804AR_HW_REV_MSB_DEFAULT) {
			dev_err(&client->dev,
				"Unexpected MSB hardware revision: 0x%02x\n", ret);
			return -ENODEV;
		}

		ret = ip804ar_read_page_reg(priv, 1, IP804AR_REG_PAGE1_HW_REV_LSB);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to read LSB hardware revision\n");
			return ret;
		}

		if (ret != IP804AR_HW_REV_LSB_DEFAULT) {
			dev_err(&client->dev,
				"Unexpected LSB hardware revision: 0x%02x\n", ret);
			return -ENODEV;
		}
	}

	dev_info(&client->dev, "IC Plus IP804AR detected (HW rev 0x04A2)\n");

	/* Mark all ports as present for non-DT case */
	for (i = 0; i < IP804AR_NUM_PORTS; i++)
		priv->port[i].present = true;

	/* Initialize chip to known state */
	ret = ip804ar_init_chip(priv);
	if (ret) {
		dev_err_probe(&client->dev, ret,
			      "Failed to initialize chip\n");
		return ret;
	}

	/* Register PSE controller */
	ret = devm_pse_controller_register(&client->dev, &priv->pcdev);
	if (ret) {
		dev_err_probe(&client->dev, ret,
			      "Failed to register PSE controller\n");
		return ret;
	}

	/* Setup IRQ handling */
	ret = ip804ar_setup_irq(priv, client->irq);
	if (ret)
		dev_warn(&client->dev, "Failed to setup IRQ: %d\n", ret);

	return 0;
}

static const struct i2c_device_id ip804ar_id[] = {
	{ "ip804ar" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ip804ar_id);

static const struct of_device_id ip804ar_of_match[] = {
	{ .compatible = "icplus,ip804ar" },
	{ }
};
MODULE_DEVICE_TABLE(of, ip804ar_of_match);

static struct i2c_driver ip804ar_driver = {
	.probe = ip804ar_i2c_probe,
	.id_table = ip804ar_id,
	.driver = {
		.name = "ip804ar",
		.of_match_table = ip804ar_of_match,
	},
};
module_i2c_driver(ip804ar_driver);

MODULE_AUTHOR("Bevan Weiss <bevan.weiss@gmail.com>");
MODULE_DESCRIPTION("IC Plus IP804AR 4-Port PoE PSE Controller Driver");
MODULE_LICENSE("GPL");
