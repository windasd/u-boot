// SPDX-License-Identifier: GPL-2.0+
/*
 * Actions Semi S900 I2C controller, polled, in its FIFO ("command list")
 * mode. Registers and command sequence as in Linux
 * drivers/i2c/busses/i2c-owl.c and the vendor kernel 3.10 driver.
 *
 * The controller sends a target address plus up to six register address
 * bytes, an optional repeated start and one data phase per command, so a
 * transfer is one message, or a short write followed by a message to the
 * same target.
 */

#include <clk.h>
#include <dm.h>
#include <i2c.h>
#include <reset.h>
#include <time.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/time.h>

#define OWL_I2C_CTL		0x00
#define OWL_I2C_CLKDIV		0x04
#define OWL_I2C_STAT		0x08
#define OWL_I2C_TXDAT		0x10
#define OWL_I2C_RXDAT		0x14
#define OWL_I2C_CMD		0x18
#define OWL_I2C_FIFOCTL		0x1c
#define OWL_I2C_FIFOSTAT	0x20
#define OWL_I2C_DATCNT		0x24

#define OWL_I2C_CTL_RB		BIT(1)	/* release the bus */
#define OWL_I2C_CTL_GBCC_STOP	(2 << 2)
#define OWL_I2C_CTL_EN		BIT(7)	/* enabling resets the state machine */

#define OWL_I2C_STAT_BEB	BIT(1)	/* bus error */
#define OWL_I2C_STAT_LAB	BIT(3)	/* arbitration lost */
#define OWL_I2C_STAT_BBB	BIT(6)	/* bus busy */
#define OWL_I2C_STAT_W1C	(BIT(7) | GENMASK(5, 1))

#define OWL_I2C_CMD_SBE		BIT(0)	/* start */
#define OWL_I2C_CMD_AS(x)	(((x) & 0x7) << 1)	/* bytes after the start */
#define OWL_I2C_CMD_RBE		BIT(4)	/* repeated start */
#define OWL_I2C_CMD_SAS(x)	(((x) & 0x7) << 5)	/* bytes after it */
#define OWL_I2C_CMD_DE		BIT(8)	/* data phase of DATCNT bytes */
#define OWL_I2C_CMD_NS		BIT(9)	/* NACK the last byte read */
#define OWL_I2C_CMD_SE		BIT(10)	/* stop */
#define OWL_I2C_CMD_MSS		BIT(11)	/* controller mode */
#define OWL_I2C_CMD_SECL	BIT(15)	/* execute */

#define OWL_I2C_FIFOCTL_RFR	BIT(1)
#define OWL_I2C_FIFOCTL_TFR	BIT(2)

#define OWL_I2C_FIFOSTAT_CECB	BIT(0)	/* command complete */
#define OWL_I2C_FIFOSTAT_RNB	BIT(1)	/* NACK received, write 1 to clear */
#define OWL_I2C_FIFOSTAT_RFE	BIT(2)	/* set: RX FIFO not empty */

#define OWL_I2C_FIFO_SIZE	128
#define OWL_I2C_MAX_REG_LEN	6	/* AS counts the target address too */
/* Every byte written goes through the TX FIFO up front, after up to 8 address bytes */
#define OWL_I2C_MAX_WRITE	(OWL_I2C_FIFO_SIZE - OWL_I2C_MAX_REG_LEN - 2)
#define OWL_I2C_MAX_READ	1023	/* DATCNT; the RX FIFO is drained while polling */
#define OWL_I2C_ADDR_MAX	0x7f	/* 7-bit addresses only */
#define OWL_I2C_DIV_MAX		0xff
#define OWL_I2C_TIMEOUT_US	100000	/* FIFO reset, idle bus; slack for a command */

struct owl_i2c_priv {
	void __iomem *base;
	ulong clk_rate;
	uint speed;
	uint div;
};

static int owl_i2c_check(struct i2c_msg *msg, int nmsgs)
{
	struct i2c_msg *data;

	if (nmsgs < 1 || nmsgs > 2)
		return -EOPNOTSUPP;
	data = &msg[nmsgs - 1];
	/* addr << 1 must fit TXDAT: 0xd0 would silently become 0x50 */
	if (data->addr > OWL_I2C_ADDR_MAX)
		return -EINVAL;
	if (data->flags & ~I2C_M_RD)
		return -EOPNOTSUPP;
	if (nmsgs == 2 && (msg[0].flags || msg[0].len > OWL_I2C_MAX_REG_LEN ||
			   msg[0].addr != data->addr))
		return -EOPNOTSUPP;
	if (data->len > (data->flags & I2C_M_RD ? OWL_I2C_MAX_READ : OWL_I2C_MAX_WRITE))
		return -EOPNOTSUPP;

	return 0;
}

static int owl_i2c_wait_clear(struct owl_i2c_priv *priv, u32 reg, u32 mask)
{
	ulong start = timer_get_us();

	while (readl(priv->base + reg) & mask)
		if (timer_get_us() - start > OWL_I2C_TIMEOUT_US)
			return -ETIMEDOUT;

	return 0;
}

/* Reset the state machine and both FIFOs, set the clock, wait for an idle bus */
static int owl_i2c_prepare(struct owl_i2c_priv *priv)
{
	int ret;

	writel(0, priv->base + OWL_I2C_CTL);
	writel(OWL_I2C_CTL_EN, priv->base + OWL_I2C_CTL);
	writel(OWL_I2C_STAT_W1C, priv->base + OWL_I2C_STAT);
	writel(priv->div, priv->base + OWL_I2C_CLKDIV);

	writel(OWL_I2C_FIFOCTL_RFR | OWL_I2C_FIFOCTL_TFR,
	       priv->base + OWL_I2C_FIFOCTL);
	ret = owl_i2c_wait_clear(priv, OWL_I2C_FIFOCTL,
				 OWL_I2C_FIFOCTL_RFR | OWL_I2C_FIFOCTL_TFR);
	if (ret)
		return ret;

	ret = owl_i2c_wait_clear(priv, OWL_I2C_STAT, OWL_I2C_STAT_BBB);
	if (ret)
		return ret;

	return readl(priv->base + OWL_I2C_STAT) & OWL_I2C_STAT_LAB ? -EAGAIN : 0;
}

/* Twice the bus time of the data and up to 8 address bytes, 9 clocks each, plus slack */
static ulong owl_i2c_timeout_us(struct owl_i2c_priv *priv, uint len)
{
	return OWL_I2C_TIMEOUT_US +
	       2 * (len + 8) * 9 * (USEC_PER_SEC / priv->speed + 1);
}

/* Run the command, draining the RX FIFO into the data message */
static int owl_i2c_run(struct owl_i2c_priv *priv, u32 cmd, struct i2c_msg *data)
{
	ulong timeout = owl_i2c_timeout_us(priv, data->len);
	bool rd = data->flags & I2C_M_RD;
	ulong start = timer_get_us();
	u32 fifostat;
	uint n = 0;

	writel(cmd, priv->base + OWL_I2C_CMD);
	do {
		/* Sampled before draining, so no byte is left in the FIFO */
		fifostat = readl(priv->base + OWL_I2C_FIFOSTAT);
		while (rd && n < data->len &&
		       readl(priv->base + OWL_I2C_FIFOSTAT) & OWL_I2C_FIFOSTAT_RFE)
			data->buf[n++] = readl(priv->base + OWL_I2C_RXDAT);
		if (fifostat & (OWL_I2C_FIFOSTAT_CECB | OWL_I2C_FIFOSTAT_RNB))
			break;
		if (timer_get_us() - start > timeout) {
			writel(OWL_I2C_CTL_EN | OWL_I2C_CTL_GBCC_STOP | OWL_I2C_CTL_RB,
			       priv->base + OWL_I2C_CTL);
			udelay(100);
			return -ETIMEDOUT;
		}
	} while (true);

	if (fifostat & OWL_I2C_FIFOSTAT_RNB) {
		writel(OWL_I2C_FIFOSTAT_RNB, priv->base + OWL_I2C_FIFOSTAT);
		return -EREMOTEIO;
	}
	if (readl(priv->base + OWL_I2C_STAT) & OWL_I2C_STAT_BEB)
		return -EIO;

	return rd && n != data->len ? -EIO : 0;
}

static int owl_i2c_xfer(struct udevice *dev, struct i2c_msg *msg, int nmsgs)
{
	struct owl_i2c_priv *priv = dev_get_priv(dev);
	struct i2c_msg *data;
	u32 cmd;
	uint i;
	bool rd;
	int ret;

	ret = owl_i2c_check(msg, nmsgs);
	if (ret)
		return ret;
	data = &msg[nmsgs - 1];
	rd = data->flags & I2C_M_RD;

	ret = owl_i2c_prepare(priv);
	if (ret)
		goto out;

	cmd = OWL_I2C_CMD_SECL | OWL_I2C_CMD_MSS | OWL_I2C_CMD_SE |
	      OWL_I2C_CMD_NS | OWL_I2C_CMD_DE | OWL_I2C_CMD_SBE;
	if (nmsgs == 2) {
		/* Target and register address, then a repeated start */
		cmd |= OWL_I2C_CMD_AS(msg[0].len + 1) | OWL_I2C_CMD_SAS(1) |
		       OWL_I2C_CMD_RBE;
		writel(msg[0].addr << 1, priv->base + OWL_I2C_TXDAT);
		for (i = 0; i < msg[0].len; i++)
			writel(msg[0].buf[i], priv->base + OWL_I2C_TXDAT);
	} else {
		cmd |= OWL_I2C_CMD_AS(1);
	}

	writel(data->len, priv->base + OWL_I2C_DATCNT);
	writel(data->addr << 1 | rd, priv->base + OWL_I2C_TXDAT);
	for (i = 0; !rd && i < data->len; i++)
		writel(data->buf[i], priv->base + OWL_I2C_TXDAT);
	writel(0, priv->base + OWL_I2C_FIFOCTL);	/* a NACK ends the command */

	ret = owl_i2c_run(priv, cmd, data);
out:
	writel(0, priv->base + OWL_I2C_CTL);

	return ret;
}

static int owl_i2c_set_bus_speed(struct udevice *dev, uint speed)
{
	struct owl_i2c_priv *priv = dev_get_priv(dev);
	uint div;

	if (!speed || speed > I2C_SPEED_FAST_RATE)
		return -EINVAL;

	/* SCL = clock / (16 * div), never faster than asked */
	div = DIV_ROUND_UP(priv->clk_rate, speed * 16);
	if (div > OWL_I2C_DIV_MAX)
		return -EINVAL;
	priv->div = div;
	priv->speed = speed;

	return 0;
}

static int owl_i2c_probe(struct udevice *dev)
{
	struct owl_i2c_priv *priv = dev_get_priv(dev);
	struct reset_ctl rst;
	struct clk clk;
	int ret;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret)
		return ret;
	ret = clk_enable(&clk);
	if (ret)
		return ret;

	/*
	 * The controller may come up held in reset, and an S900 block in
	 * reset hangs the bus when accessed: pulse it, as the vendor U-Boot
	 * does, before the first register access.
	 */
	ret = reset_get_by_index(dev, 0, &rst);
	if (!ret) {
		reset_assert(&rst);
		udelay(10);
		ret = reset_deassert(&rst);
	}
	if (ret && ret != -ENOENT)
		return ret;

	priv->clk_rate = clk_get_rate(&clk);
	if (!priv->clk_rate || IS_ERR_VALUE(priv->clk_rate))
		return -EINVAL;

	return owl_i2c_set_bus_speed(dev, I2C_SPEED_STANDARD_RATE);
}

static const struct dm_i2c_ops owl_i2c_ops = {
	.xfer		= owl_i2c_xfer,
	.set_bus_speed	= owl_i2c_set_bus_speed,
};

static const struct udevice_id owl_i2c_ids[] = {
	{ .compatible = "actions,s900-i2c" },
	{ }
};

U_BOOT_DRIVER(i2c_owl) = {
	.name		= "i2c_owl",
	.id		= UCLASS_I2C,
	.of_match	= owl_i2c_ids,
	.probe		= owl_i2c_probe,
	.priv_auto	= sizeof(struct owl_i2c_priv),
	.ops		= &owl_i2c_ops,
};
