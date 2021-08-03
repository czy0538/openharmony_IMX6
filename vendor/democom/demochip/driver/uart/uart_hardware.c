#include "linux/delay.h"
#include "asm/io.h"

#include "los_typedef.h"
#include "los_task.h"
#include "los_base.h"
#include "los_event.h"
#include "errno.h"

#include "linux/interrupt.h"
#include "linux/kernel.h"
#include "linux/spinlock.h"
#include "asm/platform.h"
#include <sys/bus.h>

#include "uart.h"
#include "uart_dev.h"
#include "string.h"

#include "los_magickey.h"
#include "uart_hardware.h"

struct demochip_port {
        int enable;
        unsigned long phys_base;
        unsigned int irq_num;
        struct uart_driver_data *udd;
};

__attribute__ ((section(".data"))) UINT32 g_uart_fputc_en = 1;


LITE_OS_SEC_BSS STATIC SPIN_LOCK_INIT(g_uartOutputSpin);


void uart_putc_phy(char c)
{
	UART_Type *uartRegs = (UART_Type *)0x02020000;

	while (!((uartRegs->USR2) & (1<<3))); /*等待上个字节发送完毕*/
	uartRegs->UTXD = (unsigned char)c;		
}

void uart_putc_virt(char c)
{
	UART_Type *uartRegs = (UART_Type *)UART_REG_BASE;

	while (!((uartRegs->USR2) & (1<<3))); /*等待上个字节发送完毕*/
	uartRegs->UTXD = (unsigned char)c;		
}



STATIC INLINE UINTPTR uart_to_ptr(UINTPTR n)
{
    (VOID)n;
    return UART_REG_BASE;
}

STATIC VOID UartPutcReg(UINTPTR base, CHAR c)
{
	UART_Type *uartRegs = (UART_Type *)base;

	while (!((uartRegs->USR2) & (1<<3))); /*等待上个字节发送完毕*/
	uartRegs->UTXD = (unsigned char)c;		

}

STATIC VOID UartPutStr(UINTPTR base, const CHAR *s, UINT32 len)
{
    UINT32 i;

    for (i = 0; i < len; i++) {
        if (*(s + i) == '\n') {
			UartPutcReg(base, *"\r");
        }
		UartPutcReg(base, *(s + i));
    }
}


UINT32 UartPutsReg(UINTPTR base, const CHAR *s, UINT32 len, BOOL isLock)
{
    UINT32 intSave;

    if (g_uart_fputc_en == 0) {
        return 0;
    }

    if (isLock) {
        LOS_SpinLockSave(&g_uartOutputSpin, &intSave);
        UartPutStr(base, s, len);
        LOS_SpinUnlockRestore(&g_uartOutputSpin, intSave);
    } else {
        UartPutStr(base, s, len);
    }

    return len;
}

VOID UartPuts(const CHAR *s, UINT32 len, BOOL isLock)
{
    UINTPTR base = uart_to_ptr(0);
    (VOID)UartPutsReg(base, s, len, isLock);
}

INT32 uart_puts(const CHAR *s, UINTPTR len, VOID *state)
{
    (VOID)state;
    UINTPTR i;

    for (i = 0; i < len; i++) {
        if (*(s + i) != '\0') {
            if (*(s + i) == '\n') {
                (VOID)uart_fputc('\r', NULL);
            }

            (VOID)uart_fputc(*(s + i), NULL);
        }
    }

    return (INT32)len;
}
VOID uart_early_init(VOID)
{
    /* enable uart transmit */
}

VOID uart_init(VOID)
{
}

#define FIFO_SIZE    128

static irqreturn_t demochip_uart_irq(int irq, void *data)
{
	char buf[FIFO_SIZE];
	unsigned int count = 0;
	struct demochip_port *port = NULL;
	struct uart_driver_data *udd = (struct uart_driver_data *)data;
	UART_Type *uartRegs;
    uint32_t status;
	
	if (udd == NULL) {
		uart_error("udd is null!\n");
		return IRQ_HANDLED;
	}
	port = (struct demochip_port *)udd->private;

	uartRegs = (UART_Type *)port->phys_base;

    status = uartRegs->USR2;
    if (status & (1<<0)) {
        do {
            buf[count++] = uartRegs->URXD;
            if (udd->num != CONSOLE_UART) {
				PRINT_RELEASE("%s %s %d, udd->num = %d, CONSOLE_UART = %d\n", __FILE__, __FUNCTION__, __LINE__, udd->num, CONSOLE_UART);
                continue;
            }
            if (CheckMagicKey(buf[count - 1])) {
                goto end;
            }

			if (buf[count-1] == '\r')
				buf[count-1] = '\n';
			status = uartRegs->USR2;
		} while (status & (1<<0));
		//buf[count] = '\0';
		//PRINT_RELEASE("%s %s %d, udd->recv %d, %s\n", __FILE__, __FUNCTION__, __LINE__, count, buf);
        udd->recv(udd, buf, count);
    }
	
end:
	/* clear all interrupt */
	return 0;
}


static int demochip_config_in(struct uart_driver_data *udd)
{
	return 0;
}

static int demochip_startup(struct uart_driver_data *udd) 
{
	int ret = 0;
	struct demochip_port *port = NULL;
	UART_Type *uartRegs;

	if (udd == NULL) {
		uart_error("udd is null!\n");
		return -EFAULT;
	}

	port = (struct demochip_port *)udd->private;
	if (!port) {
		uart_error("port is null!");
		return -EFAULT;
	}
	
	uartRegs = (UART_Type *)port->phys_base;
	
	/* enable the clock */
	LOS_TaskLock();
	//uart_clk_cfg(udd->num, true); //use for hi3518
	LOS_TaskUnlock();
	
	
	uartRegs = (UART_Type *)port->phys_base;
	
    /* enable the clock */
    /* uart disable */
	uartRegs->UCR1 &= ~(1<<0);
	
    /* clear all interrupt,set mask */
    /* mask all interrupt */

	/* enable Transmitter and Receiver */
	uartRegs->UCR2 |= (1<<2)|(1<<1);

    /* mask all interrupt */
	/* UCR3
	 * [13] : DTREN, Data Terminal Ready Interrupt Enable
	 * [12] : PARERREN, Parity Error Interrupt Enable
	 * [11] : FRAERREN, Frame Error Interrupt Enable
	 * [5]  : AIRINTEN, Asynchronous IR WAKE Interrupt Enable
	 * [4]  : AWAKEN, Asynchronous WAKE Interrupt Enable
	 * [3]  : DTRDEN, Data Terminal Ready Delta Enable
	 * [0]  : ACIEN, Autobaud Counter Interrupt Enable
	 */
	uartRegs->UCR3 &= ~((1<<13)|(1<<12)|(1<<11)|(1<<5)|(1<<4)|(1<<3)|(1<<0));

	/* UCR4
	 * [8] : ENIRI, Serial Infrared Interrupt Enable
	 * [7] : WKEN, WAKE Interrupt Enable
	 * [6] : IDDMAEN, DMA IDLE Condition Detected Interrupt Enable
	 * [3]  : TCEN, TransmitComplete Interrupt Enable
	 * [2]  : BKEN, BREAK Condition Detected Interrupt Enable
	 * [1]  : OREN, Receiver Overrun Interrupt Enable
	 * [0]  : DREN, Receive Data Ready Interrupt Enable
	 */
	uartRegs->UCR4 &= ~((1<<8)|(1<<7)|(1<<6)|(1<<3)|(1<<2)|(1<<1)|(1<<0));
	

	ret = request_irq(port->irq_num, (irq_handler_t)demochip_uart_irq,
							  0, "uart_dw", udd);

	/* enable rx interrupt */
	uartRegs->UCR4 |= (1<<0);

	/* enable uart */
	uartRegs->UCR1 |= (1<<0);
	
	demochip_config_in(udd);

	return ret;
}

static int demochip_shutdown(struct uart_driver_data *udd)
{
	return 0;
}

static int demochip_start_tx(struct uart_driver_data *udd, const char *buf, size_t count)
{
	unsigned int tx_len = count;
	struct demochip_port *port = NULL;
	char value;
	unsigned int i;
	int ret = 0;

	if (udd == NULL) {
		uart_error("udd is null!\n");
		return -EFAULT;
	}
	port = (struct demochip_port *)udd->private;
	if (!port) {
		uart_error("port is null!");
		return -EFAULT;
	}
	/* UART_WITH_LOCK: there is a spinlock in the function to write reg in order. */
	for (i = 0; i < tx_len; i++ ){
		ret = LOS_CopyToKernel((void *)&value, sizeof(char),(void *)(buf++), sizeof(char));
		if (ret) {
			return i;
		}
		(void)UartPutsReg(port->phys_base, &value, 1, UART_WITH_LOCK);
	}
	return count;
}

static int demochip_config(struct uart_driver_data *udd)
{
	return demochip_config_in(udd);
}

static struct uart_ops demochip_uops = {
	.startup        = demochip_startup,
	.shutdown       = demochip_shutdown,
	.start_tx       = demochip_start_tx,
	.config         = demochip_config,
};

#define MAX_DEV_NAME_SIZE  32
extern const struct file_operations_vfs uartdev_fops;
extern struct uart_driver_data *get_udd_by_unit(int unit);

static int demochip_attach(device_t self)
{
	struct resource *res = NULL;
	char dev_name[MAX_DEV_NAME_SIZE];
	struct demochip_port *port = NULL;
	int unit = device_get_unit(self);
	struct uart_softc *sc = device_get_softc(self);
	struct uart_driver_data *udd = sc->udd;

	if (udd == NULL) {
		uart_error("demochip_attach get uart driver data err!");
		return -1;
	}
	port = (struct demochip_port *)LOS_MemAlloc(m_aucSysMem0, sizeof(struct demochip_port));
	if (!port) {
		return -1;
	}
	memset_s(port, sizeof(struct demochip_port), 0, sizeof(struct demochip_port));
	res = bus_alloc_resource_any(self, SYS_RES_MEMORY, &unit, 0);
	if (!res) {
		goto err;
	}
	port->phys_base = (unsigned long)(uintptr_t)ioremap(res->start, res->count);
	if (!port->phys_base) {
		goto err;
	}
	res = bus_alloc_resource_any(self, SYS_RES_IRQ, &unit, 0);
	if (!res) {
		goto err;
	}
	
	port->irq_num = res->start;
	if (port->irq_num == LOS_NOK) {
		goto err;
	}

	udd->private = port;
	udd->ops = &demochip_uops;
	port->udd = udd;
	udd->recv = uart_recv_notify;
	udd->count = 0;
	memset_s(dev_name, MAX_DEV_NAME_SIZE, 0, MAX_DEV_NAME_SIZE);
	snprintf_s(dev_name, MAX_DEV_NAME_SIZE, MAX_DEV_NAME_SIZE - 1, "/dev/uartdev-%d", udd->num);
	if (register_driver(dev_name, &uartdev_fops, 0666, udd)) {
		uart_error("gen /dev/uartdev-%d fail!\n", udd->num);
		goto err;
	}

	return 0;
err:
	iounmap((void *)(uintptr_t)port->phys_base);
	(VOID)LOS_MemFree(m_aucSysMem0, port);
	port =  NULL;
	return -1;
}

static int demochip_probe(device_t self)
{
	return (BUS_PROBE_DEFAULT);
}

static int demochip_detach(device_t self)
{
	struct uart_softc *sc = device_get_softc(self);
	struct uart_driver_data *udd = sc->udd;
	struct demochip_port *port = NULL;
	char dev_name[MAX_DEV_NAME_SIZE];
	if (udd == NULL || (udd->state != UART_STATE_USEABLE)) {
		uart_error("demochip_detach uart driver data state invalid!");
		return -1;
	}

	(void)memset_s(dev_name, MAX_DEV_NAME_SIZE, 0, MAX_DEV_NAME_SIZE);
	(void)snprintf_s(dev_name, MAX_DEV_NAME_SIZE, MAX_DEV_NAME_SIZE - 1, "/dev/uartdev-%d", udd->num);
	if (unregister_driver(dev_name)) {
		uart_error("demochip_detach unregister /dev/uartdev-%d fail!\n", udd->num);
	}
	port = udd->private;
	if (port == NULL) {
		return -1;
	}
	if (port->phys_base) {
		iounmap((void *)(uintptr_t)port->phys_base);
		port->phys_base = 0;
	}
	(VOID)LOS_MemFree(m_aucSysMem0, port);
	udd->private = NULL;
	return 0;
}

static device_method_t uart_methods[] = 
{
	/* Device interface */
	DEVMETHOD(device_probe, demochip_probe),
	DEVMETHOD(device_attach, demochip_attach),
	DEVMETHOD(device_detach, demochip_detach),
	DEVMETHOD(device_shutdown, bus_generic_shutdown),
	DEVMETHOD_END
};

static driver_t uart_driver = 
{
	.name = "uart",
	.methods = uart_methods,
	.size = sizeof(struct uart_softc),
};

static devclass_t uart_devclass;
DRIVER_MODULE(uart, nexus, uart_driver, uart_devclass, 0, 0);

int uart_dev_init(void)
{
	return driver_module_handler(NULL, MOD_LOAD, &uart_nexus_driver_mod);
}

void uart_dev_exit(void)
{
	driver_module_handler(NULL, MOD_UNLOAD, &uart_nexus_driver_mod);
}
