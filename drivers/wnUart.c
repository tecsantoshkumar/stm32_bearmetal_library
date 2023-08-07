

#include "wnUart.h"

/* Driver Version */
static const wnDRIVER_VERSION DriverVersion = {
    wnUSART_API_VERSION,
    wnUSART_DRV_VERSION
};

/* Driver Capabilities */
static const wnUSART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref wnUSARTx_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref wnUSARTx_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref wnUSARTx_EVENT_CTS */
    0, /* Signal DSR change event: \ref wnUSARTx_EVENT_DSR */
    0, /* Signal DCD change event: \ref wnUSARTx_EVENT_DCD */
    0, /* Signal RI change event: \ref wnUSARTx_EVENT_RI */
    0  /* Reserved */
};

static wnDRIVER_VERSION wnUSARTGetVersion(wnVoid)
{
    return DriverVersion;
}

static wnUSART_CAPABILITIES wnUSARTGetCapabilities(wnVoid)
{
    return DriverCapabilities;
}

/* USART0 Driver wrapper functions */
static UARTx_Resources USART0_DEV = {
	.base         = CMSDK_UART0_BASE,
	.state        = 0,
    .system_clk   = 0,
    .baudrate     = DEFAULT_UART_BAUDRATE,
    .tx_nbr_bytes = 0,
    .rx_nbr_bytes = 0,
    .cb_event     = NULL
};

/* Initialises the uart */
static wnInt32 wnUSARTxInitialize(UARTx_Resources *uart,wnBool Interrupt_flag_enable )
{
	CMSDK_UART_TypeDef *dev = (CMSDK_UART_TypeDef *)uart->base;
   
	
	/* Sets baudrate and system clock */
    uart->system_clk = SystemCoreClock;
    
	/* Disable UART when changing configuration */
	dev->CTRL = 0;         

	/* Sets baudrate */
	dev->BAUDDIV = (uart->system_clk / uart->baudrate);

	/* Enables receiver and transmitter Interrupt */
    if(Interrupt_flag_enable)
    {
	    dev->CTRL = CMSDK_UART_CTRL_RXEN_Msk | CMSDK_UART_CTRL_TXEN_Msk| CMSDK_UART_CTRL_TXIRQEN_Msk |  CMSDK_UART_CTRL_RXIRQEN_Msk;
    }
    else
    {
        dev->CTRL = CMSDK_UART_CTRL_RXEN_Msk | CMSDK_UART_CTRL_TXEN_Msk;
    }
	uart->state = wnUART_INITIALIZED;

    return wnDRIVER_OK;
}
/* checks for the power control*/
static wnInt32 wnUSARTxPowerControl(UARTx_Resources *uart, wnPOWER_STATE state)
{
    ARG_UNUSED(uart);
    switch (state) {
    case wnPOWER_OFF:
    case wnPOWER_LOW:
        return wnDRIVER_ERROR_UNSUPPORTED;
    case wnPOWER_FULL:
        /* Nothing to be done */
        return wnDRIVER_OK;
    default:
		return wnDRIVER_OK;
    }
	
	return wnDRIVER_OK;
}
/* Data transmit functions
@parameters : 	pointer to UARTx_Resources 
		data - data to be trandmitted
		num - number of bytes of data
*/
static wnInt32 wnUSARTxSend(UARTx_Resources* uart, const wnVoid *data, wnUInt32 num)
{
	CMSDK_UART_TypeDef *dev = (CMSDK_UART_TypeDef *)uart->base;
    const uint8_t *p_data = (const uint8_t *)data;
	
    if (!(uart->state & wnUART_INITIALIZED)) {
        return wnUSART_ERROR_NOT_INITIALIZED;
    }
	
    if ((data == NULL) || (num == 0U)) {
        /* Invalid parameters */
        return wnDRIVER_ERROR_PARAMETER;
    }

    /* Resets previous TX counter */
    uart->tx_nbr_bytes = 0;

    while (uart->tx_nbr_bytes != num) {
        /* Waits until UART is ready to transmit */
		while (dev->STATE & CMSDK_UART_STATE_TXBF_Msk) {};
		
		dev->DATA = (wnUInt32)*p_data;		
        uart->tx_nbr_bytes++;
        p_data++;
    }
    /* Waits until lost character is transmited */
	while (dev->STATE & CMSDK_UART_STATE_TXBF_Msk) {};

    return wnDRIVER_OK;
}
/* Data Receive functions
@parameters : pointer to UARTx_Resources 
			  data - data to be received
			  num - number of bytes of data
*/
static wnInt32 wnUSARTxReceive(UARTx_Resources *uart, wnVoid *data, wnUInt32 num)
{
	CMSDK_UART_TypeDef *dev = (CMSDK_UART_TypeDef *)uart->base;
    uint8_t *p_data = (uint8_t *)data;

    if ((data == NULL) || (num == 0U)) {
        // Invalid parameters
        return wnDRIVER_ERROR_PARAMETER;
    }
    /* Resets previous RX counter */
    uart->rx_nbr_bytes = 0;

    while (uart->rx_nbr_bytes != num) {
		/* Waits until one character is received */
		while (!(dev->STATE & CMSDK_UART_STATE_RXBF_Msk)) {};

		/* Reads data */
		*p_data = (uint8_t)dev->DATA;
		p_data++;
        uart->rx_nbr_bytes++;  
    }

    return wnDRIVER_OK;
}
/* To get the data transmitted count */
static wnUInt32 wnUSARTxGetTxCount(UARTx_Resources *uart)
{
    return uart->tx_nbr_bytes;
}

/* To get the data receive count */
static wnUInt32 wnUSARTxGetRxCount(UARTx_Resources *uart)
{
    return uart->rx_nbr_bytes;
}

/*To set the uart baudrate and control bits */
static wnInt32 wnUSARTxControl(UARTx_Resources *uart, wnUInt32 control, wnUInt32 arg)
{
	CMSDK_UART_TypeDef *dev = (CMSDK_UART_TypeDef *)uart->base;
	wnUInt32 bauddiv;
	
    if (!(uart->state & wnUART_INITIALIZED)) {
        return wnUSART_ERROR_NOT_INITIALIZED;
    }	

    switch (control & wnUSART_CONTROL_Msk) {
    case wnUSART_MODE_ASYNCHRONOUS:
    
		if (arg == 0) {
			return wnUSART_ERROR_BAUDRATE;
		}
	    
        if (!(uart->state & wnUART_INITIALIZED)) {
			return wnUSART_ERROR_NOT_INITIALIZED;
		}
		/* Sets baudrate */
		bauddiv = (uart->system_clk / arg);
		uart->baudrate = arg;

		/* Minimum bauddiv value */
		if (bauddiv < 16) {
			return wnUSART_ERROR_BAUDRATE;
		}
		dev->BAUDDIV = bauddiv;
		//dev->BAUDDIV = 333;
        break;
    /* Unsupported command */
    default:
        return wnDRIVER_ERROR_UNSUPPORTED;
    }
    /* UART Data bits */
    if (control & wnUSART_DATA_BITS_Msk) {
        /* Data bit is not configurable */
        return wnDRIVER_ERROR_UNSUPPORTED;
    }

    /* UART Parity */
    if (control & wnUSART_PARITY_Msk) {
        /* Parity is not configurable */
        return wnUSART_ERROR_PARITY;
    }

    /* USART Stop bits */
    if (control & wnUSART_STOP_BITS_Msk) {
        /* Stop bit is not configurable */
        return wnUSART_ERROR_STOP_BITS;
    }

    return wnDRIVER_OK;
}

static wnInt32 wnUSARTxReceive_interupt(UARTx_Resources *uart, void *data, wnUInt32 num)
{
	CMSDK_UART_TypeDef *dev = (CMSDK_UART_TypeDef *)uart->base;
    uint8_t *p_data = (uint8_t *)data;

    if ((data == NULL) || (num == 0U)) {
        // Invalid parameters
        return wnDRIVER_ERROR_PARAMETER;
    }
    /* Resets previous RX counter */
    uart->rx_nbr_bytes = 0;

    while (uart->rx_nbr_bytes != num) {
		/* Waits until one character is received */
		//while (!(dev->STATE & CMSDK_UART_STATE_RXBF_Msk)) {};
		
		//while (!(dev->CTRL & CMSDK_UART_CTRL_RXIRQEN_Msk)) {};
        while (!(dev->INTSTATUS & (CMSDK_UART_STATUS_RXIRQ_Pos))) {
           //
        };
         dev->INTCLEAR = (dev->INTCLEAR | CMSDK_UART_STATUS_RXIRQ_Pos);
		/* Reads data */
		*p_data = (uint8_t)dev->DATA;
		p_data++;
        uart->rx_nbr_bytes++;  
    }

    return wnDRIVER_OK;
}
volatile bool tx_done = true;//This is used to do tx only if previous tx has been completed by the UARTTX1_Handler
static wnInt32 wnUSARTxSend_interupt(UARTx_Resources* uart, const void *data, wnUInt32 num)
{
	CMSDK_UART_TypeDef *dev = (CMSDK_UART_TypeDef *)uart->base;
    const uint8_t *p_data = (const uint8_t *)data;
	
    if (!(uart->state & wnUART_INITIALIZED)) {
        return wnUSART_ERROR_NOT_INITIALIZED;
    }
	
    if ((data == NULL) || (num == 0U)) {
        /* Invalid parameters */
        return wnDRIVER_ERROR_PARAMETER;
    }

    /* Resets previous TX counter */
    uart->tx_nbr_bytes = 0;
	
	dev->CTRL = dev->CTRL | CMSDK_UART_CTRL_TXIRQEN_Msk;
    tx_done = true;
    do  {
        //Send 1 st byte
        if(tx_done ){
            tx_done = false;
            dev->DATA = (wnUInt32)*p_data;
            uart->tx_nbr_bytes++;
            p_data++;
        }	

        //while ((dev->INTSTATUS & CMSDK_UART_STATUS_TXIRQ_Pos)==0){}
        //dev->INTCLEAR = dev->INTCLEAR | CMSDK_UART_STATUS_TXIRQ_Pos;
        
    }while (uart->tx_nbr_bytes != num);
    
	//while(uart->tx_nbr_bytes == num);
    /* Waits until lost character is transmited */
	//while (dev->CTRL & CMSDK_UART_CTRL_TXEN_Msk) {};
    return wnDRIVER_OK;
	
}
	


/**************************************
 * UART0 Driver
 ***************************************/
 
/* UART0 initialisation*/
static wnInt32 wnUSART0Initialize(wnUSART_SignalEvent_t cb_event, wnBool interrupt_flag)
{
    USART0_DEV.cb_event = cb_event;
    return wnUSARTxInitialize(&USART0_DEV,interrupt_flag);
}

static wnInt32 wnUSART0Uninitialize(wnVoid)
{
    /* Nothing to be done */
    return wnDRIVER_OK;
}

/*check UART0 power control*/
static wnInt32 wnUSART0PowerControl(wnPOWER_STATE state)
{
    return wnUSARTxPowerControl(&USART0_DEV, state);
}

/* UART0 data trasmit*/
static wnInt32 wnUSART0Send(const wnVoid *data, wnUInt32 num)
{
    return wnUSARTxSend(&USART0_DEV, data, num);
}

/* UART0 data receive*/
static wnInt32 wnUSART0Receive(wnVoid *data, wnUInt32 num)
{
    return wnUSARTxReceive(&USART0_DEV, data, num);
}

/* UART0 data transfer check*/
static wnInt32 wnUSART0Transfer(const wnVoid *data_out, wnVoid *data_in, wnUInt32 num)
{
    ARG_UNUSED(data_out);
    ARG_UNUSED(data_in);
    ARG_UNUSED(num);

    return wnDRIVER_ERROR_UNSUPPORTED;
}

/* To get UART0 transmit data count*/
static wnUInt32 wnUSART0GetTxCount(wnVoid)
{
    return wnUSARTxGetTxCount(&USART0_DEV);
}

/* To get UART0 receive data count*/
static wnUInt32 wnUSART0GetRxCount(wnVoid)
{
    return wnUSARTxGetRxCount(&USART0_DEV);
}

static wnInt32 wnUSART0Control(wnUInt32 control, wnUInt32 arg)
{
    return wnUSARTxControl(&USART0_DEV, control, arg);
}

static wnUSART_STATUS wnUSART0GetStatus(wnVoid)
{
    wnUSART_STATUS status = { 0, 0, 0, 0, 0, 0, 0, 0 };
    return status;
}

static wnInt32 wnUSART0SetModemControl(wnUSART_MODEM_CONTROL control)
{
    ARG_UNUSED(control);
    return wnDRIVER_ERROR_UNSUPPORTED;
}

static wnUSART_MODEM_STATUS wnUSART0GetModemStatus(wnVoid)
{
    wnUSART_MODEM_STATUS modem_status = { 0, 0, 0, 0, 0 };
    return modem_status;
}

/* UART0 data receive*/
static wnInt32 wnUSART0Receive_interupt(wnVoid *data, wnUInt32 num)
{
    return wnUSARTxReceive_interupt(&USART0_DEV, data, num);
}

static wnInt32 wnUSART0Send_interupt(const wnVoid *data, wnUInt32 num)
{
    return wnUSARTxSend_interupt(&USART0_DEV, data, num);
}

/* wrapping wnDRIVER_USART to WISIG_UART0*/
wnDRIVER_USART WISIG_UART0 = {
    wnUSARTGetVersion,
    wnUSARTGetCapabilities,
    wnUSART0Initialize,
    wnUSART0Uninitialize,
    wnUSART0PowerControl,
    wnUSART0Send,
    wnUSART0Receive,
    wnUSART0Transfer,
    wnUSART0GetTxCount,
    wnUSART0GetRxCount,
    wnUSART0Control,
    wnUSART0GetStatus,
    wnUSART0SetModemControl,
    wnUSART0GetModemStatus,
	wnUSART0Receive_interupt,
	wnUSART0Send_interupt
};

/**************************************************
 * UART1 Driver
 **************************************************/
 
/* USART0 Driver wrapper functions */
static UARTx_Resources USART1_DEV = {
	.base         = CMSDK_UART1_BASE,
	.state        = 0,
    .system_clk   = 0,
    .baudrate     = DEFAULT_UART_BAUDRATE,
    .tx_nbr_bytes = 0,
    .rx_nbr_bytes = 0,
    .cb_event     = NULL
};

/* UART1 initialisation*/
static wnInt32 wnUSART1Initialize(wnUSART_SignalEvent_t cb_event, wnBool interrupt_flag)
{
    USART0_DEV.cb_event = cb_event;
    return wnUSARTxInitialize(&USART1_DEV,interrupt_flag);
}

static wnInt32 wnUSART1Uninitialize(wnVoid)
{
    /* Nothing to be done */
    return wnDRIVER_OK;
}

/*check UART1 power control*/
static wnInt32 wnUSART1PowerControl(wnPOWER_STATE state)
{
    return wnUSARTxPowerControl(&USART1_DEV, state);
}

/* UART1 data trasmit*/
static wnInt32 wnUSART1Send(const wnVoid *data, wnUInt32 num)
{
    return wnUSARTxSend(&USART1_DEV, data, num);
}

/* UART1 data receive*/
static wnInt32 wnUSART1Receive(wnVoid *data, wnUInt32 num)
{
    return wnUSARTxReceive(&USART1_DEV, data, num);
}

/* UART1 data transfer check*/
static wnInt32 wnUSART1Transfer(const wnVoid *data_out, wnVoid *data_in, wnUInt32 num)
{
    ARG_UNUSED(data_out);
    ARG_UNUSED(data_in);
    ARG_UNUSED(num);

    return wnDRIVER_ERROR_UNSUPPORTED;
}

/* To get UART1 transmit data count*/
static wnUInt32 wnUSART1GetTxCount(wnVoid)
{
    return wnUSARTxGetTxCount(&USART1_DEV);
}

/* To get UART1 receive data count*/
static wnUInt32 wnUSART1GetRxCount(wnVoid)
{
    return wnUSARTxGetRxCount(&USART1_DEV);
}

static wnInt32 wnUSART1Control(wnUInt32 control, wnUInt32 arg)
{
    return wnUSARTxControl(&USART1_DEV, control, arg);
}

static wnUSART_STATUS wnUSART1GetStatus(wnVoid)
{
    wnUSART_STATUS status = { 0, 0, 0, 0, 0, 0, 0, 0 };
    return status;
}

static wnInt32 wnUSART1SetModemControl(wnUSART_MODEM_CONTROL control)
{
    ARG_UNUSED(control);
    return wnDRIVER_ERROR_UNSUPPORTED;
}

static wnUSART_MODEM_STATUS wnUSART1GetModemStatus(wnVoid)
{
    wnUSART_MODEM_STATUS modem_status = { 0, 0, 0, 0, 0 };
    return modem_status;
}

static wnInt32 wnUSART1Receive_interupt(wnVoid *data, wnUInt32 num)
{
    return wnUSARTxReceive_interupt(&USART1_DEV, data, num);
}

static wnInt32 wnUSART1Send_interupt(const wnVoid *data, wnUInt32 num)
{
    return wnUSARTxSend_interupt(&USART1_DEV, data, num);
}


/* wrapping wnDRIVER_USART to WISIG_UART1*/
wnDRIVER_USART WISIG_UART1 = {
    wnUSARTGetVersion,
    wnUSARTGetCapabilities,
    wnUSART1Initialize,
    wnUSART1Uninitialize,
    wnUSART1PowerControl,
    wnUSART1Send,
    wnUSART1Receive,
    wnUSART1Transfer,
    wnUSART1GetTxCount,
    wnUSART1GetRxCount,
    wnUSART1Control,
    wnUSART1GetStatus,
    wnUSART1SetModemControl,
    wnUSART1GetModemStatus,
	wnUSART1Receive_interupt,
	wnUSART1Send_interupt
};

