#include "ARMCM0.h"
#include <stdint.h>
#include <string.h>

#ifdef __GNUC__
	#define ALIGN4_U8 _Alignas(4) uint8_t
#else
	#define ALIGN4_U8 __align(4) uint8_t
#endif

#define read_reg(addr)                   (*(volatile unsigned int*)(addr))
#define write_reg(addr,data)             (*(volatile unsigned int*)(addr) = (unsigned int)(data))
#define subWriteReg(addr,high,low,value) write_reg(addr,read_reg(addr)&\
										 (~((((unsigned int)1<<((high)-(low)+1))-1)<<(low)))|\
										 ((unsigned int)(value)<<(low)))
#define PHY_REG_RD(x) (*(volatile uint32_t *)(x))
#define PHY_REG_WT(x,y) (*(volatile uint32_t *)(x) = (uint32_t)(y))

#define  XTAL            ( 5000000U)      /* Oscillator frequency             */
#define  SYSTEM_CLOCK    (5 * XTAL)

#define JUMPTABLE_BASE_ADDR 0x1fff0000
#define CONFIG_BASE_ADDR 0x1fff0400
#define AP_APB0_BASE (0x40000000UL)
#define AP_AON_BASE (AP_APB0_BASE + 0xF000)/*aon*/
#define AP_AON ((AP_AON_TypeDef  *) AP_AON_BASE)
#define AP_IOMUX_BASE (AP_APB0_BASE + 0x3800)/*iomux*/
#define AP_IOMUX ((IOMUX_TypeDef *) AP_IOMUX_BASE)
#define AP_GPIOA_BASE (AP_APB0_BASE + 0x8000)/*gpio*/
#define AP_GPIO ((AP_GPIO_TypeDef *) AP_GPIOA_BASE)

#define __NVIC_PRIO_BITS  2U /* Number of Bits used for Priority Levels */
#define IRQ_PRIO_REALTIME 0
#define IRQ_PRIO_HIGH     1
#define BB_IRQn           4
#define RTC_IRQn          6

#define BIT(n) (1ul << (n))
#define RET_SRAM0 BIT(0)  /*32K, 0x1fff0000~0x1fff7fff*/
#define RET_SRAM1 BIT(1)  /*16K, 0x1fff8000~0x1fffbfff*/
#define RET_SRAM2 BIT(2)  /*16K, 0x1fffc000~0x1fffffff*/

#define DCDC_CONFIG_SETTING(x) subWriteReg(0x4000f014,18,15, (0x0f&(x)))
#define DCDC_REF_CLK_SETTING(x) subWriteReg(0x4000f014,25,25, (0x01&(x)))
#define DIG_LDO_CURRENT_SETTING(x) subWriteReg(0x4000f014,22,21, (0x03&(x)))
#define HAL_PWRMGR_RAM_RETENTION_SET(x) subWriteReg(0x4000f01c,21,17, (((x) & 0xffffffe0) ? 0x00 : (x)))
#define HAL_PWRMGR_LOWCURRENTLDO_ENABLE(x) subWriteReg(0x4000f014,26,26, (x));
#define ENABLE_SOFTWARE_CONTROL(x) subWriteReg(&(AP_AON->PMCTL2_0),6,6, (x));
#define RTC_TICK_CURRENT (AP_AON->RTCCNT)
#define SET_SLEEP_FLAG (AP_AON->REG_S11 |= 1)
#define UNSET_SLEEP_FLAG (AP_AON->REG_S11 &= ~1)
#define ENTER_SYSTEM_SLEEP_MODE (AP_AON->PWRSLP = 0xA5A55A5A)
#define ENTER_SYSTEM_OFF_MODE (AP_AON->PWROFF = 0x5A5AA5A5)

#define LL_HW_BASE 0x40031000 // LL_HW Base address
#define LL_HW_IRQ_MASK    0x3FFF // total 14bit
#define LL_HW_CRC_BLE_FMT 0x02
#define LL_HW_CRC_ZB_FMT  0x03
#define LL_HW_IGN_CRC     0x0002 //bit1

#define PHY_SET_CHANNEL(x) PHY_REG_WT(0x400300b4, (x))
#define PHY_GET_CHANNEL (PHY_REG_RD(0x400300b4) & 0xff)
#define PHY_SET_SYNCWORD(x) PHY_REG_WT(0x4003004c,(x))
#define PHY_SET_CRC_SEED(x) subWriteReg(0x40030048,23,0,(x))
#define PHY_SET_WHITEN_SEED(x) subWriteReg(0x40030048,31,24,(x))
#define RF_PHY_TPCAL_CALC(tp0,tp1,chn) ((tp0) > (tp1) ? (((tp0<<5)-(tp0-tp1)*(chn)+16)>>5) : tp0 )
#define RF_PHY_LO_LDO_SETTING(x) subWriteReg(0x400300cc,11,10, (0x03&(x)))
#define RF_PHY_LNA_LDO_SETTING(x) subWriteReg(0x400300dc, 6, 5, (0x03&(x)))
#define RF_PHY_PA_VTRIM_SETTING(x) subWriteReg(0x400300dc, 9, 7, (0x03&(x)))

#define PHY_HW_BB_DELAY  90
#define PHY_HW_AFE_DELAY 8
#define PHY_HW_PLL_DELAY 60

#define RF_PHY_TX_POWER_EXTRA_MAX 0x3f
#define RF_PHY_TX_POWER_MAX       0x1f
#define RF_PHY_TX_POWER_MIN       0x00
#define RF_PHY_TX_POWER_5DBM      0x1d
#define RF_PHY_TX_POWER_4DBM      0x17
#define RF_PHY_TX_POWER_3DBM      0x15
#define RF_PHY_TX_POWER_0DBM      0x0d
#define RF_PHY_TX_POWER_N2DBM     0x0a
#define RF_PHY_TX_POWER_N5DBM     0x06
#define RF_PHY_TX_POWER_N6DBM     0x05
#define RF_PHY_TX_POWER_N10DBM    0x03
#define RF_PHY_TX_POWER_N15DBM    0x02
#define RF_PHY_TX_POWER_N20DBM    0x01
#define RF_PHY_EXT_PREAMBLE_US    8 // ext ble preamble length

#define PKT_FMT_ZIGBEE 0
#define PKT_FMT_BLE1M  1
#define PKT_FMT_BLE2M  2
#define PKT_FMT_BLELR3 3
#define PKT_FMT_BLELR4 4
#define PKT_FMT_500K   5
#define PKT_FMT_100K   6
#define WHITEN_SEED_CH37 0x53
#define DEFAULT_CRC_SEED 0x555555
#define DEFAULT_SYNCWORD 0x8e89bed6
#define BLE_ADV_CHN37 02
#define BLE_ADV_CHN38 26
#define BLE_ADV_CHN39 80

#define JUMP_TABLE_SZ    256
#define GLOBAL_CONFIG_SZ 256
#define NUMBER_OF_PINS   23
#define MSEC 32 // 32k RTCCounts is about a second (32kHz clock right?)
#define SEC 32768

#define RC32_TRACKINK_ALLOW         0x00000001 // enable tracking RC 32KHz clock with 16MHz hclk
#define SLAVE_LATENCY_ALLOW         0x00000002 // slave latency allow switch
#define LL_DEBUG_ALLOW              0x00000004 // enable invoke RAM project debug output fucntion
#define LL_WHITELIST_ALLOW          0x00000008 // enable whitelist filter
#define LL_RC32K_SEL                0x00000010 // select RC32K RTC, otherwise select crystal 32K RTC
#define SIMUL_CONN_ADV_ALLOW        0x00000020 // allow send adv in connect state
#define SIMUL_CONN_SCAN_ALLOW       0x00000040 // allow scan in connect state
#define CONN_CSA2_ALLOW             0x00000080 // allow using CSA2 in connection state
#define GAP_DUP_RPT_FILTER_DISALLOW 0x00000100 // duplicate report filter in GAP layer, allow default
#define ENH_CONN_CMP_EVENT_ALLOW    0x00000200 // allow LL to send enhanced connection complete event.


typedef void (*pFunc)(void);

typedef enum {
	ADV_CHANNEL_INTERVAL, SCAN_RSP_DELAY, CONN_REQ_TO_SLAVE_DELAY, SLAVE_CONN_DELAY, SLAVE_CONN_DELAY_BEFORE_SYNC, MAX_SLEEP_TIME, MIN_SLEEP_TIME, WAKEUP_ADVANCE, WAKEUP_DELAY, HDC_DIRECT_ADV_INTERVAL,
	LDC_DIRECT_ADV_INTERVAL, LL_SWITCH, NON_ADV_CHANNEL_INTERVAL, __13__, CLOCK_SETTING, LL_HW_BB_DELAY, LL_HW_AFE_DELAY, LL_HW_PLL_DELAY, LL_HW_RTLP_LOOP_TIMEOUT, LL_HW_RTLP_1ST_TIMEOUT,
	MIN_TIME_TO_STABLE_32KHZ_XOSC, LL_TX_PKTS_PER_CONN_EVT, LL_RX_PKTS_PER_CONN_EVT, DIR_ADV_DELAY, LL_TX_PWR_TO_REG_BIAS, LL_SMART_WINDOW_COEF_ALPHA, LL_SMART_WINDOW_TARGET, LL_SMART_WINDOW_INCREMENT, LL_SMART_WINDOW_LIMIT, LL_SMART_WINDOW_ACTIVE_THD,
	LL_SMART_WINDOW_ACTIVE_RANGE, LL_SMART_WINDOW_FIRST_WINDOW, LL_HW_Tx_TO_RX_INTV, LL_HW_Rx_TO_TX_INTV, INITIAL_STACK_PTR, ALLOW_TO_SLEEP_TICK_RC32K, LL_HW_BB_DELAY_ADV, LL_HW_AFE_DELAY_ADV, LL_HW_PLL_DELAY_ADV, LL_ADV_TO_SCAN_REQ_DELAY,
	LL_ADV_TO_CONN_REQ_DELAY, LL_MOVE_TO_MASTER_DELAY, LL_HW_TRLP_LOOP_TIMEOUT, LL_CONN_REQ_WIN_SIZE, LL_CONN_REQ_WIN_OFFSET, LL_MASTER_PROCESS_TARGET, LL_MASTER_TIRQ_DELAY, LL_HW_BB_DELAY_2MPHY, LL_HW_AFE_DELAY_2MPHY, LL_HW_PLL_DELAY_2MPHY,
	LL_HW_Tx_TO_RX_INTV_2MPHY, LL_HW_Rx_TO_TX_INTV_2MPHY, LL_HW_BB_DELAY_500KPHY, LL_HW_AFE_DELAY_500KPHY, LL_HW_PLL_DELAY_500KPHY, LL_HW_Tx_TO_RX_INTV_500KPHY, LL_HW_Rx_TO_TX_INTV_500KPHY, LL_HW_BB_DELAY_125KPHY, LL_HW_AFE_DELAY_125KPHY, LL_HW_PLL_DELAY_125KPHY,
	LL_HW_Tx_TO_RX_INTV_125KPHY, LL_HW_Rx_TO_TX_INTV_125KPHY, LL_HW_TRLP_TO_GAP, LL_HW_RTLP_TO_GAP, LL_TRX_NUM_ADAPTIVE_CONFIG, OSAL_SYS_TICK_WAKEUP_TRIM, __66__, __67__, __68__, __69__,
	LL_NOCONN_ADV_EST_TIME, LL_NOCONN_ADV_MARGIN, LL_SEC_SCAN_MARGIN, LL_MIN_SCAN_TIME, LL_CONN_ADV_EST_TIME, LL_SCANABLE_ADV_EST_TIME, __76__, __77__, __78__, __79__, 
	MAC_ADDRESS_LOC, LL_EXT_ADV_INTER_PRI_CHN_INT, LL_EXT_ADV_INTER_AUX_CHN_INT, LL_EXT_ADV_RSC_POOL_PERIOD, LL_EXT_ADV_RSC_POOL_UNIT, __85__, LL_EXT_ADV_TASK_DURATION, LL_PRD_ADV_TASK_DURATION, LL_CONN_TASK_DURATION, __89__,
	TIMER_ISR_ENTRY_TIME, LL_MULTICONN_MASTER_PREEMP, LL_MULTICONN_SLAVE_PREEMP, LL_EXT_ADV_INTER_SEC_CHN_INT, LL_EXT_ADV_PRI_2_SEC_CHN_INT, LL_EXT_ADV_RSC_PERIOD, LL_EXT_ADV_RSC_SLOT_DURATION, LL_PRD_ADV_RSC_PERIOD, LL_PRD_ADV_RSC_SLOT_DURATION, LL_EXT_ADV_PROCESS_TARGET,
	LL_PRD_ADV_PROCESS_TARGET,
} global_config_e;

typedef enum {
	GPIO_P00   =   0,    P0  =  GPIO_P00,
	GPIO_P01   =   1,    P1  =  GPIO_P01,
	GPIO_P02   =   2,    P2  =  GPIO_P02,
	GPIO_P03   =   3,    P3  =  GPIO_P03,
	GPIO_P07   =   4,    P7  =  GPIO_P07,
	GPIO_P09   =   5,    P9  =  GPIO_P09,
	GPIO_P10   =   6,    P10  =  GPIO_P10,
	GPIO_P11   =   7,    P11  =  GPIO_P11,   Analog_IO_0 = GPIO_P11,
	GPIO_P14   =   8,    P14  =  GPIO_P14,   Analog_IO_1 = GPIO_P14,
	GPIO_P15   =   9,    P15  =  GPIO_P15,   Analog_IO_2 = GPIO_P15,
	GPIO_P16   =   10,   P16  =  GPIO_P16,   Analog_IO_3 = GPIO_P16,XTALI = GPIO_P16,
	GPIO_P17   =   11,   P17  =  GPIO_P17,   Analog_IO_4 = GPIO_P17,XTALO = GPIO_P17,
	GPIO_P18   =   12,   P18  =  GPIO_P18,   Analog_IO_5 = GPIO_P18,
	GPIO_P20   =   13,   P20  =  GPIO_P20,   Analog_IO_6 = GPIO_P20,
	GPIO_P23   =   14,   P23  =  GPIO_P23,   Analog_IO_7 = GPIO_P23,
	GPIO_P24   =   15,   P24  =  GPIO_P24,   Analog_IO_8 = GPIO_P24,
	GPIO_P25   =   16,   P25  =  GPIO_P25,   Analog_IO_9 = GPIO_P25,
	GPIO_P26   =   17,   P26  =  GPIO_P26,
	GPIO_P27   =   18,   P27  =  GPIO_P27,
	GPIO_P31   =   19,   P31  =  GPIO_P31,
	GPIO_P32   =   20,   P32  =  GPIO_P32,
	GPIO_P33   =   21,   P33  =  GPIO_P33,
	GPIO_P34   =   22,   P34  =  GPIO_P34,
	GPIO_NUM   =   23,
	GPIO_DUMMY =  0xff,
} gpio_pin_e;

enum {
	GPIO_PIN_ASSI_NONE = 0,
	GPIO_PIN_ASSI_OUT,
	GPIO_PIN_ASSI_IN,
};

typedef enum {
	Bit_DISABLE = 0,
	Bit_ENABLE,
} bit_action_e;

typedef enum {
	GPIO_INPUT  = 0,
	GPIO_OUTPUT = 1
} gpio_dir_t;

typedef struct {
	int           enable;
	uint8_t       pin_state;
} gpioin_Ctx_t;

typedef struct {
	int           state;
	uint8_t       pin_assignments[NUMBER_OF_PINS];
	gpioin_Ctx_t  irq_ctx[NUMBER_OF_PINS];
} gpio_Ctx_t;

typedef enum {
	GPIO_FLOATING   = 0x00,     //no pull
	GPIO_PULL_UP_S  = 0x01,     //pull up strong
	GPIO_PULL_UP    = 0x02,     //pull up weak
	GPIO_PULL_DOWN  = 0x03,
} gpio_pupd_e;

typedef struct {
	uint8_t    reg_i;
	uint8_t    bit_h;
	uint8_t    bit_l;
} PULL_TypeDef;

const PULL_TypeDef c_gpio_pull[GPIO_NUM] = {
	{0,2,1},  //p0
	{0,5,4},  //p1
	{0,8,7},  //p2
	{0,11,10},//p3
	{0,23,22},//p7
	{0,29,28},//p9
	{1,2,1},  //p10
	{1,5,4},  //p11
	{1,14,13},//p14
	{1,17,16},//p15
	{1,20,19},//p16
	{1,23,22},//p17
	{1,26,25},//p18
	{2,2,1},  //p20
	{2,11,10},//p23
	{2,14,13},//p24
	{2,17,16},//p25
	{2,20,19},//p26
	{2,23,22},//p27
	{3,5,4},  //p31
	{3,8,7},  //p32
	{3,11,10},//p33
	{3,14,13},//p34
};

typedef struct {
	gpio_pin_e pin;
	gpio_pupd_e type;
} ioinit_cfg_t;

typedef enum  _SYSCLK_SEL {
	SYS_CLK_RC_32M   = 0,
	SYS_CLK_DBL_32M  = 1,
	SYS_CLK_XTAL_16M = 2,
	SYS_CLK_DLL_48M  = 3,
	SYS_CLK_DLL_64M  = 4,
	SYS_CLK_DLL_96M  = 5,
	SYS_CLK_8M       = 6,
	SYS_CLK_4M       = 7,
	SYS_CLK_NUM      = 8,
} sysclk_t;

typedef enum  _RF_PHY_CLK_SEL {
	RF_PHY_CLK_SEL_16M_XTAL  = 0,
	RF_PHY_CLK_SEL_32M_DBL_B = 1,
	RF_PHY_CLK_SEL_32M_DBL   = 2,
	RF_PHY_CLK_SEL_32M_DLL   = 3
} rfphy_clk_t;

typedef enum {
	CLK_32K_XTAL        = 0,
	CLK_32K_RCOSC       = 1,
} CLK32K_e;

typedef enum {
	MCU_SLEEP_MODE,
	SYSTEM_SLEEP_MODE,
	SYSTEM_OFF_MODE
} Sleep_Mode;

typedef struct {
	volatile uint32_t PWROFF;        //0x00
	volatile uint32_t PWRSLP;        //0x04
	volatile uint32_t IOCTL[3];      //0x08 0x0c 0x10
	volatile uint32_t PMCTL0;        //0x14
	volatile uint32_t PMCTL1;        //0x18
	volatile uint32_t PMCTL2_0;      //0x1c
	volatile uint32_t PMCTL2_1;      //0x20
	volatile uint32_t RTCCTL;        //0x24
	volatile uint32_t RTCCNT;        //0x28
	volatile uint32_t RTCCC0;        //0x2c
	volatile uint32_t RTCCC1;        //0x30
	volatile uint32_t RTCCC2;        //0x34
	volatile uint32_t RTCFLAG;       //0x38
	volatile uint32_t reserved[25];
	volatile uint32_t REG_S9;        //0xa0
	volatile uint32_t REG_S10;       //0xa4
	volatile uint32_t REG_S11;       //0xa8
	volatile uint32_t IDLE_REG;      //0xac
	volatile uint32_t GPIO_WAKEUP_SRC[2]; //0xb0 b4
	volatile uint32_t PCLK_CLK_GATE; //0xb8
	volatile uint32_t XTAL_16M_CTRL; //0xbc
	volatile uint32_t SLEEP_R[4];    //0xc0 c4 c8 cc
} AP_AON_TypeDef;

typedef struct {
	volatile uint32_t Analog_IO_en;  //0x00
	volatile uint32_t SPI_debug_en;  //0x04
	volatile uint32_t debug_mux_en;  //0x08
	volatile uint32_t full_mux0_en;  //0x0c
	volatile uint32_t full_mux1_en;  //0x10 reserved in some soc
	volatile uint32_t gpio_pad_en;   //0x14
	volatile uint32_t gpio_sel[9];   //0x18
	volatile uint32_t pad_pe0;       //0x3c
	volatile uint32_t pad_pe1;       //0x40
	volatile uint32_t pad_ps0;       //0x44
	volatile uint32_t pad_ps1;       //0x48
	volatile uint32_t keyscan_in_en; //0x4c
	volatile uint32_t keyscan_out_en;  //0x50
} IOMUX_TypeDef;

typedef struct {
	volatile uint32_t swporta_dr;    //0x00
	volatile uint32_t swporta_ddr;   //0x04
	volatile uint32_t swporta_ctl;   //0x08
	uint32_t reserved1[9];           //0x18-0x2c portC&D
	volatile uint32_t inten;         //0x30
	volatile uint32_t intmask;       //0x34
	volatile uint32_t inttype_level; //0x38
	volatile uint32_t int_polarity;  //0x3c
	volatile uint32_t int_status;   //0x40
	volatile uint32_t raw_instatus;  //0x44
	volatile uint32_t debounce;      //0x48
	volatile uint32_t porta_eoi;    //0x4c
	volatile uint32_t ext_porta;    //0x50
	uint32_t reserved2[3];           //0x58 0x5c
	volatile uint32_t ls_sync;       //0x60
	volatile uint32_t id_code;      //0x64
	uint32_t reserved3[1];          //0x68
	volatile uint32_t ver_id_code;  //0x6c
	volatile uint32_t config_reg2;  //0x70
	volatile uint32_t config_reg1;  //0x74
} AP_GPIO_TypeDef;



/*********************************************************************
	OSAL LARGE HEAP CONFIG
*/
typedef uint8_t halDataAlign_t; //!< Used for byte alignment
typedef struct {
	// The 15 LSB's of 'val' indicate the total item size, including the header, in 8-bit bytes.
	unsigned short len : 15;   // unsigned short len : 15;
	// The 1 MSB of 'val' is used as a boolean to indicate in-use or freed.
	unsigned short inUse : 1;  // unsigned short inUse : 1;
} osalMemHdrHdr_t;

typedef union {
	/*  Dummy variable so compiler forces structure to alignment of largest element while not wasting
		space on targets when the halDataAlign_t is smaller than a UINT16.
	*/
	halDataAlign_t alignDummy;
	uint32_t val;            // uint16    // TODO: maybe due to 4 byte alignment requirement in M0, this union should be 4 byte, change from uint16 to uint32, investigate more later -  04-25
	osalMemHdrHdr_t hdr;
} osalMemHdr_t;

#define LARGE_HEAP_SIZE (1*1024)
#define __STACK_SIZE 0x00000400
#define __HEAP_SIZE 0x00000C00

ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];
static uint8_t stack[__STACK_SIZE] __attribute__((aligned(8), used, section(".stack")));
static uint8_t heap[__HEAP_SIZE] __attribute__((aligned(8), used, section(".heap")));

/*********************************************************************
	GLOBAL VARIABLES
*/
uint32_t SystemCoreClock = SYSTEM_CLOCK;  /* System Core Clock Frequency      */
volatile uint8_t g_clk32K_config;
volatile sysclk_t g_spif_clk_config;
ALIGN4_U8 phyBufTx[256];
static gpio_Ctx_t m_gpioCtx = {
	.state = 0,
	.pin_assignments = {0,},
};

/*********************************************************************
	EXTERNAL VARIABLES
*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

extern void _start(void) __attribute__((noreturn)); /* PreeMain (C library entry point) */
extern void Reset_Handler(void);
extern volatile sysclk_t g_system_clk;
extern volatile rfphy_clk_t g_rfPhyClkSel;
extern volatile uint8_t g_rfPhyTpCal0;
extern volatile uint8_t g_rfPhyTpCal1;
extern int clk_init(sysclk_t h_system_clk_sel);
extern void osal_mem_set_heap(osalMemHdr_t* hdr, uint32_t size);
extern uint8_t osal_init_system( void );
extern void osal_start_system( void );
extern void* osal_memcpy( void*, const void*, unsigned int );
extern void* osal_memset( void* dest, uint8_t value, int len );
extern void enableSleep(void);
extern void setSleepMode(Sleep_Mode mode);
extern void WaitRTCCount(uint32_t rtcDelyCnt);

extern void drv_irq_init(void);
extern int drv_enable_irq(void);
extern int drv_disable_irq(void);
extern void ll_hw_go(void);
extern void ll_hw_ign_rfifo(uint8_t ignCtrl);
extern void ll_hw_rst_tfifo(void);
extern void ll_hw_tx2rx_timing_config(uint8_t pkt);
extern void ll_hw_set_crc_fmt(uint8_t txCrc,uint8_t rxCrc);
extern void ll_hw_set_rx_tx_interval(uint32_t intvTime);
extern void ll_hw_set_stx(void);
extern void ll_hw_set_trx_settle(uint8_t tmBb,uint8_t tmAfe,uint8_t tmPll);
extern void ll_hw_set_tx_rx_interval(uint32_t intvTime);
extern void ll_hw_set_tx_rx_release(uint16_t txTime,uint16_t rxTime);
extern uint8_t ll_hw_write_tfifo(uint8_t* rxPkt, uint16_t pktLen);
extern void phy_hw_set_stx(void);
extern void set_max_length(uint32_t length);



const pFunc __initial_sp = (pFunc)(&__StackTop);

void Default_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) __attribute__((noreturn));

/*----------------------------------------------------------------------------
	Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
extern const pFunc __Vectors[48];
const pFunc __Vectors[48] __attribute__((used, section(".vectors"))) = {
	(pFunc)(&__StackTop), /* Initial Stack Pointer */
	Reset_Handler,        /* Reset Handler */
	0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
};

void Reset_Handler(void) {
	uint32_t *pSrc, *pDest;
	
	pSrc = &__etext;
	pDest = &__data_start__;
	for (; pDest < &__data_end__;) {
			*pDest++ = *pSrc++;
	}

	pDest = &__bss_start__;
	for (; pDest < &__bss_end__;) {
			*pDest++ = 0UL;
	}

	_start(); /* Enter PreeMain (C library entry point) */
}

void _exit(int status) {
	(void)status;
	while (1);
}


void hal_gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type) {
	uint8_t i = c_gpio_pull[pin].reg_i;
	uint8_t h = c_gpio_pull[pin].bit_h;
	uint8_t l = c_gpio_pull[pin].bit_l;

	if(pin < P31) {
		subWriteReg(&(AP_AON->IOCTL[i]),h,l,type);
	}
	else {
		subWriteReg(&(AP_AON->PMCTL0),h,l,type);
	}
}

static void hal_low_power_io_init(void) {
	//========= pull all io to gnd by default
	ioinit_cfg_t ioInit[] = {
		//TSOP6252 10 IO
		{GPIO_P02, GPIO_FLOATING},/*SWD*/
		{GPIO_P03, GPIO_FLOATING},/*SWD,LED*/
		{GPIO_P09, GPIO_FLOATING},/*UART TX*/
		{GPIO_P10, GPIO_FLOATING},/*UART RX*/
		{GPIO_P11, GPIO_FLOATING},
		{GPIO_P14, GPIO_FLOATING},
		{GPIO_P15, GPIO_FLOATING},
		{GPIO_P16, GPIO_FLOATING},
		{GPIO_P18, GPIO_FLOATING},
		{GPIO_P20, GPIO_FLOATING},
	};

	for(uint8_t i=0; i<sizeof(ioInit)/sizeof(ioinit_cfg_t); i++) {
		hal_gpio_pull_set(ioInit[i].pin,ioInit[i].type);
	}

	DCDC_CONFIG_SETTING(0x0a);
	DCDC_REF_CLK_SETTING(1);
	DIG_LDO_CURRENT_SETTING(0x01);
	HAL_PWRMGR_RAM_RETENTION_SET(RET_SRAM0);
	HAL_PWRMGR_LOWCURRENTLDO_ENABLE(1);
}

void hal_rtc_clock_config(CLK32K_e clk32Mode) {
	if(clk32Mode == CLK_32K_RCOSC) {
		subWriteReg(&(AP_AON->PMCTL0),31,27,0x05);
		subWriteReg(&(AP_AON->PMCTL2_0),16,7,0x3fb);
		subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x01);
	}
	else if(clk32Mode == CLK_32K_XTAL) {
		// P16 P17 for 32K XTAL input
		hal_gpio_pull_set(GPIO_P16, GPIO_FLOATING);
		hal_gpio_pull_set(GPIO_P17, GPIO_FLOATING);
		subWriteReg(&(AP_AON->PMCTL2_0),9,8,0x03);   //software control 32k_clk
		subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x00);   //disable software control
		subWriteReg(&(AP_AON->PMCTL0),31,27,0x16);
	}
}

static void hal_init(void) {
	hal_low_power_io_init();
	clk_init(g_system_clk); //system init
	hal_rtc_clock_config((CLK32K_e)g_clk32K_config);
	enableSleep();
	setSleepMode(SYSTEM_SLEEP_MODE);
}

void hal_gpio_init(void) {
	memset(&m_gpioCtx, 0, sizeof(m_gpioCtx));
	m_gpioCtx.state = 1;
}

void hal_gpio_fmux(gpio_pin_e pin, bit_action_e value) {
	if(value) {
		AP_IOMUX->full_mux0_en |= BIT(pin);
	}
	else {
		AP_IOMUX->full_mux0_en &= ~BIT(pin);
	}
}

void hal_gpio_pin2pin3_control(gpio_pin_e pin, uint8_t en) {//0:sw,1:other func
	if(en) {
		AP_IOMUX->gpio_pad_en |= BIT(pin-2);
	}
	else {
		AP_IOMUX->gpio_pad_en &= ~BIT(pin-2);
	}
}

void hal_gpio_cfg_analog_io(gpio_pin_e pin, bit_action_e value) {
	if(value) {
		hal_gpio_pull_set(pin,GPIO_FLOATING);
		AP_IOMUX->Analog_IO_en |= BIT(pin - P11);
	}
	else {
		AP_IOMUX->Analog_IO_en &= ~BIT(pin - P11);
	}
}

void hal_gpio_pin_init(gpio_pin_e pin, gpio_dir_t type) {
	hal_gpio_fmux(pin,Bit_DISABLE);

	if((pin == P2) || (pin == P3)) {
		hal_gpio_pin2pin3_control(pin,1);
	}

	hal_gpio_cfg_analog_io(pin,Bit_DISABLE);

	if(type == GPIO_OUTPUT) {
		AP_GPIO->swporta_ddr |= BIT(pin);
	}
	else {
		AP_GPIO->swporta_ddr &= ~BIT(pin);
		m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_IN;
	}
}

void hal_gpioretention_register(gpio_pin_e pin) {
	m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_OUT;
	hal_gpio_pin_init(pin, GPIO_OUTPUT);
}

void hal_gpio_write(gpio_pin_e pin, uint8_t en) {
	if(en) {
		AP_GPIO->swporta_dr |= BIT(pin);
	}
	else {
		AP_GPIO->swporta_dr &= ~BIT(pin);
	}
}

void config_RTC(uint32_t time) {
	uint32_t tick_curr = RTC_TICK_CURRENT;
	WaitRTCCount(1); //align to rtc clock edge
	AP_AON->RTCCC0 = tick_curr + time;  //set RTC comparator0 value
	//enable (20) comparator0 event, (18) counter overflow interrupt, (15) comparator0 interrupt
	AP_AON->RTCCTL |= (BIT(15) | BIT(18) | BIT(20));
}

void rf_phy_ana_cfg(uint8_t txPower) {
	//-------------------------------------------------------------------
	//               RF_PHY RX ADC CLOCK Config
	subWriteReg(0x4000f040,18,18, 0x01);  // xtal output to digital enable : ALWAYS Set 1
	subWriteReg(0x4000f044,23,22, g_rfPhyClkSel);
	subWriteReg(0x4000f044, 6,   5, 0x03);    // trim dll/dbl ldo vout

	if((g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DBL) || (g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DBL_B) ) {
		// enable dbl for rf
		subWriteReg(0x4000f044, 8, 8, 0x01);  // DBL EN,DLL EN,DLL LDO EN
	}

	if(g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DLL) {
		// enable dll for rf
		subWriteReg(0x4000f044, 7, 7, 0x01);  // DLL ensable
	}

	subWriteReg(0x4000f044, 19, 18, 0x03);    // Rx adc clk en, rf phy clk en

	if((g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL) && (g_system_clk == SYS_CLK_DLL_48M)) {
		subWriteReg( 0x4003008c,23,23,0x01);
	}
	else {
		subWriteReg( 0x4003008c,23,23,0x00);
	}

	//-------------------------------------------------------------------
	//               PLL
	PHY_REG_WT(0x400300cc,0x20000bc0);  // i_pll_ctrl0 :
	//-------------------------------------------------------------------
	//TX PLL BW
	PHY_REG_WT(0x400300d0,0x00000180);  // i_pll_ctrl1
	PHY_REG_WT(0x400300d4,0x076a3e7a);  // i_pll_ctrl2 pll lpf, boost vco current[7:4]
	PHY_REG_WT(0x400300d8,0x04890000);  // i_pll_ctrl3 vco/tp varactor
	//-------------------------------------------------------------------
	//RX PLL BW active when rx_en
	PHY_REG_WT(0x40030104,0x00000180);  // i_pll_ctrl5
	PHY_REG_WT(0x40030108,0x076a3e7a);  // i_pll_ctrl6 pll lpf, boost vco current[7:4]
	PHY_REG_WT(0x4003010c,0x04898000);  // i_pll_ctrl7 vco/tp varactor
	//-------------------------------------------------------------------
	//VCO Coarse Tuning Setting
	PHY_REG_WT(0x40030080,0x000024cc);  //[11:10] vco coarse tune slot time
	//[9:7] delay from pll reset ends to start vco coarse tune

	//-------------------------------------------------------------------
	//PLL config for rfPhyClk=16M
	if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL) {
		subWriteReg(0x40030080, 0, 0, 1);//indicate 16M reference clk to rfpll
	}

	//-------------------------------------------------------------------
	//               Tx PA
	PHY_REG_WT(0x400300b8, 0x0825|(((txPower)&0x1f)<<12));  // pa ramp reg, txPower for g_rfPhyTxPower
	//-------------------------------------------------------------------
	//               Rx FrontEnd
	PHY_REG_WT(0x400300dc,0x01a6fc2f);      // boost TIA current
	//-------------------------------------------------------------------
}

void rf_phy_bb_cfg(uint8_t pktFmt)
{
	//BW Sel and Gauss sel
	if(pktFmt == PKT_FMT_ZIGBEE || pktFmt == PKT_FMT_BLE2M) {
		PHY_REG_WT( 0x400300e0,0x00000080);   // set pga bw use small bw for zigbee and BLE2M
		subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var

		if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL) {
			PHY_REG_WT( 0x40030090,0x00080000);   // set reg_dc
		}
		else {
			PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc
		}

		//PHY_REG_WT( 0x40030094,0x00001048);   // tp_cal val
	}
	else if(pktFmt < PKT_FMT_500K) {
		PHY_REG_WT( 0x400300e0,0x00000100);   // set pga bw
		subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var

		//subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var
		if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL) {
			PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc
		}
		else {
			PHY_REG_WT( 0x40030090,0x00020000);   // set reg_dc
		}

		//PHY_REG_WT( 0x40030094,0x00001025);   // tp_cal val
	}
	else {
		PHY_REG_WT( 0x400300e0,0x00000100);   // set pga bw
		subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var

		if(g_rfPhyClkSel == RF_PHY_CLK_SEL_16M_XTAL) {
			PHY_REG_WT( 0x40030090,0x00040000);   // set reg_dc
		}
		else {
			PHY_REG_WT( 0x40030090,0x00020000);   // set reg_dc
		}
	}

	PHY_REG_WT( 0x400300b0,0x01000003);                 // dac dly
	PHY_REG_WT( 0x40030094,0x00001000+g_rfPhyTpCal0);   // tp_cal val

	//pktFmt Setting and syncThd
	if(pktFmt == PKT_FMT_ZIGBEE) {
		PHY_REG_WT( 0x40030000,0x78068000);
		PHY_REG_WT( 0x40030048,0x00000000);   //clr crc and wtSeed
		PHY_REG_WT( 0x40030040,0x000b2800);   // disable gauss
		PHY_REG_WT( 0x4003004c,0x3675ee07);
		ll_hw_set_crc_fmt (LL_HW_CRC_ZB_FMT, LL_HW_CRC_ZB_FMT);
	}
	else if(pktFmt == PKT_FMT_BLE1M) {
		PHY_REG_WT( 0x40030000,0x3d068001);
		PHY_REG_WT( 0x40030048,0x37555555);
		PHY_REG_WT( 0x40030040,0x00032800);   // enable gauss
		PHY_REG_WT( 0x4003004c,0x8e89bed6);
		ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
	}
	else if(pktFmt == PKT_FMT_BLE2M) {
		PHY_REG_WT( 0x40030000,0x3d068002);
		PHY_REG_WT( 0x40030048,0x37555555);
		PHY_REG_WT( 0x40030040,0x00032800);   // enable gauss
		PHY_REG_WT( 0x4003004c,0x8e89bed6);
		ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
	}
	else if(pktFmt == PKT_FMT_BLELR3 || pktFmt == PKT_FMT_BLELR4) {
		//pktFmt=3 or pktFmt=4
		PHY_REG_WT( 0x40030000,0x98068000|pktFmt);  //for tx set differnt phy
		PHY_REG_WT( 0x40030004,0x50985a54);         //for RSSI >-90 set higher syncThd=0x98
		PHY_REG_WT( 0x40030040,0x00032800);   // enable gauss
		PHY_REG_WT( 0x40030048,0x37555555);
		PHY_REG_WT( 0x4003004c,0x8e89bed6);
		ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
	}
	else {
		PHY_REG_WT( 0x40030000,0x42068000|pktFmt);
		PHY_REG_WT( 0x40030048,0x00555555);
		PHY_REG_WT( 0x40030040,0x000b2800);   // enable gauss
		PHY_REG_WT( 0x4003004c,0x8e89bed6);
		ll_hw_set_crc_fmt (LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
	}

	//Agc Control Setting
	if(pktFmt == PKT_FMT_ZIGBEE) {
		PHY_REG_WT( 0x40030050,0x22086680);
	}
	else if(pktFmt == PKT_FMT_BLE2M) {
		PHY_REG_WT( 0x40030050,0x22084580);
	}
	else {
		PHY_REG_WT( 0x40030050,0x22085580);
	}

	// add by ZQ 20181030 for DLE feature
	// set to 255
	// need considering the ADV PDU in BLE 5.0
	subWriteReg(0x4003000c,7,0, 0xff);
	//AGC TAB with LNA two gain step,no bypass lna mode
	//20200721 for tsop 6252
	PHY_REG_WT(0x40030054, 0x545c9ca4);
	PHY_REG_WT(0x40030058, 0x4243444c);
	PHY_REG_WT(0x4003005c, 0x464c5241);
	PHY_REG_WT(0x40030060, 0x2e343a40);
	PHY_REG_WT(0x40030064, 0x557f0028);
	PHY_REG_WT(0x40030068, 0x3d43494f);
	PHY_REG_WT(0x4003006c, 0x4c2b3137);
	PHY_REG_WT(0x40030070, 0x343a4046);
	PHY_REG_WT(0x40030074, 0x1c22282e);

	//ext preamble for BLE 1M/2M, nByte
	if(pktFmt == PKT_FMT_BLE1M) {
		subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>3) ); // 1byte -> 8us
	}
	else if(pktFmt == PKT_FMT_BLE2M) {
		subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>2) );//2 byte -> 8us
	}
	else {
		subWriteReg(0x40030040, 7, 5, (0) );
	}
}

void rf_phy_set_txPower(uint8_t txPower) {
	if(txPower == RF_PHY_TX_POWER_EXTRA_MAX) {
		DCDC_CONFIG_SETTING(0x08);//set dcdc to highest
		RF_PHY_LO_LDO_SETTING(0);
		RF_PHY_LNA_LDO_SETTING(0);
		RF_PHY_PA_VTRIM_SETTING(1);
	}
	else {
		DCDC_CONFIG_SETTING(0x0a);
		RF_PHY_LO_LDO_SETTING(2);
		RF_PHY_LNA_LDO_SETTING(1);
		RF_PHY_PA_VTRIM_SETTING(0);
	}

	PHY_REG_WT(0x400300b8,(PHY_REG_RD(0x400300b8)&0x0fff) | ((txPower&0x1f)<<12));
}

void phy_hw_go(void) {
	PHY_REG_WT(LL_HW_BASE +0x14, LL_HW_IRQ_MASK);   //clr  irq status
	PHY_REG_WT(LL_HW_BASE +0x0c, 0x0001);           //mask irq :only use mode done
	PHY_REG_WT(LL_HW_BASE +0x00, 0x0001);           //trig
	uint8_t rfChnIdx = PHY_GET_CHANNEL;

	if(rfChnIdx < 2) {
		rfChnIdx = 2;
	}
	else if(rfChnIdx > 80) {
		rfChnIdx = 80;
	}

	subWriteReg(0x40030094,7,0, RF_PHY_TPCAL_CALC(g_rfPhyTpCal0, g_rfPhyTpCal1, (rfChnIdx-2)>>1));
}

void phy_hw_timing_setting(void) {
	ll_hw_set_tx_rx_release (10,     1);
	ll_hw_set_rx_tx_interval(       60);        //T_IFS=150us for BLE 1M
	ll_hw_set_tx_rx_interval(       66);        //T_IFS=150us for BLE 1M
	ll_hw_set_trx_settle    (57, 8, 52);        //TxBB,RxAFE,PL
}

void phy_hw_pktFmt_Config() {
	//baseband cfg
	rf_phy_bb_cfg(PKT_FMT_BLE1M);

	ll_hw_set_crc_fmt(LL_HW_CRC_BLE_FMT, LL_HW_CRC_BLE_FMT);
	PHY_SET_CRC_SEED(DEFAULT_CRC_SEED);
	ll_hw_ign_rfifo(LL_HW_IGN_CRC);

	//whiten
	PHY_SET_WHITEN_SEED(WHITEN_SEED_CH37);
	//syncword
	PHY_SET_SYNCWORD(DEFAULT_SYNCWORD);
}

void rf_phy_ini(void) {
	g_rfPhyClkSel = RF_PHY_CLK_SEL_16M_XTAL;
	uint8_t txPower = RF_PHY_TX_POWER_N2DBM; // global?

	phy_hw_pktFmt_Config();
	rf_phy_ana_cfg(txPower);
	rf_phy_set_txPower(txPower);
}

void debug_blink(uint32_t nblink) {
	gpio_pin_e pin = P3; // LED is on pin 3
	hal_gpioretention_register(pin);
	while(nblink--) {
		hal_gpio_write(pin, 1);
		WaitRTCCount(5*MSEC);
		hal_gpio_write(pin, 0);
		if(nblink) {
			WaitRTCCount(195*MSEC);
		}
	}
}
void debug_blink3(void) { debug_blink(3); }
void debug_blink5(void) { debug_blink(5); }

void RF_init() {
	gpio_pin_e pin = P3; // LED is on pin 3
	hal_gpioretention_register(pin);
	hal_gpio_write(pin, 1);
	WaitRTCCount(5*MSEC);
	hal_gpio_write(pin, 0);

	uint8_t pubAddr[6] = {0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
	uint8_t advBuffer[31] = {0x02,0x01,0x06,0x1B,0xFF,0x04,0x05,0x01,0x02,0x03,0xcc,0x00,0x03,0xaa,0x00,0x93,0xaa,0x67,0xF7,0xDB,0x34,0xC4,0x03,0x8E,0x5C,0x0B,0xAA,0x97,0x30,0x56,0xE6};
	uint8_t advHead[2]= {0x00,(sizeof(pubAddr) + sizeof(advBuffer))};

	osal_memcpy(&(phyBufTx[0]), &(advHead[0]), sizeof(advHead));
	osal_memcpy(&(phyBufTx[2]), &(pubAddr[0]), sizeof(pubAddr));
	osal_memcpy(&(phyBufTx[8]), &(advBuffer[0]), sizeof(advBuffer));
}

void RF_send() {
	gpio_pin_e pin = P3; // LED is on pin 3
	hal_gpioretention_register(pin);
	hal_gpio_write(pin, 1);
	WaitRTCCount(5*MSEC);
	hal_gpio_write(pin, 0);

	// do advertisement update here


	uint8_t channels[3] = {BLE_ADV_CHN37, BLE_ADV_CHN38, BLE_ADV_CHN39};
	for(size_t i = 0; i < sizeof(channels); i++) {
		drv_disable_irq();

		phy_hw_timing_setting();
		PHY_SET_CHANNEL(channels[i]);
		
		ll_hw_set_stx();
		ll_hw_set_trx_settle(PHY_HW_BB_DELAY, PHY_HW_AFE_DELAY, PHY_HW_PLL_DELAY);

		ll_hw_rst_tfifo();
		set_max_length(0xff);

		ll_hw_write_tfifo(phyBufTx, phyBufTx[1] +2);
		phy_hw_go();

		WaitRTCCount(1*MSEC); // wait for packet to be send
		
		drv_enable_irq();
	}

	config_RTC(500*MSEC);
	// ENTER_SYSTEM_SLEEP_MODE;
}

void populate_jump_table() {
	uint32_t* pJump_table = (uint32_t*)(JUMPTABLE_BASE_ADDR);

	for(int i = 0; i < JUMP_TABLE_SZ; i++) {
		switch(i) {
		case 2: // task array
		case 3: // task count
		case 4: // task events
		case 5: // osal mem init, on wakeup
		case 58: // LL_DIRECT_TEST_TX_TEST, called on wakeup and blocks
		case 59: // LL_DIRECT_TEST_RX_TEST, called on wakeup and blocks
		case 60:  // OSAL_POWER_CONSERVE called by osal_pwrmgr_powerconserve()
		case 61:  // ENTER_SLEEP_PROCESS called by enterSleepProcess()
		case 62:  // WAKEUP_PROCESS, called on wakeup and blocks
		case 63:  // CONFIG_RTC called by enterSleepProcess()
		case 64:  // ENTER_SLEEP_OFF_MODE called by enterSleepProcess()
		case 100: // APP_SLEEP_PROCESS called by enterSleepProcess()
		case 101: // APP_WAKEUP_PROCESS
		case 212: // called by drv_irq_init()
		case 213: // called by drv_enable_irq()
		case 214: // called by osal_pwrmgr_powerconserve()
		case 228: // V4_IRQ_HANDLER
		case 230: // called when RTC comparator0 is reached (event?)
			pJump_table[i] = 0;
			break;
		case 1: // tasks init, should have an entry or osal crashes. Here abused as WAKEUP_PROCESS and sleep again
			pJump_table[i] = (uint32_t)RF_send;
			break;
		case 102:
			pJump_table[i] = (uint32_t)rf_phy_ini;
			break;
		default:
			pJump_table[i] = (uint32_t)debug_blink3;
			break;
		}
	}
}

void global_config() {
	uint32_t* pGlobal_config = (uint32_t*)(CONFIG_BASE_ADDR);
	osal_memset(pGlobal_config, 0x00, GLOBAL_CONFIG_SZ);

	//save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
	pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
	// LL switch setting
	pGlobal_config[LL_SWITCH] =  LL_DEBUG_ALLOW | SLAVE_LATENCY_ALLOW | LL_WHITELIST_ALLOW | SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW; //RC32_TRACKINK_ALLOW

	if(g_clk32K_config == CLK_32K_XTAL) {
		pGlobal_config[LL_SWITCH] &= 0xffffffee;
	}
	else {
		pGlobal_config[LL_SWITCH] |= (RC32_TRACKINK_ALLOW | LL_RC32K_SEL);
	}

	// sleep delay
	pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10;      // 10ms, temporary set
	// system clock setting
	pGlobal_config[CLOCK_SETTING] = g_system_clk;//CLOCK_32MHZ;
	//------------------------------------------------------------------------
	// wakeup time cose
	// t1. HW_Wakeup->MCU relase 62.5us
	// t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
	// t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
	// t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
	// WAKEUP_ADVANCE should be larger than t1+t2+t3+t4
	//------------------------------------------------------------------------
	// wakeup advance time, in us
	pGlobal_config[WAKEUP_ADVANCE] = 1850;//650;//600;//310;

	if(g_system_clk==SYS_CLK_XTAL_16M) {
		pGlobal_config[WAKEUP_DELAY] = 16;
	}
	else if(g_system_clk==SYS_CLK_DBL_32M) {
		pGlobal_config[WAKEUP_DELAY] = 16;
	}
	else if(g_system_clk==SYS_CLK_DLL_48M) {
		pGlobal_config[WAKEUP_DELAY] = 16;
	}
	else if(g_system_clk==SYS_CLK_DLL_64M) {
		pGlobal_config[WAKEUP_DELAY] = 16;
	}

	// sleep time, in us
	pGlobal_config[MAX_SLEEP_TIME] = 30000000;
	pGlobal_config[MIN_SLEEP_TIME] = 1600;
	pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 55;// 30.5 per tick
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	// LL engine settle time
	pGlobal_config[LL_HW_BB_DELAY] = 54;//54-8;
	pGlobal_config[LL_HW_AFE_DELAY] = 8;
	pGlobal_config[LL_HW_PLL_DELAY] = 40;//45;//52;
	// Tx2Rx and Rx2Tx interval
	//Tx2Rx could be advanced a little
	//Rx2Tx should be ensure T_IFS within150us+-2us
	pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 62-RF_PHY_EXT_PREAMBLE_US;
	pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 50;//65
	//------------------------------------------------2MPHY
	// LL engine settle time
	pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
	pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
	pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 40;//45;//52;
	// Tx2Rx and Rx2Tx interval
	//Tx2Rx could be advanced a little
	//Rx2Tx should be ensure T_IFS within150us+-2us
	pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 73-RF_PHY_EXT_PREAMBLE_US;//20200822 ZQ
	pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 57;//72
	//------------------------------------------------CODEPHY 500K
	// LL engine settle time CODEPHY 500K
	pGlobal_config[LL_HW_BB_DELAY_500KPHY] = 50;//54-8;
	pGlobal_config[LL_HW_AFE_DELAY_500KPHY] = 8;
	pGlobal_config[LL_HW_PLL_DELAY_500KPHY] = 40;//45;//52;
	// Tx2Rx and Rx2Tx interval
	//Tx2Rx could be advanced a little
	//Rx2Tx should be ensure T_IFS within150us+-2us
	pGlobal_config[LL_HW_Rx_TO_TX_INTV_500KPHY] =  2;
	pGlobal_config[LL_HW_Tx_TO_RX_INTV_500KPHY] = 66;//72
	//------------------------------------------------CODEPHY 125K
	// LL engine settle time CODEPHY 125K
	pGlobal_config[LL_HW_BB_DELAY_125KPHY] = 30;//54-8;
	pGlobal_config[LL_HW_AFE_DELAY_125KPHY] = 8;
	pGlobal_config[LL_HW_PLL_DELAY_125KPHY] = 40;//45;//52;
	// Tx2Rx and Rx2Tx interval
	//Tx2Rx could be advanced a little
	//Rx2Tx should be ensure T_IFS within150us+-2us
	pGlobal_config[LL_HW_Rx_TO_TX_INTV_125KPHY] = 5;
	pGlobal_config[LL_HW_Tx_TO_RX_INTV_125KPHY] = 66;//72
	// LL engine settle time, for advertisement
	pGlobal_config[LL_HW_BB_DELAY_ADV] = 90;
	pGlobal_config[LL_HW_AFE_DELAY_ADV] = 8;
	pGlobal_config[LL_HW_PLL_DELAY_ADV] = 60;
	// adv channel interval
	pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
	pGlobal_config[NON_ADV_CHANNEL_INTERVAL] = 666;//6250;

	//20201207 Jie modify
	if(g_system_clk==SYS_CLK_XTAL_16M) {
		// scan req -> scan rsp timing
		pGlobal_config[SCAN_RSP_DELAY] = 13+RF_PHY_EXT_PREAMBLE_US;//23;
	}
	else if(g_system_clk==SYS_CLK_DBL_32M) {
		pGlobal_config[SCAN_RSP_DELAY] = 8+RF_PHY_EXT_PREAMBLE_US;//23;
	}
	else if(g_system_clk==SYS_CLK_DLL_48M) {
		// scan req -> scan rsp timing
		pGlobal_config[SCAN_RSP_DELAY] = 6+RF_PHY_EXT_PREAMBLE_US;//20201207 set           //4;        // 12    //  2019/3/19 A2: 12 --> 9
	}
	else if(g_system_clk == SYS_CLK_DLL_64M) {
		pGlobal_config[SCAN_RSP_DELAY] = 4+RF_PHY_EXT_PREAMBLE_US;//2020.12.07 set         //3;
	}

	// conn_req -> slave connection event calibration time, will advance the receive window
	pGlobal_config[CONN_REQ_TO_SLAVE_DELAY] = 300;//192;//500;//192;
	// calibration time for 2 connection event, will advance the next conn event receive window
	// SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
	pGlobal_config[SLAVE_CONN_DELAY] = 300;//0;//1500;//0;//3000;//0;          ---> update 11-20
	pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 500;//160 NG//500 OK
	// RTLP timeout
	pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
	pGlobal_config[LL_HW_RTLP_TO_GAP]       = 1000;
	pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 2000 + pGlobal_config[SLAVE_CONN_DELAY] * 2;//500;
	// direct adv interval configuration
	pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
	pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;
	// A1 ROM metal change for HDC direct adv,
	pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.
	// A1 ROM metal change
	pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 6;//8;
	pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 6;//8;
	pGlobal_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8;     //0:        disable adaptive
	//other:    adaptive max limitation
//    pGlobal_config[LL_TX_PWR_TO_REG_BIAS]   = 0x15;   // assume when g_rfPhyTxPower = 0x1f, tx power = 10dBm
	//smart window configuration
	pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]      = 2;
	pGlobal_config[LL_SMART_WINDOW_TARGET]          = 600;
	pGlobal_config[LL_SMART_WINDOW_INCREMENT]       = 9;
	pGlobal_config[LL_SMART_WINDOW_LIMIT]           = 20000;
	pGlobal_config[LL_SMART_WINDOW_ACTIVE_THD]      = 8;
	pGlobal_config[LL_SMART_WINDOW_ACTIVE_RANGE]    = 0;//300
	pGlobal_config[LL_SMART_WINDOW_FIRST_WINDOW]    = 5000;

	//====== A2 metal change add, for scanner & initiator
	if(g_system_clk==SYS_CLK_XTAL_16M) {
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 18+RF_PHY_EXT_PREAMBLE_US;//20;      //  2019/3/19 A2: 20 --> 18
		pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 25+RF_PHY_EXT_PREAMBLE_US;//27;      //  2019/3/19 A2: 27 --> 25
	}
	else if(g_system_clk==SYS_CLK_DBL_32M) {
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 12+RF_PHY_EXT_PREAMBLE_US;                //  2019/3/26 add
		pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 16+RF_PHY_EXT_PREAMBLE_US;
	}
	else if(g_system_clk==SYS_CLK_DLL_48M) {
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 8+RF_PHY_EXT_PREAMBLE_US;//12;       //  2019/3/19 A2: 12 --> 10
		pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 11+RF_PHY_EXT_PREAMBLE_US;
	}
	else if(g_system_clk==SYS_CLK_DLL_64M) {
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 6+RF_PHY_EXT_PREAMBLE_US;                //  2019/3/26 add
		pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 8+RF_PHY_EXT_PREAMBLE_US;
	}

	// TRLP timeout
	pGlobal_config[LL_HW_TRLP_LOOP_TIMEOUT] = 50000;    // enough for 8Tx + 8Rx : (41 * 8 + 150) * 16 - 150 = 7498us
	pGlobal_config[LL_HW_TRLP_TO_GAP]       = 1000;
	pGlobal_config[LL_MOVE_TO_MASTER_DELAY] = 100;
	pGlobal_config[LL_CONN_REQ_WIN_SIZE] = 5;
	pGlobal_config[LL_CONN_REQ_WIN_OFFSET] = 2;
	pGlobal_config[LL_MASTER_PROCESS_TARGET] = 200;   // reserve time for preparing master conn event, delay should be insert if needn't so long time
	pGlobal_config[LL_MASTER_TIRQ_DELAY] = 0;         // timer IRQ -> timer ISR delay
	pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;  // 0.125us
	pGlobal_config[MAC_ADDRESS_LOC] = 0x11004000;
	// for simultaneous conn & adv/scan
	pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400*3;
	pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;
	pGlobal_config[LL_SEC_SCAN_MARGIN] = 2500;//1400;  to avoid mesh proxy llTrigErr 0x15
	pGlobal_config[LL_MIN_SCAN_TIME] = 2000;
	//  BBB new
	pGlobal_config[TIMER_ISR_ENTRY_TIME] = 30;//15;
	pGlobal_config[LL_MULTICONN_MASTER_PREEMP] = 0;
	pGlobal_config[LL_MULTICONN_SLAVE_PREEMP] = 0;
	pGlobal_config[LL_EXT_ADV_TASK_DURATION] = 20000;
	pGlobal_config[LL_PRD_ADV_TASK_DURATION] = 20000;
	pGlobal_config[LL_CONN_TASK_DURATION] = 5000;
	pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] = 5000;
	pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT] = 5000;
	pGlobal_config[LL_EXT_ADV_PRI_2_SEC_CHN_INT] = 1500;
	pGlobal_config[LL_EXT_ADV_RSC_PERIOD] = 1000000;
	pGlobal_config[LL_EXT_ADV_RSC_SLOT_DURATION] = 10000;
	pGlobal_config[LL_PRD_ADV_RSC_PERIOD] = 1000000;
	pGlobal_config[LL_PRD_ADV_RSC_SLOT_DURATION] = 10000;
	pGlobal_config[LL_EXT_ADV_PROCESS_TARGET] = 500;
	pGlobal_config[LL_PRD_ADV_PROCESS_TARGET] = 500;
}


int main(void) {
	populate_jump_table();
	global_config();
	
	g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_64M;
	g_clk32K_config = CLK_32K_RCOSC;//CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC

	osal_mem_set_heap((osalMemHdr_t*)g_largeHeap, LARGE_HEAP_SIZE);
	hal_init();
	hal_gpio_init();
	RF_init();

	// CMSIS enable interrupts (device specific interrupt flags also need to be set!)
	drv_enable_irq();
	NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
	NVIC_SetPriority(RTC_IRQn, IRQ_PRIO_HIGH);
	NVIC_EnableIRQ(RTC_IRQn);
	
	config_RTC(3); // fire RTC irq as fast as you can
	//ENABLE_SOFTWARE_CONTROL(0x00); // software control disable
	//SET_SLEEP_FLAG;
	//AP_AON->SLEEP_R[0] = 4; //RSTC_WAKE_RTC;
	
	/* Initialize the operating system */
	osal_init_system();
	/* Start OSAL */
	osal_start_system(); // No Return from here

	return 0; // should never reach
}
