#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/desig.h> // desig_get_unique_id
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>

#include <canard.h>
#include <drivers/stm32/canard_stm32.h>


static volatile uint32_t systick_millis;

void usleep(void);

static uint32_t millis(void)
{
    return systick_millis;
}

// called with usleep(1000) from canard.c
extern void usleep(void)
{
    // TODO exact 1 ms delay using micros()
    uint32_t ms = millis();
    while(millis() - ms < 2U);
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_CAN1);

    // CAN1_RX is PA11, CAN1_TX is PA12
    /* Configure CAN pin: RX (input pull-up). */
    gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
    gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);

    /* Configure CAN pin: TX. */
    gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);
}

static void gpio_setup(void)
{
    /* Set GPIO to 'output push-pull'. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
}

void sys_tick_handler(void)
{
    systick_millis++;
}

static CanardInstance g_canard;             // The library instance
static uint8_t g_canard_memory_pool[1024];  // Arena for memory allocation, used by the library

#define UNIQUE_ID_LENGTH_BYTES 16
#define APP_VERSION_MAJOR                       99

#define APP_VERSION_MINOR                       99
#define APP_NODE_NAME                           "wave.canard"
#define APP_NODE_ID                             100

#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1
#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)

/* see zubax Basic Tutorial */
static bool shouldAcceptTransfer(const CanardInstance* ins,
        uint64_t* out_data_type_signature,
        uint16_t data_type_id,
        CanardTransferType transfer_type,
        uint8_t source_node_id)
{
    if ((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
        return true;
    }

    return false;
}

static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    const uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    const uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    const uint32_t uptime_sec = millis() / 1000;
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

static void readUniqueID(uint8_t buffer[UNIQUE_ID_LENGTH_BYTES])
{
    memset(buffer, 0, UNIQUE_ID_LENGTH_BYTES);
    uint32_t uid_buf[3];
    desig_get_unique_id(uid_buf);
    memcpy(&buffer[2], uid_buf, 12);
}

static uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);

    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;

#ifdef GIT_HASH
    buffer[9] = 1;  // Optional field flags, VCS commit is set
    const uint32_t git_hash = GIT_HASH;
    canardEncodeScalar(buffer, 80, 32, &git_hash);
#endif // GIT_HASH

    uint8_t my_unique_id[UNIQUE_ID_LENGTH_BYTES];
    readUniqueID(my_unique_id);
    memcpy(&buffer[24], my_unique_id, UNIQUE_ID_LENGTH_BYTES);

    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);
    return 41 + name_len ;
}

static void getNodeInfoHandle(CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    const uint16_t len = makeNodeInfoMessage(buffer);
    int result = canardRequestOrRespond(&g_canard,
                                        transfer->source_node_id,
                                        UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                        UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                        &transfer->transfer_id,
                                        transfer->priority,
                                        CanardResponse,
                                        &buffer[0],
                                        (uint16_t)len);
    if (result < 0)
    {
        // TODO: handle the error
    }
}


static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
            (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        getNodeInfoHandle(transfer);
    }
}

static void uavcan_init(void)
{
    CanardSTM32CANTimings timings;
    int result = canardSTM32ComputeCANTimings(rcc_apb1_frequency, 1000000, &timings);
    if (result)
    {
    }
    result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
    if (result)
    {
    }

    canardInit(&g_canard,                         // Uninitialized library instance
               g_canard_memory_pool,              // Raw memory chunk used for dynamic allocation
               sizeof(g_canard_memory_pool),      // Size of the above, in bytes
               onTransferReceived,                // Callback, see CanardOnTransferReception
               shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
               NULL);

    canardSetLocalNodeID(&g_canard, APP_NODE_ID);
}

static void sendCanard(void)
{
    const CanardCANFrame* txf = canardPeekTxQueue(&g_canard);
    while(txf)
    {
        const int tx_res = canardSTM32Transmit(txf);
        if (tx_res < 0)         // Failure - drop the frame and report
        {
            // TODO: handle the error properly
        }
        if (tx_res > 0)
        {
            canardPopTxQueue(&g_canard);
        }
        txf = canardPeekTxQueue(&g_canard);
    }
}

static void receiveCanard(void)
{
    CanardCANFrame rx_frame;
    int res = canardSTM32Receive(&rx_frame);
    if(res)
    {
        canardHandleRxFrame(&g_canard, &rx_frame, millis() * 1000);
    }
}

#define CANARD_SPIN_PERIOD    1000

static void spinCanard(void)
{
    static uint32_t spin_time = 0;
    if (millis() < spin_time + CANARD_SPIN_PERIOD)
    {
        return;
    }  // rate limiting
    spin_time = millis();
    gpio_toggle(GPIOA, GPIO4); /* Toggle LED. */

    uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
    static uint8_t transfer_id = 0;                          // This variable MUST BE STATIC; refer to the libcanard documentation for the background

    makeNodeStatusMessage(buffer);

    canardBroadcast(&g_canard,
            UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
            UAVCAN_NODE_STATUS_DATA_TYPE_ID,
            &transfer_id,
            CANARD_TRANSFER_PRIORITY_LOW,
            buffer,
            UAVCAN_NODE_STATUS_MESSAGE_SIZE);
}

int main(void)
{
    clock_setup();
    gpio_setup();

    /* 72MHz / 8 => 9000000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    uint32_t counts_per_ms = rcc_ahb_frequency / 1000U;

    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(counts_per_ms - 1);
    systick_interrupt_enable();
    systick_counter_enable();

    uavcan_init();

    while (1) {
        sendCanard();
        receiveCanard();
        spinCanard();
    }

    return 0;
}
