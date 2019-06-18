#include <stdint.h>
#include <string.h>
#include "app_timer.h"
#include "nrf_delay.h"
#include "boards.h"
#include "simple_hal.h"
#include "log.h"
#include "access_config.h"
#include "simple_on_off_server.h"
#include "light_switch_example_common.h"
#include "mesh_app_utils.h"
#include "net_state.h"
#include "rtt_input.h"
#include "mesh_stack.h"
#include "mesh_softdevice_init.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"
#include "simple_on_off_common.h"
#include "nrf_mesh_events.h"
#include "nrf_nvic.h"


#define RTT_INPUT_POLL_PERIOD_MS (100)
#define LED_PIN_NUMBER           (BSP_LED_0)
#define LED_PIN_MASK             (1u << LED_PIN_NUMBER)
#define LED_BLINK_INTERVAL_MS    (200)
#define LED_BLINK_CNT_START      (2)
#define LED_BLINK_CNT_RESET      (3)
#define LED_BLINK_CNT_PROV       (4)
#define DURATION_BETWEEN_ADV_MSECS 5000
#define DURATION_BETWEEN_MSECS 10000
APP_TIMER_DEF(m_auto_mode_timer);
APP_TIMER_DEF(sleep_timer);
static simple_on_off_server_t m_server;
static nrf_mesh_evt_handler_t m_evt_handler;
bool status_msg=false;
bool st=false;
bool send_complete=false;
bool wake_up=false;
static bool                   m_device_provisioned;
bool send_msg= false;
    char data[100];
    char data1[10];
     char data2[50];
  //  char msg[128];
uint32_t message(simple_on_off_server_t * p_server,int8_t *value);
//uint8_t data[50];

static void advertising_start_timer_handler(void *p_context) {
//memcpy(data2,data,sizeof(data2));
 nrf_mesh_rx_cb_clear();//to stop scanning in server after 5 seconds of advertisemsnts from main
nrf_delay_ms(200);
    //simple_on_off_server_status_publish(&m_server,1);
//simple_on_off_server_status_publish(&m_server,2);
//st=true;
 simple_on_off_server_status_publish(&m_server,2,1);//node 1 second parameter to 1 and then 2 accordingly
//memset(data,0,sizeof(data));
}

static void sleep_timer_handler(void *p_context){
nrf_mesh_enable();
nrf_delay_ms(100);
  //simple_on_off_server_status_publish(&m_server,3);
        
 while(!nrf_mesh_process());
 
}
static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static bool on_off_server_get_cb(const simple_on_off_server_t * p_server)
{
    return hal_led_pin_get(LED_PIN_NUMBER);
}
static void start_auto_mode() {

  ret_code_t err_code;

  err_code = app_timer_start(m_auto_mode_timer, APP_TIMER_TICKS(DURATION_BETWEEN_ADV_MSECS), NULL);
  APP_ERROR_CHECK(err_code);
}
static void start_sleep_mode() {

  ret_code_t err_code;
nrf_mesh_disable();
  err_code = app_timer_start(sleep_timer, APP_TIMER_TICKS(DURATION_BETWEEN_MSECS), NULL);
  APP_ERROR_CHECK(err_code);
  (void)sd_app_evt_wait();
}

static bool on_off_server_set_cb(const simple_on_off_server_t * p_server, uint8_t value)
{

__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Value%d\n", value);
        if(value){
        send_msg=true;
        }else
        start_sleep_mode();
 //  nrf_delay_ms(100);
//   start_auto_mode();
    return value;
}
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
  err_code=app_timer_create(&m_auto_mode_timer,APP_TIMER_MODE_SINGLE_SHOT,advertising_start_timer_handler);
 APP_ERROR_CHECK(err_code);
   err_code=app_timer_create(&sleep_timer,APP_TIMER_MODE_SINGLE_SHOT,sleep_timer_handler);
 APP_ERROR_CHECK(err_code);
}
static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void rx_cb(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    LEDS_OFF(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
 //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "message received\n");
    char msg[128];
 static uint8_t cb_id=0;
   if(p_rx_data->adv_type ==6){
    (void) sprintf(msg, "RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]\n",
                   p_rx_data->p_metadata->params.scanner.timestamp,
                   p_rx_data->p_metadata->params.scanner.rssi,
                   p_rx_data->adv_type,
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[0],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[1],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[2],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[3],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[4],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[5]);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);
    //LEDS_ON(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
//p_rx_data->p_metadata->params.scanner.adv_addr;

  memcpy(data,&p_rx_data->p_metadata->params.scanner.adv_addr,sizeof(p_rx_data->p_metadata->params.scanner.adv_addr));
  memcpy(data1,(const int8_t*)&p_rx_data->p_metadata->params.scanner.rssi,sizeof( p_rx_data->p_metadata->params.scanner.rssi));
  memcpy(data2,&p_rx_data->p_payload[7],sizeof(p_rx_data->p_payload[7]));
  //cb_id++;
//strcat(data,(char *)&p_rx_data->p_metadata->params.scanner.adv_addr);
strcat(data,data1);
//strcat(data,(const char *)&p_rx_data->p_metadata->params.scanner.adv_addr);
//strcat(data,(const int8_t *)&p_rx_data->p_metadata->params.scanner.rssi);

//strcat(data,(const uint8_t*) &p_rx_data->p_payload[7]);

//data34=(uint8_t*)p_rx_data->p_payload;
//data34=(uint8_t *)data34 & 0xff;
strcat(data,data2);
  // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " code %d\n",p_rx_data->length);

  message(&m_server,data);   

}
memset(data,0,sizeof(data));
//memset(data1,0,sizeof(data1));
//memset(data2,0,sizeof(data2));
}

void send_mesh_message(simple_on_off_server_t * p_server, bool value){
uint32_t status=0;
uint8_t buffer[30]="hello";
uint8_t length;
uint16_t address;
access_message_tx_t msg;
//length= SEGGER_RTT_Read(0,buffer, sizeof(buffer));
           
              msg.opcode.opcode =SIMPLE_ON_OFF_OPCODE_SET;
              msg.opcode.company_id = 0x0059; // Nordic's company ID
      
              msg.p_buffer = (const uint8_t *) &buffer[0];
              msg.length =length;
             
             address = 0x0100;
             // access_model_publish_address_set(address);
           //   SEGGER_RTT_printf(0,"Sending to group address 0x%04x\n", address);
              status= access_model_publish(p_server->model_handle, &msg);

              if (status == NRF_ERROR_INVALID_STATE ||
              status == NRF_ERROR_BUSY||status == NRF_ERROR_NO_MEM)
               {
                 __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot send. Device is busy.\n");
                //  hal_led_blink_ms(LEDS_MASK, 50, 4);
               }
               else
               {
                     ERROR_CHECK(status);
               }
            
}

void dc_dc_enable(void){
    ret_code_t err_code;
err_code=sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
   APP_ERROR_CHECK(err_code);

}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    m_server.get_cb = on_off_server_get_cb;
    m_server.set_cb = on_off_server_set_cb;
    ERROR_CHECK(simple_on_off_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
}
static void mesh_evt_cb(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_TX_COMPLETE:
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
        if(send_complete)
          
            break;
        default:
            /* Ignore */
            break;
    }

}
static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[SERVER_NODE_UUID_PREFIX_SIZE] = SERVER_NODE_UUID_PREFIX;
   //m_evt_handler.evt_cb = mesh_evt_cb;
    //nrf_mesh_evt_handler_add(&m_evt_handler);
    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, SERVER_NODE_UUID_PREFIX_SIZE));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

    nrf_clock_lf_cfg_t lfc_cfg = DEV_BOARD_LF_CLK_CFG;
    ERROR_CHECK(mesh_softdevice_init(lfc_cfg));
    mesh_init();
}

static void start(void)
{
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    } 

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);
//send_mesh_message(&m_server,true);
}

//trasmitting the message received by the server 
uint32_t message(simple_on_off_server_t * p_server,int8_t *value){
access_message_tx_t msg;
   // uint8_t val[20]="590064";
   //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " Value %02x\n", &data);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "transmitting message to client\n");
    msg.opcode.opcode = SIMPLE_ON_OFF_OPCODE_SET;//acknowledged data
    msg.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    msg.p_buffer =(const uint8_t*) &data;
    msg.length = sizeof(data);
    msg.force_segmented = true;
    msg.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    return access_model_publish(p_server->model_handle, &msg);
}

static void token_get_cb(){

nrf_mesh_rx_cb_set(rx_cb);
}

int main(void)
{
    timers_init();
    initialize();
    execution_start(start);
    dc_dc_enable();
    memset(data,0,sizeof(data));

//simple_on_off_server_status_publish(&m_server,true);
    for (;;)
    {  
          if(send_msg)
          {
            nrf_mesh_rx_cb_set(rx_cb);//to start receiving from the beacon
            start_auto_mode();
            nrf_delay_ms(100);
           
            send_msg=false;
          }
         if(send_complete)
         {    
         __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");
           //simple_on_off_server_status_publish(&m_server,2);
           
          send_complete=false;
         }
    }
}
