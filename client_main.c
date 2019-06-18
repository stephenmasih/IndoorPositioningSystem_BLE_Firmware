#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "boards.h"
#include "nrf_delay.h"
#include "simple_hal.h"
#include "log.h"
#include "access_config.h"
#include "simple_on_off_client.h"
#include "rtt_input.h"
#include "device_state_manager.h"
#include "light_switch_example_common.h"
#include "mesh_app_utils.h"
#include "mesh_stack.h"
#include "mesh_softdevice_init.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"
#include "app_timer.h"
//#include "nrf_gpio.h"
#define RTT_INPUT_POLL_PERIOD_MS (100)
#define GROUP_MSG_REPEAT_COUNT   (2)
//#define APP_TIMER_ENABLED 1
#define LED_BLINK_INTERVAL_MS       (200)
#define LED_BLINK_SHORT_INTERVAL_MS (50)
#define LED_BLINK_CNT_START         (2)
#define LED_BLINK_CNT_RESET         (3)
#define LED_BLINK_CNT_PROV          (4)
#define LED_BLINK_CNT_NO_REPLY      (6)
bool st=false;
bool sleep_t=false;
uint8_t RELAY_NODES=2;
#define DURATION_BETWEEN_ADV_MSECS 15000
#define SLEEP_TIME 10000
APP_TIMER_DEF(m_auto_mode_timer);
APP_TIMER_DEF(sleep_timer);
void start_auto_mode();
void sleep_mode();
static simple_on_off_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static const uint8_t          m_client_node_uuid[NRF_MESH_UUID_SIZE] = CLIENT_NODE_UUID;
static bool                   m_device_provisioned;
extern bool opcode_status;
 //access_message_rx_t message;
static void advertising_start_timer_handler(void *p_context);
static void sleep_timer_handler(void *p_context);
static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src);
void send_token(uint8_t token);

static bool client_publication_configured(void)
{
    dsm_handle_t pub_addr_handle;
    for (uint8_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; i++)
    {
        
        //So this basically is subscribing to the server address so that whenever the server sends a msg it listens to it
        if (access_model_publish_address_get(m_clients[i].model_handle, &pub_addr_handle) == NRF_SUCCESS)
        {
            if (pub_addr_handle == DSM_HANDLE_INVALID)
            {
            //__LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "servers information failed to retrive from DSM\n");
                return false;
            }
        }
        else
        {
           // __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "servers information failed to retrive from DSM\n");
            return false;
        }
    }
//__LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "servers information retrieved from DSM\n");
    return true;
}
static void advertising_start_timer_handler(void *p_context) {

            if(client_publication_configured()){
           
           simple_on_off_client_set_unreliable(&m_clients[0],1,2);
           __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Wake up server 0\n");
           simple_on_off_client_set_unreliable(&m_clients[1],1,2);
           __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Wake up server 1\n");
           
}
}

static void sleep_timer_handler(void *p_context){
__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "sleep timer started \n");
start_auto_mode();
}
static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

}

static uint32_t server_index_get(const simple_on_off_client_t * p_client)
{
    uint32_t index = p_client - &m_clients[0];
    NRF_MESH_ASSERT(index < SERVER_NODE_COUNT);
    return index;
}

static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

void send_token(uint8_t token){
//static simple_on_off_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];

//address_set(address);

for(uint8_t server_index=0;server_index<=1;server_index++){
simple_on_off_client_set_unreliable(&m_clients[server_index],token,2);
__LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Wake up server %d\n",server_index);
}
}

static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{   uint8_t count=0;
    uint32_t server_index = server_index_get(p_self);
 access_message_rx_t message;
  // __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "server status received \n");
    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
             __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "server ON received \n");
               
            // simple_on_off_client_set(&m_clients[0],1);
             
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
       
            if(client_publication_configured()){
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "server OFF received \n");
           //setting the second server also to off state
              //simple_on_off_client_set(&m_clients[1],1);
          
            }
        break;


        case SLEEP_MODE:
         memset(&message.p_data,0,sizeof(message.p_data));
       __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Making both the servers sleep \n");
 
        if(client_publication_configured())
        {
            //__LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "SLEEP server 0 \n");
               simple_on_off_client_set_unreliable(&m_clients[0],0,2);//would turn the server value to false and put it to sleep
               nrf_delay_ms(100);
            //__LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "SLEEP server 1 \n");
               simple_on_off_client_set_unreliable(&m_clients[1],0,2);//would turn the server value to false and put it to sleep
               sleep_mode();
        }   
          break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "error on no reply \n");
            break;

        case SIMPLE_ON_OFF_STATUS_CANCELLED:
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unknown status \n");
            break;
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
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

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].status_cb = client_status_cb;
        m_clients[i].timeout_cb = client_publish_timeout_cb;
        ERROR_CHECK(simple_on_off_client_init(&m_clients[i], i + 1));
        ERROR_CHECK(access_model_subscription_list_alloc(m_clients[i].model_handle));
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = m_client_node_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}
void start_auto_mode() {

  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Advertising timer started \n");
  st=false;
  ret_code_t err_code;
  err_code = app_timer_start(m_auto_mode_timer, APP_TIMER_TICKS(DURATION_BETWEEN_ADV_MSECS), NULL);
  APP_ERROR_CHECK(err_code);

}
void sleep_mode(){

  ret_code_t err_code;

  err_code = app_timer_start(m_auto_mode_timer, APP_TIMER_TICKS(SLEEP_TIME), NULL);
  APP_ERROR_CHECK(err_code);
  (void)sd_app_evt_wait();
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
static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");
  
    //nrf clock initialize
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
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID", p_uuid, NRF_MESH_UUID_SIZE);

}

int main(void)
{
    timers_init();
    initialize();
    execution_start(start);
    start_auto_mode();
    
    for (;;)
    {
 send_token(1);//force ping servers to wake up
 nrf_delay_ms(5000);
//   if(st){
//  
//    simple_on_off_client_set_unreliable(&m_clients[1],1,2);
//    
//    
//    st=false;
//   }
    
    }
}
