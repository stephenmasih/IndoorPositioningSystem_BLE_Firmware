/*
 * Sigfox Hacking House Taipei
 * Connected Workforce
 * Ver: 0
 */

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <misc/printk.h>
#include <uart.h>

#include <device.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <stdio.h>
#define limit 50

#define AT_BUF_SIZE 1024
struct device *uart;
static void uart_rx_handler(u8_t character) {

		static size_t at_cmd_len;
		static char at_buf[AT_BUF_SIZE]; /* AT command and modem response buffer */

	 /* Handle control characters */
	 switch (character) {
		 case 0x08: /* Backspace. */ /* Fall through. */
		 case 0x7F: /* DEL character */
		 if (at_cmd_len > 0) {
			 at_cmd_len--;
		  }
			return;
		 }
		 switch (character) {
			  case '\0':
				 goto send;
				 return;
				case '\r':
				 goto send;
				 return;
				case '\n':
				 goto send;
				 return;
			 }
		 /* Detect AT command buffer overflow, leaving space for null */
		 if (at_cmd_len + 1 > sizeof(at_buf) - 1) {
			 printk("Buffer overflow, dropping '%c'\n", character);
			 return;
		 }

	  /* Write character to AT buffer */
		at_buf[at_cmd_len] = character;
		at_cmd_len++;

		return;

		send:
		at_buf[at_cmd_len] = '\0';
	 	/* Terminate the command string */

		/* Reset UART handler state */
		at_cmd_len = 0;

		/* Send the command, if there is one to send */
		if (at_buf[0]) {
			uart_irq_rx_disable(uart); /* Stop UART to protect at_buf */
		  printk("%s\n",at_buf);
			uart_irq_rx_enable(uart);
		}
	}
static void isr(struct device *dev) {

		u8_t character;
		uart_irq_update(dev);
		if (!uart_irq_rx_ready(dev)) {
			return;
		}
		/* * Check that we are not sending data (buffer must be preserved then), * and that a new character is available before handling each character */
		while ((uart_fifo_read(dev, &character, 1))) {
			uart_rx_handler(character);
		}
	}
void myuart_send(char * str) {
		for (size_t i = 0; str[i]; i++) {
			uart_poll_out(uart, str[i]);
		}
	}
void DataSend(char Maj0, char Maj1, char Min0, char Min1){
    char strH[20];

    memset(strH,0,sizeof(strH));

    sprintf(strH, "AT$SF=%02x%02x%02x%02x\n", Maj0, Maj1, Min0, Min1);

    printk("%s\n",strH);
    myuart_send(strH);
  }

// GPIO for the buttons
#define PIN_A 11
#define PORT DT_GPIO_P0_DEV_NAME
#define EDGE (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE)
#define PULL_UP SW0_GPIO_FLAGS

static struct gpio_callback gpio_btnA_cb;
static struct k_work buttonA_work;


#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

#define SCAN_WINDOW 1000

int err;

char addr_str[BT_ADDR_LE_STR_LEN], uuid_str[25];
int RSSI;

struct beacon
{
    char Major[2];
    char Minor[2];
    float distance;
    struct
    {
        float _err_measure;
        float _err_estimate;
        float _current_estimate;
        float _last_estimate;
        float _kalman_gain;
    } Filter;
} beacon_list[3] =
{
    {{0xAA, 0x00}, {0x00, 0x00}, 99, {1, 1}},
    {{0xBB, 0x00}, {0x00, 0x00}, 98, {1, 1}},
    {{0xCC, 0x00}, {0x00, 0x00}, 97, {1, 1}}
};

int count=0;
struct Queue *q;

typedef struct node                // a node in a linked list
{
    int counter;
    char Major[2];
    char Minor[2];
    struct node *next;            // and a pointer to the next item in the linked list
};

typedef struct Queue
{
    struct node *front;
    struct node *rear;
};

int empty()
{
    if(q->front == NULL) return 0;
    else return 1;
}

void enqueue(char NewMaj0, char NewMaj1, char NewMin0, char NewMin1)
{
    struct node *temp;
    temp = (struct node *) k_malloc(sizeof(struct node));

    temp->Major[0] = NewMaj0;
    temp->Major[1] = NewMaj1;
    temp->Minor[0] = NewMin0;
    temp->Minor[1] = NewMin1;
    temp->counter++;

    temp->next = NULL;
    if(empty(q) != 0)                    // if the queue is not empty, add it at the rear
    {
        // adjusting the last node's next pointer to point
        q->rear->next = temp;            // at it
    }
    else
        q->front = temp;                 // otherwise it is the only item, adjust front to point at it
    q->rear = temp;                      // in either case point rear at the newly added item

}

void dequeue(){
    struct node *temp;
    if(empty(q) != 0)
    {
        temp = q->front;
        q->front = q->front->next;
        k_free(temp);
        if(q->front == NULL) q->rear = NULL;
    }
    else printf("ERROR, Queue empty, cannot dequeue\n");
}

void CompareIncrementOrEnqueue(char Maj0, char Maj1, char Min0, char Min1)
{
    int flag = 1;
    struct node *temp;
    temp = q->front;
      while(empty(q) != 0 && temp != NULL)
      {
          if(Maj0 == temp->Major[0] && Maj1 == temp->Major[1] && Min0 == temp->Minor[0] && Min1 == temp->Minor[1])
          {
                temp->counter++;
                printk("%02X%02X%02X%02X ",temp->Major[0],temp->Major[1],temp->Minor[0],temp->Minor[1]);
                printk("matched %d times\n", temp->counter);
                flag = 0;
                if(temp->counter >=100){
                  //send sigfox message
                  myuart_send("ATS410=1\n");
                  k_sleep(100);
                  myuart_send("AT$GI?\n");
                  k_sleep(100);
                  myuart_send("AT$RC\n");
                  DataSend(temp->Major[0],temp->Major[1],temp->Minor[0],temp->Minor[1]);
                  k_sleep(2000);
                  temp->counter=0;
                }

                break;
          }
          temp = temp->next;
      }

      if(count>limit)
      {
        dequeue();
        enqueue(Maj0, Maj1, Min0, Min1);
        printk("dequeued and added\n");
      }
      else if (flag == 1)
      {
          enqueue(Maj0, Maj1, Min0, Min1);
          printk("added\n");
          count++;
      }

}

int absolute(int v)
{
    unsigned int r;  // the result goes here
    int const mask = ((v >> sizeof(int)) * (CHAR_BIT - 1));

    r = (v + mask) ^ mask;
    return r;
}

void ImproveKalman0(float mea)
{
    beacon_list[0].Filter._kalman_gain = beacon_list[0].Filter._err_estimate / (beacon_list[0].Filter._err_estimate + beacon_list[0].Filter._err_measure);
    beacon_list[0].Filter._current_estimate = beacon_list[0].Filter._last_estimate + beacon_list[0].Filter._kalman_gain * (mea - beacon_list[0].Filter._last_estimate);
    beacon_list[0].Filter._err_estimate =  (1.0 - beacon_list[0].Filter._kalman_gain) * beacon_list[0].Filter._err_estimate + absolute(beacon_list[0].Filter._last_estimate - beacon_list[0].Filter._current_estimate) * 0.01;
    beacon_list[0].distance = beacon_list[0].Filter._last_estimate = beacon_list[0].Filter._current_estimate;
}

void ImproveKalman1(float mea)
{
    beacon_list[1].Filter._kalman_gain = beacon_list[1].Filter._err_estimate / (beacon_list[1].Filter._err_estimate + beacon_list[1].Filter._err_measure);
    beacon_list[1].Filter._current_estimate = beacon_list[1].Filter._last_estimate + beacon_list[1].Filter._kalman_gain * (mea - beacon_list[1].Filter._last_estimate);
    beacon_list[1].Filter._err_estimate =  (1.0 - beacon_list[1].Filter._kalman_gain) * beacon_list[1].Filter._err_estimate + absolute(beacon_list[1].Filter._last_estimate - beacon_list[1].Filter._current_estimate) * 0.01;
    beacon_list[1].distance = beacon_list[1].Filter._last_estimate = beacon_list[1].Filter._current_estimate;
}

void ImproveKalman2(float mea)
{
    beacon_list[2].Filter._kalman_gain = beacon_list[2].Filter._err_estimate / (beacon_list[2].Filter._err_estimate + beacon_list[2].Filter._err_measure);
    beacon_list[2].Filter._current_estimate = beacon_list[2].Filter._last_estimate + beacon_list[2].Filter._kalman_gain * (mea - beacon_list[2].Filter._last_estimate);
    beacon_list[2].Filter._err_estimate =  (1.0 - beacon_list[2].Filter._kalman_gain) * beacon_list[2].Filter._err_estimate + absolute(beacon_list[2].Filter._last_estimate - beacon_list[2].Filter._current_estimate) * 0.01;
    beacon_list[2].distance = beacon_list[2].Filter._last_estimate = beacon_list[2].Filter._current_estimate;
}

void UpdateBeaconList(char *NewMaj0, char *NewMaj1, char *NewMin0, char *NewMin1, float NewDistance, int *index)
{
    beacon_list[*index].Major[0] = *NewMaj0;
    beacon_list[*index].Major[1] = *NewMaj1;
    beacon_list[*index].Minor[0] = *NewMin0;
    beacon_list[*index].Minor[1] = *NewMin1;
    beacon_list[*index].distance = NewDistance;
}

float pow (float n, float exp)
{
    float y = 1.0;
    int count = 1;
    while(count <= exp)
    {
        y = y * n;
        count++;
    }
    return y;
}

float calculateDistance(int rssi)
{

    int txPower = -59; //hard coded power value. Usually ranges between -59 to -65

    if (rssi == 0)
    {
        return -1.0;
    }

    float ratio = rssi * 1.0 / txPower;
    if (ratio < 1.0)
    {
        return pow(ratio, 10);
    }
    else
    {
        float distance =  (0.89976) * pow(ratio, 7.7095) + 0.111;
        return distance;
    }
}

void UpdateCounter(float NewDistance)
{
  // Finding the smallest distance
  float smallest_distance;
  u8_t IndexOfSmallestDistance;
  if (beacon_list[0].distance < beacon_list[1].distance)
  {
      if (beacon_list[0].distance < beacon_list[2].distance)
      {
          smallest_distance = beacon_list[0].distance;
          IndexOfSmallestDistance = 0;
      }
      else
      {
          smallest_distance = beacon_list[2].distance;
          IndexOfSmallestDistance = 2;
      }
  }
  else
  {
      if (beacon_list[1].distance < beacon_list[2].distance)
      {
          smallest_distance = beacon_list[1].distance;
          IndexOfSmallestDistance = 1;
      }
      else
      {
          smallest_distance = beacon_list[2].distance;
          IndexOfSmallestDistance = 2;
      }
  }
    // LL traversal and counter increments
    CompareIncrementOrEnqueue(
        beacon_list[IndexOfSmallestDistance].Major[0],
        beacon_list[IndexOfSmallestDistance].Major[1],
        beacon_list[IndexOfSmallestDistance].Minor[0],
        beacon_list[IndexOfSmallestDistance].Minor[1]
    );
}

void ProcessNewBeacon(char *NewMaj0, char *NewMaj1, char *NewMin0, char *NewMin1, float NewDistance)
{

    // Calculating Largest Distance var i.e it represent the farthest beacon

    // CodeBlock#1

    float largest_distance;
    int IndexOfLargestDistance;
    if (beacon_list[0].distance > beacon_list[1].distance)
    {
        if (beacon_list[0].distance > beacon_list[2].distance)
        {
            largest_distance = beacon_list[0].distance;
            IndexOfLargestDistance = 0;
        }
        else
        {
            largest_distance = beacon_list[2].distance;
            IndexOfLargestDistance = 2;
        }
    }
    else
    {
        if (beacon_list[1].distance > beacon_list[2].distance)
        {
            largest_distance = beacon_list[1].distance;
            IndexOfLargestDistance = 1;
        }
        else
        {
            largest_distance = beacon_list[2].distance;
            IndexOfLargestDistance = 2;
        }
    }

    // Compare with existing MajMin

    if (  *NewMaj0 == beacon_list[0].Major[0] &&
            *NewMaj1 == beacon_list[0].Major[1] &&
            *NewMin0 == beacon_list[0].Minor[0] &&
            *NewMin1 == beacon_list[0].Minor[1] )
    {
        // CodeBlock 4 - Calling Kalman Improvement on 1st beacon
        ImproveKalman0(NewDistance);
    }

    else if (*NewMaj0 == beacon_list[1].Major[0] &&
             *NewMaj1 == beacon_list[1].Major[1] &&
             *NewMin0 == beacon_list[1].Minor[0] &&
             *NewMin1 == beacon_list[1].Minor[1] )
    {
        // CodeBlock 4 - Calling Kalman Improvement on 2nd beacon
        ImproveKalman1(NewDistance);
    }

    else if (*NewMaj0 == beacon_list[2].Major[0] &&
             *NewMaj1 == beacon_list[2].Major[1] &&
             *NewMin0 == beacon_list[2].Minor[0] &&
             *NewMin1 == beacon_list[2].Minor[1] )
    {
        // CodeBlock4 - Calling Kalman Improvement on 3rd beacon
        ImproveKalman2(NewDistance);
    }

    else if (NewDistance < largest_distance)
    {
        UpdateBeaconList(NewMaj0, NewMaj1, NewMin0, NewMin1, NewDistance, &IndexOfLargestDistance);
    }
    UpdateCounter(NewDistance);
}

void printUUID()
{
    for (int i = 0; i < 3; i++)
    {
        printk("MajMin: %02X%02X %02X%02X ", beacon_list[i].Major[0], beacon_list[i].Major[1], beacon_list[i].Minor[0], beacon_list[i].Minor[1] );
        printf("Distance: %f \n", beacon_list[i].distance );
    }
    printk("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
}

// uuid to match 18ee1516016b4becad96bcb96d166e97
static bool uid_located(struct bt_data *data, void *user_data)
{

    if(data->data_len > 2 && data->data_len <= 25)
    {
        int i;
        memset (uuid_str, '\0', 25);
        for (i = 0; i < data->data_len; i++)
        {
            u16_t u16;
            memcpy(&u16, &data->data[i], sizeof(u16));
            uuid_str[i] = u16;
        }
        if(uuid_str[4] == 0x18 && uuid_str[5] == 0xee && uuid_str[6] == 0x15 && uuid_str[7] == 0x16 && uuid_str[8] == 0x01 && uuid_str[9] == 0x6b
                && uuid_str[10] == 0x4b && uuid_str[11] == 0xec && uuid_str[12] == 0xad && uuid_str[13] == 0x96 && uuid_str[14] == 0xbc && uuid_str[15] == 0xb9
                && uuid_str[16] == 0x6d && uuid_str[17] == 0x16 && uuid_str[18] == 0x6e && uuid_str[19] == 0x97)
        {

            ProcessNewBeacon(&uuid_str[20], &uuid_str[21], &uuid_str[22], &uuid_str[23], calculateDistance(RSSI));
            printUUID();

        }
    }
    return true;
}

static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
                         struct net_buf_simple *ad)
{
    RSSI = (int) rssi;
    bt_data_parse(ad, uid_located, NULL);
}

// Thread for scanning
void t_ble_scan()
{
  q= (struct Queue *) k_malloc(sizeof(struct Queue));
  q->front = NULL;            // initialize the queue's pointers to NULL
  q->rear = NULL;

    //Scan Parameter
    struct bt_le_scan_param scan_param =
    {
        .type       = BT_LE_SCAN_TYPE_ACTIVE,
        .interval   = 0x1388,
        .window     = 0x03E8,
    };

    //enable bluetooth
    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    //start scan
    err = bt_le_scan_start(&scan_param, device_found);
    if (err)
    {
        printk("Scanning failed to start (err %d)\n", err);
        return;
    }

    printk("Scanning successfully started\n");

}

void buttonA_work_handler(struct k_work *work){
  //send sigfox SOS message
  myuart_send("ATS410=1\n");
  k_sleep(100);
  myuart_send("AT$GI?\n");
  k_sleep(100);
  myuart_send("AT$RC\n");
  DataSend('H','E','L','P');
  k_sleep(2000);
}
void button_A_pressed(struct device *gpiob, struct gpio_callback *cb,u32_t pins){
	//printk("Button A pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonA_work);
}
void configureButtons(void){
	struct device *gpiob;
	gpiob = device_get_binding(PORT);
	if (!gpiob)
	{
		printk("error\n");
		return;
	}

	// Button A
	k_work_init(&buttonA_work, buttonA_work_handler);
	gpio_pin_configure(gpiob, PIN_A, GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	gpio_init_callback(&gpio_btnA_cb, button_A_pressed, BIT(PIN_A));
	gpio_add_callback(gpiob, &gpio_btnA_cb);
	gpio_pin_enable_callback(gpiob, PIN_A);
}

void main(void) {
  configureButtons();
  uart = device_get_binding("UART_1");
  uart_irq_callback_set(uart,isr);
  uart_irq_rx_enable(uart);
}

K_THREAD_DEFINE(t_ble_scan_id, MY_STACK_SIZE,
                t_ble_scan,
                NULL, NULL, NULL,
                MY_PRIORITY, 0, K_NO_WAIT);
