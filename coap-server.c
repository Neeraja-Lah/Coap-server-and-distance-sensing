#include <logging/log.h>
LOG_MODULE_REGISTER(net_coap_server_sample, LOG_LEVEL_DBG);

#include <device.h>
#include <drivers/gpio.h>
#include <devicetree.h>
#include <kernel.h>
#include <stdio.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <string.h>
#include <net/socket.h>
#include <net/net_mgmt.h>
#include <net/net_ip.h>
#include <net/udp.h>
#include <net/coap.h>
#include <net/coap_link_format.h>
#include <stdlib.h>
#include <sys/printk.h>
#include <fsl_iomuxc.h>
#include <drivers/sensor.h>
#include "net_private.h"

#if defined(CONFIG_NET_IPV6)
#include "ipv6.h"
#endif

#define MAX_COAP_MSG_LEN 256

#define MY_COAP_PORT 5683

#define BLOCK_WISE_TRANSFER_SIZE_GET 2048

#if defined(CONFIG_NET_IPV6)
#define ALL_NODES_LOCAL_COAP_MCAST \
	{ { { 0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xfd } } }

#define MY_IP6ADDR \
	{ { { 0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1 } } }
#endif

#define NUM_OBSERVERS 2

#define NUM_PENDINGS 3

#define MAX_RETRANSMIT_COUNT 2

#define R_LED 0
#define G_LED 1
#define B_LED 2

#define RED_LED				red_led

#define R_LED_NODE 		DT_NODELABEL(RED_LED)

#define R_LED_LABEL 		DT_GPIO_LABEL(R_LED_NODE, gpios)

#define R_PIN 		DT_GPIO_PIN(R_LED_NODE, gpios)

#define R_FLAG 		DT_GPIO_FLAGS(R_LED_NODE, gpios)

#define GREEN_LED 				gree_led

#define G_LED_NODE 		DT_NODELABEL(GREEN_LED)

#define G_LED_LABEL		DT_GPIO_LABEL(G_LED_NODE, gpios)

#define G_PIN 		DT_GPIO_PIN(G_LED_NODE, gpios)

#define G_FLAG		DT_GPIO_FLAGS(G_LED_NODE, gpios)

#define BLUE_LED 				blue_led

#define B_LED_NODE 		DT_NODELABEL(BLUE_LED)

#define B_LED_LABEL 		DT_GPIO_LABEL(B_LED_NODE, gpios)

#define B_PIN 		DT_GPIO_PIN(B_LED_NODE, gpios)

#define B_FLAG 		DT_GPIO_FLAGS(B_LED_NODE, gpios)

#define SENSOR_1 				sensor_dist1

#define HCSR04_1_NODE 		DT_NODELABEL(SENSOR_1)

#define HCSR04_1_LABEL 		DT_PROP(HCSR04_1_NODE, label)

#define SENSOR_2 				sensor_dist2

#define HCSR04_2_NODE 		DT_NODELABEL(SENSOR_2)

#define HCSR04_2_LABEL 		DT_PROP(HCSR04_2_NODE, label)
	
//Thread stack size and priority level
#define NEW_STACK 500
#define PRIORITY_LEVEL 5


/* CoAP socket */
static int sock;

static struct coap_observer observers[NUM_OBSERVERS];

static struct coap_pending pendings[NUM_PENDINGS];

static struct k_work_delayable observer_work;

static struct coap_resource *resource_to_notify;

static struct k_work_delayable retransmit_work;

uint8_t red_led_status=0, blue_led_status=0, green_led_status=0 , num_led=0, num_hcsensor=1;

static int change_distance1, change_distance2, change_distance3, change_distance4;

const static struct device *led_red, *led_green, *led_blue, *HCSR04_1, *HCSR04_2;

//Timer Definition
K_TIMER_DEFINE(timer_synchronization, NULL, NULL);

//Distance Measurement and Comparison
uint16_t final_pos1_1=0, final_pos1_2=0, initial_pos1_1 = 0, initial_pos1_2 =0, final_pos2_1=0, final_pos2_2=0, initial_pos2_1=0, initial_pos2_2=0 ;

static int measure(const struct device *dev)
{
    int ret;
    struct sensor_value distance;
    ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
    switch (ret) {
    case 0:
        ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
        if (ret) {
            LOG_ERR("sensor_channel_get failed ret %d", ret);
            return ret;
        }
		if (!strcmp(dev-> name, HCSR04_1_LABEL)){
		final_pos1_1=distance.val1;
		final_pos1_2=(distance.val2)/1000;
		}
		if (!strcmp(dev-> name, HCSR04_2_LABEL)){
		final_pos2_1=distance.val1;
		final_pos2_2=(distance.val2)/1000;
		}
        break;
    case -EIO:
        LOG_WRN("Failed to read Device: %s", dev->name);
        break;
    default:
        LOG_ERR("Error in reading device: %s", dev->name);
        break;
    }
    return 0;
}

//Thread Function
void sensor_data(void* dummy1, void* dummy2, void* dummy3)
{	
	k_timer_start(&timer_synchronization, K_MSEC(1000), K_MSEC(1000));
	
	volatile uint16_t ret=0;
    while (1) {
		k_timer_status_sync(&timer_synchronization);
		initial_pos1_1=final_pos1_1;
		initial_pos1_2=final_pos1_2;
		ret = measure(HCSR04_1);
		if(ret){
			LOG_ERR("Failed to measure HCSR04_1");
		}
		initial_pos2_1=final_pos2_1;
		initial_pos2_2=final_pos2_2;
		ret = measure(HCSR04_2);
		if(ret){
			LOG_ERR("Failed to measure HCSR04_2");
		}

    }
}

//Thread creation
K_THREAD_DEFINE(new_threadid, NEW_STACK,
                sensor_data, NULL, NULL, NULL,
                PRIORITY_LEVEL, 0, 0);

#define ADDRLEN(sock) \
	(((struct sockaddr *)sock)->sa_family == AF_INET ? \
		sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6))

#if defined(CONFIG_NET_IPV4)
static int start_coap_server(void)
{
	struct sockaddr_in addr;
	int r;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(MY_COAP_PORT);

	sock = socket(addr.sin_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create UDP socket %d", errno);
		return -errno;
	}

	r = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (r < 0) {
		LOG_ERR("Failed to bind UDP socket %d", errno);
		return -errno;
	}

	return 0;
}
#endif

static int send_coap_reply(struct coap_packet *cpkt,
			   const struct sockaddr *addr,
			   socklen_t addr_len)
{
	int r;

	net_hexdump("Response", cpkt->data, cpkt->offset);

	r = sendto(sock, cpkt->data, cpkt->offset, 0, addr, addr_len);
	if (r < 0) {
		LOG_ERR("Failed to send %d", errno);
		r = -errno;
	}

	return r;
}

//well_known_core resource for discovering resources on coap server
static int well_known_core_get(struct coap_resource *resource,
			       struct coap_packet *request,
			       struct sockaddr *addr, socklen_t addr_len)
{
	struct coap_packet response;
	uint8_t *data;
	int r;

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_well_known_core_get(resource, request, &response,
				     data, MAX_COAP_MSG_LEN);
	if (r < 0) {
		goto end;
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}

//Scheduling next transmission
static void prepare_new_retransmission(void)
{
	struct coap_pending *new_pending_ptr;
	int32_t time_remaining;
	uint32_t curr_time = k_uptime_get_32();
	new_pending_ptr = coap_pending_next_to_expire(pendings, NUM_PENDINGS);
	if (!new_pending_ptr) {
		return;
	}

	time_remaining = new_pending_ptr->t0 + new_pending_ptr->timeout - curr_time;
	if (time_remaining < 0) {
		time_remaining = 0;
	}

	k_work_reschedule(&retransmit_work, K_MSEC(time_remaining));
}

static void delete_observer(struct sockaddr *addr);

static void retransmit_request(struct k_work *work)
{
	struct coap_pending *pending;
	int r;

	pending = coap_pending_next_to_expire(pendings, NUM_PENDINGS);
	if (!pending) {
		return;
	}

	if (!coap_pending_cycle(pending)) {
		delete_observer(&pending->addr);
		k_free(pending->data);
		coap_pending_clear(pending);
	} else {
		net_hexdump("Retransmit", pending->data, pending->len);

		r = sendto(sock, pending->data, pending->len, 0,
			   &pending->addr, ADDRLEN(&pending->addr));
		if (r < 0) {
			LOG_ERR("Failed to send %d", errno);
		}
	}

	prepare_new_retransmission();
}

//Difference of distance measurements of each sensor 
static void diff_distance(struct k_work *work)
{
	change_distance1 = abs(final_pos1_1 - initial_pos1_1);
	change_distance2 = abs(final_pos1_2 - initial_pos1_2);

	change_distance3 = abs(final_pos2_1 - initial_pos2_1);
	change_distance4 = abs(final_pos2_2 - initial_pos2_2);

	if (resource_to_notify) {
		coap_resource_notify(resource_to_notify);
	}

	k_work_reschedule(&observer_work, K_SECONDS(5));
}


static int led_get(struct coap_resource *resource,
		     struct coap_packet *request,
		     struct sockaddr *addr, socklen_t addr_len)

{	
	static struct coap_block_context ctx;
	struct coap_packet response;
	uint8_t payload[64];
	uint8_t token[COAP_TOKEN_MAX_LEN];
	uint8_t *data;
	uint16_t id;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	int r;
	
	if (ctx.total_size == 0) {
		coap_block_transfer_init(&ctx, COAP_BLOCK_64,
					 BLOCK_WISE_TRANSFER_SIZE_GET);
	}

	r = coap_update_from_block(request, &ctx);
	if (r < 0) {
		return -EINVAL;
	}

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, COAP_TYPE_ACK, tkl, token,
			     COAP_RESPONSE_CODE_CONTENT, id);
	if (r < 0) {
		return -EINVAL;
	}

	r = coap_append_option_int(&response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_TEXT_PLAIN);
	if (r < 0) {
		goto end;
	}

	r = coap_append_block2_option(&response, &ctx);
	if (r < 0) {
		goto end;
	}

	r = coap_packet_append_payload_marker(&response);
	if (r < 0) {
		goto end;
	}

num_led = (int)((struct coap_core_metadata *)resource->user_data)->user_data;

	switch(num_led)
	{
		case R_LED:
			r = (red_led_status == 0) ? snprintk((char *)payload, sizeof(payload), "RED LED IS OFF") : snprintk((char *)payload, sizeof(payload), "RED LED IS ON");
			break;
		case G_LED:
			r = (green_led_status == 0) ? snprintk((char *)payload, sizeof(payload), "GREEN LED IS OFF") : snprintk((char *)payload, sizeof(payload), "GREEN LED IS ON");
			break;

		case B_LED:
			r = (blue_led_status == 0) ? snprintk((char *)payload, sizeof(payload), "BLUE LED IS OFF") : snprintk((char *)payload, sizeof(payload), "BLUE LED IS ON");
			break;
	}

	r = coap_packet_append_payload(&response, (uint8_t*)payload, strlen(payload));
	if (r < 0) {
		goto end;
	}

	r = coap_next_block(&response, &ctx);
	if (!r) {
		memset(&ctx, 0, sizeof(ctx));
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}

static int led_put(struct coap_resource *resource,
		    struct coap_packet *request,
		    struct sockaddr *addr, socklen_t addr_len)
{
	struct coap_packet response;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	const uint8_t *payload;
	uint8_t *data;
	uint16_t payload_len;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	uint16_t id;
	int r,ret;
	int value;

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	payload = coap_packet_get_payload(request, &payload_len);
	if (payload) {
		net_hexdump("PUT Payload", payload, payload_len);
	}
	
	value = atoi(payload);

	num_led = (int)((struct coap_core_metadata *)resource->user_data)->user_data;

	switch(num_led)
	{
		case R_LED:
		ret = gpio_pin_set(led_red, R_PIN, value);
		red_led_status=value;
		if(ret){
			LOG_WRN("Error: GPIO for RED led is not set!\n");
			return -1;
			}
			break;

		case G_LED:
		ret = gpio_pin_set(led_green, G_PIN, value);
		green_led_status=value;
		if(ret){
			LOG_WRN("Error: GPIO for GREEN led is not set!\n");
			return -1;
			}
			break;

		case B_LED:
		ret = gpio_pin_set(led_blue, B_PIN, value);
		blue_led_status=value;
		if(ret){
			LOG_WRN("Error: GPIO for BLUE led is not set!\n");
			return -1;
			}
			break;
	}
	type = (type == COAP_TYPE_CON) ? COAP_TYPE_ACK : COAP_TYPE_NON_CON;

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, type, tkl, token,
			     COAP_RESPONSE_CODE_CHANGED, id);
	if (r < 0) {
		goto end;
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}

static int create_pending_request(struct coap_packet *response,
				  const struct sockaddr *addr)
{
	struct coap_pending *pending;
	int r;

	pending = coap_pending_next_unused(pendings, NUM_PENDINGS);
	if (!pending) {
		return -ENOMEM;
	}

	r = coap_pending_init(pending, response, addr,
			      COAP_DEFAULT_MAX_RETRANSMIT);
	if (r < 0) {
		return -EINVAL;
	}

	coap_pending_cycle(pending);

	pending = coap_pending_next_to_expire(pendings, NUM_PENDINGS);
	if (!pending) {
		return 0;
	}

	k_work_reschedule(&retransmit_work, K_MSEC(pending->timeout));

	return 0;
}


static int send_notification_packet(const struct sockaddr *addr,
				    socklen_t addr_len,
				    uint16_t age, uint16_t id,
				    const uint8_t *token, uint8_t tkl,
				    bool is_response, struct coap_resource *resource)
{
	struct coap_packet response;
	char payload[40];
	uint8_t *data;
	uint8_t type;
	int r;

	if (is_response) {
		type = COAP_TYPE_ACK;
	} else {
		type = COAP_TYPE_CON;
	}

	if (!is_response) {
		id = coap_next_id();
	}

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, type, tkl, token,
			     COAP_RESPONSE_CODE_CONTENT, id);
	if (r < 0) {
		goto end;
	}

	if (age >= 2U) {
		r = coap_append_option_int(&response, COAP_OPTION_OBSERVE, age);
		if (r < 0) {
			goto end;
		}
	}

	r = coap_append_option_int(&response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_TEXT_PLAIN);
	if (r < 0) {
		goto end;
	}

	r = coap_packet_append_payload_marker(&response);
	if (r < 0) {
		goto end;
	}

	if(change_distance1 >= 1 || change_distance2 > 500){
		r = snprintk((char *) payload, sizeof(payload),"Change of distance measurement for sensor 1 > 0.5");
	}
	else if(change_distance3 >= 1 || change_distance3 > 500){
			r = snprintk((char *) payload, sizeof(payload),"Change of distance measurement for sensor 1 > 0.5");
	}
	else{
		num_hcsensor = (int)((struct coap_core_metadata *)resource->user_data)->user_data;
		r = (num_hcsensor == 1) ? snprintk((char *) payload, sizeof(payload), "distance 1: %d.%03din\n", final_pos1_1, final_pos1_2) :
			snprintk((char *) payload, sizeof(payload), "distance 2: %d.%03din\n", final_pos2_1, final_pos2_2);
	}

	if (r < 0) {
		goto end;
	}

	r = coap_packet_append_payload(&response, (uint8_t *)payload,
				       strlen(payload));
	if (r < 0) {
		goto end;
	}

	if (type == COAP_TYPE_CON) {
		r = create_pending_request(&response, addr);
		if (r < 0) {
			goto end;
		}
	}

	k_work_reschedule(&observer_work, K_SECONDS(5));

	r = send_coap_reply(&response, addr, addr_len);

	/* On succesfull creation of pending request, do not free memory */
	if (type == COAP_TYPE_CON) {
		return r;
	}

end:
	k_free(data);

	return r;
}


static int sensor_get(struct coap_resource *resource,
		     struct coap_packet *request,
		     struct sockaddr *addr, socklen_t addr_len)

{
	struct coap_observer *observer;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	uint16_t id;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	bool observe = true;

	if (!coap_request_is_observe(request)) {
		observe = false;
		goto done;
	}

	observer = coap_observer_next_unused(observers, NUM_OBSERVERS);
	if (!observer) {
		return -ENOMEM;
	}

	coap_observer_init(observer, request, addr);

	coap_register_observer(resource, observer);

	resource_to_notify = resource;

done:
	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	return send_notification_packet(addr, addr_len,
					observe ? resource->age : 0,
					id, token, tkl, true , resource);
}	


static int hcsr_period_put(struct coap_resource *resource,
		    struct coap_packet *request,
		    struct sockaddr *addr, socklen_t addr_len)
{
	struct coap_packet response;
	uint8_t token[COAP_TOKEN_MAX_LEN];
	const uint8_t *payload;
	uint8_t *data;
	uint16_t payload_len;
	uint8_t code;
	uint8_t type;
	uint8_t tkl;
	uint16_t id;
	int r,val;

	code = coap_header_get_code(request);
	type = coap_header_get_type(request);
	id = coap_header_get_id(request);
	tkl = coap_header_get_token(request, token);

	LOG_INF("*******");
	LOG_INF("type: %u code %u id %u", type, code, id);
	LOG_INF("*******");

	payload = coap_packet_get_payload(request, &payload_len);
	if (payload) {
		net_hexdump("PUT Payload", payload, payload_len);
	}
	
	val = atoi(payload);

	k_timer_start(&timer_synchronization, K_MSEC(val), K_MSEC(val));

	if (type == COAP_TYPE_CON) {
		type = COAP_TYPE_ACK;
	} else {
		type = COAP_TYPE_NON_CON;
	}

	data = (uint8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&response, data, MAX_COAP_MSG_LEN,
			     COAP_VERSION_1, type, tkl, token,
			     COAP_RESPONSE_CODE_CHANGED, id);
	if (r < 0) {
		goto end;
	}

	r = send_coap_reply(&response, addr, addr_len);

end:
	k_free(data);

	return r;
}

static void sensor_notify(struct coap_resource *resource,
		       struct coap_observer *observer)
{
	send_notification_packet(&observer->addr,
				 sizeof(observer->addr),
				 resource->age, 0,
				 observer->token, observer->tkl, false, resource);
}

static const char * const red_led_path[] = { "led", "led_red", NULL };
static const char * const green_led_path[] = { "led", "led_green", NULL };
static const char * const blue_led_path[] = { "led", "led_blue", NULL };
static const char * const sensor1_path[] = { "sensor", "distsensor_1", NULL };
static const char * const sensor2_path[] = { "sensor", "distsensor_2", NULL };
static const char * const hcsr_samplingperiod_path[] = {"sensor", "sampling_period", NULL};

static struct coap_resource resources[] = {
	{ .get = well_known_core_get,
	  .path = COAP_WELL_KNOWN_CORE_PATH,
	},
	{ .get = led_get,
	  .put = led_put,
	  .path = red_led_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)0,
        }),
	},
	{ .get = led_get,
	  .put = led_put,
	  .path = green_led_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)1,
        }),
	},
	{ .get = led_get,
	  .put = led_put,
	  .path = blue_led_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)2,
        }),
	},
	{ .get = sensor_get,
	  .path = sensor1_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)1,
        }),
		.notify = sensor_notify,
	},
	{ .get = sensor_get,
	  .path = sensor2_path,
	  .user_data = &((struct coap_core_metadata){
            .attributes = NULL,
            .user_data = (void*)2,
        }),
	   .notify = sensor_notify,
	},
	{ .put = hcsr_period_put,
	  .path = hcsr_samplingperiod_path,
	},
};

static struct coap_resource *find_resource_by_observer(
		struct coap_resource *resources, struct coap_observer *o)
{
	struct coap_resource *r;

	for (r = resources; r && r->path; r++) {
		sys_snode_t *node;

		SYS_SLIST_FOR_EACH_NODE(&r->observers, node) {
			if (&o->list == node) {
				return r;
			}
		}
	}

	return NULL;
}
static void delete_observer(struct sockaddr *soc_addr)
{
	struct coap_resource *r;
	struct coap_observer *o;

	o = coap_find_observer_by_addr(observers, NUM_OBSERVERS, soc_addr);
	if (!o) {
		return;
	}

	r = find_resource_by_observer(resources, o);
	if (!r) {
		LOG_ERR("Failed to find resource by observer\n");
		return;
	}

	LOG_INF("Observer removed %p", o);

	coap_remove_observer(r, o);
	memset(o, 0, sizeof(struct coap_observer));
}

static struct coap_resource *find_resouce_by_observer(
		struct coap_resource *resources, struct coap_observer *o)
{
	struct coap_resource *r;

	for (r = resources; r && r->path; r++) {
		sys_snode_t *node;

		SYS_SLIST_FOR_EACH_NODE(&r->observers, node) {
			if (&o->list == node) {
				return r;
			}
		}
	}

	return NULL;
}


static void process_coap_request(uint8_t *data, uint16_t data_len,
				 struct sockaddr *client_addr,
				 socklen_t client_addr_len)
{
	struct coap_packet request;
	struct coap_pending *pending;
	struct coap_option options[16] = { 0 };
	uint8_t opt_num = 16U;
	uint8_t type;
	int r;

	r = coap_packet_parse(&request, data, data_len, options, opt_num);
	if (r < 0) {
		LOG_ERR("Invalid data received (%d)\n", r);
		return;
	}

	type = coap_header_get_type(&request);

	pending = coap_pending_received(&request, pendings, NUM_PENDINGS);
	if (!pending) {
		goto not_found;
	}

	/* Clear CoAP pending request */
	if (type == COAP_TYPE_ACK) {
		k_free(pending->data);
		coap_pending_clear(pending);
	}

	return;

not_found:

	if (type == COAP_TYPE_RESET) {
		struct coap_resource *r;
		struct coap_observer *o;

		o = coap_find_observer_by_addr(observers, NUM_OBSERVERS,
					       client_addr);
		if (!o) {
			LOG_ERR("Observer not found\n");
			goto end;
		}

		r = find_resouce_by_observer(resources, o);
		if (!r) {
			LOG_ERR("Observer found but Resource not found\n");
			goto end;
		}

		coap_remove_observer(r, o);

		return;
	}

end:
	r = coap_handle_request(&request, resources, options, opt_num,
				client_addr, client_addr_len);
	if (r < 0) {
		LOG_WRN("No handler for such request (%d)\n", r);
	}
}

static int process_client_request(void)
{
	int received;
	struct sockaddr client_addr;
	socklen_t client_addr_len;
	uint8_t request[MAX_COAP_MSG_LEN];

	do {
		client_addr_len = sizeof(client_addr);
		received = recvfrom(sock, request, sizeof(request), 0,
				    &client_addr, &client_addr_len);
		if (received < 0) {
			LOG_ERR("Connection error %d", errno);
			return -errno;
		}

		process_coap_request(request, received, &client_addr,
				     client_addr_len);
	} while (true);

	return 0;
}


void main(void)
{
	int x,retVal=0;

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));


	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_GPIO3_IO15, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_GPIO3_IO15,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
			    
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 1);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));	
   IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
			    
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02, 1);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,
			    IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	 if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT)) {
        k_sleep(K_MSEC(500));
    }
	//Sensors & LED Device Configurations
	HCSR04_1= device_get_binding(HCSR04_1_LABEL);
	if(HCSR04_1 == NULL)
    {
        LOG_WRN("Error: Failed to get HCSR04 1 Device\n");
        return;
    }
	LOG_INF("Successfully configured HCSR04 1 sensor");

	HCSR04_2= device_get_binding(HCSR04_2_LABEL);
	if(HCSR04_2 == NULL)
    {
        LOG_WRN("Error: Failed to get HCSR04 2 Device\n");
        return;
    }
	LOG_INF("Successfully configured HCSR04 2 sensor");

	k_thread_start(new_threadid);

	led_red = device_get_binding(R_LED_LABEL);
    if(led_red == NULL)
    {
        LOG_WRN("Error: Failed to get RED LED\n");
        return;
    }

	retVal = gpio_pin_configure(led_red, R_PIN, GPIO_OUTPUT_ACTIVE | R_FLAG); 
	if(retVal < 0)
		LOG_WRN("Error: Failed to configure RED LED\n");
	
	retVal = gpio_pin_set(led_red, R_PIN, red_led_status);
	if(retVal){
		LOG_WRN("Error in setting the GPIO Pin for RED LED!\n");
		return;
	}

	led_green = device_get_binding(G_LED_LABEL);
    if(led_green == NULL)
    {
        LOG_WRN("Failed to get GREEN LED Device\n");
        return;
    }

	retVal = gpio_pin_configure(led_green, G_PIN, GPIO_OUTPUT_ACTIVE | G_FLAG); 
	if(retVal < 0)
		LOG_WRN("Error: Failed to configure GREEN LED\n");
	
	retVal = gpio_pin_set(led_green, G_PIN, green_led_status);
	if(retVal){
		LOG_WRN("Error in setting the GPIO Pin for GREEN LED!\n");
		return;
	}

	/* BLUE_LED LED Config */
	led_blue = device_get_binding(B_LED_LABEL);
    if(led_blue == NULL)
    {
        LOG_WRN("Failed to get BLUE LED Device\n");
        return;
    }

	retVal = gpio_pin_configure(led_blue, B_PIN, GPIO_OUTPUT_ACTIVE | B_FLAG); 
	if(retVal < 0)
		LOG_WRN("Error: Failed to configure BLUE LED\n");
	
	retVal = gpio_pin_set(led_blue, B_PIN, blue_led_status);
	if(retVal){
		LOG_WRN("Error in setting the GPIO Pin for BLUE LED!\n");
		return;
	}


	x = start_coap_server();
	if (x < 0) {
		goto quit;
	}

	k_work_init_delayable(&retransmit_work, retransmit_request);
	k_work_init_delayable(&observer_work, diff_distance);

	while (1) {
		x = process_client_request();
		if (x < 0) {
			goto quit;
		}
	}

	LOG_DBG("Done");
	return;

quit:
	LOG_ERR("Quit");
}

