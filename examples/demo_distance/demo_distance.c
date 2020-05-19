/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "udp_netinit.h"

#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <demo_msgs/msg/demo_distance.h>
#include <std_msgs/msg/int32.h>
#include <rmw_uros/options.h>

#define OLIMEX_TFMINI "/dev/ttyS1"

/* Peripheral Map */
#define RCC_AHB1ENR         *(volatile uint32_t *) (RCC_BASE + 0x30)    /* AHB1 Enable Register */

#define PERIPH_BASE         (uint32_t) (0x40000000)                     /* Peripheral base address */
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000)
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000)
#define RCC_BASE            (AHB1PERIPH_BASE + 0x3800)                  /* Reset and Clock Control base address */
#define RCC_AHB1ENR         *(volatile uint32_t *) (RCC_BASE + 0x30)    /* AHB1 Enable Register */
#define GPIOC_BASE          (AHB1PERIPH_BASE + 0x0800)                  /* GPIO Port C base address */
#define GPIOC_MOUTPUT       *(volatile uint32_t *) (GPIOC_BASE + 0x04)  /* Port C mode register */
#define GPIOC_MODER         *(volatile uint32_t *) (GPIOC_BASE + 0x00)  /* Port C mode register */
#define GPIOC_ODR           *(volatile uint32_t *) (GPIOC_BASE + 0x14)  /* LED Output Data Register */



#define TFMINI_FRAME_HEADER_BYTE        (0x59)
#define TFMINI_FRAME_SPARE_BYTE         (0x00)
#define LED_HEARTBEAT			(9)

struct tfmini_frame {
        uint8_t headers[2];
        uint8_t dist_l;
        uint8_t dist_h;
        uint8_t strength_l;
        uint8_t strength_h;
        uint8_t integr_time;
        uint8_t spare_byte;
        uint8_t checksum;
} __attribute__((packed)) ;

static void led_toggle(void) {
	static int status = 0;
	static int half_seconds = 0;

	if (half_seconds == 5) {
		half_seconds = 0;
		if (status) {
			status = 0;
  			board_autoled_off(LED_HEARTBEAT);

		} else {
			status = 1;
  			board_autoled_on(LED_HEARTBEAT);
		}
	}

	half_seconds++;
}

static int read_all(int fd, char *data, size_t size)
{
	int readd = 0;
	int n;

	while ((n = read(fd, &data[readd], size - readd)) >= 0) {
		if (readd == size)
			return -1;

		readd += n;
	}

	return readd;
}

static int synchronize(int fd, struct tfmini_frame *tmf)
{
        ssize_t rc;
        unsigned int i=0;
	char data;
       	char *datas;
	unsigned int checksum;

	memset(tmf, 0, sizeof(*tmf));
        while (rc = read(fd, &data, 1) == 1) {

                if (data == TFMINI_FRAME_HEADER_BYTE && !i) {
			checksum = data;
                        i=1;
                } else if (data == TFMINI_FRAME_HEADER_BYTE && i) {
			checksum += data;
                        break;
                } else {
                        i=0;
		}
        }

	if (rc < 0) {
		return rc;
	}

	if ((rc = read_all(fd, (char *) &tmf->dist_l, sizeof(struct tfmini_frame) - 2))
			< sizeof(struct tfmini_frame) - 2) {
		printf("Error, could not read tmf, rc = %d\n", rc);
		printf("Value gotten tfm->distl=%d\n", tmf->dist_l);
	}

	datas = tmf;

	for (i = 0; i < 8; i ++) {
		checksum += datas[i];
	}

	if ((checksum & 0xFF) != tmf->checksum) {
		synchronize(fd, tmf);
	}

        return 0;
}

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int demo_distance_main(int argc, char* argv[])
#endif
{
	(void)argc;
	(void)argv;

	struct sched_param param;
    // rclc_publisher_t *publisher;
	// rclc_node_t *node;
    struct tfmini_frame tmf;
	unsigned int value;
	
	int fd = open(OLIMEX_TFMINI, O_RDONLY);
	printf("Starting application distance publisher\n");
	if (fd < 0) {
		printf("Could not open %s\n", OLIMEX_TFMINI);
		goto end;
	}

     if (synchronize(fd, &tmf)) {
		printf("Could not synchronize %s\n", OLIMEX_TFMINI);
		goto end;
    }
	
	param.sched_priority = 100;
	(void)sched_setparam(0, &param);

	udp_netinit();
// =================================
    rcl_ret_t rv;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    rv = rcl_init_options_init(&options, rcl_get_default_allocator());
    if (RCL_RET_OK != rv) {
        printf("rcl init options error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    // Set the IP and the port of the Agent
    // rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
    // rmw_uros_options_set_udp_address(argv[1], argv[2], rmw_options);

    rcl_context_t context = rcl_get_zero_initialized_context();
    // rv = rcl_init(0, NULL, &options, &context);												// 0, NULL
    rv = rcl_init(argc, argv, &options, &context);										// argc, argv
    if (RCL_RET_OK != rv) {
        printf("rcl initialization error: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rcl_node_t node = rcl_get_zero_initialized_node();
    rv = rcl_node_init(&node, "int32_publisher_rcl", "", &context, &node_ops);
    if (RCL_RET_OK != rv) {
        printf("Node initialization error: %s\n", rcl_get_error_string().str);
		// board_reset(0);
        return 1;
    }

    rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &publisher_ops);
    if (RCL_RET_OK != rv) {
        printf("Publisher initialization error: %s\n", rcl_get_error_string().str);
		// board_reset(0);
        return 1;
    }


	demo_msgs__msg__DemoDistance msg;
	msg.heartbeats = 0;

    // std_msgs__msg__Int32 msg;
    // msg.data = 0;
    usleep(3000000); // As we are sending low number mensajes we need to wait discovery of the subscriber. (Do not have a notification on discovery)
    do {
		int i;

    	while (synchronize(fd, &tmf)) {
			printf("Error during synchro \n");
    	}
		value = (tmf.dist_h << 8 | tmf.dist_l);
		msg.heartbeats++;
		if (value < 12000) {
			msg.sensor_distance = value;
			// msg.data = value;
		}
        
		rv = rcl_publish(&publisher, (const void*)&msg, NULL);
        if (RCL_RET_OK == rv ) {
            // printf("TFMINI sent: '%i'\n", msg.data);
            printf("TFMINI sent: '%i'\n", msg.sensor_distance);
        }
		// rclc_spin_node_once(node, 1000);
		led_toggle();
		for (i = 0; i < 10; i++) { 
			usleep(10000);
	    }

    // } while (RCL_RET_OK == rv && msg.data < 1000 );
    } while (RCL_RET_OK == rv && msg.heartbeats < 10 );

    rv = rcl_publisher_fini(&publisher, &node);
    rv = rcl_node_fini(&node);

end:

	// board_reset(0);
    return 0;
}

// ======================================

	// rclc_init(1, "");
 

	// // const rclc_message_type_support_t type_support = 
	// // 	RCLC_GET_MSG_TYPE_SUPPORT(demo_msgs, msg, DemoDistance);
	// const rclc_message_type_support_t type_support = 
	// 	RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);	

	// while (1) {
	// 	node = rclc_create_node("polimexdst", "");
	// 	if (node) {
	// 		break;
	// 	}
		
	// 	else if (!node && i == 100) {
	// 		printf("Issues creating node\n");
	// 		board_reset(0);
	// 	}
	// 	i++;

	// 	usleep(10000);
	// }

	// publisher = rclc_create_publisher(node, type_support, "distance_oli", 1);
	// if (!publisher) {
	// 	rclc_destroy_node(node);
	// 	printf("Issues creating publisher\n");
	// 	return -1;
	// }

	// // demo_msgs__msg__DemoDistance msg;
	// // msg.heartbeats = 0;
    // // std_msgs__msg__Int32 msg;
    // // msg.data = 0;

// -=-=-=-=-=-=-=-=-=-=-=-=
	// if (fd < 0) {
	// 	printf("Could not open %s\n", OLIMEX_TFMINI);
	// 	goto end;
	// }

    //  if (synchronize(fd, &tmf)) {
	// 	printf("Could not synchronize %s\n", OLIMEX_TFMINI);
	// 	goto end;
    // }
// -=-=-=-=-=-=-=-=-=-=-=-=

    // while (rclc_ok()) {

    // while (synchronize(fd, &tmf)) {
	// 	printf("Error during synchro \n");
    // }

// 	value = (tmf.dist_h << 8 | tmf.dist_l);
// 	// msg.heartbeats++;

// 	if (value < 12000) {
// 		// msg.sensor_distance = value;
// 		msg.data = value;
// 	}

// 	// if (rclc_publish(publisher, (const void*)&msg)) {
// 	// 	board_reset(0);
// 	// }

// 	// rclc_spin_node_once(node, 1000);

// 	led_toggle();
// 	for (i = 0; i < 10; i++) { 
// 		usleep(10000);
//     }

// end:
// 	// rclc_destroy_publisher(publisher);
// 	// rclc_destroy_node(node);

// 	// board_reset(0);

// ======================================
