/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rcutils/time.h>
#include <rclc_parameter/rclc_parameter.h>

// messages includes
#include <diagnostic_msgs/msg/Diagnostic_Array.h>
#include <std_msgs/msg/U_Int16_Multi_Array.h>
#include <sensor_msgs/msg/Joy.h>
#include <sensor_msgs/msg/Fluid_Pressure.h>
#include <sensor_msgs/msg/Temperature.h>
#include <sensor_msgs/msg/Imu.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>
#include <nereo_interfaces/msg/thruster_statuses.h>
#include <nereo_interfaces/srv/set_navigation_mode.h>

#include "stabilize_mode.h"
#include "navigation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef enum {
	ROS2_OK,
	ROS2_WARNING,
	ROS2_ERROR,
	ROS2_STALE
} RosErrors;

typedef enum {
	NAVIGATION_MODE_MANUAL,
    NAVIGATION_MODE_STABILIZE_FULL,
	NAVIGATION_MODE_STABILIZE_DEPTH,
	NAVIGATION_MODE_STABILIZE_R_P,
	NAVIGATION_MODE_STABILIZE_ANGLES
} NavigationModes;

typedef enum {
	ROV_DISARMED,
	ROV_ARMED,
} RovArmModes;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile RovArmModes is_rov_armed = ROV_ARMED;
volatile NavigationModes navigation_mode = NAVIGATION_MODE_MANUAL;
uint8_t MAX_DIAG_MESSAGES = 10;
//static RovArmModes is_rov_armed = ROV_DISARMED;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 60000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void joystick_subscription_callback (const void * msgin) {}
void imu_subscription_callback (const void * msgin) {}
void pressure_subscription_callback (const void * msgin) {}
void temperature_subscription_callback (const void * msgin) {}

void set_pwm_idle();
void start_pwm_channels_1_to_4(TIM_HandleTypeDef *timer_handle);
void set_pwms(uint16_t pwms[8]);

void arm_disarm_service_callback(const void *, void *);
void set_nav_mode_service_callback(const void *, void *);
size_t add_diagnostic_status(diagnostic_msgs__msg__DiagnosticArray * array, char * hardware_id, RosErrors level, char * name, char * message);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
    // micro-ROS configuration

    rmw_uros_set_custom_transport(
      true,
      (void *) &huart1,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read
    );

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
        Error_Handler();
    }

    // uROS app

    // time
	rcutils_time_point_value_t now;
	// publishers
	rcl_publisher_t thruster_status_publisher;
	rcl_publisher_t diagnostic_publisher;


	// subscribers
	rcl_subscription_t joystick_subscriber;
	rcl_subscription_t imu_subscriber;
	rcl_subscription_t pressure_subscriber;
	rcl_subscription_t temperature_subscriber;



	// messages
	diagnostic_msgs__msg__DiagnosticArray diagnostic_values_array;

	// does it work like this or do I have to declare a char * and then assign it to the data field, updating also size and capacity?? need to test on f303re
	diagnostic_values_array.header.frame_id.capacity = 14;
	diagnostic_values_array.header.frame_id.data = "navigator_fc";

	diagnostic_msgs__msg__DiagnosticStatus diagnostic_statuses[MAX_DIAG_MESSAGES];
	diagnostic_values_array.status.data = diagnostic_statuses;
	diagnostic_values_array.status.capacity = MAX_DIAG_MESSAGES;
	diagnostic_values_array.status.size = 0;

	nereo_interfaces__msg__ThrusterStatuses thruster_status;
	sensor_msgs__msg__Joy joystick_input;
	sensor_msgs__msg__Imu imu_data;
	sensor_msgs__msg__FluidPressure fluid_pressure;
	sensor_msgs__msg__Temperature water_temperature;

    // support , allocator, node
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "navigator_node", "", &support);

    // create executor
	rclc_executor_t executor;
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(
	    &executor, &support.context,
		RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);

	/*
	 * PUBLISHERS
	 */
	// thruster status publisher
	rclc_publisher_init_default(
		&thruster_status_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, ThrusterStatuses),
		"thruster_status"
	);

	// diagnostic publisher
	rclc_publisher_init_default(
		&diagnostic_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
		"diagnostic_messages"
	);

	/*
	 * SUBSCRIBERS
	 */
	// joystick subscriber
	rclc_subscription_init_default(
		&joystick_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
		"joy"
	);
	rclc_executor_add_subscription(
		&executor,
		&joystick_subscriber,
		&joystick_input,
		&joystick_subscription_callback,
		ON_NEW_DATA
	);

	// IMU subscriber
	rclc_subscription_init_default(
		&imu_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"imu_data"
	);
	rclc_executor_add_subscription(
		&executor,
		&imu_subscriber,
		&imu_data,
		&imu_subscription_callback,
		ON_NEW_DATA
	);

	// pressure subscriber
	rclc_subscription_init_default(
		&pressure_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
		"fluid_pressure"
	);
	rclc_executor_add_subscription(
		&executor,
		&pressure_subscriber,
		&fluid_pressure,
		&pressure_subscription_callback,
		ON_NEW_DATA
	);

	// temperature subscriber
	rclc_subscription_init_default(
		&temperature_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
		"water_temperature"
	);
	rclc_executor_add_subscription(
		&executor,
		&temperature_subscriber,
		&water_temperature,
		&temperature_subscription_callback,
		ON_NEW_DATA
	);

	// arm/disarm ROV service
	rcl_service_t arm_disarm_service;
	const char * service_name_0 = "/arm_disarm";

	// Get message type support
	const rosidl_service_type_support_t * type_support_0 = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);

	// Initialize server with default configuration
	rcl_ret_t rc = rclc_service_init_default(
	  &arm_disarm_service,
	  &node,
	  type_support_0,
	  service_name_0
	);

	if (rc != RCL_RET_OK) {
		add_diagnostic_status(&diagnostic_values_array, "service_init", ROS2_STALE,
				"arm_disarm_service", "Error initializing arm_disarm_service.");
		Error_Handler();
	}

	std_srvs__srv__SetBool_Request arm_disarm_request_msg;
	std_srvs__srv__SetBool_Response arm_disarm_response_msg;

	rc = rclc_executor_add_service(
	  &executor, &arm_disarm_service, &arm_disarm_request_msg,
	  &arm_disarm_response_msg, arm_disarm_service_callback
	);

	if (rc != RCL_RET_OK) {
		add_diagnostic_status(&diagnostic_values_array, "executor_add", ROS2_STALE,
				"arm_disarm_service", "Error adding arm_disarm_service to executor.");
		Error_Handler();
	}

	// set navigation mode service
	rcl_service_t set_nav_mode_srv;
	const char * service_name_1 = "/set_navigation_mode";

	const rosidl_service_type_support_t * type_support_1 = ROSIDL_GET_SRV_TYPE_SUPPORT(nereo_interfaces, srv, SetNavigationMode);

	rc = rclc_service_init_default(
	  &set_nav_mode_srv,
	  &node,
	  type_support_1,
	  service_name_1
	);

	if (rc != RCL_RET_OK) {
		add_diagnostic_status(&diagnostic_values_array, "service_init", ROS2_STALE,
				"set_nav_mode_service", "Error initializing set_nav_mode_service.");
		Error_Handler();
	}

	nereo_interfaces__srv__SetNavigationMode_Request nav_mode_request_msg;
	nereo_interfaces__srv__SetNavigationMode_Response nav_mode_response_msg;

	rc = rclc_executor_add_service(
	  &executor, &set_nav_mode_srv, &nav_mode_request_msg,
	  &nav_mode_response_msg, set_nav_mode_service_callback
	);

	if (rc != RCL_RET_OK) {
		add_diagnostic_status(&diagnostic_values_array, "executor_add", ROS2_STALE,
				"set_nav_mode_service", "Error adding set_nav_mode_service to executor.");
		Error_Handler();
	}

	// controller parameter server: it takes parameter for the maximum number of PIDs possible (ie 4)
	// only uses the needed parameters for the current navigation mode.
	rclc_parameter_server_t controller_constants_parameter_server;

	const rclc_parameter_options_t contr_const_param_server_opt = {
			.notify_changed_over_dds = false,
			.max_params = 12,
			.allow_undeclared_parameters = true,
			.low_mem_mode = false
	};

	rc = rclc_parameter_server_init_with_option(&controller_constants_parameter_server, &node, &contr_const_param_server_opt);

	if (RCL_RET_OK != rc)
	{
		add_diagnostic_status(&diagnostic_values_array, "parameter_server_init", ROS2_ERROR,
				"parameter_server", "Error initializing parameter server.");
	}

	rc = rclc_executor_add_parameter_server(&executor, &controller_constants_parameter_server, NULL);
	if (RCL_RET_OK != rc)
	{
		add_diagnostic_status(&diagnostic_values_array, "parameter_server_exec_add", ROS2_ERROR,
				"parameter_server", "Error adding parameter server to executor.");
	}

	// Spin executor to receive requests
	rclc_executor_spin(&executor);

	start_pwm_channels_1_to_4(&htim2);
	start_pwm_channels_1_to_4(&htim3);
	set_pwm_idle();

    float joy_input[6] = {0};

    // need either to calculate this in the subscription callbacks, or to set it to a fixed value. In this case, 20Hz
    float integration_intervals[4] = {0.05};
	uint16_t pwm_output[8] = {1500};

	uint8_t pwm_error = 0;

    while(1)
    {
    	rc = rcutils_system_time_now(&now);
    	if(RCL_RET_OK != rc)
		{
			// TODO add to diagnostic, blink status LED?, write logs to flash?
		}
    	diagnostic_values_array.header.stamp.sec = RCUTILS_NS_TO_S(now);
    	diagnostic_values_array.header.stamp.nanosec = now % (1000*1000*1000);
    	if (is_rov_armed == ROV_ARMED)
    	{
    		// save joystick input
    		joy_input[0] = joystick_input.axes.data[0]; // sway
    		joy_input[1] =joystick_input.axes.data[1]; // forward
    		joy_input[2] =joystick_input.axes.data[3]; // heave
			joy_input[6] =joystick_input.axes.data[2]; // yaw
			switch (navigation_mode)
			{
			case NAVIGATION_MODE_MANUAL:
				pwm_error = calculate_pwm(joy_input, pwm_output);
				break;
			case NAVIGATION_MODE_STABILIZE_FULL:
				pwm_error = calculate_pwm_with_pid(joy_input, pwm_output, (Quaternion *)&imu_data.orientation, (float *) &fluid_pressure.fluid_pressure, integration_intervals);
			default:
				set_pwm_idle();
				for(uint8_t i = 0; i < 8; i++) thruster_status.thrusters_pwms[i] = 1500;
				break;
			}
			// send pwms
			for(uint8_t i = 0; i < 8; i++)
			{
				// check pwm is in range, else publish to diagnostic
			}
			set_pwms(pwm_output);
		} else
		{
			set_pwm_idle();
			for(uint8_t i = 0; i < 8; i++) thruster_status.thrusters_pwms[i] = 1500;
		}

    	// publish thruster state
    	for(uint8_t i = 0; i < 8; i++) thruster_status.thrusters_pwms[i] = pwm_output[i];
    	rc = rcl_publish(&thruster_status_publisher, &thruster_status, NULL);
    	if(RCL_RET_OK != rc)
    	{
    		// TODO add to diagnostic, blink status LED?, write logs to flash?
    	}
    	// publish diagnostic

    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void set_pwms(uint16_t pwms[8])
{
	TIM2 -> CCR1 = pwms[0];
	TIM2 -> CCR2 = pwms[1];
	TIM2 -> CCR3 = pwms[2];
	TIM2 -> CCR4 = pwms[3];

	// vertical thrusters
	TIM3 -> CCR1 = pwms[4];
	TIM3 -> CCR2 = pwms[5];
	TIM3 -> CCR3 = pwms[6];
	TIM3 -> CCR4 = pwms[7];
}

void start_pwm_channels_1_to_4(TIM_HandleTypeDef *timer_handle)
{
	HAL_TIMEx_PWMN_Start(timer_handle, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(timer_handle, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(timer_handle, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(timer_handle, TIM_CHANNEL_4);
}

void set_pwm_idle()
{
	TIM2 -> CCR1 = PWM_IDLE;
	TIM2 -> CCR2 = PWM_IDLE;
	TIM2 -> CCR3 = PWM_IDLE;
	TIM2 -> CCR4 = PWM_IDLE;

	// vertical thrusters
	TIM3 -> CCR1 = PWM_IDLE;
	TIM3 -> CCR2 = PWM_IDLE;
	TIM3 -> CCR3 = PWM_IDLE;
	TIM3 -> CCR4 = PWM_IDLE;
}

void arm_disarm_service_callback(const void * request_msg, void * response_msg)
{
  // Cast messages to expected types
  std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) request_msg;
  std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) response_msg;

  // Handle request message and set the response message values
  is_rov_armed = req_in->data ? ROV_ARMED : ROV_DISARMED;
  res_in->success = true;
  if(is_rov_armed == ROV_ARMED)
  {
	  res_in->message.data = "ROV ARMED";
  } else
  {
	  res_in->message.data = "ROV DISARMED";
  }

}

void set_nav_mode_service_callback(const void * request_msg, void * response_msg)
{
	nereo_interfaces__srv__SetNavigationMode_Request * req_in = (nereo_interfaces__srv__SetNavigationMode_Request *) request_msg;
	nereo_interfaces__srv__SetNavigationMode_Response * res_in = (nereo_interfaces__srv__SetNavigationMode_Response *) response_msg;
	switch (req_in->navigation_mode)
	{
		case NAVIGATION_MODE_MANUAL:
			navigation_mode = NAVIGATION_MODE_MANUAL;
			res_in->success = true;
			res_in->mode_after_set = NAVIGATION_MODE_MANUAL;
			res_in->message.data = "Navigation mode set to manual.";
			break;
		case NAVIGATION_MODE_STABILIZE_FULL:
			res_in->success = true;
			res_in->mode_after_set = NAVIGATION_MODE_STABILIZE_FULL;
			res_in->message.data = "Navigation mode set to stabilize: full.";
			navigation_mode = NAVIGATION_MODE_STABILIZE_FULL;
			break;
		default:
			res_in->success = false;
			res_in->mode_after_set = NAVIGATION_MODE_MANUAL;
			res_in->message.data = "Wrong request: navigation mode does not exist. Reverted back to manual mode.";
			navigation_mode = NAVIGATION_MODE_MANUAL;
			break;
	}
}

size_t add_diagnostic_status(diagnostic_msgs__msg__DiagnosticArray * array, char * hardware_id, RosErrors level, char * name, char * message)
{
	if(array->status.size >= array->status.capacity) return 0;

	array->status.data[array->status.size].hardware_id.data = hardware_id;
	array->status.data[array->status.size].level = level;
	array->status.data[array->status.size].name.data = name;
	array->status.data[array->status.size].message.data = message;
	array->status.size++;

	return array->status.size;

}
/* USER CODE END Application */
