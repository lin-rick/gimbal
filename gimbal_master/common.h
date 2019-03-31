/*******************************************************************************
* File Name: common.h
*
* Version: 1.0
*
* Description:
*  This is the header file for the common functionality in the project.
*
* Hardware Dependency:
*  CY8CKIT-042-BLE
*
********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined (_COMMON_H)
#define _COMMON_H

/*******************************************************************************
* Included headers
*******************************************************************************/

/*******************************************************************************
* Macros
*******************************************************************************/

#define MAX_MTU_SIZE									(512)
#define DEFAULT_MTU_SIZE								(64)
#define TASK_LOOP_HZ									(50)
#define SITSTAND_BUF_SIZE                               (200)

/*******************************************************************************
* Typedefs
*******************************************************************************/

typedef enum device_state_s
{
	SLEEP,
	INIT,
	ACTIVE,
	ALERT
} device_state_t;

typedef enum calib_s
{
	/* Model calibration */
	CALIB_FORWARD,
	CALIB_SIDE_LEFT,
	CALIB_SIDE_RIGHT,
	CALIB_SQUAT,
	CALIB_UPRIGHT,
	CALIB_ARRAY_SIZE,

	/* States */
	CALIB_NONE,
	CALIB_ZERO_START,
	CALIB_ZERO_RESET,
	CALIB_MODEL_START,
	CALIB_MODEL_RESET
} calib_t;

typedef enum calib_flags_s
{
	CALIB_FORWARD_RDY			= 1,
	CALIB_SIDE_LEFT_RDY			= 2,
	CALIB_SIDE_RIGHT_RDY		= 4,
	CALIB_SQUAT_RDY				= 8,
	CALIB_UPRIGHT_RDY			= 16,

	CALIB_ZERO_RDY				= 17,
	CALIB_MODEL_RDY				= 31,
	CALIB_ZERO_GENERATED		= 32,
	CALIB_MODEL_GENERATED		= 64,
} calib_flags_t;

typedef enum imu_error_s
{
	IMU_NO_ERR,
	IMU_RW_ERR,
	IMU_M_INIT_ERR,
	IMU_XG_INIT_ERR,
	IMU_MXG_INIT_ERR
} imu_error_t;

typedef struct dev_status_s {
	uint8_t notify;
	uint8_t req_sleep;
	calib_t req_calibrate;
	calib_flags_t calib_status;
	uint8_t charging;
	device_state_t device_state;
	imu_error_t imu1_error;
	imu_error_t imu2_error;
} dev_status_t;

typedef struct dev_data_s {
	int16_t ax_1, ay_1, az_1;
	int16_t ax_2, ay_2, az_2;
	float pitch, yaw, roll;
	uint8_t state, posture, leaning, bad_leaning;
	uint32_t step_count;
	uint32_t bad_posture_count;
    uint32_t leaning_count;
    uint32_t bad_posture_leaning_count;
} dev_data_t;

typedef struct dev_buffer_s {
	uint8_t in[MAX_MTU_SIZE];
	uint8_t out[MAX_MTU_SIZE];
	uint16_t negotiated_mtu;
	uint16_t msg_size;
} dev_buffer_t;

typedef struct mdl_settings_s {
	uint8_t thresh_stand[3];
	uint8_t thresh_sit[3];
	uint8_t req_model_update;
} mdl_settings_t;

typedef struct statistics_s {
	uint8_t req_stats_reset;
    uint8_t in_session;
    uint16_t curr_sess_time;
	uint16_t sit_stand_transitions[SITSTAND_BUF_SIZE];
	uint16_t num_transitions;
} statistics_t;

typedef struct dev_s {
	dev_status_t status;
	dev_data_t data;
	dev_buffer_t buffer;
	mdl_settings_t model;
	statistics_t statistics;
	int motor1_dir;
	int motor2_dir;
	int axis_lock;
} dev_t;

#endif

/* [] END OF FILE */
