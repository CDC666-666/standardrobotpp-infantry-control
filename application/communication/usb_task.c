/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "IMU.h"//包含IMU头文件
#include "IMU_task.h"//包含IMU任务头文件
#include "cmsis_os.h"
#include "data_exchange.h"
#include "gimbal.h"//包含Gimbal头文件
#include "macro_typedef.h"//包含宏定义头文件
#include "supervisory_computer_cmd.h"//包含自瞄文件
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#include "gimbal_yaw_pitch_direct.h"
extern Gimbal_s gimbal_direct;
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_Event       10  // ms
#define SEND_DURATION_Pid         10  // ms
#define SEND_DURATION_AllRobotHp  10  // ms
#define SEND_DURATION_GameStatus  10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
#define SEND_DURATION_GroundRobotPosition   10// ms
#define SEND_DURATION_RfidStatus   10// ms
#define SEND_DURATION_RobotStatus  10// ms
#define SEND_DURATION_JointState   10// ms
#define SEND_DURATION_Buff         10// ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte
#define VISION_TO_GIMBAL_FRAME_LEN ((uint16_t)sizeof(VisionToGimbal_s))

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataDebug_s       SEND_DATA_DEBUG;
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;
static SendDataEvent_s       SEND_DATA_EVENT;
static SendDataPidDebug_s    SEND_DATA_PID;
static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
static SendDataRfidStatus_s  SEND_RFID_STATUS_DATA;
static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
static SendDataBuff_s        SEND_BUFF_DATA;

//__packed__ 是为了让结构体在内存里的布局和 USB 协议帧逐字节一致，禁止编译器自动插入对齐填充字节。
//这个结构体不是普通 C 内部数据结构，而是 USB 通信协议帧，必须保证二进制布局固定。
typedef struct __packed__
{
    uint8_t head[2];
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} GimbalToVision_s;

static GimbalToVision_s SEND_GIMBAL_TO_VISION;//声明发送变量 
volatile fp32 gimbal_to_vision_yaw_now = 0.0f;
volatile fp32 gimbal_to_vision_q0_now = 0.0f;
volatile fp32 gimbal_to_vision_q1_now = 0.0f;

typedef struct __packed__
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal_s;//接受变量声明

// clang-format on

// 数据接收结构体
static VisionToGimbal_s RECEIVE_VISION_TO_GIMBAL_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Debug;
    uint32_t Imu;
    uint32_t RobotStateInfo;
    uint32_t Event;
    uint32_t Pid;
    uint32_t AllRobotHp;
    uint32_t GameStatus;
    uint32_t RobotMotion;
    uint32_t GroundRobotPosition;
    uint32_t RfidStatus;
    uint32_t RobotStatus;
    uint32_t JointState;
    uint32_t Buff;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendDebugData(void);
//static void UsbSendImuData(void);
// static void UsbSendRobotStateInfoData(void);
// static void UsbSendEventData(void);
//static void UsbSendPidData(void);
// static void UsbSendAllRobotHpData(void);
// static void UsbSendGameStatusData(void);
// static void UsbSendRobotMotionData(void);
// static void UsbSendGroundRobotPositionData(void);
// static void UsbSendRfidStatusData(void);
// static void UsbSendRobotStatusData(void);
static void UsbSendJointStateData(void);
// static void UsbSendBuffData(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

/*******************************************************************************/
/* CRC16 Function Declaration                                                  */
/*******************************************************************************/

static uint16_t calculate_crc16(const uint8_t * data, uint32_t length);
static bool verify_crc16(const uint8_t * data, uint32_t length);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();//把收到的视觉数据转换成 ROBOT_CMD_DATA
        GetVirtualRcCtrlData();//当前直接清零虚拟遥控器

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    IMU = Subscribe(IMU_NAME);                             // 获取IMU数据指针
    FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 获取底盘速度矢量指针

    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_VISION_TO_GIMBAL_DATA, 0, sizeof(VisionToGimbal_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/

    // 1.初始化调试数据包
    // 帧头部分
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }

    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/

    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;

    // 4.初始化事件数据包
    SEND_DATA_EVENT.frame_header.sof = SEND_SOF;
    SEND_DATA_EVENT.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    SEND_DATA_EVENT.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_EVENT.frame_header), sizeof(SEND_DATA_EVENT.frame_header));

    // 5.初始化pid调参数据
    SEND_DATA_PID.frame_header.sof = SEND_SOF;
    SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 6.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),
        sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));

    // 7.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),
        sizeof(SEND_DATA_GAME_STATUS.frame_header));

    // 8.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));

    // 9.初始化地面机器人位置数据
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.sof = SEND_SOF;
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.len =
        (uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_GROUND_ROBOT_POSITION_DATA.frame_header),
        sizeof(SEND_GROUND_ROBOT_POSITION_DATA.frame_header));

    // 10.初始化RFID状态数据
    SEND_RFID_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_RFID_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    SEND_RFID_STATUS_DATA.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_RFID_STATUS_DATA.frame_header),
        sizeof(SEND_RFID_STATUS_DATA.frame_header));

    // 11.初始化机器人状态数据
    SEND_ROBOT_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    SEND_ROBOT_STATUS_DATA.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_STATUS_DATA.frame_header),
        sizeof(SEND_ROBOT_STATUS_DATA.frame_header));

    // 12.初始化云台发视觉数据
    SEND_GIMBAL_TO_VISION.head[0] = 'S';
    SEND_GIMBAL_TO_VISION.head[1] = 'P';
    SEND_GIMBAL_TO_VISION.mode = 0;
    SEND_GIMBAL_TO_VISION.bullet_speed = 18.0f;
    SEND_GIMBAL_TO_VISION.bullet_count = 18;

    // 13.初始化机器人增益和底盘能量数据
    SEND_BUFF_DATA.frame_header.sof = SEND_SOF;
    SEND_BUFF_DATA.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    SEND_BUFF_DATA.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_BUFF_DATA.frame_header),
        sizeof(SEND_BUFF_DATA.frame_header));
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 发送Debug数据
    //CheckDurationAndSend(Debug);
    // // 发送Imu数据
    //CheckDurationAndSend(Imu);
    // // 发送RobotStateInfo数据
    // CheckDurationAndSend(RobotStateInfo);
    // // 发送Event数据
    // CheckDurationAndSend(Event);
    // // 发送PidDebug数据
    //CheckDurationAndSend(Pid);
    // // 发送AllRobotHp数据
    // CheckDurationAndSend(AllRobotHp);
    // // 发送GameStatus数据
    // CheckDurationAndSend(GameStatus);
    // // 发送RobotMotion数据
    // CheckDurationAndSend(RobotMotion);
    // // 发送GroundRobotPosition数据
    // CheckDurationAndSend(GroundRobotPosition);
    // // 发送RfidStatus数据
    // CheckDurationAndSend(RfidStatus);
    // // 发送RobotStatus数据
    // CheckDurationAndSend(RobotStatus);
    // // 发送JointState数据
    CheckDurationAndSend(JointState); //调用 UsbSendJointStateData()
    // // 发送Buff数据UsbSendJointStateData
    // CheckDurationAndSend(Buff);
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint16_t cache_len = 0;
    uint32_t recv_len = USB_RECEIVE_LEN - cache_len;
    uint16_t parse_idx = 0;

    USB_Receive(USB_RX_BUF + cache_len, &recv_len);
    if (recv_len > (USB_RX_DATA_SIZE - cache_len)) {
        recv_len = USB_RX_DATA_SIZE - cache_len;
    }
    cache_len += (uint16_t)recv_len;

    while ((parse_idx + VISION_TO_GIMBAL_FRAME_LEN) <= cache_len) {
        if (USB_RX_BUF[parse_idx] == 'S' && USB_RX_BUF[parse_idx + 1] == 'P') {
            uint8_t * frame = &USB_RX_BUF[parse_idx];

            // 使用查表法验证CRC16
            if (verify_crc16(frame, VISION_TO_GIMBAL_FRAME_LEN)) {
                memcpy(&RECEIVE_VISION_TO_GIMBAL_DATA, frame, sizeof(VisionToGimbal_s));
                RECEIVE_TIME = HAL_GetTick();
                parse_idx += VISION_TO_GIMBAL_FRAME_LEN;
                continue;
            }
        }
        parse_idx++;
    }

    if (parse_idx < cache_len) {
        memmove(USB_RX_BUF, USB_RX_BUF + parse_idx, cache_len - parse_idx);
        cache_len -= parse_idx;
    } else {
        cache_len = 0;
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
    USB_Transmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
}

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = IMU->angle[AX_Z];
    SEND_DATA_IMU.data.pitch = IMU->angle[AX_Y];
    SEND_DATA_IMU.data.roll = IMU->angle[AX_X];

    SEND_DATA_IMU.data.yaw_vel = IMU->gyro[AX_Z];
    SEND_DATA_IMU.data.pitch_vel = IMU->gyro[AX_Y];
    SEND_DATA_IMU.data.roll_vel = IMU->gyro[AX_X];

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = HAL_GetTick();

    append_CRC16_check_sum(
        (uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}

/**
 * @brief 发送事件数据
 * @param duration 发送周期
 */
static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
    USB_Transmit((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
}

/**
 * @brief 发送PidDubug数据
 * @param duration 发送周期
 */
static void UsbSendPidData(void)
{
    // Use a local snapshot to avoid CRC/data mismatch caused by concurrent updates.
    SendDataPidDebug_s pid_snapshot;
    memcpy(&pid_snapshot, &SEND_DATA_PID, sizeof(SendDataPidDebug_s));
    pid_snapshot.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&pid_snapshot, sizeof(SendDataPidDebug_s));
    USB_Transmit((uint8_t *)&pid_snapshot, sizeof(SendDataPidDebug_s));
}

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    SEND_DATA_ALL_ROBOT_HP.data.red_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.red_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.red_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.red_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.red_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.red_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.red_base_hp = 9;
    SEND_DATA_ALL_ROBOT_HP.data.blue_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.blue_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.blue_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.blue_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.blue_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.blue_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.blue_base_hp = 9;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_progress = 1;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = 100;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

/**
 * @brief 发送地面机器人位置数据
 * @param duration 发送周期
 */
static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum(
        (uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
    USB_Transmit(
        (uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
    USB_Transmit((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
}

/**
 * @brief 发送机器人状态数据
 * @param duration 发送周期
 */
static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
}

/**
 * @brief 发送云台状态数据
 * @param duration 发送周期
 */
static void UsbSendJointStateData(void)
{
    const fp32 * quat = get_INS_quat_point();

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
    SEND_GIMBAL_TO_VISION.mode = (uint8_t)gimbal_direct.mode;
#else
    SEND_GIMBAL_TO_VISION.mode = 0;
#endif

    if (quat != NULL) {
        SEND_GIMBAL_TO_VISION.q[0] = quat[0];//INS 四元数
        SEND_GIMBAL_TO_VISION.q[1] = quat[1];
        SEND_GIMBAL_TO_VISION.q[2] = quat[2];
        SEND_GIMBAL_TO_VISION.q[3] = quat[3];
    } else {
        SEND_GIMBAL_TO_VISION.q[0] = 1.0f;
        SEND_GIMBAL_TO_VISION.q[1] = 0.0f;
        SEND_GIMBAL_TO_VISION.q[2] = 0.0f;
        SEND_GIMBAL_TO_VISION.q[3] = 0.0f;
    }
    gimbal_to_vision_q0_now = SEND_GIMBAL_TO_VISION.q[0];
    gimbal_to_vision_q1_now = SEND_GIMBAL_TO_VISION.q[1];
    gimbal_to_vision_yaw_now = SEND_GIMBAL_TO_VISION.yaw;

    SEND_GIMBAL_TO_VISION.yaw = CmdGimbalJointState(AX_YAW);
    SEND_GIMBAL_TO_VISION.yaw_vel = GetImuVelocity(AX_YAW);
    SEND_GIMBAL_TO_VISION.pitch = CmdGimbalJointState(AX_PITCH);
    SEND_GIMBAL_TO_VISION.pitch_vel = GetImuVelocity(AX_PITCH);
    SEND_GIMBAL_TO_VISION.bullet_speed = 22.0f;
    SEND_GIMBAL_TO_VISION.bullet_count = 18;

    {
        // 使用查表法计算CRC16，发送数据时附带 CRC16，收到数据后重新计算 CRC16，如果两个 CRC16 一样，通常说明数据没被破坏。
        uint16_t crc =
            calculate_crc16((uint8_t *)&SEND_GIMBAL_TO_VISION, sizeof(GimbalToVision_s) - 2);
        // 存储CRC（低字节在前，高字节在后）
        SEND_GIMBAL_TO_VISION.crc16 = crc;
           }

    USB_Transmit((uint8_t *)&SEND_GIMBAL_TO_VISION, sizeof(GimbalToVision_s));
}

/**
 * @brief 发送机器人增益和底盘能量数据
 * @param duration 发送周期
 */
static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    USB_Transmit((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
}

/*******************************************************************************/
/* CRC16 校验函数 - 与上位机tools/crc.cpp完全一致                              */
/*******************************************************************************/

// CRC16查表
const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief      计算CRC16校验值（查表法）
 * @param[in]  data: 数据指针
 * @param[in]  length: 数据长度
 * @retval     CRC16校验值
 */
static uint16_t calculate_crc16(const uint8_t * data, uint32_t length)
{
    uint16_t crc16 = 0xFFFF;
    uint32_t i;

    for (i = 0; i < length; i++) {
        uint8_t byte = data[i];
        uint8_t index = (uint8_t)((crc16 ^ byte) & 0x00FF);
        crc16 = (uint16_t)((crc16 >> 8) ^ CRC16_TABLE[index]);
    }

    return crc16;
}

/**
 * @brief      验证CRC16校验
 * @param[in]  data: 数据指针
 * @param[in]  length: 数据长度
 * @retval     校验结果
 */
static bool verify_crc16(const uint8_t * data, uint32_t length)
{
    if (data == NULL || length < sizeof(uint16_t)) {
        return false;
    }

    uint16_t received_crc = ((uint16_t)data[length - 1] << 8) | data[length - 2];
    uint16_t calculated_crc = calculate_crc16(data, length - sizeof(uint16_t));

    return (received_crc == calculated_crc);
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    // CRC only checks transport integrity; reject invalid commands before any control use.
    if (RECEIVE_VISION_TO_GIMBAL_DATA.mode == 0 ||
        RECEIVE_VISION_TO_GIMBAL_DATA.mode > 2 ||
        !isfinite(RECEIVE_VISION_TO_GIMBAL_DATA.yaw) ||
        !isfinite(RECEIVE_VISION_TO_GIMBAL_DATA.pitch) ||
        !isfinite(RECEIVE_VISION_TO_GIMBAL_DATA.yaw_vel) ||
        !isfinite(RECEIVE_VISION_TO_GIMBAL_DATA.pitch_vel) ||
        !isfinite(RECEIVE_VISION_TO_GIMBAL_DATA.yaw_acc) ||
        !isfinite(RECEIVE_VISION_TO_GIMBAL_DATA.pitch_acc)) {
        ROBOT_CMD_DATA.gimbal.yaw = CmdGimbalJointState(AX_YAW);
        ROBOT_CMD_DATA.gimbal.pitch = CmdGimbalJointState(AX_PITCH);
        ROBOT_CMD_DATA.gimbal.yaw_vel = 0.0f;
        ROBOT_CMD_DATA.gimbal.yaw_acc = 0.0f;
        ROBOT_CMD_DATA.gimbal.pitch_vel = 0.0f;
        ROBOT_CMD_DATA.gimbal.pitch_acc = 0.0f;
        ROBOT_CMD_DATA.shoot.fire = false;
        ROBOT_CMD_DATA.shoot.fric_on = false;
        return;
    }

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_VISION_TO_GIMBAL_DATA.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_VISION_TO_GIMBAL_DATA.pitch;
    ROBOT_CMD_DATA.gimbal.yaw_vel = RECEIVE_VISION_TO_GIMBAL_DATA.yaw_vel;
    ROBOT_CMD_DATA.gimbal.yaw_acc = RECEIVE_VISION_TO_GIMBAL_DATA.yaw_acc;
    ROBOT_CMD_DATA.gimbal.pitch_vel = RECEIVE_VISION_TO_GIMBAL_DATA.pitch_vel;
    ROBOT_CMD_DATA.gimbal.pitch_acc = RECEIVE_VISION_TO_GIMBAL_DATA.pitch_acc;
    ROBOT_CMD_DATA.shoot.fire = (RECEIVE_VISION_TO_GIMBAL_DATA.mode == 2);
    ROBOT_CMD_DATA.shoot.fric_on = true;
}

static void GetVirtualRcCtrlData(void) { memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t)); }

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 获取USB连接状态
 * @return true: USB离线, false: USB在线
 */
inline uint8_t GetUsbStatus(void) { return (uint8_t)USB_OFFLINE; }

/**
 * @brief 修改调试数据包
 * @param index 索引
 * @param data  发送数据
 * @param name  数据名称
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char * name)
{
    SEND_DATA_DEBUG.packages[index].data = data;

    if (SEND_DATA_DEBUG.packages[index].name[0] != '\0') {
        return;
    }

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        SEND_DATA_DEBUG.packages[index].name[i] = name[i];
        i++;
    }

    //TODO:添加对数据名称的一些检查工作
}

void ModifyPidDebugDataPackage(float fdb, float ref, float pid_out)
{
    SEND_DATA_PID.data.fdb = fdb;
    SEND_DATA_PID.data.ref = ref;
    SEND_DATA_PID.data.pid_out = pid_out;
}

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
inline float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return ROBOT_CMD_DATA.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return ROBOT_CMD_DATA.gimbal.pitch;
    }
    return 0.0f;
}

/** @brief 获取上位机云台轨迹速度参考，单位 rad/s */
inline float GetScCmdGimbalVelocity(uint8_t axis)
{
    if (axis == AX_YAW) return ROBOT_CMD_DATA.gimbal.yaw_vel;
    if (axis == AX_PITCH) return ROBOT_CMD_DATA.gimbal.pitch_vel;
    return 0.0f;
}

/**
 * @brief 获取上位机云台轨迹加速度，单位 rad/s^2
 */
inline float GetScCmdGimbalAcceleration(uint8_t axis)
{
    if (axis == AX_YAW) return ROBOT_CMD_DATA.gimbal.yaw_acc;
    if (axis == AX_PITCH) return ROBOT_CMD_DATA.gimbal.pitch_acc;
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X) {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } else if (axis == AX_Y) {
        return ROBOT_CMD_DATA.speed_vector.vy;
    } else if (axis == AX_Z) {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z) {
        return ROBOT_CMD_DATA.speed_vector.wz;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void) { return ROBOT_CMD_DATA.chassis.leg_length; }

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
inline bool GetScCmdFire(void) { return ROBOT_CMD_DATA.shoot.fire; }

/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
inline bool GetScCmdFricOn(void) { return ROBOT_CMD_DATA.shoot.fric_on; }
/*------------------------------ End of File ------------------------------*/
