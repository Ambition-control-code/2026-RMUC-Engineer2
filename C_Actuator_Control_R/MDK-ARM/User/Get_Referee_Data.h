#ifndef __GET_REFEREE_DATA_H__
#define __GET_REFEREE_DATA_H__

#include "stm32f4xx.h"                // Device header

#define REFEREE_RXFRAME_LENGTH  136   //frame_header5字节，cmd_id2字节，data_max127字节，crc16 2bytes=136字节

/**
 * @brief 裁判系统通信协议格式
 */
#define FrameHeader_Length    5U      /*!< the length of frame header 	协议帧头部（Frame Header）的长度为5字节*/
#define CMDID_Length          2U      /*!< the length of CMD ID       	命令标识符（Command ID）的长度为2字节***/
#define CRC16_Length          2U      /*!< the length of CRC ID CRC16 	校验码的长度为2字节*******************/

/**
 * @brief Cmd id  命令码ID
 * @remeber 详细的信息去翻看大疆裁判系统串口协议，有每个ID对应所传数据类型和数据长度，需要细读，研读，方可磨刀不误砍柴工！！！
 * @know 如果没有看错的话，以下所有的ID除了0x0302是图传链路，其余所有ID均是常规链路！！！
 * @must 对于图传链路的宏定义这里只写了使用的自定义控制器，其余想加可以在仔细阅读理解裁判系统通讯协议后加，小心引发雪崩！！！
 */
#define GAME_STATUS_ID                    0x0001U  /* game status data 比赛状态数据*/
#define GAME_RESULT_ID                    0x0002U  /* game result data 比赛结果数据*/
#define GAME_ROBOT_HP_ID                  0x0003U  /* robot HP data 机器人血量数据*/
#define EVENET_DATA_ID                    0x0101U  /* site event data 场地事件数据*/
#define REFEREE_WARNING_ID                0x0104U  /* referee warning data 裁判警告数据，己方判罚/判负时触发发送*/
#define DART_INFO_ID                      0x0105U  /* dart shoot data 飞镖发射相关数据*/
#define ROBOT_STATUS_ID                   0x0201U  /* robot status data 机器人性能体系数据*/
#define POWER_HEAT_DATA_ID                0x0202U  /* real power heat data 实时底盘缓冲能量和射击热量数据*/
#define ROBOT_POS_ID                      0x0203U  /* robot position data 机器人位置数据*/
#define BUFF_ID                           0x0204U  /* robot buff data 机器人增益和底盘能量数据*/
#define HURT_DATA_ID                      0x0206U  /* robot hurt data 伤害状态数据，伤害发生后发送*/
#define SHOOT_DATA_ID                     0x0207U  /* real robo t shoot data 实时射击数据，弹丸发射后发送*/
#define PROJECTILE_ALLOWANCE_ID           0x0208U  /* bullet remain data 允许发弹量*/
#define RFID_STATUS_ID                    0x0209U  /* RFID status data 机器人 RFID 模块状态*/
#define DART_CLIENT_CMD_ID                0x020AU  /* DART Client cmd data 飞镖选手端指令数据*/
#define GROUND_ROBOT_POSITION_ID          0x020BU  /* ground robot position 地面机器人位置数据*/
#define RADAR_MARAKING_DATA_ID            0x020CU  /* Radar marking progress 雷达标记进度数据*/
#define SENTRY_INFO_ID                    0x020DU  /* SENTRY make autonomous decisions 哨兵自主决策信息同步*/
#define RADAR_INFO_ID                     0x020EU  /* RADAR make autonomous decisions 雷达自主决策信息同步*/
#define ROBOT_INTERACTION_DATA_ID         0x0301U  /* robot interactive data 机器人交互数据，发送方触发发送*/
#define MAP_COMMAND_ID                    0x0303U  /* mini map interactive data 选手端小地图交互数据，选手端触发发送*/
#define MAP_ROBOT_DATA_ID                 0x0305U  /* mini map receive data 选手端小地图接收雷达数据*/
#define MAP_DATA_ID                       0x0307U  /* mini map Auto Robot path 选手端小地图接收哨兵数据*/
#define CUSTOM_INFO_ID                    0x0308U  /* mini map robot  path 选手端小地图接收机器人数据*/
//-------------------------以上是常规链路的命令码ID,以下是图传链路的命令码ID-------------------------------------------------//
#define custom_controller_ID              0x0302U  /* robot interactive data 自定义控制器与机器人交互数据，发送方触发发送*/
#define Keymouse_Control_ID               0X0304u  /* Keyboard and Mouse Remote Control Data 图传链路的键鼠遥控控制数据*/
//---------------------------------------------------------------------------------------------------------------------//


//------以下内容是什么本人尚未参透，不理解没有用但是写出来，但是大家都写在这里了，然后代码中并没有使用（可能查询的不仔细），需要注意！！！----------//
/**
 * @brief Robot id
 */
#define ROBOT_RED_HERO_ID                 0x0001U
#define ROBOT_RED_ENGINEER_ID             0x0002U
#define ROBOT_RED_3_INFANTEY_ID           0x0003U
#define ROBOT_RED_4_INFANTEY_ID           0x0004U
#define ROBOT_RED_5_INFANTEY_ID           0x0005U
#define ROBOT_RED_AERIAL_INFANTEY_ID      0x0006U
#define ROBOT_RED_SENTEY_INFANTEY_ID      0x0007U
#define ROBOT_RED_DART_INFANTEY_ID        0x0008U
#define ROBOT_RED_RADAR_INFANTEY_ID       0x0009U
#define ROBOT_RED_OUTPOST_INFANTEY_ID     0x0010U
#define ROBOT_RED_BASE_INFANTEY_ID        0x0011U

#define ROBOT_BLUE_HERO_ID                0x0101U
#define ROBOT_BLUE_ENGINEER_ID            0x0102U
#define ROBOT_BLUE_3_INFANTEY_ID          0x0103U
#define ROBOT_BLUE_4_INFANTEY_ID          0x0104U
#define ROBOT_BLUE_5_INFANTEY_ID          0x0105U
#define ROBOT_BLUE_AERIAL_INFANTEY_ID     0x0106U
#define ROBOT_BLUE_SENTEY_INFANTEY_ID     0x0107U
#define ROBOT_BLUE_DART_INFANTEY_ID       0x0108U
#define ROBOT_BLUE_RADAR_INFANTEY_ID      0x0109U
#define ROBOT_BLUE_OUTPOST_INFANTEY_ID    0x0110U
#define ROBOT_BLUE_BASE_INFANTEY_ID       0x0111U

/**
 * @brief client id
 */
#define CLIENT_RED_HERO_ID                0x0101U
#define CLIENT_RED_ENGINEER_ID            0x0102U
#define CLIENT_RED_3_INFANTEY_ID          0x0103U
#define CLIENT_RED_4_INFANTEY_ID          0x0104U
#define CLIENT_RED_5_INFANTEY_ID          0x0105U
#define CLIENT_RED_AERIAL_INFANTEY_ID     0x0106U

#define CLIENT_BLUE_HERO_ID               0x0165U
#define CLIENT_BLUE_ENGINEER_ID           0x0166U
#define CLIENT_BLUE_3_INFANTEY_ID         0x0167U
#define CLIENT_BLUE_4_INFANTEY_ID         0x0168U
#define CLIENT_BLUE_5_INFANTEY_ID         0x0169U
#define CLIENT_BLUE_AERIAL_INFANTEY_ID    0x016AU

/* Exported types ------------------------------------------------------------*/
/* cancel byte alignment */
#pragma  pack(1)

//----------------------------------------------------------------------------------------------------------------//

//-----------------------------------以下开始结构体内容-------------------------------------------------------//
/**
 * @brief typedef structure that contains the information of frame header 
 * @translate 包含帧头信息的类型定义结构体
 */
typedef struct 
{
  uint8_t  SOF;           /*!< Data frame start byte, fixed value is 0xA5 	数据帧起始字节，固定值为 0xA5*/
  uint16_t Data_Length;   /*!< the length of data in the data frame       	数据帧中数据的长度*/
  uint8_t  Seq;           /*!< package serial number                      	包序列号*/
  uint8_t  CRC8;          /*!< Frame header CRC8 checksum                 	帧头的 CRC8 校验和*/
}FrameHeader_TypeDef;

/**
 * @brief typedef structure that contains the information of game status, id: 0x0001U 
 * @translate 包含游戏状态信息的类型定义结构体，标识符：0x0001U
 */
typedef struct          
{
	/**
	* @brief the type of game,
						1:RMUC,（RoboMaster 大学组）
						2:RMUT,（RoboMaster 职业组）
						3:RMUA,（RoboMaster 国际组）
						4:RMUL,3v3,（RoboMaster 青少年组）3V3
						5:RMUL,1v1,（RoboMaster 青少年组）1V1
	*/
	uint8_t game_type : 4;         /*                                		游戏类型（4位位域）*/
	uint8_t game_progress : 4;	   /*!< the progress of game         		游戏进程（4位位域）*/
	uint16_t stage_remain_time;	   /*!< remain time of real progress 		当前阶段剩余时间*/
	uint64_t SyncTimeStamp;        /*!< unix time                     	同步时间戳（Unix时间格式）*/
       
}game_status_t;

/**
 * @brief typedef structure that contains the information of game result, id: 0x0002U
 * @translate 这个结构体包含比赛结果的信息，ID号为0x0002
 */
typedef struct
{
  /**
   * @brief the result of game   比赛结果
            0:draw               平局
            1:Red wins           红方胜
            2:Blue wins          蓝方胜
   */
 uint8_t winner;                 //胜者
}game_result_t;

/**
 * @brief typedef structure that contains the information of robot HP data, id: 0x0003U
 * @translate 这个结构体包含机器人血量的数据信息，ID为0x0003
 */
typedef struct
{
  uint16_t red_1_robot_HP;    /*!< Red Hero HP       	红方英雄血量*/
  uint16_t red_2_robot_HP;    /*!< Red Engineer HP   	红方工程血量*/
  uint16_t red_3_robot_HP;    /*!< Red 3 Infantry HP 	红方三号步兵血量*/
  uint16_t red_4_robot_HP;    /*!< Red 4 Infantry HP 	红方四号步兵血量*/
  uint16_t red_reserved;      /*                      红方预留字段，未使用*/
  uint16_t red_7_robot_HP;    /*!< Red Sentry HP      红方AI哨兵血量*/
  uint16_t red_outpost_HP;    /*!< Red Outpost HP     红方前哨站血量*/
  uint16_t red_base_HP;       /*!< Red Base HP        红方基地血量*/

  uint16_t blue_1_robot_HP;   /*!< Blue Hero HP       蓝方英雄血量*/
  uint16_t blue_2_robot_HP;   /*!< Blue Engineer HP   蓝方工程血量*/
  uint16_t blue_3_robot_HP;   /*!< Blue 3 Infantry HP 蓝方三号步兵血量*/
  uint16_t blue_4_robot_HP;   /*!< Blue 4 Infantry HP 蓝方四号步兵血量*/
  uint16_t blue_reserved;     /*                      蓝方预留字段，未使用*/
  uint16_t blue_7_robot_HP;   /*!< Blue Sentry HP     蓝方哨兵血量*/
  uint16_t blue_outpost_HP;   /*!< Blue Outpost HP    蓝方前哨站血量*/
  uint16_t blue_base_HP;      /*!< Blue Base HP       蓝方基地血量*/
}game_robot_HP_t;

/**
 * @brief typedef structure that contains the information of site event data, id: 0x0101U
 * @translate 包含场上事件数据的结构体，ID为0x0101
 */
typedef union
{
  /**
   * @brief the event of site   场地事件
	          bits 0-2:
            bit 0:  Status of the supply zone that does not overlap with the exchange zone, 1 for occupied
						位 0 ：  与交换区不重叠的补给区状态，1表示被占领
            bit 1:  Status of the supply zone that overlaps with the exchange zone, 1 for occupied
						位 1 ：  与交换区重叠的补给区状态，1表示被占领
            bit 2:  Status of the supply zone, 1 for occupied (applicable only to RMUL)
						位 2 ：  补给区状态，1表示被占领（仅适用于RMUL赛事）
	
	          bits 3-5: Status of the energy mechanism:
						位 3-5 ：  能量机制状态：
            bit 3:    Status of the small energy mechanism, 1 for activated
						位 3 ：    小能量机关状态，1表示已激活
            bit 4:    Status of the large energy mechanism, 1 for activated
						位 4 ：    大能量机关状态，1表示已激活
            bit 5-6:  Status of the central highland, 1 for occupied by own side, 2 for occupied by the opponent
						位 5-6 ：  中央高地状态，1表示被己方占领，2表示被敌方占领
            bit 7-8:  Status of the trapezoidal highland, 1 for occupied
						位 7-8 ：  梯形高地状态，1表示被占领
            bit 9-17: Time of the opponent's last dart hit on the own side's outpost or base (0-420, default is 0 at the start)
						位 9-17 ： 敌方最后一次用飞镖击中己方前哨站或基地的时间（0-420，开局默认0）
	          bit 18-20:  Specific target of the opponent's last dart hit on the own side's outpost or base, default is 0 at the start, 
	                      1 for hitting the outpost, 2 for hitting the fixed target on the base, 3 for hitting the random fixed target on the base, 
	                      4 for hitting the random moving target on the base
						位 18-20 ：  敌方最后一次用飞镖击中己方前哨站或基地的具体目标，开局默认0，
                        1表示击中前哨站，2表示击中基地固定目标，
												3表示击中基地随机固定目标，4表示击中基地随机移动目标						
	          bit 21-22: Status of the central gain point, 0 for unoccupied, 1 for occupied by own side, 
	                     2 for occupied by the opponent, 3 for occupied by both sides (applicable only to RMUL)  
						位 21-22：  中央增益点状态，0表示未被占领，1表示被己方占领，2表示被敌方占领，3表示双方共同占领（仅适用于RMUL赛事）
            bit 23-31: Reserved
						位 23-31 ： 预留位
                      
  */
	uint32_t  event_data;
}event_data_t;

/**
 * @brief typedef structure that contains the warning  of Referee , id: 0x0104U
 * @translate 包含裁判警告的结构体，ID为0x0104
 */
typedef struct 
{ 
	/**
   * @brief the type of game,     	警告类型
            1: Double Yellow Card,	双黄牌
            2: Yellow Card,        	黄牌
            3:RMUA,									RMUA（可能为特定赛事的警告类型，需结合协议文档确认）
            4:RMUL,3v3,							RMUL 3v3（青少年组3v3赛事的警告类型）
            5:RMUL,1v1,							RMUL 1v1（青少年组1v1赛事的警告类型）
   */
  uint8_t level;                    //警告等级/类型
  uint8_t offending_robot_id;       //犯规机器人ID
  uint8_t count;  								  //警告次数
}referee_warning_t; 

/**
 * @brief typedef structure that contains the information of dart, id: 0x0105U
 * @translate 包含飞镖信息的结构体，ID为0x0105
 */
typedef  struct
{
 
 uint8_t dart_remaining_time;/* The remaining time for our side's dart launcher, in seconds.己方飞镖发射器的剩余可用时间，单位为秒*/
 
 uint16_t dart_info;
  /**
   * @brief dart_info			飞镖信息
   *       bit 0-2 The most recent target hit by our side's dart is defaulted to 0 at the start, 
	                 where 1 indicates hitting the outpost, 2 indicates hitting the base's fixed target, 
	                 3 indicates hitting the base's random fixed target, and 4 indicates hitting the base's random moving target.
					 位 0-2 ：己方飞镖最近一次击中的目标，开局默认值为0，
	                 其中1表示击中前哨站，2表示击中基地固定目标，
	                 3表示击中基地随机固定目标，4表示击中基地随机移动目标。
   *       bit 3-5 The cumulative hit count of the opponent's recently hit target (defaulting to 0 at the start, with a maximum of 4)
					 位 3-5 ：敌方最近被击中目标的累计命中次数（开局默认值为0，最大值为4）
   *       bit 6-7 The currently selected target for the dart (defaulting to 0 at the start or when not selected/selecting the outpost, 
	                 1 for selecting the base's fixed target, 2 for selecting the base's random fixed target, 
									 3 for selecting the base's random moving target)
					 位 6-7 ：当前飞镖选定的目标（开局时或未选择目标/选择前哨站时默认值为0，
	                 1表示选择基地固定目标，2表示选择基地随机固定目标，3表示选择基地随机移动目标）
   *       bit 8-15 Reserved
					 位 8-15 ：预留位
   */
}dart_info_t;

/**
 * @brief typedef structure that contains the information of robot status, id: 0x0201U
 * @translate 包含机器人状态信息的结构体，ID为0x0201
 */
typedef struct
{
  /**
   * @brief      robot id				          机器人ID
   *             0: robot none					  没有此机器人
   *             1: red hero              红方英雄
   *             2: red engineer          红方工程
   *         3/4/5: red infantry          红方三号，四号，五号步兵
   *             6: red aerial            红方空中机器人
   *             7: red sentry            红方哨兵
   *             8: red dart              红方飞镖
   *             9: red radar station     红方雷达
   *           101: blue hero             蓝方英雄
   *           102: blue engineer         蓝方工程
   *   103/104/105: blue infantry         蓝方三号，四号，五号步兵
   *           106: blue aerial           蓝方空中机器人
   *           107: blue sentry           蓝方哨兵
   *           108: blue dart             蓝方飞镖
   *           109: blue radar station    蓝方雷达
   */
  uint8_t robot_id;                       //机器人ID
  uint8_t robot_level;                    //机器人等级
  uint16_t current_HP;                    //当前血量
  uint16_t maximum_HP;                    //最大血量

  uint16_t shooter_barrel_cooling_value;  //发射装置炮管冷却值
  uint16_t shooter_barrel_heat_limit;     //发射装置炮管热量上限
  uint16_t chassis_power_limit;           //底盘功率限制

  uint8_t mains_power_gimbal_output : 1;  //主电源云台输出（1位位域）
  uint8_t mains_power_chassis_output : 1; //主电源底盘输出（1位位域）
  uint8_t mains_power_shooter_output : 1; //主电源发射机构输出（1位位域）
} robot_status_t;

/**
 * @brief typedef structure that contains the information of power heat data, id: 0x0202U
 * @translate 包含能量和热量数据信息的类型定义结构体，ID为0x0202
 */
typedef struct
{
  uint16_t buffer_energy;                 //缓冲能量
  uint16_t shooter_17mm_1_barrel_heat;    //17mm口径发射装置1号炮管热量
  uint16_t shooter_17mm_2_barrel_heat;    //17mm口径发射装置2号炮管热量
  uint16_t shooter_42mm_barrel_heat;      //42mm口径发射装置炮管热量
} power_heat_data_t;

/**
 * @brief typedef structure that contains the information of robot position data, id: 0x0203U
 * @translate 包含机器人位置信息的结构体，ID为0x0203
 */
typedef struct
{
  float x;      /* position x coordinate, unit: m 	位置x坐标，单位：米*/
  float y;      /* position y coordinate, unit: m 	位置y坐标，单位：米*/
  float angle;  /* Position muzzle, unit: degrees 	炮口朝向角度，单位：度*/
} robot_pos_t;

/**
 * @brief typedef structure that contains the information of robot buff data, id: 0x0204U
 * @translate 包含机器人buff增益信息数据的结构体，ID为0x0204
 */
typedef struct
{
   uint8_t recovery_buff;       /* Robot healing gain (percentage, value of 10 represents 10% of the maximum healing volume per second) */
																/* 机器人治疗增益（百分比，值为10表示每秒恢复最大生命值的10%） */
   uint8_t cooling_buff;        /* Robot shooting heat cooling rate (direct value, a value of 5 indicates 5 times cooling; the fixed heat cooling gain provided by the fortress gain point is temporarily not applicable) */
																/* 机器人射击热量冷却速率（直接数值，值为5表示冷却速度为5倍；要塞增益点提供的固定热量冷却增益暂不适用） */
	 uint8_t defence_buff;        /* Robot defense gain (percentage, a value of 50 represents a 50% defense gain) */
																/* 机器人防御增益（百分比，值为50表示50%的防御增益） */
	 uint8_t vulnerability_buff;  /* Robot negative defense gain (percentage, a value of 30 represents -30% defense gain) */
																/* 机器人负面防御增益（百分比，值为30表示-30%的防御增益） */
	 uint16_t attack_buff;        /* Robot attack gain (percentage, a value of 50 represents 50% attack gain) */
																/* 机器人攻击增益（百分比，值为50表示50%的攻击增益） */
	 /**
	 * @brief remaining_energy    // 剩余能量
	 *	bit0-4: Feedback on the remaining energy value of the robot, identifying the proportion of the remaining energy value of the robot in hexadecimal, only
	 *	Feedback when the remaining energy of the robot is less than 50%, and default feedback for the rest is 0x32.
	    位0-4：机器人剩余能量值反馈，以十六进制标识机器人剩余能量值的比例，仅当机器人剩余能量小于50%时反馈，其余情况默认反馈0x32。
	 *	bit0: When the remaining energy is greater than 50%, it is 1, and in other cases, it is 0
	    位0 ： 当剩余能量大于50%时为1，其他情况为0
	 *	bit 1: When the remaining energy is greater than 30%, it is 1, and in other cases, it is 0
	    位 1 ：当剩余能量大于30%时为1，其他情况为0
	 *	bit2: When the remaining energy is greater than 15%, it is 1, and in other cases, it is 0
	    位 2 ：当剩余能量大于15%时为1，其他情况为0
	 *	bit3: When the remaining energy is greater than 5%, it is 1, and in other cases it is 0 Bit4: when the remaining energy is greater than 1%
	    位 3 ：当剩余能量大于5%时为1，其他情况为0
	 *	When is 1, in other cases it is 0
	    位 4 ：当剩余能量大于1%时为1，其他情况为0
	 */
	 uint8_t remaining_energy;
}buff_t;

/**
 * @brief typedef structure that contains the information of robot hurt, id: 0x0206U
 * @translate 包含机器人损伤信息的结构体，ID为0x0206
 */
typedef struct
{
 uint8_t armor_id : 4; /* hurt armor id */
  /**
   * @brief hurt type              损伤类型
   *        0: armor hurt          装甲板受损
   *        1: module offline      模块离线
   *        2: over fire rate      超射速
   *        3: over fire heat      发射机构过热
   *        4: over chassis power  超底盘功率
   *        5: armor bump          装甲碰撞
   */
 uint8_t HP_deduction_reason : 4;  //血量扣除的原因
}hurt_data_t;

/**
 * @brief typedef structure that contains the information of real shoot data, id: 0x0207U
 * @translate 包含实际射击数据信息的类型定义结构体，标识符ID：0x0207U
 */
typedef  struct
{
  uint8_t bullet_type;          /* 1:17mm 2:42mm 1:17毫米弹 2:42毫米弹*/
  uint8_t shooter_number; 
	/* 
	     1:First 17mm Shoot       第一个17毫米发射器
	     2:Second 17mm Shoot      第二个17毫米发射器
	     3:42mm Shoot             42毫米发射器
	*/
  uint8_t launching_frequency;  /* Hz 发射频率*/
  float initial_speed;          /* m/s 初始速度，单位：米/秒（m/s）*/
}shoot_data_t;

/**
 * @brief typedef structure that contains the information of bullet remaining number, id: 0x0208U
 * @translate 包含剩余子弹数量信息的类型定义结构体，标识符：0x0208U
 */
typedef  struct
{
  uint16_t projectile_allowance_17mm;  	/* 17毫米子弹余量 */
  uint16_t projectile_allowance_42mm;  	/* 42毫米子弹余量 */
  uint16_t remaining_gold_coin;        	/* 剩余金币数量 */
}projectile_allowance_t;

/**
 * @brief typedef structure that contains the information of RFID status, id: 0x0209U
 * @translate 包含RFID状态信息的类型定义结构体，标识符：0x0209U
 * @teach RFID技术常被用于标识场地特定区域（如补给区、得分区）或机器人身份，该结构体的作用是将RFID相关的状态信息标准化，
          便于裁判系统与机器人之间的数据交互
 */
typedef struct
{
 uint32_t rfid_status;                /* RFID状态 */
}rfid_status_t;

/**
 * @brief typedef structure that contains the information of dart client data, id: 0x020AU
 * @translate 包含飞镖客户端数据信息的类型定义结构体，标识符：0x020AU
 */
typedef  struct
{
 uint8_t dart_launch_opening_status;      /* 飞镖发射口状态 */
 uint8_t reserved;                        /* 预留字段 */
 uint16_t target_change_time;             /* 目标切换时间 */
 uint16_t latest_launch_cmd_time;         /* 最近一次发射命令时间 */
}dart_client_cmd_t;

/**
 * @brief typedef structure that contains the information of robot position in mimi map, id: 0x020BU
 * @translate 包含迷你地图中机器人位置信息的类型定义结构体，标识符：0x020BU
 */
typedef struct
{
  float hero_x;           // 英雄机器人的x坐标
  float hero_y;           // 英雄机器人的y坐标
  float engineer_x;       // 工程机器人的x坐标
  float engineer_y;       // 工程机器人的y坐标
  float standard_3_x;     // 3号标准机器人的x坐标
  float standard_3_y;     // 3号标准机器人的y坐标
  float standard_4_x;     // 4号标准机器人的x坐标
  float standard_4_y;     // 4号标准机器人的y坐标
}ground_robot_position_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020C
 * @translate 包含机器人标记信息的类型定义结构体，标识符：0x020C
 */
typedef struct
{
  uint8_t mark_progress;     /* 标记进度 */
}radar_mark_data_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020D
 * @translate 包含机器人标记信息的类型定义结构体，标识符：0x020D
 */
typedef  struct
{
  uint32_t sentry_info;       //哨兵信息
  uint16_t sentry_info_2;     //哨兵信息2
} sentry_info_t;

/**
 * @brief typedef structure that contains the information of radar, id: 0x020E
 * @translate 包含雷达信息的类型定义结构体，标识符：0x020E
 */
typedef  struct
{
  uint8_t radar_info;			/* 雷达信息 */
}radar_info_t;

/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0301U
 * @translate 包含自定义控制器交互信息的类型定义结构体，标识符：0x0301U
 * @remember  该结构体看似和自定义控制器0x0302很像，但并不是，注意辨别
 */
typedef struct{ 
	uint16_t data_cmd_id;       /* 数据命令标识符 */
	uint16_t sender_id;         /* 发送者ID */
	uint16_t receiver_id;       /* 接收者ID */
	uint8_t user_data[113];     /* 用户数据（长度为113字节） */
}robot_interaction_data_t;

/**
 * @brief typedef structure that contains the information of client transmit data, id: 0x0303U
 * @translate 包含客户端传输数据信息的类型定义结构体，标识符：0x0303U
 */
typedef struct
{
  /**
   * @brief target position coordinate, is 0 when transmit target robot id
	 * @translate 目标位置坐标，当传输目标机器人ID时该值为0
   */
  float target_position_x;   /* 目标位置x坐标 */
  float target_position_y;   /* 目标位置y坐标 */
  uint8_t cmd_keyboard;      /* 键盘命令 */
  uint16_t target_robot_id;  /* 目标机器人ID */
  uint16_t cmd_source;       /* 命令来源 */
}map_command_t;

/**
 * @brief typedef structure that contains the information of client receive data, id: 0x0305U
 * @translate 包含客户端接收数据信息的类型定义结构体，标识符：0x0305U
 */
typedef struct
{
	uint16_t hero_position_x;         // 英雄机器人x坐标
	uint16_t hero_position_y;         // 英雄机器人y坐标
	uint16_t engineer_position_x;     // 工程机器人x坐标
	uint16_t engineer_position_y;     // 工程机器人y坐标
	uint16_t infantry_3_position_x;   // 3号步兵机器人x坐标
	uint16_t infantry_3_position_y;   // 3号步兵机器人y坐标
	uint16_t infantry_4_position_x;   // 4号步兵机器人x坐标
	uint16_t infantry_4_position_y;   // 4号步兵机器人y坐标
	uint16_t infantry_5_position_x;   // 5号步兵机器人x坐标
	uint16_t infantry_5_position_y;   // 5号步兵机器人y坐标
	uint16_t sentry_position_x;       // 哨兵机器人x坐标
	uint16_t sentry_position_y;       // 哨兵机器人y坐标
}map_robot_data_t;

/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0307U
 * @translate 包含哨兵路径信息的类型定义结构体，标识符：0x0307U
 */
typedef struct
{
  /**
   * @brief  sentry status                   哨兵状态
   *         1: attack on target point       在目标点进攻
   *         2: defend on target point       在目标点防御
   *         3: move to target point         移动至目标点
   */
  uint8_t intention;                         // 意图（状态）
  uint16_t start_position_x;                 // 起始位置x坐标
  uint16_t start_position_y;                 // 起始位置y坐标
  int8_t delta_x[49];                        // x方向增量数组（长度49）
  int8_t delta_y[49];                        // y方向增量数组（长度49）
	uint16_t sender_id;                        // 发送者ID
}map_data_t;

/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0308U
 * @translate 包含哨兵路径信息的类型定义结构体，标识符：0x0308U
 * @role （基于我的理解）我认为该结构体大概率是传哨兵路径数据的结构体！！！
 */
typedef  struct
{
	uint16_t sender_id;          /* 发送者ID */
	uint16_t receiver_id;        /* 接收者ID */
	uint8_t user_data[30];       /* 用户数据（长度为30字节） */
}custom_info_t;

/**
 * @brief typedef structure that contains the information of custom robot, id: 0x0302U
 * @translate 包含自定义控制器（真正的工程自定义控制器结构体）信息的类型定义结构体，标识符：0x0302U
 * @remeber 由于图传链路的自定义控制器是后加的所以放在了所有常规链路结构体的后面，在我写时易于查找
 */
typedef  struct
{
	uint8_t data[30];      /****接收的数据包是39个字节，但所有经裁判系统的数据帧头校验和都是相同的，被同一个函数解析完就OK了，
	                            只需要他的ID，所以这个结构体只需要存储他30字节的有效数据*******************************/
}custom_robot_data_t;

/***@role： 图传链路的键鼠遥控控制数据的结构体，标识符： 0x03040***/
typedef  struct
{
	int16_t Mouse_X;																		//鼠标X轴移动速度，负值标识向左移动
	int16_t Mouse_Y;																		//鼠标y轴移动速度，负值标识向下移动
	int16_t Mouse_Z;																		//鼠标滚轮移动速度，负值标识向后滚动
	int8_t  Left_button_down;														//鼠标左键是否按下：0为未按下，1为按下
	int8_t  Right_button_down;													//鼠标右键是否按下：0为未按下，1为按下
	uint16_t keyboard_value_t;													//键盘信息，每个bit对应一个按键，0为未按下，1为按下
	uint16_t reserved;																	//保留位
}KeyMouse_control_t;

typedef union
{
	struct
	{
		uint16_t W:1;
	  uint16_t S:1;
	  uint16_t A:1;
	  uint16_t D:1;
	  uint16_t SHIFT:1;
	  uint16_t CTRL:1;
	  uint16_t Q:1;
	  uint16_t E:1;
	  uint16_t R:1;
	  uint16_t F:1;
	  uint16_t G:1;
	  uint16_t Z:1;
	  uint16_t X:1;
	  uint16_t C:1;
	  uint16_t V:1;
	  uint16_t B:1;
	}keyboard_value;
	uint16_t keyboard_value_data;
}KeyBoard_Value_Data_t;
extern KeyBoard_Value_Data_t KeyBoard_Value_Data;



/**
 * @brief typedef structure that contains the information of Referee
 * @translate 包含裁判系统信息的类型定义结构体
 */
typedef struct 
{
  uint8_t Index;                                     //索引
  uint16_t DataLength;                               //数据长度
																			               
#ifdef GAME_STATUS_ID                                
  game_status_t game_status;                         //比赛状态信息
#endif                                               
																			               
#ifdef GAME_RESULT_ID                                
  game_result_t game_result;                         //比赛结果信息
#endif                                               
																			               
#ifdef  GAME_ROBOT_HP_ID                             
	game_robot_HP_t game_robot_HP;                     //比赛机器人血量信息
#endif	                                             
																			               
#ifdef EVENET_DATA_ID                                
  event_data_t event_data;                           //事件数据信息
#endif                                               
																			               
#ifdef REFEREE_WARNING_ID                            
    referee_warning_t referee_warning;               //裁判警告信息
#endif                                               
																			               
#ifdef DART_INFO_ID                                  
  dart_info_t dart_info;                             //飞镖信息
#endif                                               
																			               
#ifdef ROBOT_STATUS_ID                               
  robot_status_t robot_status;                       //机器人状态信息
#endif                                               
																			               
#ifdef POWER_HEAT_DATA_ID                            
  power_heat_data_t power_heat_data;                 //能量热量数据信息
#endif                                               
																			               
#ifdef ROBOT_POS_ID                                  
  robot_pos_t  robot_pos;                            //机器人位置信息
#endif                                               
																			               
#ifdef BUFF_ID                                       
  buff_t  buff;                                      //机器人增益效果状态信息
#endif                                               
																			               
#ifdef HURT_DATA_ID                                  
  hurt_data_t  hurt_data;                            //机器人损伤信息
#endif                                               
																			               
#ifdef SHOOT_DATA_ID                                 
  shoot_data_t  shoot_data;                          //实际射击数据信息
#endif                                               
																								     
#ifdef PROJECTILE_ALLOWANCE_ID                       
  projectile_allowance_t projectile_allowance;       //剩余子弹数量信息
#endif                                               
																								     
#ifdef  RFID_STATUS_ID                               
    rfid_status_t rfid_status;                       //RFID状态信息
#endif                                               
																								     
#ifdef DART_CLIENT_CMD_ID                            
    dart_client_cmd_t  dart_client_cmd;              //飞镖客户端状态信息
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    ground_robot_position_t  ground_robot_position;  //迷你地图中机器人位置信息
#endif 

#ifdef RADAR_MARAKING_DATA_ID
    radar_mark_data_t  radar_mark_data;              //机器人标记信息
#endif

#ifdef SENTRY_INFO_ID
    sentry_info_t  sentry_info;                      //哨兵机器人相关信息
#endif

#ifdef RADAR_INFO_ID
    radar_info_t  radar_info;                        //雷达相关信息
#endif

#ifdef ROBOT_INTERACTION_DATA_ID
    robot_interaction_data_t robot_interaction_data; //自定义控制器相关信息（此自定义控制器非工程的自定义控制器）
#endif

#ifdef MAP_COMMAND_ID
    map_command_t map_command;                       //客户端传输数据信息
#endif

#ifdef MAP_ROBOT_DATA_ID
    map_robot_data_t  map_robot_data;                //客户端接收数据信息
#endif

#ifdef MAP_DATA_ID
    map_data_t map_data;                             //哨兵路径信息
#endif

#ifdef CUSTOM_INFO_ID
    custom_info_t custom_info;                       //自定义信息（大概是传哨兵路径信息的）
#endif

#ifdef custom_controller_ID
    custom_robot_data_t custom_robot_data;           //自定义控制器接收到的信息（其中不包括帧头，校验和等验证信息，只有三十位的有效字节）
#endif

#ifdef Keymouse_Control_ID
    KeyMouse_control_t KeyMouse_control;           	 //裁判系统发来的键鼠的数据
#endif
}Referee_System_Info_TypeDef;
extern Referee_System_Info_TypeDef Referee_System_Info;

/* restore byte alignment 恢复字节对齐 */
#pragma  pack()

//----------------------以上为裁判系统数据的结构体，有时间的话可以仔细阅读一下---------------------------------------------//

//--------------------------------------------------------------------------------------------------------//
extern uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];

//********************************************************************************************************//

//----------------------------------------------------------------------------------------------------------//
void Referee_System_Frame_Update(uint8_t *Buff);
//**************************************************************************************************************

#endif


