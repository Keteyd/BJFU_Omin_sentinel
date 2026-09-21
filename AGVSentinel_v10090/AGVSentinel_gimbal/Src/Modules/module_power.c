#include "module_power.h"
#include "cmsis_os.h"
#include "periph_motor.h"
#include "stdio.h"
#include "stdlib.h"

CAN_TxHeaderTypeDef CapHeader;
SuperCap_Data_t SuperCap ;
/**
 * @brief 解析超级电容控制板的反馈数据 (ID: 0x211)
 * @param rx_data CAN接收到的8字节数据缓冲区
 * @param cap_data 指向存储解析后数据的结构体指针
 */
void Parse_SuperCap_Feedback(uint8_t *rx_data, SuperCap_Data_t *cap_data) {
    if (rx_data == NULL || cap_data == NULL) return;

    // 将 8 个 uint8_t 字节直接看作 4 个 uint16_t 数据
    // 由于文档指明发送端是小端，且 Cortex-M 主控也是小端，直接强转完全没问题
    uint16_t *pPowerData = (uint16_t *)rx_data;

    // 依序解析并除以 100.0 还原真实物理量
    cap_data->input_voltage = (float)pPowerData[0] / 100.0f; // 字节 1, 2    
    cap_data->cap_voltage   = (float)pPowerData[1] / 100.0f; // 字节 3, 4
    cap_data->input_current = (float)pPowerData[2] / 100.0f; // 字节 5, 6
    cap_data->target_power  = (float)pPowerData[3] / 100.0f; // 字节 7, 8
}

/**
 * @brief  初始化超级电容的发送报文头 (ID: 0x210)
 * @param  pheader 指向需要初始化的 CAN_TxHeaderTypeDef 结构体指针
 */
void SuperCap_InitHeader(CAN_TxHeaderTypeDef *pheader) {
    if (pheader == NULL) return;
    Can_InitTxHeader(pheader, 0x210, 0, 8);
}


/**
 * @brief  设定超级电容的控制功率 (ID: 0x210)
 * @param  phcan          CAN 句柄指针 (如 &hcan1)
 * @param  pheader        CAN 发送报文头指针 (如 &(pgroup->can_header))
 * @param  target_power_w 目标功率，单位：瓦特(W)，范围：45.0 ~ 100.0
 * @note   使用此函数前，请确保 pheader 已经通过 Can_InitTxHeader 初始化过。
 */
void SuperCap_SetPower(CAN_HandleTypeDef* phcan, CAN_TxHeaderTypeDef* pheader, float target_power_w)
{
    // 1. 严格的安全限幅 (45W 到 100W)
    if (target_power_w < 45.0f) {
        target_power_w = 45.0f;
    }
    if (target_power_w > 100.0f) {
        target_power_w = 100.0f;
    }

    // 2. 将浮点数转换为无符号 16 位整数 (放大 100 倍)
    uint16_t tempPower = (uint16_t)(target_power_w * 100.0f);

    // 3. 准备 CAN 发送缓冲区 (长度为8)
    uint8_t txdata[8] = {0}; // 数组自动清零
    
    // 4. 文档要求的“大端模式”：高位在前，低位在后
    txdata[0] = (uint8_t)(tempPower >> 8);   // 功率设定数据高八位
    txdata[1] = (uint8_t)(tempPower & 0xFF); // 功率设定数据低八位

    // ?? 原来的第 5 步被删掉了，因为外部已经初始化过了！

    // 6. 调用你的自定义发送函数
    Can_SendMessage(phcan, pheader, txdata);
}

/**
 * @brief  检测超级电容功率并在非95W时强制校正（带100ms发送限速）
 * @param  pcap_data 指向超级电容数据结构体的指针
 * @note   推荐在主循环或定时任务中高频调用（比如 2ms 周期调用一次也没问题）
 */
void SuperCap_CheckPower(SuperCap_Data_t *pcap_data)
{
    if (pcap_data == NULL) return;
	if (pcap_data->target_power ==95) return;
    // 1. 静态变量，用于记录上一次成功发送的时间戳
    static uint32_t last_send_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // 2. 时间限制：如果距离上次发送不足 100ms，直接返回，绝不发送
    if (current_tick - last_send_tick < 100)
    {
        return; 
    }
    if (fabs(pcap_data->target_power - 95.0f) > 0.1f)
    {
        // 4. 调用你之前写好的发送函数，强制拉回 95.0W
        SuperCap_SetPower(&hcan1, &CapHeader, 95.0f);
        
        // 5. 更新发送时间戳，重新开始 100ms 的倒计时
        last_send_tick = current_tick;

    }
}