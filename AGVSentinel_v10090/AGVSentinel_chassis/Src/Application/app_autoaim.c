#include "app_autoaim.h"

//--------------------------------------------------------------------------------------------------------
//																						澶栭儴鍙橀噺鍜屽嚱鏁?extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

//extern shoot_control_t shoot_control;
//extern gimbal_control_t gimbal_control;
extern INS_INSTypeDef INS;
extern fp32 INS_angle[3];
extern fp32 INS_accel[3];
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern int shoot_speed_add_little;
//--------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------
//																						鍙橀噺瀹氫箟
__ALIGNED(8) uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN]; 	 //鍙戦€佺紦鍐?鏈€澶SART1_MAX_SEND_LEN(400)瀛楄妭
extern UART_HandleTypeDef huart1;
uint8_t ChariqotRecognition_data[2][ChariotRecognition_data_dma_buf_len];
int16_t ChariotRecognitionDirection[2];
int16_t Chariot_Rec_Dir_rotate[2];
int CameraDetectTarget_Flag = 0;
float ChariotRecognition_pitch = 0.0f;
float last_ChariotRecognition_yaw = 0.0f;
float ChariotRecognition_yaw = 0.0f;
float PitchCurrentPositionSave = 0.0f;
uint16_t Target_Distance = 150;
uint8_t Dis_Buf_Index = 0;
uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];      //鎺ユ敹缂撳啿,鏈€澶SART_REC_LEN(400)涓瓧鑺?
uint16_t USART1_RX_STA=0;						 //鎺ユ敹鐘舵€佹爣璁?

uint32_t usart1_this_time_rx_len = 0;              //USART1鏀跺埌鐨勬暟鎹釜鏁?uint8_t ChariqotRecognition_data[2][ChariotRecognition_data_dma_buf_len];   //DMA鎺ユ敹濡欑畻鏁版嵁鐨勫弻缂撳瓨鏁扮粍
int32_t ChariotRecognitionTemp[2] = {0,0};      //瑙ｆ瀽瑙掑害鍐呭鐨勬暟缁?int16_t ChariotRecognitionDirection[2];  //鎺ュ彈鎽勫儚澶翠紶鏉ョ殑澶ц鐢叉暟鎹?int16_t Chariot_Rec_Dir_rotate[2];       //鎺ュ彈鎽勫儚澶翠紶鏉ョ殑澶ц鐢插拰灏忚鐢叉暟鎹?int CameraDetectTarget_Flag = 0;     //鎽勫儚澶村彂鐜扮洰鏍囨爣蹇?
int TempShootingFlag=0;      //鍙戝脊鏍囧織,淇敼璇ユ爣蹇楋紝鍙€夋嫨鏄惁鍙戝脊

float last_ChariotRecognition_pitch = 0.0f;        //涓婁竴涓猵itch瑙掑害鍊?float ChariotRecognition_pitch = 0.0f;             //pitch瑙掑害鍊?float last_ChariotRecognition_yaw = 0.0f;          //涓婁竴涓獃aw杞磋搴﹀€?float ChariotRecognition_yaw = 0.0f;               //yaw瑙掑害鍊?
float YawCurrentPositionSave   = 0.0f;     //淇濆瓨褰撳墠Yaw杞翠綅缃?float PitchCurrentPositionSave = 0.0f;     //淇濆瓨褰撳墠Pitch杞翠綅缃?
uint16_t last_Target_Distance = 0;         //涓婃鎽勫儚澶翠笌鐩爣鐨勮窛绂?uint16_t Target_Distance = 150;            //鎽勫儚澶翠笌鐩爣鐨勮窛绂?
uint16_t Distance_Buf[Dis_buf_Size];     //璺濈缂撳啿鍖?uint8_t Dis_Buf_Index = 0;
uint8_t Pitch_Add_Angle = 0;
uint8_t enter_CNT = 0;
int Armor_R_Flag_Before=0;
int Armor_R_Flag_Behind=0;

int GM_Rotete_flag_Before=0;        //鍓嶅浐瀹氭憚鍍忓ご璇嗗埆鐩爣
int GM_Rotete_flag_Behind=0;        //鍚庡浐瀹氭憚鍍忓ご璇嗗埆鐩爣
int Time_count=0;

CRringBuffer_t CR_ringBuffer;

float CR_yaw_Angle[20];
uint8_t CR_yaw_Angle_Index = 0;
uint8_t CR_yaw_Angle_CNT   = 0;
int8_t loop_j;

char Sendtosight[Sendtosight_len];            //鍙戦€佺粰瑙嗚

int friction_wheel_count = 0;
float kalman_yaw = 0;
float kalman_pitch = 0;
float kalman_yaw_feedforward = 0;
uint8_t update_flag = 1;
int Last_CameraDetectTarget_Flag=0;
float E_TEST=0;
float E_TEST1=0;
float E_TEST2=0;
float E_TEST3=0;
int camera_send_flag = 0;

int autoaim_mode_flag
	= 0;   //鑷瀯鐘舵€佸紑鍚?鍏抽棴鏍囧織
//--------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------
//																						涓插彛澶勭悊鍣ㄥ嚱鏁?//void USART1_IRQHandler(void)
//{  
//	 if(huart1.Instance->SR & UART_FLAG_RXNE)//鎺ユ敹鍒版暟鎹?//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }

//    else if(USART1->SR & UART_FLAG_IDLE)
//    {
//        static uint8_t this_time_rx_len = 0;
//			
//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //澶辨晥DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //鑾峰彇鎺ユ敹鏁版嵁闀垮害,闀垮害 = 璁惧畾闀垮害 - 鍓╀綑闀垮害
//            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=15 buf_len =27,rxlen=12 鐒跺悗灏变笉鎺ユ敹浜?搴旇鎶奲uf鏀规垚30


//            //reset set_data_lenght
//            //閲嶆柊璁惧畾鏁版嵁闀垮害
//            hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

//            //set memory buffer 1
//            //璁惧畾缂撳啿鍖?
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //浣胯兘DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);
//	   
//            if(this_time_rx_len == ChariotRecognition_data_len )
//            {
//							GetVisionData(&visionDataGet ,ChariqotRecognition_data[0]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //澶辨晥DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //鑾峰彇鎺ユ敹鏁版嵁闀垮害,闀垮害 = 璁惧畾闀垮害 - 鍓╀綑闀垮害
//            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=21 

//            //reset set_data_lenght
//            //閲嶆柊璁惧畾鏁版嵁闀垮害
//           hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

//            //set memory buffer 0
//            //璁惧畾缂撳啿鍖?
//            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //浣胯兘DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == ChariotRecognition_data_len)
//            {
//							GetVisionData(&visionDataGet ,ChariqotRecognition_data[0]);
//            }
//        }
//    }

//}

//--------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------
//																							鏈哄櫒浜篒D锛屾満鍣ㄤ汉绛夌骇锛屽皠閫?.....
State_distance state_distacne=closedistance;

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

int count_Sendtosight = 0;
//extern int windmill_mode;
float yaw_add_little = 0;
float pitch_add_little = 0;

int shoot_num = 0;
float real_shoot_speed_save[5] = {1.36,1.36,1.36,1.36,1.36};
float remaining_shoot_num = 0;
float last_remaining_shoot_num = 0;
float average_shoot_speed = 1.36;
float last_shoot_speed ;
//--------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------
//             姹?娆″彂寮圭殑骞冲潎鍊硷紝瀹炴椂鏇存柊,鏆傛椂娌℃湁鍐欏ソ锛屽畬鍠勫浘浼犻摼璺椂淇敼
void shoot_speed_sending_culculate(void)
{
//	remaining_shoot_num = get_bullet_remaining_num();
  remaining_shoot_num = 1.36f;
	if(last_remaining_shoot_num - remaining_shoot_num)
	{
		shoot_num++;
	}
	
	if(shoot_num)
	{
//		real_shoot_speed_save[shoot_num%5] = get_bullet_speed();
		real_shoot_speed_save[shoot_num%5] = 1.36f;
	}
	
	average_shoot_speed = (real_shoot_speed_save[0] + real_shoot_speed_save[1] + real_shoot_speed_save[2] + real_shoot_speed_save[3] + real_shoot_speed_save[4]) / 5;	
}
//--------------------------------------------------------------------------------------------------------





VisionDataSend_Typedef visionDataSend;					    //鍙戦€佺粰瑙嗚缁勭殑鏁版嵁
VisionDataGet_Typedef  visionDataGet;               //鎺ユ敹鏉ヨ嚜瑙嗚缁勭殑鏁版嵁
uint8_t RXdata[15];                                 //鎺ユ敹瑙嗚缁勭殑鏁扮粍
//             鍙戦€佹暟鎹垵濮嬪寲
void SendVisionData_Init(void)
{
	visionDataSend.head = 0x53u;
	visionDataSend.color = 0x02u;
	visionDataSend.mode = 0x01u;
	visionDataSend.robotID = 0x03u;
	visionDataSend.yaw_angle.yaw_gyro = 5;
	visionDataSend.pitch_angle.pitch_gyro = 6;
	visionDataSend.yaw_acc_data.yaw_acc = 0;
	visionDataSend.pitch_acc_data.pitch_acc = 0;
	visionDataSend.shoot_speed = 0u;
	visionDataSend.CRCcode = 0u;
	visionDataSend.end = 0x45u;
}
void MyUART_Init(void)
{
	SendVisionData_Init();
}

uint8_t AutoAim_CalculateCRC8(const uint8_t *data, uint16_t len)
{
	uint8_t crc = 0x00u;

	if (data == NULL) {
		return 0u;
	}

	for (uint16_t i = 0u; i < len; ++i) {
		crc ^= data[i];
		for (uint8_t bit = 0u; bit < 8u; ++bit) {
			crc = (crc & 0x80u) != 0u
				? (uint8_t)((crc << 1u) ^ 0x31u)
				: (uint8_t)(crc << 1u);
		}
	}

	return crc;
}

uint8_t AutoAim_VerifyFrameCRC8(const uint8_t *frame, uint16_t len)
{
	if (frame == NULL || len != ChariotRecognition_data_len) {
		return 0u;
	}
	return (uint8_t)(AutoAim_CalculateCRC8(frame, VISION_FRAME_CRC_OFFSET) ==
					 frame[VISION_FRAME_CRC_OFFSET]);
}
//                鍚戣瑙夊彂閫佹暟鎹?
void SendVisionData(VisionDataSend_Typedef* RAW_Data)
{
	static uint8_t data_buffer[15];
	static uint8_t delay_num = 0u;

	if (RAW_Data == NULL) {
		return;
	}

	if (++delay_num < 10u) {
		return;
	}
	delay_num = 0u;

	data_buffer[0]  = RAW_Data->head;
	data_buffer[1]  = RAW_Data->color;
	data_buffer[2]  = RAW_Data->mode;
	data_buffer[3]  = RAW_Data->robotID;
	data_buffer[4]  = RAW_Data->yaw_angle.yaw_angle_send[0];
	data_buffer[5]  = RAW_Data->yaw_angle.yaw_angle_send[1];
	data_buffer[6]  = RAW_Data->pitch_angle.pitch_angle_send[0];
	data_buffer[7]  = RAW_Data->pitch_angle.pitch_angle_send[1];
	data_buffer[8]  = RAW_Data->yaw_acc_data.yaw_acc_send[0];
	data_buffer[9]  = RAW_Data->yaw_acc_data.yaw_acc_send[1];
	data_buffer[10] = RAW_Data->pitch_acc_data.pitch_acc_send[0];
	data_buffer[11] = RAW_Data->pitch_acc_data.pitch_acc_send[1];
	data_buffer[12] = RAW_Data->shoot_speed;
	RAW_Data->CRCcode = AutoAim_CalculateCRC8(data_buffer, VISION_FRAME_CRC_OFFSET);
	data_buffer[VISION_FRAME_CRC_OFFSET] = RAW_Data->CRCcode;
	data_buffer[VISION_FRAME_END_OFFSET] = RAW_Data->end;

	HAL_UART_Transmit_DMA(&huart1, data_buffer, sizeof(data_buffer));
}
static volatile uint32_t s_autoaim_last_update_time = 0U;

void GetVisionData(VisionDataGet_Typedef* myVisionDataGet, uint8_t RAW_Data[30])
{
	if (myVisionDataGet == NULL || RAW_Data == NULL ||
		RAW_Data[0] != 0x53u || RAW_Data[VISION_FRAME_END_OFFSET] != 0x45u ||
		!AutoAim_VerifyFrameCRC8(RAW_Data, ChariotRecognition_data_len)) {
		return;
	}

	myVisionDataGet->head = RAW_Data[0];
	myVisionDataGet->detect_signal = RAW_Data[1];
	myVisionDataGet->shoot_msg = RAW_Data[2];
	myVisionDataGet->yaw_angle.yaw_angle_get[0] = RAW_Data[3];
	myVisionDataGet->yaw_angle.yaw_angle_get[1] = RAW_Data[4];
	myVisionDataGet->pitch_angle.pitch_angle_get[0] = RAW_Data[5];
	myVisionDataGet->pitch_angle.pitch_angle_get[1] = RAW_Data[6];
	myVisionDataGet->x_data.x_get[0] = RAW_Data[7];
	myVisionDataGet->x_data.x_get[1] = RAW_Data[8];
	myVisionDataGet->y_data.y_get[0] = RAW_Data[9];
	myVisionDataGet->y_data.y_get[1] = RAW_Data[10];
	myVisionDataGet->depth_data.depth_get[0] = RAW_Data[11];
	myVisionDataGet->depth_data.depth_get[1] = RAW_Data[12];
	myVisionDataGet->CRCcode = RAW_Data[13];
	myVisionDataGet->end = RAW_Data[14];
	s_autoaim_last_update_time = HAL_GetTick();
}

uint8_t AutoAim_IsOnline(uint32_t timeout_ms)
{
	return (uint8_t)(s_autoaim_last_update_time != 0U &&
	                 (HAL_GetTick() - s_autoaim_last_update_time) <= timeout_ms);
}

uint32_t AutoAim_GetLastUpdateTime(void)
{
	return s_autoaim_last_update_time;
}
void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend)
{
	Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
	if (visionDataSend == NULL || referee == NULL) {
		return;
	}

	visionDataSend->yaw_angle.yaw_gyro = (int16_t)(INS.Yaw * 100.0f);
	visionDataSend->pitch_angle.pitch_gyro = (int16_t)(INS.Roll * 100.0f);
	visionDataSend->yaw_acc_data.yaw_acc = (int16_t)(INS.Accel[0] * 100.0f);
	visionDataSend->pitch_acc_data.pitch_acc = (int16_t)(INS.Accel[2] * 100.0f);

	if ((referee->bullet_speed <= 0.0f) || (referee->bullet_speed * 10.0f > 255.0f)) {
		last_shoot_speed = 0.0f;
	} else {
		last_shoot_speed = referee->bullet_speed;
	}
	visionDataSend->shoot_speed = (uint8_t)(last_shoot_speed * 10.0f);
	visionDataSend->robotID = (uint8_t)referee->robot_id;
}
uint8_t get_shoot_msg(VisionDataGet_Typedef *visionDataGet)
{
	uint8_t shoot_msg = 1u;
	static uint8_t shoot_msg_buf[SHOOT_MSG_BUF_SIZE] = {0};

	if (visionDataGet == NULL) {
		return 0u;
	}

	for (uint8_t i = SHOOT_MSG_BUF_SIZE - 1u; i > 0u; --i) {
		shoot_msg_buf[i] = shoot_msg_buf[i - 1u];
	}
	shoot_msg_buf[0] = visionDataGet->shoot_msg ? 1u : 0u;

	for (uint8_t i = 0u; i < SHOOT_MSG_BUF_SIZE; ++i) {
		shoot_msg &= shoot_msg_buf[i];
	}
	return shoot_msg;
}



