/*
 * uart.c
 *
 *  Created on: 2026骞�9鏈�16鏃�
 *      Author: 浣�
 *
 *  鏈枃浠讹細涓插彛鐩稿叧鍑芥暟瀹炵幇 + PID 鍦ㄧ嚎璋冨弬
 *
 *  PID 璋冨弬鏁版嵁娴侊紙閲嶈锛岀悊瑙ｅ悗鍐嶆敼锛夛細
 *    涓婁綅鏈�(VOFA) --鏃犵嚎涓插彛--> TC264 UART_2
 *        --> 纭欢鎺ユ敹涓柇 isr.c:uart2_rx_isr()
 *        --> 妗嗘灦鍒嗗彂 wireless_module_uart_handler()
 *        --> zf_device_wireless_uart.c:wireless_uart_callback()
 *        --> 鍐欏叆鏃犵嚎涓插彛鍐呴儴 FIFO锛�64瀛楄妭锛�
 *    鎴戜滑鍦ㄤ富寰幆璋冪敤 pid_tuning_handler()锛�
 *        --> wireless_uart_read_buffer() 鎶� FIFO 鏁版嵁璇诲嚭鏉�
 *        --> 缁勫抚锛堜互 '!' 涓轰竴甯х粨鏉熸爣蹇楋級
 *        --> 瑙ｆ瀽 "Xn=鏁板��!" 骞舵洿鏂板叏灞� NUM 缁撴瀯浣�
 *
 *  娉ㄦ剰锛歸ireless_uart_init() 宸插湪 code/init.c 璋冪敤锛屾湰鏂囦欢涓嶅啀鍒濆鍖栥��
 */

#include "uart.h"
#include <math.h>

/* 环岛识别后开始累积、出环后清零的相对陀螺仪角度。 */
extern volatile float ring_gyro_angle_deg;
/* 0=初始化失败，2=IMU660RB。 */
extern volatile uint8 ring_imu_type;

//==================== 鍘熸湁娴嬭瘯鍑芥暟锛堜繚鐣欙紝鍕垮垹锛� ====================
void uart_test_init(void){
    uart_init(UART_3, 115200 , UART3_TX_P15_6, UART3_RX_P15_7);
}

void uart_test_transmit(const char *str){
    uart_write_string(UART_3, str);
}

//==================== PID 鍦ㄧ嚎璋冨弬 ====================

// 鍏ㄥ眬 PID 鍙傛暟鍙橀噺銆傝繖閲岀粰涓�缁勯粯璁ゅ�硷紝涓婄數鍚庝笂浣嶆満娌″彂鎸囦护鍓嶇敤杩欑粍鍊硷紱
// 浣犲彲浠ユ寜鑷繁杞︾殑瀹為檯鎯呭喌淇敼涓嬮潰鐨勯粯璁ゆ暟鍊笺��
// 娉ㄦ剰锛氫笅闈㈡暟鍊肩殑椤哄簭蹇呴』鍜� uart.h 閲� PIDNUM 缁撴瀯浣撶殑鎴愬憳椤哄簭涓�涓�瀵瑰簲锛�
//       angle_p, angle_i, angle_d, speed_p, speed_i, speed_d, anglev_p, anglev_i, anglev_d
PIDNUM NUM1 =
{
    0.0f,   // angle_p   瑙掑害鐜� P
    0.0f,   // angle_i   瑙掑害鐜� I
    45.0f,   // angle_d   瑙掑害鐜� D
    26.0f,   // speed_p
    1.92f,   // speed_i
    0.0f,   // speed_d   閫熷害鐜� D
    1.1f,   // anglev_p  圆环转向 P
    0.0f,   // anglev_i  瑙掗�熷害鐜� I
    45.0f,  // anglev_d  圆环转向 D
};

PIDNUM NUM2 =
{
    0.0025f,   // angle_p   瑙掑害鐜� P
    0.0f,   // angle_i   瑙掑害鐜� I
    0.0f,   // angle_d   瑙掑害鐜� D
    26.0f,   // speed_p
    1.92f,   // speed_i
    0.0f,   // speed_d   閫熷害鐜� D
    0.0025f,   // anglev_p  瑙掗�熷害鐜� P
    0.0f,   // anglev_i  瑙掗�熷害鐜� I
    0.0f,   // anglev_d  瑙掗�熷害鐜� D
};

// ---- 鎺ユ敹缁勫抚鐢ㄧ殑闈欐�佸彉閲忥紙鍙湪鏈枃浠跺唴浣跨敤锛屽澶栦笉鍙锛� ----
static uint8  pid_rx_buf[64];       // 鏆傚瓨涓�甯у皻鏈В鏋愮殑鏁版嵁
static uint8  pid_rx_idx = 0;       // 褰撳墠宸叉敹鍒扮殑瀛楄妭鏁�

// ---- 瀹氭椂鍥炰紶璁℃暟鐢ㄧ殑闈欐�佸彉閲� ----
// volatile锛氬洜涓哄畠浠細琚腑鏂拰涓诲惊鐜悓鏃惰闂紝蹇呴』鍔� volatile锛�
//           闃叉缂栬瘧鍣ㄤ紭鍖栨妸瀹冧滑浼樺寲鎴愬瘎瀛樺櫒鍙橀噺锛屽鑷翠富寰幆璇讳笉鍒颁腑鏂噷鐨勪慨鏀广��
static volatile uint8 send_tick_cnt = 0;   // 5ms 涓柇璁℃暟鍣�
static volatile uint8 send_flag     = 0;   // 鍒扮偣鍙戦�佹爣蹇楋細1 = 涓诲惊鐜鍙戜簡
volatile uint8 car_stop      = 0;

//-------------------------------------------------------------------------------------------------------------------
//  鍑芥暟璇存槑锛氫粠涓�娈靛瓧绗﹂噷瑙ｆ瀽鍑烘诞鐐规暟锛堟敮鎸佽礋鏁般�佸皬鏁帮級
//            閫昏緫涓� vofa.c 涓� Get_Data() 淇濇寔涓�鑷达紝浣嗘搷浣滅殑鏄櫘閫氬瓧绗︽寚閽堛��
//  鍙傛暟     s      : 瀛楃涓茶捣濮嬪湴鍧�
//           len    : 瀛楃闀垮害
//  杩斿洖鍊�  瑙ｆ瀽鍑虹殑娴偣鏁帮紱閬囧埌闈炴硶瀛楃杩斿洖 0
//-------------------------------------------------------------------------------------------------------------------
static float pid_parse_float(const uint8 *s, uint8 len)
{
//    uint8 minus_flag = 0;       // 鏄惁涓鸿礋鏁�
//    uint8 i = 0;
//    float result = 0.0f;
//    uint8 decimal_cnt = 0;      // 灏忔暟鐐瑰悗杩樻湁鍑犱綅
//
//    // 1. 鍒ゆ柇璐熷彿
//    if(0 < len && s[0] == '-')
//    {
//        minus_flag = 1;
//        i = 1;
//    }
//
//    // 2. 閫愬瓧绗﹁浆鏁板��
//    for(; i < len; i++)
//    {
//        if(s[i] == '.')                 // 閬囧埌灏忔暟鐐�
//        {
//            decimal_cnt = 1;            // 鏍囪寮�濮嬪鐞嗗皬鏁伴儴鍒�
//            continue;
//        }
//
//        uint8 digit = s[i] - '0';      // 瀛楃杞暟瀛�
//        if(digit > 9) return 0.0f;     // 涓嶆槸鏁板瓧涔熶笉鏄皬鏁扮偣锛屾暟鎹潪娉�
//
//        if(decimal_cnt)
//        {   // 灏忔暟閮ㄥ垎锛氭寜浣嶆潈绱姞 0.1, 0.01, 0.001...
//            result += digit * powf(0.1f, (float)decimal_cnt);
//            decimal_cnt++;
//        }
//        else
//        {   // 鏁存暟閮ㄥ垎锛氫箻 10 绱姞
//            result = result * 10.0f + digit;
//        }
//    }
//
//    return minus_flag ? -result : result;
        uint8  minus_flag = 0;
        uint8  i = 0;
        uint32 int_val  = 0;      /* 整数部分 */
        uint32 frac_val = 0;      /* 小数部分（按位累积的整数） */
        uint8  frac_cnt = 0;      /* 小数位数计数 */
        float  result;

        if(0 < len && s[0] == '-') { minus_flag = 1; i = 1; }

        for(; i < len; i++)
        {
            if(s[i] == '.') { frac_cnt = 1; continue; }
            if(s[i] < '0' || s[i] > '9') return 0.0f;   /* 非法字符直接丢 */
            uint8 digit = (uint8)(s[i] - '0');
            if(frac_cnt) { frac_val = frac_val * 10u + digit; frac_cnt++; }
            else         { int_val  = int_val  * 10u + digit; }
        }

        /* 手动算 10^(frac_cnt-1)，代替 powf：
           "2.5"  → frac_cnt=2 → div=10
           "0.02" → frac_cnt=3 → div=100 */
        float div = 1.0f;
        for(uint8 k = 1u; k < frac_cnt; k++) { div *= 10.0f; }

        result = (float)int_val + (float)frac_val / div;
        return minus_flag ? -result : result;
}

/*
 * T3 串口手动指定 ring_state，只接受 0/1/2/3。
 */
static void pid_set_ring_state_from_serial(float value)
{
    uint8 state;

    if((value < 0.0f) || (value > 3.0f))
    {
        return;
    }

    state = (uint8)value;
    if(value != (float)state)
    {
        return;
    }

    ring_state = state;
}

//-------------------------------------------------------------------------------------------------------------------
//  鍑芥暟璇存槑锛氳В鏋愪竴瀹屾暣甯э紙涓嶅惈缁撳熬 '!'锛夛紝鏇存柊 NUM 缁撴瀯浣�
//            甯ф牸寮忥細棣栧瓧绗� P/I/D 琛ㄧず绫诲瀷锛屾瀛楃 1/2/3 琛ㄧず鐜彿锛�
//            鐒跺悗鏄� '=' 鍜屾暟鍊笺�備緥濡� "P1=2.5"銆�"I3=0.02"銆�"D2=0"
//  鍙傛暟     frame : 甯ф暟鎹紙浠� pid_rx_buf 涓鸿捣鐐癸級
//           len   : 甯ч暱搴�
//-------------------------------------------------------------------------------------------------------------------
static void pid_apply_frame(const uint8 *frame, uint8 len)
{
    uint8 eq_pos = 0;       // '=' 鍦ㄥ抚涓殑浣嶇疆
    uint8 found_eq = 0;
    uint8 i;

    // 1. 鎵惧埌 '=' 鐨勪綅缃�
    for(i = 0; i < len; i++)
    {
        if(frame[i] == '=')
        {
            eq_pos = i;
            found_eq = 1;
            break;
        }
    }
    if(!found_eq) return;   // 甯ч噷娌℃湁 '='锛屼笉鏄悎娉曡皟鍙傛寚浠わ紝涓㈠純

    // 2. 鑷冲皯瑕佹湁 绫诲瀷瀛楁瘝 + 鐜彿 + '='锛屽嵆 eq_pos 鑷冲皯涓� 2
    if(eq_pos < 2) return;

    // 3. 瑙ｆ瀽 '=' 鍚庨潰鐨勬暟鍊�
    float value = pid_parse_float(&frame[eq_pos + 1], len - eq_pos - 1);

    // 4. 鏍规嵁棣栧瓧姣�(绫诲瀷)鍜屾鏁板瓧(鐜彿)鏇存柊瀵瑰簲鍙傛暟
    //    瀵瑰簲鍏崇郴涓� vofa.c 鐨� USART_PID_Adjust() 瀹屽叏涓�鑷达細
    //      P1/I1/D1 -> 瑙掑害鐜�   ;  P2/I2/D2 -> 瑙掗�熷害鐜� ;  P3/I3/D3 -> 閫熷害鐜�
    if(frame[0] == 'P')
    {
        if(frame[1] == '1')      NUM1.angle_p  = value;
        else if(frame[1] == '2') NUM1.anglev_p = value;  //圆环 ring_pid.Kp
        else if(frame[1] == '3') NUM1.speed_p  = value;
        else if(frame[1] == '5') NUM2.speed_p  = value;
    }
    else if(frame[0] == 'I')
    {
        if(frame[1] == '1')      NUM1.angle_i  = value;
        else if(frame[1] == '2') NUM1.anglev_i = value;
        else if(frame[1] == '3') NUM1.speed_i  = value;
        else if(frame[1] == '5') NUM2.speed_i  = value;
    }
    else if(frame[0] == 'D')
    {
        if(frame[1] == '1')      NUM1.angle_d  = value;
        else if(frame[1] == '2') NUM1.anglev_d = value;  //圆环 ring_pid.Kd
        else if(frame[1] == '3') NUM1.speed_d  = value;
        else if(frame[1] == '5') NUM2.speed_d  = value;
    }else if(frame[0] == 'T'){
        if(frame[1] == '1')      car_stop = (uint8)value;    //收到即停车
        else if(frame[1] == '2') ring_done_cnt = (uint8)value; //清空环数
        else if(frame[1] == '3') pid_set_ring_state_from_serial(value);  //清空标志位
        else if(frame[1] == '4') debug_test = (uint8_t)value; //错误显示标志位
        else if(frame[1] == '5') NUM2.angle_p  = value;
        else if(frame[1] == '6') NUM2.anglev_p = value;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  鍑芥暟璇存槑锛歅ID 璋冨弬鏁版嵁澶勭悊鍑芥暟锛堜富寰幆閲屽弽澶嶈皟鐢級
//-------------------------------------------------------------------------------------------------------------------
void pid_tuning_handler(void)
{
    uint8  tmp[32];                 // 涓�娆′粠 FIFO 璇诲嚭鐨勪复鏃剁紦鍐插尯
    uint32 read_num;                // 瀹為檯璇诲埌鐨勫瓧鑺傛暟
    uint8  i;

    // 1. 鎶婃棤绾夸覆鍙� FIFO 涓幇鏈夌殑鏁版嵁灏介噺璇诲嚭鏉ワ紙涓�娆℃渶澶氳 32 瀛楄妭锛�
    read_num = wireless_uart_read_buffer(tmp, sizeof(tmp));

    // 2. 閫愬瓧鑺傚杺缁欑粍甯ч�昏緫
    for(i = 0; i < read_num; i++)
    {
        uint8 ch = tmp[i];

        if(ch == '!')                       // 鏀跺埌 '!'锛岃鏄庝竴甯х粨鏉�
        {
            if(pid_rx_idx > 0)              // 缂撳啿鍖洪噷纭疄鏈夊唴瀹规墠瑙ｆ瀽
            {
                pid_apply_frame(pid_rx_buf, pid_rx_idx);
                pid_rx_idx = 0;             // 娓呯┖锛屽噯澶囨帴鏀朵笅涓�甯�
            }
        }
        else if(ch == '=' || (ch >= '0' && ch <= '9') || ch == '.' ||
                ch == '-' || ch == 'P' || ch == 'I' || ch == 'D' ||ch == 'T'||
                ch == '1' || ch == '2' || ch == '3')
        {
            // 鍙妸鍗忚鍏佽鐨勫瓧绗︽敹杩涚紦鍐插尯锛岃繃婊ゆ帀鎹㈣銆佸洖杞︾瓑鏉傚瓧绗�
            if(pid_rx_idx < sizeof(pid_rx_buf) - 1)
            {
                pid_rx_buf[pid_rx_idx++] = ch;
            }
            else
            {
                pid_rx_idx = 0;             // 缂撳啿鍖烘弧浜嗚繕娌℃敹鍒� '!'锛岃鏄庡抚鎹熷潖锛岀洿鎺ヤ涪寮�
            }
        }
        // 鍏跺畠瀛楃锛堝 '\r'銆�'\n'锛夌洿鎺ュ拷鐣ワ紝涓嶈繘缂撳啿鍖�
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  鍑芥暟璇存槑锛氬湪 5ms 瀹氭椂涓柇閲岃皟鐢ㄧ殑璁℃暟鍑芥暟
//  鍘熺悊锛氭瘡杩涗竴娆′腑鏂� +1锛涚疮璁� 4 娆★紙20ms锛夊氨鎶� send_flag 缃� 1锛岀劧鍚庢竻闆惰鏁板櫒銆�
//       鍙仛鈥滆鏁� + 缃爣蹇椻�濊繖绉嶆瀬鐭搷浣滐紝缁濅笉鍦ㄨ繖閲岀洿鎺ヤ覆鍙ｅ彂閫侊紒
//-------------------------------------------------------------------------------------------------------------------
void pid_tuning_tick(void)
{
    /* 本函数由 CPU0 的 5ms PIT 调用，统一负责 IMU yaw 积分。 */
    imu660rb_yaw_update(0.005f);

    send_tick_cnt++;

    if(send_tick_cnt >= 4)          // 2 脳10ms = 20ms锛屽埌鍙戦�佸懆鏈熶簡
    {
        send_tick_cnt = 0;
        send_flag = 1;              // 鍛婅瘔涓诲惊鐜細鍙互鍙戜簡
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  鍑芥暟璇存槑锛氫富寰幆閲岃皟鐢紝鍒扮偣灏辨妸鍙傛暟鍙戝嚭鍘�
//-------------------------------------------------------------------------------------------------------------------
void pid_tuning_poll_send(void)
{
    if(send_flag)                   // 涓柇閲岀疆浜嗗彂閫佹爣蹇�
    {
        send_flag = 0;              // 鍏堟竻鏍囧織锛屽啀鍙戦�侊紙鍗充娇鍙戦�佷腑閫旇鎵撴柇涔熶笉浼氶噸澶嶅彂锛�
        pid_tuning_send_current();   // 鐪熸鍦ㄨ繖閲屽彂閫侊紙鑰楁椂鎿嶄綔鏀惧湪涓诲惊鐜紝涓嶈�借涓柇锛�
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  鍑芥暟璇存槑锛氭妸褰撳墠 9 涓� PID 鍙傛暟閫氳繃鏃犵嚎涓插彛鍥炰紶缁欎笂浣嶆満
//  上位机 CSV 共 25 列：第 21 列=环岛相对 yaw，第 24 列=IMU 原始 yaw，
//  第 25 列=IMU Z 轴原始角速度（deg/s）。前 23 列保持原顺序不变。
//-------------------------------------------------------------------------------------------------------------------
void pid_tuning_send_current(void)
{
    char buf[252];
    int len;
    PID *active_turn_pid;
    float imu_yaw_deg;
    float imu_gyro_z_dps;

    active_turn_pid = (ring_state == 0U) ? &image_pid : &ring_pid;
    imu_yaw_deg = imu660rb_yaw_get();
    imu_gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

    len = snprintf(
        buf, sizeof(buf),
        "%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%d,%d,%d,%.2f,%.2f,"
        "%.2f,%.2f,%d,%d,%d,"
        "%.5f,%.5f,%.2f,%d,%d,"
        "%.2f,%.2f,%d\n",
        motor_L.encoder_speed,
        motor_L.target_speed,
        motor_pid_L.Kp,
        motor_pid_L.Ki,
        motor_R.encoder_speed,
        motor_R.target_speed,
        motor_pid_R.Kp,
        motor_pid_R.Ki,
        motor_L.duty,
        motor_R.duty,
        pid_error,
        get_turn_pid_output,
        get_turn_cmd,
        active_turn_pid->Kp,
        active_turn_pid->Kd,
        ring_state,
        ring_done_cnt,
        debug_test,
        NUM2.angle_p,
        NUM2.anglev_p,
        ring_gyro_angle_deg,
        imu_ready,
        ring_imu_type,
        imu_yaw_deg,
        imu_gyro_z_dps,
        zebra_count
    );

//    len = snprintf(
//        buf, sizeof(buf),
//        "%.2f\n",
//        angle_z
//        );
    /* 格式化失败或内容被截断时，不发送残缺帧 */
    if(len < 0 || len >= (int)sizeof(buf))
    {
        return;
    }

    wireless_uart_send_string(buf);
}
/* 调试观察变量 */
//volatile uint32 uart_tx_attempts = 0;
//volatile uint32 uart_tx_returns = 0;
//volatile uint32 uart_tx_left = 0xFFFFFFFFu;
//volatile uint8  uart_rts_level = 255;
//
//void pid_tuning_send_current(void)
//{
//    static const uint8 test_frame[] = "1,2,3\n";
//
//    /* 初始化失败时，先解决初始化问题 */
//    if (wireless_init_result != 0)
//    {
//        return;
//    }
//
//    uart_rts_level = gpio_get_level(WIRELESS_UART_RTS_PIN);
//
//    uart_tx_attempts++;
//
//    /*
//     * 返回值是“还有多少字节没发出去”：
//     * 0 = 本次发送完成
//     * 大于 0 = 没发完
//     */
//    uart_tx_left = wireless_uart_send_buffer(
//        test_frame,
//        (uint32)(sizeof(test_frame) - 1u)
//    );
//
//    uart_tx_returns++;
//}
