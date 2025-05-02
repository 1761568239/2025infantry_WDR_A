#include "ano.h"
#include "usart.h"

/*!
  * @brief    Send_Data函数是协议中所有发送数据功能使用到的发送函数
  *
  * @param    dataToSend   :   要发送的数据首地址
  * @param    length       :   要发送的数据长度
  *
  * @return   无
  *
  * @note     移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
  *
  * @see      内部调用
  *
  * @date     2019/5/28 星期二
  */
void ANO_DT_Send_Data(unsigned char *dataToSend , unsigned short length)
{
	HAL_UART_Transmit(&huart1, dataToSend, length, 1000);
}

/*
 * 函数: Vofa_SendData
 * -------------------
 * 发送数据通过特定协议，将浮点数值打包成字节数组。
 *
 * 参数:
 *     data0 - float: 要发送的第一个浮点数值。
 *     data1 - float: 要发送的第二个浮点数值。
 *     data2 - float: 要发送的第三个浮点数值。
 *     data3 - float: 要发送的第四个浮点数值。
 *     data4 - float: 要发送的第五个浮点数值。
 *     data5 - float: 要发送的第六个浮点数值。
 *     len   - uint8_t: 要发送的数据长度。
 *
 * 返回值:
 *     无
 *
 * 注意:
 *     该函数将提供的浮点数据打包到一个字节数组中，确保与通信端的兼容性。
 *     然后在发送它之前，将特定的控制字节附加到数组中。
 *     数据长度不应超过64字节。
 */
void Vofa_SendData(float data0, float data1, float data2, float data3, float data4, float data5, uint8_t len)
{
    static uint8_t tmp_dat[64]; // 临时数组，用于存储打包的数据
    uint8_t cnt = len * sizeof(float); // 计算数据所需的总字节数
    float data[6] = {data0, data1, data2, data3, data4, data5}; // 创建一个数组以保存输入数据
    uint8_t i = 0; // 循环计数器

    // 将浮点数据打包到字节数组中
    for (i = 0; i < cnt; i++)
    {
        tmp_dat[i] = *((uint8_t *)data + i);
    }

    // 将控制字节附加到字节数组中
    tmp_dat[i++] = 0x00; // 控制字节1
    tmp_dat[i++] = 0x00; // 控制字节2
    tmp_dat[i++] = 0x80; // 控制字节3
    tmp_dat[i++] = 0x7f; // 控制字节4

    // 通过通信接口发送打包的数据
    ANO_DT_Send_Data(tmp_dat, i);
}