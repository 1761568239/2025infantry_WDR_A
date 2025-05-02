#include "ano.h"
#include "usart.h"

/*!
  * @brief    Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
  *
  * @param    dataToSend   :   Ҫ���͵������׵�ַ
  * @param    length       :   Ҫ���͵����ݳ���
  *
  * @return   ��
  *
  * @note     ��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
  *
  * @see      �ڲ�����
  *
  * @date     2019/5/28 ���ڶ�
  */
void ANO_DT_Send_Data(unsigned char *dataToSend , unsigned short length)
{
	HAL_UART_Transmit(&huart1, dataToSend, length, 1000);
}

/*
 * ����: Vofa_SendData
 * -------------------
 * ��������ͨ���ض�Э�飬��������ֵ������ֽ����顣
 *
 * ����:
 *     data0 - float: Ҫ���͵ĵ�һ��������ֵ��
 *     data1 - float: Ҫ���͵ĵڶ���������ֵ��
 *     data2 - float: Ҫ���͵ĵ�����������ֵ��
 *     data3 - float: Ҫ���͵ĵ��ĸ�������ֵ��
 *     data4 - float: Ҫ���͵ĵ����������ֵ��
 *     data5 - float: Ҫ���͵ĵ�����������ֵ��
 *     len   - uint8_t: Ҫ���͵����ݳ��ȡ�
 *
 * ����ֵ:
 *     ��
 *
 * ע��:
 *     �ú������ṩ�ĸ������ݴ����һ���ֽ������У�ȷ����ͨ�Ŷ˵ļ����ԡ�
 *     Ȼ���ڷ�����֮ǰ�����ض��Ŀ����ֽڸ��ӵ������С�
 *     ���ݳ��Ȳ�Ӧ����64�ֽڡ�
 */
void Vofa_SendData(float data0, float data1, float data2, float data3, float data4, float data5, uint8_t len)
{
    static uint8_t tmp_dat[64]; // ��ʱ���飬���ڴ洢���������
    uint8_t cnt = len * sizeof(float); // ����������������ֽ���
    float data[6] = {data0, data1, data2, data3, data4, data5}; // ����һ�������Ա�����������
    uint8_t i = 0; // ѭ��������

    // ���������ݴ�����ֽ�������
    for (i = 0; i < cnt; i++)
    {
        tmp_dat[i] = *((uint8_t *)data + i);
    }

    // �������ֽڸ��ӵ��ֽ�������
    tmp_dat[i++] = 0x00; // �����ֽ�1
    tmp_dat[i++] = 0x00; // �����ֽ�2
    tmp_dat[i++] = 0x80; // �����ֽ�3
    tmp_dat[i++] = 0x7f; // �����ֽ�4

    // ͨ��ͨ�Žӿڷ��ʹ��������
    ANO_DT_Send_Data(tmp_dat, i);
}