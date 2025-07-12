from pyb import UART


uart = UART(1, 115200 , timeout_char=200)

#--------------------串口------------------
def send_custom_packet(frame_type, data):
    """
    发送指定帧类型的数据包。
    参数：
        frame_type: int (1~15)，例如1表示帧头0xF1、帧尾0xE1
        data: list 或 bytearray，要发送的数据内容
    """
    #assert 1 <= frame_type <= 15, "帧类型必须在1~15之间"
    header = 0xF0 + frame_type
    footer = 0xE0 + frame_type
    packet = bytearray([header]) + bytearray(data) + bytearray([footer])
    uart.write(packet)

if __name__ == "__main__":
    # 示例：发送帧类型1的数据包
    frame_type = 1
    data = [0x01, 0x02, 0x03]  # 示例数据
    send_custom_packet(frame_type, data)
    print("数据包已发送")
