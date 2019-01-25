#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "crc.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Accel.h"
#include "draw_blood_hw/Puncture.h"

#define PLCON   0Xff00
#define PLCOFF  0X00ff
#define TBUFF_DEEP  100

serial::Serial ser; //声明串口对象

int recevie_reg;
struct tx_buff
{
    int ByteNum;
    int Reg;
    uint8_t Buffer[100];
};

tx_buff C_Buff[TBUFF_DEEP];

uint8_t tbuff_now,buff_full;
//回调函数
void write_callback(const draw_blood_hw::Puncture &msg)
{
    crc::CRC cc_crc;
    unsigned int test_crc,cnt;
    static uint8_t s_buffer[100];
    int cmd_tmp;
    memset(s_buffer,0,sizeof(s_buffer));
	ROS_INFO("Puncture_CMD:ID=d Cmd=%d data=%d",msg.Puncture_id,msg.Puncture_cmd,msg.Puncture_data);
    cmd_tmp=(msg.Puncture_cmd-1)%40000;
    cnt=0;
    switch(msg.Puncture_cmd)
    {
        case 7:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x05;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = (PLCON & 0xff00)>>8;
            s_buffer[5] = (PLCON & 0x00ff)>>0;
            cnt=6;
        }
        break;
        case 8:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x05;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = (PLCON & 0xff00)>>8;
            s_buffer[5] = (PLCON & 0x00ff)>>0;
            cnt=6;
        }
        break;
        case 10:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x05;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = (PLCON & 0xff00)>>8;
            s_buffer[5] = (PLCON & 0x00ff)>>0;
            cnt=6;
        }
        break;
        case 11:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x05;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = (PLCON & 0xff00)>>8;
            s_buffer[5] = (PLCON & 0x00ff)>>0;
            cnt=6;
        }
        break;
        case 40009:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x06;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = (msg.Puncture_data & 0x0000ff00)>>8;
            s_buffer[5] = (msg.Puncture_data & 0x000000ff);
            cnt=6;
        }
        break;
        case 40010:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x10;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = 0x00;
            s_buffer[5] = 0X02;
            s_buffer[6] = 0X04;
            s_buffer[7] = (msg.Puncture_data & 0x0000ff00)>>8;
            s_buffer[8] = (msg.Puncture_data & 0x000000ff);
            s_buffer[9] = (msg.Puncture_data & 0xff000000)>>24;
            s_buffer[10] = (msg.Puncture_data & 0x00ff0000)>>16;
            cnt=11;
        }
        break;
        case 40012:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x06;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = (msg.Puncture_data & 0x0000ff00)>>8;
            s_buffer[5] = (msg.Puncture_data & 0x000000ff);
            cnt=6;
        }
        break;
        case 40013:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x03;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = 0X00;
            s_buffer[5] = 0x01;
            cnt=6;
        }
        break;
        case 40023:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x03;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = 0X00;
            s_buffer[5] = 0x02;
            cnt=6;
        }
        break;
        case 40043:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x03;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = 0X00;
            s_buffer[5] = 0x01;
            cnt=6;
        }
        break;
        case 40201:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x03;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = 0X00;
            s_buffer[5] = 0x02;
            cnt=6;
        }
        break;
        case 40016:
        {
            //数据打包
            s_buffer[0] = msg.Puncture_id;
            s_buffer[1] = 0x03;
            s_buffer[2] = (cmd_tmp & 0xff00)>>8;
            s_buffer[3] = (cmd_tmp & 0x00ff)>>0;
            s_buffer[4] = 0X00;
            s_buffer[5] = 0x02;
            cnt=6;
        }
        break;
        default :
        break;
    }
    if(cnt>0)
    {
        test_crc= cc_crc.CRC16(s_buffer,cnt);
        s_buffer[cnt++] = test_crc;
        s_buffer[cnt++] = test_crc>>8;
        ROS_INFO("buff_full: %d ", buff_full);
        C_Buff[buff_full].ByteNum=cnt;
        C_Buff[buff_full].Reg=msg.Puncture_cmd;
        memcpy(C_Buff[buff_full].Buffer,s_buffer,cnt);

        buff_full++;
        buff_full%=TBUFF_DEEP;
    }
}

int main (int argc, char** argv)
{
    crc::CRC cc_crc;
    static int counter;
    static int smx=0;
    int i;
    static int p,ptmp;
    unsigned int test_crc;
    uint8_t r_buff[200];
    tbuff_now=0;
    buff_full=0;
    C_Buff[0].ByteNum=0;
    recevie_reg=0;
    std_msgs::UInt8MultiArray  r_buffer;
    draw_blood_hw::Puncture PunctureSta;

    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;

    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    //ros::Publisher read_pub = nh.advertise<std_msgs::UInt8MultiArray>("read", 1000);
    //ros::Publisher read_pub = nh.advertise<geometry_msgs::Accel>("read", 1000);
    ros::Publisher read_pub = nh.advertise< draw_blood_hw::Puncture >("read", 1000);
    try
    {
    //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //指定循环的频率
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        if(ser.available())
        {
            //std_msgs::String serial_data;
            //std_msgs::UInt8MultiArray  serial_data;

            p=0;
            while(ser.available())
            {
                ptmp=ser.available();
                ser.read (&r_buff[p],ptmp);
                p+=ptmp;
                loop_rate.sleep();
            }

            //获取串口数据
            // p=ser.available();
            // ser.read (serial_data.data,p);

            ROS_INFO("serial  heard: %d ", p);
            r_buffer.data.resize(p);
            for(i=0; i<p; i++)
            {
                r_buffer.data[i]=r_buff[i];
                ROS_INFO("r_buffer.data[%d]=%x",i,r_buffer.data[i]);
            }
            if(r_buffer.data[0]==0x01 || r_buffer.data[0]==0x02|| r_buffer.data[0]==0xf1|| r_buffer.data[0]==0x10)
            {
                if(cc_crc.CRC16(r_buff,p)==0);
                {
                    switch(recevie_reg)
                    {
                        case 7:
                        case 8:
                        case 10:
                        case 11:
                        case 40009:
                        case 40010:
                        case 40012:
                        {
                            PunctureSta.Puncture_id=r_buffer.data[0];
                            PunctureSta.Puncture_cmd=recevie_reg;
                            PunctureSta.Puncture_data=0;
                            recevie_reg=0;
                            read_pub.publish(PunctureSta);
                        }
                        break;
                        case 40013:
                        case 40043:
                        {
                            PunctureSta.Puncture_id=r_buffer.data[0];
                            PunctureSta.Puncture_cmd=recevie_reg;
                            PunctureSta.Puncture_data=(r_buffer.data[3]<<8)|(r_buffer.data[4]<<0);
                            recevie_reg=0;
                            read_pub.publish(PunctureSta);
                        }
                        break;
                        case 40023:
                        {
                            PunctureSta.Puncture_id=r_buffer.data[0];
                            PunctureSta.Puncture_cmd=recevie_reg;
                            PunctureSta.Puncture_data=(r_buffer.data[5]<<24)|(r_buffer.data[6]<<16)|(r_buffer.data[3]<<8)|(r_buffer.data[4]<<0);
                            recevie_reg=0;
                            read_pub.publish(PunctureSta);
                        }
                        break;
                        case 40201:
                        {
                            PunctureSta.Puncture_id=r_buffer.data[0];
                            PunctureSta.Puncture_cmd=recevie_reg;
                            PunctureSta.Puncture_data=(r_buffer.data[5]<<24)|(r_buffer.data[6]<<16)|(r_buffer.data[3]<<8)|(r_buffer.data[4]<<0);
                            recevie_reg=0;
                            read_pub.publish(PunctureSta);
                        }
                        break;
                        case 40016:
                        {
                            PunctureSta.Puncture_id=r_buffer.data[0];
                            PunctureSta.Puncture_cmd=recevie_reg;
                            PunctureSta.Puncture_data=(r_buffer.data[3]<<24)|(r_buffer.data[4]<<16)|(r_buffer.data[5]<<8)|(r_buffer.data[6]<<0);
                            recevie_reg=0;
                            read_pub.publish(PunctureSta);
                        }
                        break;
                        
                        default:
                        break;
                    }
                }
            }
        }

        if(C_Buff[tbuff_now].ByteNum!=0 && recevie_reg==0)
        {
            ROS_INFO("tbuff_now: %d ", tbuff_now);
            ser.write(C_Buff[tbuff_now].Buffer,C_Buff[tbuff_now].ByteNum);
            C_Buff[tbuff_now].ByteNum=0;
            recevie_reg=C_Buff[tbuff_now].Reg;

            tbuff_now++;
            tbuff_now%=TBUFF_DEEP;
        }
        loop_rate.sleep();
    }
}


//   <build_depend>message_generation</build_depend>
//   <build_depend>message_runtime</build_depend>
//   <run_depend>message_runtime</run_depend>

