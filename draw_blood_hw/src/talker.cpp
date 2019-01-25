#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <geometry_msgs/Transform.h>

#include "geometry_msgs/Accel.h"
#include "draw_blood_hw/Puncture.h"
int p_sta;
int run_sta;
int p_cmd;

int subrun=0;
 int z_sta1 = 0;
 int z_sta2=0;

 int ar_sta1 = 0;
 int ar_sta2=0;

 int pos_sta1 = 0;
 int pos_sta2=0;

 int range1 = 0;
 int range2 =0;

int yali = 0;
int juli = 0;

int sta_pub_en = 0;
int yali_pub_en = 0;
int juli_pub_en = 0;

//回调函数
// void puncture_cmd_callback(const draw_blood_hw::Puncture &msg)
// {
//   ROS_INFO("puncture_cmd:[%d,%d,%d]",msg.Puncture_id,msg.Puncture_cmd,msg.Puncture_data);
//   if(p_cmd==0)
//   {
//     switch(msg.Puncture_cmd)
//     {
//       case 1:
//       {
//         subrun=1;
//         p_cmd=msg.Puncture_cmd;
//         run_sta=1;//ing
//       }
//       break;
//       case 2:
//       {
//         if(p_sta==1)
//         {
//           subrun=1;
//           p_cmd=msg.Puncture_cmd;
//           range1=msg.Puncture_xpos;
//           range2=msg.Puncture_ypos;
//           run_sta=1;//ing
//         }
//       }
//       break;
//       case 3:         //获取压力
//       case 4:         //获取距离
//       {
//         subrun=1;
//         p_cmd=msg.Puncture_cmd;
//         run_sta=1;//ing
//       }
//       break;
//       default:
//       break;
//     }
//   }
// }

void puncture_cmd_callback(const geometry_msgs::Transform &msg)
{

  if(p_cmd==0)
  {
    switch((int)msg.rotation.x)
    {
      case 1:
      {
        subrun=1;
        p_cmd=(int)msg.rotation.x;
        run_sta=1;//ing
      }
      break;
      case 2:
      {
        if(p_sta==1)
        {
          subrun=1;
          p_cmd=(int)msg.rotation.x;
          range1=(int)msg.rotation.y;
          range2=(int)msg.rotation.z;
          run_sta=1;//ing
        }
      }
      break;
      case 3:         //获取压力
      case 4:         //获取距离
      {
        subrun=1;
        p_cmd=(int)msg.rotation.x;
        run_sta=1;//ing
      }
      break;
      default:
      break;
    }
  }
}

//回调函数
class Listener
{
  public:
  void callback(const draw_blood_hw::Puncture &msg);
};
//void Listener::callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
void Listener::callback(const draw_blood_hw::Puncture &msg)
{
	ROS_INFO("Puncture message:[%d,%d,%d]",msg.Puncture_id,msg.Puncture_cmd,msg.Puncture_data);
    switch(msg.Puncture_cmd)
    {
        case 40013:
        {
          if(msg.Puncture_id==1)
            ar_sta1=msg.Puncture_data;
          else if(msg.Puncture_id==2)
            ar_sta2=msg.Puncture_data;
        }
        break;
        case 40023:
        {
          if(msg.Puncture_id==1)
            pos_sta1=msg.Puncture_data;
          else if(msg.Puncture_id==2)
            pos_sta2=msg.Puncture_data;
        }
        break;
        case 40043:
        {
          if(msg.Puncture_id==1)
            z_sta1=msg.Puncture_data;
          else if(msg.Puncture_id==2)
            z_sta2=msg.Puncture_data;
        }
        break;
        case 40201:
        {
          if(msg.Puncture_id==0xf1)
          {
            yali=msg.Puncture_data;
            yali_pub_en=1;
          }
        }
        break;
        case 40016:
        {
          if(msg.Puncture_id==0x10)
          {
            juli=msg.Puncture_data;
            juli_pub_en=1;
          }
        }
        break;
        
        default :
        break;
    }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  int count = 0;
  p_sta=0;
  p_cmd=0;
  range1=0;
  range2=0;
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  //订阅主题
  Listener listener;
  ros::Subscriber read_sub = n.subscribe("read", 1000, &Listener::callback, &listener);
  ros::Subscriber puncture_sub = n.subscribe("/ur_driver/effort_command", 1000, puncture_cmd_callback);
  ros::AsyncSpinner Spinner(2);
  //发布主题
  ros::Publisher write_pub = n.advertise< draw_blood_hw::Puncture >("write", 1000);
  ros::Publisher puncture_pub = n.advertise< draw_blood_hw::Puncture >("puncture_sta", 1000);

  ros::Publisher yali_pub = n.advertise< draw_blood_hw::Puncture >("puncture_yali", 1000);
  ros::Publisher juli_pub = n.advertise< draw_blood_hw::Puncture >("puncture_juli", 1000);
  
  ros::Rate loop_rate(20);
  draw_blood_hw::Puncture msg;
  while (ros::ok())
  {
    if(p_cmd!=0)
    {
      switch(p_cmd)
      {
        case 1:                   //init
        {
          switch(subrun)          //setup
          {
            case 1:
            {
              z_sta1=0;
              z_sta2=0;
              
              msg.Puncture_id=2;
              msg.Puncture_cmd=10;
              msg.Puncture_data=0;
              write_pub.publish(msg);
              subrun=2;
            }
            break;
            case 2:
            {
              if(z_sta2==0x01)
              {
                subrun=3;
                break;
              }
              msg.Puncture_id=2;
              msg.Puncture_cmd=40043;
              msg.Puncture_data=0;
              write_pub.publish(msg);
            }
            break;
            case 3:
            {
              msg.Puncture_id=1;
              msg.Puncture_cmd=10;
              msg.Puncture_data=0;
              write_pub.publish(msg);
              subrun=4;
            }
            break;
            case 4:
            {
              if(z_sta1==0x01)
              {
                subrun=5;
                break;
              }
              msg.Puncture_id=1;
              msg.Puncture_cmd=40043;
              msg.Puncture_data=0;
              write_pub.publish(msg);
            }
            break;
            case 5:
            {
              msg.Puncture_id=2;
              msg.Puncture_cmd=11;
              msg.Puncture_data=0;
              write_pub.publish(msg);
              subrun=6;
            }
            break;
            case 6:
            {
              msg.Puncture_id=1;
              msg.Puncture_cmd=11;
              msg.Puncture_data=0;
              write_pub.publish(msg);
              p_sta=1;//init ok
              run_sta=2;//done
              p_cmd=0;
            }
            break;
            
            default:
            break;
          }
        }
        break;
        case 2:
        {
          switch(subrun)          //setup
          {
            case 1:
            {
              msg.Puncture_id=1;
              msg.Puncture_cmd=40010;
              msg.Puncture_data=range1;
              write_pub.publish(msg);
              subrun=2;
            }
            break;
            case 2:
            {
              msg.Puncture_id=2;
              msg.Puncture_cmd=40010;
              msg.Puncture_data=range2;
              write_pub.publish(msg);
              subrun=3;
            }
            case 3:
            {
              ar_sta1 = 0;
              ar_sta2 = 0;
              msg.Puncture_id=1;
              msg.Puncture_cmd=8;
              msg.Puncture_data=0;
              write_pub.publish(msg);
              subrun=4;
            }
            break;
            case 4:
            {
              if(ar_sta1==0x01)
              {
                subrun=5;
                break;
              }
              msg.Puncture_id=1;
              msg.Puncture_cmd=40013;
              msg.Puncture_data=0;
              write_pub.publish(msg);
            }
            break;
            case 5:
            {
              msg.Puncture_id=2;
              msg.Puncture_cmd=8;
              msg.Puncture_data=0;
              write_pub.publish(msg);
              subrun=6;
            }
            break;
            case 6:
            {
              if(ar_sta2==0x01)
              {
                run_sta=2;//done
                p_cmd=0;
                break;
              }
              msg.Puncture_id=2;
              msg.Puncture_cmd=40013;
              msg.Puncture_data=0;
              write_pub.publish(msg);
            }
            break;
            default:
            break;
          }
        }
        break;
        case 3:
        {
          msg.Puncture_id=0xf1;
          msg.Puncture_cmd=40201;
          msg.Puncture_data=0;
          write_pub.publish(msg);
          run_sta=2;//done
          p_cmd=0;
        }
        break;
        case 4:
        {
          msg.Puncture_id=0x10;
          msg.Puncture_cmd=40016;
          msg.Puncture_data=0;
          write_pub.publish(msg);
          run_sta=2;//done
          p_cmd=0;
        }
        break;
        
        default:
        break;
      }
    }
    if(count%5==0)
    {
      msg.Puncture_id=1;
      msg.Puncture_cmd=40023;
      msg.Puncture_data=0;
      write_pub.publish(msg);
      msg.Puncture_id=2;
      msg.Puncture_cmd=40023;
      msg.Puncture_data=0;
      write_pub.publish(msg);
    }
    if(count%2==1)
    {
      msg.Puncture_id=1;
      msg.Puncture_cmd=2;
      msg.Puncture_data=run_sta;
      msg.Puncture_xpos=pos_sta1;
      msg.Puncture_ypos=pos_sta2;
      puncture_pub.publish(msg);
    }

    if(yali_pub_en==1)
    {
      yali_pub_en=0;
      msg.Puncture_id=0xf1;
      msg.Puncture_cmd=3;
      msg.Puncture_data=yali;
      msg.Puncture_xpos=0;
      msg.Puncture_ypos=0;
      yali_pub.publish(msg);
    }
    if(juli_pub_en==1)
    {
      juli_pub_en=0;
      msg.Puncture_id=0x10;
      msg.Puncture_cmd=4;
      msg.Puncture_data=juli;
      msg.Puncture_xpos=0;
      msg.Puncture_ypos=0;
      juli_pub.publish(msg);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

