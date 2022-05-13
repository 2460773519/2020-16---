#include "Control.h"
#include "MENU.h"

#define PB 5
#define PM 4
#define PS 3
#define ZO 0
#define NS 3
#define NM 4
#define NB 5



//方向环变量
int duty=0;             
int k_mid=0;
int P1=21,d1=0;
int LOW1=0;
int uwb_flag=0;
int kp_g=25,kd_g=5;    // 25 5
float turn_on_out=0;
float err_on=0;
float fuzz_p=0;
float fuzz_kp=10;
int stop_cuan=0,p_g=0;


//速度环变量
int val,var;
int las_v=0;
int pwm_speed=0,expd_speed=240,kp_speed=7,speed,kp_low=5,expd_speed_low=250; 
float Left_OutPWM = 0.f;
float Right_OutPWM = 0.f;
float speed_P = 0.01f; 
float speed_I = 0.08f;
int d_speed=0;
int pwm_tai=0;
int kp_tai=31;
int ca_su=1240;
int left_duty,right_duty;

int L_error[2]={0},R_error[2]={0};
int las_speed=0;
char abs_stop=0;
int i5=0,s_p_copy=0;
int s_p_1=0;
int speed_Angle=0;
int zhili_gogogo=0;
int sancha_z=0;
int huaxin=0;



//直立变量
int err,Angle,expd=0;   //73
int pwm_station,last_icm;
double fGyroscopeAngleIntegral=0;  //融合角度
double g_fGyroscopeAngleSpeed;    //归一陀螺仪值
double g_fGravityAngle;           //归一加速度
int GYROSCOPE_ANGLE_SIGMA_FREQUENCY=167;  //陀螺仪调整频率   小会超调(大跟随时间变长)  //2.1 140// 1.3 167 // 1.0  62  
float GYROSCOPE_ANGLE_RATIO=1.3;     

int kp_ping,kd_ping;
float g_x=0;
int s_p=0,sp_yaun_su=0;
int use_time=0;
int s_p_Angel=-20;
int s_p_speed=20;
int kai=0;
int s_p_mc=2200;
int wan_point=0,sum_m=0;



//uwb通信

int stop_time=35; //发送次数  35
int gogogo_dis=1000;
char fon='B',las_fon=0;
int yaokong=0,ZUOYOU=0;
int jianjianjian=0;
int go_pao=0;
int first_qiu=0;
int a1=0;


//编码器
int stop_js=0;
int mc_quan=0;
int temp1=1;
int banma_mc=0;
int cha_speed=0;
int very_long_mc=0;
int very_long=0;
int very_long_dis=3000;
int very_long_con=2;

//蓝牙
unsigned short int send_data[6];  
int time_s=0,cont=0,quan_while=0;

//电赛变量
int shuzi_num=0;
int first_sizi=0;
int VP=97,VI=17;       //速度环
int kp=40,kd=25;     //转向环
int hope=0;
int hopes=105;
int left_hope=0,right_hope=0;
int left_duty,right_duty;
int jifen_angle=2650;
float angle_ji=0;    //积分角度 90度
int mc_dis=80;// 十字走的脉冲
int sum_mc=0;
int lu_sz[2]={5,5};
int err_kz=30;

int car_zk=1;  //状态1 跑车
int go_flash=1; 
int shizi=0;
int shizi_num=0,shizi_fx=0;
int delay_time=0,delay_time_1=0;
int v=0;
int no_ft=0;
int car_4_mc=0;
int bbb=0;
int banma_4=0;
int first_4_sizi=0;
int first_go_go=0;
int stop_mc_flash=0;
int choci=0;
int delay_ttt=0;
int data_las=0;
int time_go=0;
int fx_fx_fx=0;
int num_go_ci=0;
int baaba=0;


uint8  wuxian_data=0,opmv_data=0;

/*---------------------------------------------------------------
---------------------------------------------------------
-----------------------------------------------------------------*/

void TIM6_zhoduan()
{
  static int i1=0,i2=0;
  
  get_icm20602_accdata();
  get_icm20602_gyro();	
  
  if(gpio_get(B5))
	val = tim_counter_get_count(TIM_3);
  else
    val = -tim_counter_get_count(TIM_3);
  if(gpio_get(B7))
    var = -tim_counter_get_count(TIM_4);
  else
    var = tim_counter_get_count(TIM_4);
  
  tim_counter_rst(TIM_3);
  tim_counter_rst(TIM_4);
  var=var*6;
  val=val*6;
  
  v=(var+val)/20.0;
  
  if(shizi==3)
  {
    if(beepds==0) beepds=3;
    sum_mc=sum_mc+v;
    if(sum_mc>=250)
    {
      shizi=2;
      sum_mc=0;
      
    }
  }
  if(time_go>=1)
  {
    if(time_go==1)
    {
      if(fx_fx_fx==1)  opmv_data=0x11;
      if(fx_fx_fx==2)  opmv_data=0x12;
    }
    time_go--;
  }
  
  if(delay_ttt>=1)
  {
    
    if(delay_ttt==1)
    {
      expd=1;
    }
    delay_ttt--;
     if(opmv_data!=0) delay_ttt=0;
  }
  if(shizi==2&&(opmv_data!=0||car_zk==4)) 
  {
    expd=0;
   //  if(beepds==0) beepds=3;
    sum_mc=sum_mc+v; //十字延迟打角
    no_ft=1;
    hope=hopes;
   // move=0;
  }
  if((sum_mc>=370&&shizi==2&&car_zk!=4)||(sum_mc>=600&&shizi==2&&car_zk==4))
  {
    
    no_ft=0;
    shizi=1;
    sum_mc=0;
    
  }
  
  if(car_zk==5&&shizi==1)
  {
    sum_mc=sum_mc+v;
    if(sum_mc>=1700)
    {
      beepds=20;
      sum_mc=0;
      shizi=0;
    }
  }
  
  if(car_4_mc==1)   //后端十字延迟打角
  {
    sum_mc=sum_mc+v; 
    
    if(sum_mc>=300)
    {
     // banma_4=1;
      car_4_mc=0;
      sum_mc=0;
      car_zk=4;
      banma_4=1;
    }
  }
  
  
  if(first_go_go==1)
  {
     no_ft=1;
    sum_mc=sum_mc+v;
    if(sum_mc>=30)
    {
      no_ft=0;
      sum_mc=0;
      first_go_go=0;
      //beepds=5;
    }
  }
  
  if(stop_mc_flash==1)
  {
    sum_mc=sum_mc+v;
    if((sum_mc>=200&&choci==0)||(sum_mc>=50&&choci==1))
    {
      choci=0;
      stop_mc_flash=0;
      hope=0;
      no_ft=1;
      sum_mc=0;
    }
  }
  if(delay_time==1)
  {
    if(shuzi_num==1) opmv_data=0x11;
    if(shuzi_num==2) opmv_data=0x12;
  }
  if(delay_time>0) delay_time--;
  
  if(delay_time_1==1) car_zk=3; //识别到病房停车}
  if(delay_time_1>0&&(gpio_get(bo_4)==1)) 
 // if(delay_time_1>0) 
  {
    uart_putchar(UART_4,'M');// opmv 识别数字
    gpio_set(red,0);
   // if(beepds==0) beepds=3;
    delay_time_1--;
  }
//  var= tim_encoder_get_count(TIM_4)*200/170;								// 采集对应编码器数据
//  val=-tim_encoder_get_count(TIM_3)*200/170;// 采集对应编码器数据    
   turn();             //究极转向环控制
   //if(stop==0) hope=0;
    if(stop==0) Speed_loop();
  
}
void turn() 
{
  int i=0;
  int sum=0;
  if(expd==10)
  {
    for(i=53;i<=58;i++)
    {
       sum=sum+(mid[i]-79);  
    }
    err_ft=(err_ft*8+(sum/5)*2)/10;
  }
  else
  {
  if(car_zk==3||car_zk==5) 
  {
     for(i=50;i<=55;i++)
    {
       sum=sum+(mid[i]-79);  
    }
    err_ft=(err_ft*8+(sum/5)*2)/10;
  }
  else
  {
    for(i=40;i<=50;i++)
    {
       sum=sum+(mid[i]-79);  
    }
    err_ft=(err_ft*8+(sum/10)*2)/10;
  }
  }
    
    if(shizi==1&&car_zk==0) //提车时十字的控制 A:左转到病房 B：右转到病房 C：直行
    {        
        // D:左转没到   E：右转没到
       if(opmv_data==0x11)  //左转到病房
       {
         // if(beepds==0) beepds=3;
         no_ft=0;
         sum_mc=sum_mc+v;
         lu_sz[0]=0;//左转
         hope=hopes;
         if(fabs(angle_ji)<=jifen_angle+20)
         {
          
           angle_ji=angle_ji+icm_gyro_x/100.0;
           err_ft=-(err_kz*v)/10.0; 
         }
         else
         {
           no_ft=1;
         }
         if(sum_mc>=1600) 
         {
           
           gpio_set(red,1);
           no_ft=1;
           delay_time_1=200;
           sum_mc=0;
         //  car_zk=13; //识别到病房停车
           hope=0;
           //beepds=20;
           shizi=0;
           opmv_data=0;    //状态清零
           angle_ji=0;
         } 
       }
       
       if(opmv_data==0x13)  //左转没到
       {
         no_ft=0;
        // sum_mc=sum_mc+v;
         lu_sz[1]=0;//左转
         hope=hopes;
         if(fabs(angle_ji)<=jifen_angle+20)
         {
           angle_ji=angle_ji+icm_gyro_x/100.0;
           err_ft=-(err_kz)*v/10.0; 
         }
        else
         {
           no_ft=1;
           sum_mc=0;
           car_zk=1; //识别到病房停车
           hope=hopes;
         //  beepds=20;
           shizi=0;
           opmv_data=0;    //状态清零
           angle_ji=0;
         } 
       }
       
       if(opmv_data==0x12)  //右转到病房
       {
         no_ft=0;
         sum_mc=sum_mc+v;
         lu_sz[0]=1;//右转
         hope=hopes;
         if(fabs(angle_ji)<=jifen_angle)
         {
           angle_ji=angle_ji+icm_gyro_x/100.0;
           err_ft=(err_kz*v)/10.0; 
         }
         else
         {
           no_ft=1;
         }
         if(sum_mc>=1600) 
         {
           gpio_set(red,1);
           no_ft=1;
           delay_time_1=200;
           sum_mc=0;
          // car_zk=13; //识别到病房停车
           hope=0;
       //    beepds=20;
           shizi=0;
           opmv_data=0;    //状态清零
           angle_ji=0;
         } 
       }
       
       if(opmv_data==0x14)  //右转没到
       {
        no_ft=0;
       //  sum_mc=sum_mc+v;
         lu_sz[1]=1;//右转
         hope=hopes;
         if(fabs(angle_ji)<=jifen_angle)
         {
           angle_ji=angle_ji+icm_gyro_x/100.0;
           err_ft=(err_kz)*v/10.0; 
         }
         else
         {
           no_ft=1;
           sum_mc=0;
           car_zk=1; //识别到病房停车
           hope=hopes;
          // beepds=20;
           shizi=0;
           opmv_data=0;    //状态清零
           angle_ji=0;
         } 
       }
       if(opmv_data==0x15) //直行
       {
         //no_ft=1;
         data_las=opmv_data;
         sum_mc=sum_mc+v;
         hope=hopes;
         
         if(sum_mc>=1350) 
         {
           no_ft=0;
           sum_mc=0;
           car_zk=1; //识别到病房停车
        //   beepds=20;
           shizi=0;
           opmv_data=0;    //状态清零
         } 
       }
    }
    

   if(car_zk==3&&car_4_mc==0)                  //货物被卸下 倒车
   {
      if(sum_mc<=1330)
      {
         hope=-hopes;    
         no_ft=1;
         sum_mc=sum_mc-v;  //倒车控制
      }
      else
      {  
         hope=0;
         no_ft=0;
         if(lu_sz[0]==1)  err_ft=25 ;//右转
         if(lu_sz[0]==0)  err_ft=-25;
         
        angle_ji=angle_ji+icm_gyro_x/100.0;
        if((fabs(angle_ji)>=jifen_angle&&lu_sz[0]==1)||(fabs(angle_ji)>=jifen_angle&&lu_sz[0]==0))
        {
          sum_mc=0;
          angle_ji=0;
          no_ft=0;
          hope=hopes;  
        //  beepds=5;
          if(lu_sz[1]==5)
          {
            if(shuzi_num==1||shuzi_num==2) hope=hope-20;
            else  hope=hope+30;
            car_zk=5;
          }  //冲刺回家}
          else{car_4_mc=1;shizi=0;}           //等待十字 
        }
      }  
   }
   if(car_zk==4&&shizi==1)
   {
    // if(beepds==0)    beepds=3;
     if(lu_sz[1]==0)  err_ft=(err_kz)*v/10.0;
     if(lu_sz[1]==1)  err_ft=-(err_kz)*v/10.0;
    angle_ji=angle_ji+icm_gyro_x/100.0;
    if(fabs(angle_ji)>=jifen_angle-100)
    {
     // STOP();
      choci=1;
      hope=hope+90; ///
     // no_ft=1;
      angle_ji=0;
      car_zk=5;
      first_4_sizi=1;
     // err_ft=0;
    }
     
   }

//   
   if(no_ft==1) err_ft=0;
   // if(no_ft==1&&beepds==0) beepds=3;
    duty=kp*err_ft/10.0-kd*icm_gyro_x/1000.0;//5毫秒更新一次转向外环 

    left_hope=hope+duty;
    right_hope=hope-duty;

//     if(left_hope>=900) left_hope=900;
//     if(left_hope<=-800) left_hope=-800;
}



void Speed_loop()
{   
//    var= tim_encoder_get_count(TIM_4)*200/170;								// 采集对应编码器数据
//    val=-tim_encoder_get_count(TIM_3)*200/170;// 采集对应编码器数据
     static int  ii_l=0,ii_r=0;
    static int mc_mc_l=0,mc_mc_r=0;
     

    
       
    R_error[0]=(right_hope-var);
    L_error[0]=(left_hope-val);
    
    left_duty=left_duty+97.0/30.0*(L_error[0]-L_error[1])+17.0/50.0*L_error[0];
    L_error[1]=L_error[0];
    
    right_duty=right_duty+97.0/30.0*(R_error[0]-R_error[1])+17.0/50.0*R_error[0];
    R_error[1]=R_error[0];
    
//    left_duty=left_hope;
//    right_duty=right_hope;
//    if(first_go_go==1)
//    {
//      if(left_duty>100) left_duty=100;
//      if(right_duty>100) right_duty=100;
//    }
    
     if(expd==1)
    {
     // if(beepds==0) beepds=3;
      mc_mc_l=mc_mc_l+val;
      if(mc_mc_l>=300) {ii_l=~ii_l;mc_mc_l=0;}
      if(mc_mc_l<=-300) {ii_l=~ii_l;mc_mc_l=0;}
      if(ii_l==0) left_duty=-300;
      else       left_duty=300;
      
      right_duty=left_duty;
//       mc_mc_r=mc_mc_r+var;
//      if(mc_mc_r>=70) {ii_r=~ii_r;mc_mc_r=0;}
//      if(mc_mc_r<=-70) {ii_r=~ii_r;mc_mc_r=0;}
//      if(ii_r==0) right_duty=-250;
//      else       right_duty=+250;
    }
    
    
   if(first_go_go==1)
    {
      left_duty=300;
      right_duty=300;
    }
    if(left_duty>=0)
    {
      if(left_duty>999) left_duty=999;
      pwm_duty_updata(TIM_5, zuon, 0);
      pwm_duty_updata(TIM_5, zuop, left_duty); 
    }
    else
    {
      if(left_duty<-999) left_duty=-999;
      pwm_duty_updata (TIM_5, zuon, -left_duty);
      pwm_duty_updata (TIM_5, zuop, 0);
    }
    if(right_duty>=0)
    {
      if(right_duty>999) right_duty=999;
      pwm_duty_updata (TIM_5, youn, 0);
      pwm_duty_updata (TIM_5, youp, right_duty);
    }
    else
    {
      if(right_duty<-999) right_duty=-999;
      pwm_duty_updata (TIM_5, youn,-right_duty);
      pwm_duty_updata (TIM_5, youp, 0);
    } 
}



int16 Filter(int16 a)
{
    static float Filter_win[4]={0,0,0,0};//
	uint8  mid_weight[4]={0,1,2,7};
	float b;
	uint8 i;
	uint8 mid_coe=0;
    float sumErr=0;
	int16 c;
	b = (float)a;
	
	
		  //加权滑动平均滤波
  Filter_win[3]=b; //Servo_Control0ut
  for(i=0;i<4;i++)
  {
    sumErr+=Filter_win[i]*mid_weight[i];
    mid_coe+=mid_weight[i];
  }
  b = sumErr/mid_coe;
  for(i=0;i<3;i++) //先进先出原则
  {
    Filter_win[i]=Filter_win[i+1];
  }
  c = (int16)b;
	return c;
}

void AngleCalculate(void)
{   
       double   fDeltaValue=0;//中间变量为补偿量 
       icm_acc_z = 0.8*icm_acc_z+0.2*last_icm;
       icm_acc_z = Filter(icm_acc_z);
        //    加速度    =       （Z轴加速度-加速度零偏值）  * 加速度角度比例
	g_fGravityAngle =       (icm_acc_z-240) * 1;// accle_z/185; //加速度    
        //            陀螺仪值    =  （陀螺仪Z轴值-陀螺仪零偏值）  * 陀螺仪角度比例
        g_fGyroscopeAngleSpeed = -(icm_gyro_y+12) * GYROSCOPE_ANGLE_RATIO;          //gyro 角速度	 
	
      //中间补偿量  =（加速度           -融合角度      ）/ 加速度常量
        fDeltaValue =(g_fGravityAngle - fGyroscopeAngleIntegral) / 2;      //补偿量

      //融合角度                +=（陀螺仪值               + 中间补偿量 ）/ 陀螺仪调整频率
	fGyroscopeAngleIntegral +=(g_fGyroscopeAngleSpeed + fDeltaValue) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
        
        Angle = fGyroscopeAngleIntegral/19;
       // err = Angle +52+75-expd;
       // pwm_station =(-g_fGyroscopeAngleSpeed+(err)*kp*5)*kd/100; // 2ms 一次角速度控制
        last_icm = icm_acc_z;     
}


void Data_Send(UARTN_enum uartn,unsigned short int *pst)
{
        unsigned char _cnt=0;	unsigned char sum = 0;
	unsigned char data_to_send[23];         //发送缓存
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //高8位
	data_to_send[_cnt++]=(unsigned char)pst[0];  //低8位
	data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[1];
	data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[2];
	data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[3];
	data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[4];
	data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[5];
	data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[6];
	data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[7];
	data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
	data_to_send[_cnt++]=(unsigned char)pst[8];
	data_to_send[3] = _cnt-4;
	sum = 0;
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
        
	data_to_send[_cnt++] = sum;
        for(unsigned char i=0;i<_cnt;i++)
        uart_putchar(uartn,data_to_send[i]);
}

void laya_send()              //蓝牙发送函数
{
//    send_data[0]=low_mov;    
//    send_data[1]=speed;
//    send_data[2]=err_ft;
//    send_data[3]=icm_gyro_x+44;
//   send_data[4]=-icm_gyro_y;
//  send_data[5]=icm_gyro_z;
   // send_data[6]=fGyroscopeAngleIntegral;
   // send_data[2]=icm_acc_z-300;
    send_data[0]=var;    
    send_data[1]=hope;
//    send_data[2]=Angle;
//    send_data[3]=icm_gyro_x+44;
    Data_Send(UART_4,send_data);   
    uart_putchar(UART_4,0x33);
}

void STOP()
{ 
   stop = 1; 
   if(shuzi_num==1||shuzi_num==2)
   {
   pwm_duty_updata (TIM_5, zuon, 80);
   pwm_duty_updata (TIM_5, zuop, 0);
   pwm_duty_updata (TIM_5, youn, 80);
   pwm_duty_updata (TIM_5, youp, 0);
   systick_delay_ms(50);
   pwm_init(TIM_5, TIM_5_CH1_A00, 16000, 0);   //左                           // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   pwm_init(TIM_5, TIM_5_CH2_A01, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   pwm_init(TIM_5, TIM_5_CH3_A02, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   pwm_init(TIM_5, TIM_5_CH4_A03, 16000, 0);  
   }
   else
   {
     pwm_duty_updata (TIM_5, zuon, 300);
     pwm_duty_updata (TIM_5, zuop, 0);
     pwm_duty_updata (TIM_5, youn, 300);
     pwm_duty_updata (TIM_5, youp, 0);
     systick_delay_ms(hope*1.5);
     pwm_init(TIM_5, TIM_5_CH1_A00, 16000, 0);   //左                           // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
     pwm_init(TIM_5, TIM_5_CH2_A01, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
     pwm_init(TIM_5, TIM_5_CH3_A02, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
     pwm_init(TIM_5, TIM_5_CH4_A03, 16000, 0);  
   }
   
   hope=0;
}





int regression(int *LINE,int startline,int endline)        
{    
    int B;
    int i;
    int sumX=0,sumY=0,avrX=0,avrY=0 ;    
    int num=0,B_up1=0,B_up2=0,B_up,B_down;
    for(i=startline;i<=endline;i++)
    {
        num++;  //总的行数
        sumX+=i;    
        sumY+= (int)LINE[i];
    }
    avrX=sumX/num;//中间行
    avrY=sumY/num;//中间列
    B_up=0;
    B_down=0;
    for(i=startline;i<=endline;i++)
    {        
        B_up1=(int)((int)LINE[i]-avrY);
        B_up2=i-avrX;//有正负值
        B_up+=(int)((B_up1*B_up2));
//        B_up=B_up/100*100;
        B_down+=(int)(((i-avrX)*(i-avrX)));
    }
    if(B_down==0) 
        B=0;
    else 
        B=B_up*16/B_down;//16为放大系数
    return -B;
}



  

void v_speed_set2pwm(int16 leftSpeed,int16 rightSpeed)//ABS停车
{
    
    int16 v_e_L = leftSpeed + val;//S.CL S.CR是左右轮编码器的值，不需要经过处理，直接用真实值
    int16 v_e_R = rightSpeed + (var);
    static int32 v_e_Counter_L = 0;
    static int32 v_e_Counter_R = 0;
    
//    if( meet.turn_C&&meet.turn_C!=2)
//          v_e_L=0;
    int32 v_e_Counter_MayBeNotChange = (v_e_Counter_L + v_e_L + v_e_Counter_R + v_e_R)/2;
            
    int16 v_e = (v_e_L + v_e_R)/2;
    
    float I_endless = (float)v_e_Counter_MayBeNotChange * speed_I;
    if((I_endless > 480)&&(v_e > 0))
	{
		v_e_Counter_MayBeNotChange = (v_e_Counter_L  + v_e_Counter_R)/2;
	}else 
          if((I_endless < -480)&&(v_e <0))
	{
		v_e_Counter_MayBeNotChange = (v_e_Counter_L + v_e_Counter_R)/2;
	}else
	{
                //累加积分
		v_e_Counter_L = v_e_Counter_L + v_e_L;
		v_e_Counter_R = v_e_Counter_R + v_e_R;
	}
    
    int32 v_e_counter = (v_e_Counter_L + v_e_Counter_R)/2;
            
    float OutPWM = v_e * speed_P + v_e_counter * speed_I;
    Left_OutPWM  = OutPWM;
    Right_OutPWM = OutPWM;
    
    if(Left_OutPWM > 950.f)  {Left_OutPWM = 950.f;}
    if(Left_OutPWM < -950.f) {Left_OutPWM = -950.f;}
    
    if(Right_OutPWM > 950.f){Right_OutPWM = 950.f;}
    if(Right_OutPWM < -950.f){Right_OutPWM = -950.f;}
    
}

void ABS()
{
   v_speed_set2pwm(0,0);
          
   if(Right_OutPWM>=0)
   {
      pwm_duty_updata(TIM_5, zuon, 0);
      pwm_duty_updata(TIM_5, zuop, Right_OutPWM);//右轮正    
   }
   else
   {
      pwm_duty_updata(TIM_5, zuon, -Right_OutPWM);
      pwm_duty_updata(TIM_5, zuop, 0);//右轮正    
     
   }
   if(Left_OutPWM>=0)
   {
      pwm_duty_updata(TIM_5, youn, 0); 
      pwm_duty_updata(TIM_5, youp, Left_OutPWM);
   }
   else
   {
     pwm_duty_updata(TIM_5, youn, -Left_OutPWM); 
     pwm_duty_updata(TIM_5, youp, 0); //
   }
}


//float Q_bias, Angle_err;
//float PCt_0, PCt_1, E;
//float K_0, K_1, t_0, t_1;
//float Pdot[4] ={0,0,0,0};
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };
//
//void KalmanFilter(int16 Accel,int16 Gyro)		
//{
//    static float  Gyro_y = 0;
//	fGyroscopeAngleIntegral+=(Gyro - Q_bias)/GYROSCOPE_ANGLE_SIGMA_FREQUENCY; //先验估计
// 
//	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
// 
//	Pdot[1] = -PP[1][1];
//	Pdot[2] = -PP[1][1];
//	Pdot[3]= Q_gyro;
//	
//	PP[0][0] += Pdot[0]/GYROSCOPE_ANGLE_SIGMA_FREQUENCY;   // Pk-先验估计误差协方差微分的积分
//	PP[0][1] += Pdot[1]/GYROSCOPE_ANGLE_SIGMA_FREQUENCY;   // =先验估计误差协方差
//	PP[1][0] += Pdot[2]/GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
//	PP[1][1] += Pdot[3]/GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
//		
//	Angle_err = Accel - fGyroscopeAngleIntegral;	//-先验估计
//	
//	PCt_0 = GYROSCOPE_ANGLE_RATIO * PP[0][0];
//	PCt_1 = GYROSCOPE_ANGLE_RATIO * PP[1][0];
//	
//	E = R_angle + GYROSCOPE_ANGLE_RATIO * PCt_0;
//	
//	K_0 = PCt_0 / E;
//	K_1 = PCt_1 / E;
//	
//	t_0 = PCt_0;
//	t_1 = GYROSCOPE_ANGLE_RATIO * PP[0][1];
// 
//	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
//	PP[0][1] -= K_0 * t_1;
//	PP[1][0] -= K_1 * t_0;
//	PP[1][1] -= K_1 * t_1;
//		
//	fGyroscopeAngleIntegral	+= K_0 * Angle_err;	 //后验估计
//	Q_bias	+= K_1 * Angle_err;	 //后验估计
//	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
//         Angle = fGyroscopeAngleIntegral/18;
//	
//}

#define Peried 1/100.0f		//卡尔曼积分周期
#define Q GYROSCOPE_ANGLE_RATIO				//过程噪声2.0		越小积分越慢，跟踪加速度计越慢越平滑
#define R GYROSCOPE_ANGLE_SIGMA_FREQUENCY			//测量噪声5000.0	越小跟踪加速度计越快
float KalmanGain = 1.0f;	//卡尔曼增益

void KalmanFilter(float ACC_Angle)
{
	//卡尔曼滤波局部参量
    static float Priori_Estimation = 0;//先验估计
    static float Posterior_Estimation = 0;//后验估计
    static float Priori_Convariance = 0;//先验方差
    static float Posterior_Convariance = 0;//后验方差
    static float las=0;
    ACC_Angle = (ACC_Angle*8+las*2)/10;
     ACC_Angle = Filter(ACC_Angle);
    las = ACC_Angle;
    g_fGyroscopeAngleSpeed = -(icm_gyro_y-12) * GYROSCOPE_ANGLE_RATIO;
	//卡尔曼滤波
    //1.时间更新(预测) : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k) 
    Priori_Estimation = Posterior_Estimation - (-icm_gyro_y-12)*Peried;		//先验估计，积分获得角度
	if (Priori_Estimation != Priori_Estimation)
	{
		Priori_Estimation = 0;
	}
	
    //2.更新先验协方差  : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k) 
    Priori_Convariance = (float)sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );
	if (Priori_Convariance != Priori_Convariance)
	{
		Priori_Convariance = 0;
	}
	
    //  卡尔曼后验估计：测量更新  
    // 1.计算卡尔曼增益  : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) /
    KalmanGain = (float)sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
	if (KalmanGain != KalmanGain)
	{
		KalmanGain = 1;
	}
	
    //2.测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1)) 
    Posterior_Estimation  = Priori_Estimation + KalmanGain * (ACC_Angle - Priori_Estimation );
	if (Posterior_Estimation != Posterior_Estimation)
	{
		Posterior_Estimation = 0;
	}
	
    // 3.更新后验协方差  : P(k|k) =（I-K(k)*H(k)）*P(k|k-1) 
    Posterior_Convariance = (float)sqrt(( 1 - KalmanGain ) * Priori_Convariance * Priori_Convariance );
	if (Posterior_Convariance != Posterior_Convariance)
	{
		Posterior_Convariance = 0;
	}
	
    //得到最终角度 
        
     
    fGyroscopeAngleIntegral=Posterior_Estimation;
    Angle = -fGyroscopeAngleIntegral/19;
	
//	if (Attitude_Angle.Y != Attitude_Angle.Y)
//	{
//		Attitude_Angle.Y = 1;
//	}
}




    