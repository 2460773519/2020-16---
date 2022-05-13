#include "MENU.h"
//
//菜单相关变量
int t=0;
uint8 se=0;
int stop=1;
int kaishi=0;
int beepds=0;
int ok_data=0;
int yaun_s_point=34;
int zhen_go=0;



///*--------------------------------------------------
//-------------------初始化函数-----------------------
//----------------------------------------------------*/
void csh()
{ 
//   gpio_init(B13, GPO, GPIO_HIGH, GPO_PUSH_PULL);        // 单片机LED灯
//   gpio_init(H2 , GPO, GPIO_HIGH, GPO_PUSH_PULL);        // 单片机LED灯
   gpio_init(beepd, GPO, GPIO_LOW, GPO_PUSH_PULL);        // 
   gpio_init(B5, GPI, 1, GPI_PULL_UP);                        //B4和B5为一对
   gpio_init(B7, GPI, 1, GPI_PULL_UP);                        //B6和B7为一对
   gpio_init(bo_6, GPI, 1, GPI_PULL_UP); 
    gpio_init(bo_7, GPI, 1, GPI_PULL_UP); 
   
   gpio_init(up, GPI, GPIO_HIGH, GPI_FLOATING_IN);      //按键：上
   gpio_init(down, GPI, GPIO_HIGH, GPI_FLOATING_IN);	//按键：下
   gpio_init(ok, GPI, GPIO_HIGH, GPI_FLOATING_IN);	//按键：确定
   gpio_init(returnd, GPI, GPIO_HIGH, GPI_FLOATING_IN);	//按键：返回
   gpio_init(jia, GPI, GPIO_HIGH, GPI_FLOATING_IN);	//按键：加
   gpio_init(jian, GPI, GPIO_HIGH, GPI_FLOATING_IN);	//按键：减
   
   gpio_init(bo_1, GPO, GPIO_LOW, GPO_PUSH_PULL);      
   gpio_init(bo_2, GPO, GPIO_LOW, GPO_PUSH_PULL);
   gpio_init(bo_3, GPO, GPIO_LOW, GPO_PUSH_PULL);
   
   gpio_init(bo_4, GPI, GPIO_LOW, GPI_FLOATING_IN);
   gpio_init(bo_5, GPI, GPIO_LOW, GPI_FLOATING_IN);  
   
//   exti_interrupt_init(bo_4, EXTI_Trigger_Falling, 0x01, 0x01);
//   exti_interrupt_init(bo_5, EXTI_Trigger_Falling, 0x02, 0x02);
//   exti_interrupt_enable(bo_4);
//   exti_interrupt_enable(bo_5);

   
//   gpio_init(bo_6, GPO, GPIO_HIGH, GPI_FLOATING_IN);
//   gpio_init(bo_7, GPO, GPIO_HIGH, GPI_FLOATING_IN);
//   gpio_init(bo_8, GPO, GPIO_HIGH, GPI_FLOATING_IN);
   
   //编码器初始化
   tim_counter_init(TIM_3, TIM_3_ENC1_B04);                    //左编码器
    tim_counter_init(TIM_4, TIM_4_ENC1_B06);  			// 初始化 TIM_8 C00/C01 正交编码器采集
   
  //电机初始化
   pwm_init(TIM_5, TIM_5_CH1_A00, 16000, 0);   //左                           // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   pwm_init(TIM_5, TIM_5_CH2_A01, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   pwm_init(TIM_5, TIM_5_CH3_A02, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   pwm_init(TIM_5, TIM_5_CH4_A03, 16000, 0);                              // 初始化TIM2 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%
   
   // flash_page_program(FLASH_SECTION_127, FLASH_PAGE_3,(const uint32 *)flash_image_init,4);
   flash_page_read(FLASH_SECTION_127,FLASH_PAGE_3,flash_image,4); //读出摄像头需要的4个数据

   //外设初始化
    lcd_init(); 
    icm20602_init();
    mt9v03x_init();

    
   //中断初始化
    uart_init(UART_4,115200,UART4_TX_C10,UART4_RX_C11);  //无线
    uart_init(UART_3,115200,UART3_TX_B10,UART3_RX_B11);  //opmv
    tim_interrupt_init_ms(TIM_6,5, 0,0);
     //tim_interrupt_disable(TIM_6);
  //  tim_interrupt_init_ms(TIM_7,5,1,1);
    //tim_interrupt_enable(TIM_7);
   // tim_interrupt_disable(TIM_7);
    uart_rx_irq(UART_3,1);
    uart_rx_irq(UART_4,1);
    
//    gpio_set(beepd,1);
//     systick_delay_ms(1000);
//     gpio_set(beepd,0);
     
}
//
//
void caidan()
{
  beep();
  
  
  
    if(wuxian_data=='G')
    {
      wuxian_data=0;
      zhen_go=1;
    }

  if(gpio_get(bo_6)==0)
  {
    zhen_go=1;
       systick_delay_ms(500);
  }
  
  
  if(key_judge(up)) 
  {
    
    t++;
    lcd_clear(WHITE);
    if(t>7) t=0;
  }
  
  if(key_judge(down)) 
  {
    t--;
    lcd_clear(WHITE);
    if(t<0) t=7;
  }
   lcd_showstr(0,(uint8)t,">");
/*---------------直立-----------------*/
  if(se==1)
  {         
      lcd_showstr(10,0,"go_flash");
     lcd_showint16(75,0,(int)(go_flash));

     lcd_showint16(75,1,(int)(hopes));
     lcd_showstr(10,1,"hopes");
//    
//    lcd_showstr(120,1,"err");
//    lcd_showint16(105,2,(err));
                 
     lcd_showstr(10,2,"kp");
     lcd_showint16(75,2,(int)(kp));
    
     lcd_showstr(10,3,"kd");
    lcd_showint16(75,3,kd);
    
    lcd_showstr(10,4,"VP");
    lcd_showint16(75,4,VP);
    
    lcd_showstr(10,5,"VI");
    lcd_showint16(75,5,VI);
    
    lcd_showstr(10,6,"jifen_angle");
    lcd_showint16(75,6,jifen_angle);
    
    lcd_showstr(10,7,"mc_dis");
    lcd_showint16(75,7,mc_dis);
    
    

    
                  

    if(key_judge(jia))
    {

      if(t==0) go_flash=go_flash+1; 
      if(t==1) hopes=hopes+10;
      if(t==2) kp=kp+1;
      if(t==3) kd=kd+1;
      if(t==4) VP=VP+1;
      if(t==5) VI=VI+1;
      if(t==6) jifen_angle=jifen_angle+20;
      if(t==7) mc_dis=mc_dis+20;

      if(t==8) se=8;
    }
    
    if(key_judge(jian))
    {
      if(t==0) go_flash=go_flash-1;
      if(t==1) hopes=hopes-10;
      if(t==2) kp=kp-1;
      if(t==3) kd=kd-1;
      if(t==4) VP=VP-1;
      if(t==5) VI=VI-1;
      if(t==6) jifen_angle=jifen_angle-20;
      if(t==7) mc_dis=mc_dis-20;

      if(t==8) se=8;
    }
  }
  
  /***************************转向**************/
  
  if(se==3)
  {    
    
    lcd_showstr(10,0,"err_kz");
    lcd_showint8(75,0,err_kz);
    
    lcd_showstr(10,1,"shuzi_num");
    lcd_showint8(75,1,shuzi_num);
    
    lcd_showstr(10,2,"kp_g");
    lcd_showint8(75,2,kp_g);
    
    lcd_showstr(10,3,"kd_g");
    lcd_showint8(75,3,kd_g);
    
    lcd_showstr(120,1,"duty");
    lcd_showint16(105,2,(duty));
    
    lcd_showstr(10,4,"ca_su");
    lcd_showint16(75,4,(ca_su));
    
    
    if(key_judge(jia))
    {
      if(t==0) err_kz=err_kz+1;
      if(t==1) shuzi_num=shuzi_num+1;
      if(t==2) kp_g=kp_g+1;
      if(t==3) kd_g=kd_g+1;
      if(t==4) ca_su=ca_su+100;
      if(t==5) kp_tai=kp_tai+1;
      if(t==6) fuzz_kp=fuzz_kp+0.5;

      if(t==8) se=8;
    }
    
    if(key_judge(jian))
    {
      if(t==0) err_kz=err_kz-1;
      if(t==1) shuzi_num=shuzi_num-1;
      if(t==2) kp_g=kp_g-1;
      if(t==3) kd_g=kd_g-1;
      if(t==4) ca_su=ca_su-100;
      if(t==5) kp_tai=kp_tai-1;
      if(t==6) fuzz_kp=fuzz_kp-0.5;

      if(t==8) se=8;
    }
  }
  
  /***************************传球**************/
  if(se==4)
  {    
      
    lcd_showstr(10,0,"mc_dis");     lcd_showint16(75,0,mc_dis);
    lcd_showstr(10,1,"stop_time");  lcd_showint16(75,1,(stop_time)); 
    lcd_showstr(10,2,"s_cha_num");  lcd_showint16(75,2,s_cha_num);
    lcd_showstr(10,3,"s_cha_fx");   lcd_showint16(75,3,s_cha_fx);
    lcd_showstr(10,4,"gogogo_dis"); lcd_showint16(75,4,gogogo_dis);
    
    lcd_showstr(10,5,"quan_while"); lcd_showint16(75,5,quan_while);
    lcd_showstr(10,6,"yaun_s_point"); lcd_showint16(75,6,yaun_s_point);
    lcd_showstr(10,7,"Total score"); lcd_showint16(75,7,(quan_while*yaun_s_point));
    

        

    if(key_judge(jia))
    {
      if(t==0) mc_dis=mc_dis+100;
      if(t==1) stop_time=stop_time+1;
      if(t==2) s_cha_num=s_cha_num+1;
      if(t==3) s_cha_fx=s_cha_fx+1;
      if(t==4) gogogo_dis=gogogo_dis+10;
      if(t==5) quan_while=quan_while+1;
      if(t==6) yaun_s_point=yaun_s_point+1;
      
    }
    
    if(key_judge(jian))
    {
      if(t==0) mc_dis=mc_dis-100;
      if(t==1) stop_time=stop_time-1;
      if(t==2) s_cha_num=s_cha_num-1;
      if(t==3) s_cha_fx=s_cha_fx-1;
      if(t==4) gogogo_dis=gogogo_dis-10;
      if(t==5) quan_while=quan_while-1;
      if(t==6) yaun_s_point=yaun_s_point-1;
    }
    
    
  }
  
   /***************************高速*************/
  
  if(se==5)
  {    
    lcd_showstr(10,0,"kp_low");
    lcd_showint16(75,0,kp_low);
    
    lcd_showstr(10,1,"ex_sped");
    lcd_showint16(75,1,(int)(expd_speed));
    
    lcd_showstr(10,2,"kp_speed");
    lcd_showint16(75,2,kp_speed);
    
    lcd_showstr(10,3,"ex_spedlo");
    lcd_showint16(75,3,expd_speed_low);
    
    lcd_showstr(10,4,"very_l_con");
    lcd_showint16(75,4,very_long_con);
    
    lcd_showstr(10,5,"very_l_dis");
    lcd_showint16(75,5,very_long_dis);
    
    
    


    if(key_judge(jia))
    {
        if(t==0) kp_low=kp_low+1;
        if(t==1) expd_speed=expd_speed+1;
        if(t==2) kp_speed=kp_speed+1;
        if(t==3) expd_speed_low=expd_speed_low+1;
        if(t==4) very_long_con=very_long_con+1;
        if(t==5) very_long_dis=very_long_dis+1000;
      
    }
    
    if(key_judge(jian))
    {
       if(t==0) kp_low=kp_low-1;
       if(t==1) expd_speed=expd_speed-1;
       if(t==2) kp_speed=kp_speed-1;
       if(t==3) expd_speed_low=expd_speed_low-1;
        if(t==4) very_long_con=very_long_con-1;
        if(t==5) very_long_dis=very_long_dis-1000;
    }
    
    
  }
  
   /***************************yuan_su**************/
  if(se==6)
  {    
      
    lcd_showstr(10,0,"low_yaun");
    lcd_showint16(75,0,(low_yaun));
    
    lcd_showstr(10,1,"into_yuan");
    lcd_showint16(75,1,into_yuan);
    
    lcd_showstr(10,2,"out_yuan");
    lcd_showint16(75,2,out_yuan);
    
    lcd_showstr(10,3,"in_h_d");
    lcd_showint16(75,3,in_huan_dian);

    
    lcd_showstr(10,4,"drath_banma");
    lcd_showint16(75,4,drath_banma);
    
    lcd_showstr(10,5,"s_p_Angel");
    lcd_showint8(75,5,s_p_Angel);
    
    lcd_showstr(10,6,"s_p_mc");
    lcd_showint16(75,6,s_p_mc);
    
    lcd_showstr(10,7,"x_p_Angle");
    lcd_showint16(75,7,s_p_speed);
    

    
    
    
//    lcd_showstr(10,0,"mc_dis");
//    lcd_showint16(75,0,mc_dis);
//    
//    lcd_showstr(10,1,"stop_time");
//    lcd_showint16(75,1,(int)(stop_time));
        

    if(key_judge(jia))
    {
      if(t==0) low_yaun=low_yaun+1;
      if(t==1) into_yuan=into_yuan+1;
      if(t==2) out_yuan=out_yuan+1;
      if(t==3) in_huan_dian=in_huan_dian+1;
      if(t==4) drath_banma=drath_banma+1;
      if(t==5) s_p_Angel=s_p_Angel+1;
      if(t==6) s_p_mc=s_p_mc+100;
      if(t==7) s_p_speed=s_p_speed+1;
      
    }
    
    if(key_judge(jian))
    {
      if(t==0) low_yaun=low_yaun-1;
      if(t==1) into_yuan=into_yuan-1;
      if(t==2) out_yuan=out_yuan-1;
      if(t==3) in_huan_dian=in_huan_dian-1;
      if(t==4) drath_banma=drath_banma-1;
      if(t==5) s_p_Angel=s_p_Angel-1;
      if(t==6) s_p_mc=s_p_mc-100;
      if(t==7) s_p_speed=s_p_speed-1;
    }
    
    
  }
  
     /***************************flash**************/
  if(se==7)
  {    
      
    lcd_showstr(10,0,"EXP_TIME");
    lcd_showint16(75,0,flash_image[0]);
    
    lcd_showstr(10,1,"LR_OFFSET");
    lcd_showint16(75,1,flash_image[1]);
    
    lcd_showstr(10,2,"UD_OFFSET");
    lcd_showint16(75,2,flash_image[2]);
    
    lcd_showstr(10,3,"GAIN");
    lcd_showint16(75,3,flash_image[3]);

    if(key_judge(jia))
    {
      if(t==0) flash_image[0]=flash_image[0]+20;
      if(t==1) flash_image[1]=flash_image[1]+1;
      if(t==2) flash_image[2]=flash_image[2]+1;
      if(t==3) flash_image[3]=flash_image[3]+1;
      
      flash_page_program(FLASH_SECTION_127, FLASH_PAGE_3,flash_image,4);
      
    }
    
    if(key_judge(jian))
    {
      if(t==0) flash_image[0]=flash_image[0]-20;
      if(t==1) flash_image[1]=flash_image[1]-1;
      if(t==2) flash_image[2]=flash_image[2]-1;
      if(t==3) flash_image[3]=flash_image[3]-1;
      
     flash_page_program(FLASH_SECTION_127, FLASH_PAGE_3,flash_image,4);
      
      
    }
    
    
  }
  
/*---------------图像-----------------*/
  if(se==2) 
  {
    if(t<4)
    {
      lcd_clear(WHITE);
      t=4;
    } 
    
    lcd_showfloat(60,4,err_ft,2,3);
    lcd_showstr(10,4,"err_ft");
   
    
    lcd_showstr(100,4,"drath");
    lcd_showint8(125,4,drath );
    
    lcd_showstr(10,5,"yidong");
    lcd_showint16(65,5,yidong);
    
    lcd_showint8(125,5,White);
    
    lcd_showstr(10,7,"add");
    lcd_showint16(40,7,add);
    
    lcd_showint16(100,6,sobel_yun);
    lcd_showstr(10,6,"sobel_yun");
    
    lcd_showint8(115,7,(Angle));
    
   if(err_ft_bu==0) lcd_displayimage032(mt9v03x_image[0],160,60);
   // if(err_ft_bu==0) lcd_displayimage032_hui(image_avg[0],160,60);
    
   else lcd_displayimage032_hui(mt9v03x_image[0],160,60);
       
    

    if(key_judge(jia))
    {
      if(t==5) yidong=yidong+1;
      if(t==7) add=add+1;
      if(t==4) drath=drath+1;
      if(t==6) sobel_yun=sobel_yun+1;
//      if(t==2) kp_g=kp_g+1;
//      if(t==3) kd_g=kd_g+1;
    
      if(t==8) se=8;
    }
    
    if(key_judge(jian))
    {
      if(t==5) yidong=yidong-1;
      if(t==7) add=add-1;
      if(t==4) drath=drath-1;
      if(t==6) sobel_yun=sobel_yun-1;
//      if(t==2) kp_g=kp_g-1;
//      if(t==3) kd_g=kd_g-1;
//      
//    
//      if(t==8) se=8;
    }
    if(key_judge(ok)) err_ft_bu=~err_ft_bu;
    
 
  }

 

  
/*---------------目录-----------------*/  
  if(se==0||zhen_go==1)
  {
    if(zhen_go==1)
    {
      exti_even_disable(bo_5);
      exti_even_disable(bo_4);
    }
    
    lcd_showstr(25,0,"go_go_go");
    lcd_showstr(25,1,"zhili");
    lcd_showstr(25,2,"image");
    lcd_showstr(25,3,"turn");
    lcd_showstr(25,4,"chuanqiu");
    lcd_showstr(25,5,"High speed");
    lcd_showstr(25,6,"yuan_su");
    lcd_showstr(25,7,"flash");
    
    lcd_showfloat(90,0,err_ft,2,3);
      lcd_showint16(90,2,shuzi_num);
    
     lcd_showint16(90,3,val);
      lcd_showint16(90,4,var);
    

    if(key_judge(ok)||zhen_go==1)
    {
      if(t==0||zhen_go==1)  
      {
        opmv_data=0;
        first_4_sizi=0;
        t=0;
        zhen_go=0;
        lu_sz[0]=5;
        lu_sz[1]=5;
        shizi=0;
        first_sizi=0;
        angle_ji=0;
        no_ft=0;
        opmv_data=0;    
        stop = 0;
        hope=hopes-30;
        first_go_go=1;
       
        car_zk=1;
        angle_ji=0;
        err_ft=0;
        duty=0;
        systick_delay_ms(250);
//        left_duty=0;
//        right_duty=0;
      }
      if(t==0) {se=0;t=1;}
      if(t==1) se=1;
      if(t==2) se=2;
      if(t==3) se=3;
      if(t==4) se=4;
      if(t==5) se=5;
      if(t==6) se=6;
      if(t==7) se=7;
      if(t==8) se=8;
      t=t-1;
      lcd_clear(WHITE);
    }
  }

/*---------------返回最初-----------------*/    
  if(key_judge(returnd))
  {
    lcd_clear(WHITE);
    se=0;
    t=0;
  }
}

//
//
////--------------------------------------------------------------------------
////-------------------------------判断按键是否按下---------------------------
////--------------------------------------------------------------------------
char key_judge(PIN_enum pin)
{
  if(gpio_get(pin)==0)
  {
    systick_delay_ms(30);
    if(gpio_get(pin)==0)
    {
      gpio_set(beepd,1);
      systick_delay_ms(10);
      gpio_set(beepd,0);
      return 1;
    }
    else return  0;
  }
  else return 0;
}
//
//
///*----------------------------------------------------------------------
//-----------------------------------蜂鸣器-------------------------------
//------------------------------------------------------------------------*/
void beep()
{
  if(beepds>temp1)   temp1++;
  if(temp1==2)  gpio_set(beepd,1);
  if(temp1==beepds)
  {
    temp1=0;
    gpio_set(beepd,0);
    beepds=0;
  }
}


void caidan_sta()
{
    lcd_showstr(25,0,"g_Adjust");
    lcd_showint16(90,0,GYROSCOPE_ANGLE_SIGMA_FREQUENCY);
    
    lcd_showstr(25,1,"g_pili*");
    lcd_showint16(90,1,(int)(GYROSCOPE_ANGLE_RATIO*10));
    if(key_judge(jian)) GYROSCOPE_ANGLE_SIGMA_FREQUENCY=GYROSCOPE_ANGLE_SIGMA_FREQUENCY-1;
    if(key_judge(jia)) GYROSCOPE_ANGLE_SIGMA_FREQUENCY=GYROSCOPE_ANGLE_SIGMA_FREQUENCY+1;
    if(key_judge(up)) GYROSCOPE_ANGLE_RATIO=GYROSCOPE_ANGLE_RATIO-0.2;
    if(key_judge(down)) GYROSCOPE_ANGLE_RATIO=GYROSCOPE_ANGLE_RATIO+0.2;
}


