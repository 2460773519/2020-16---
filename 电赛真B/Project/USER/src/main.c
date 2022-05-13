#include "headfile.h"
/**            
 *                             _ooOoo_                                                       
 *                            o8888888o
 *                            88" . "88
 *                            (| -_- |)
 *                            O\  = /O
 *                          ____/`---'\____
 *                       .'  \\|     |//  `.
 *                      /  \\|||  :  |||//  \   
 *                     /  _||| v|| 金 |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|   ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. / 
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"". 
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |  
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /  
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=    ---='
 *          ^^^^^^^^^^^7月8日代码^^^^^^^^^^^^^^^^^^^^^^^^^  哈尔滨和桂林    
*/     //7月4

///car_zk=0 //停车状态
// car_zk=1 //跑车状态
// if(car_zk==3)      //货物被卸下 倒车
// car_zk=4 //等待十字 
// car_zk=5 //冲刺回家
// car_zk=6 //
// car_zk=7 //到家了
// 0左 1右
// err_ft 正右  B车新
int main(void)  
{
  //int y=0,n,m;
    csh();//初始化
   
    //uart_putchar(UART_3,'G');// opmv 识别数字
//    systick_delay_ms(200);
//     uart_putchar(UART_3,'S');// opmv 识别数字
      while(1)        
      {        

          if(mt9v03x_finish_flag) 
          {
            
           // if(gpio_get(bo_4)==0&&beepds==0)  beepds=5;
           // uart_putchar(UART_4,0x12);
           // laya_send();  
            
            image_yun=OTSU(mt9v03x_image[0],0,ROW-5,0,COL);
            find_side(mt9v03x_image);           
              if(stop==1)  
              {    
                caidan();  
             //   into_home();
              }
              else
              {    //dada
                 lcd_showint16(75,0,(angle_ji));
                 lcd_showint16(75,1,(sum_mc));
                  lcd_showint16(75,3,(car_zk));
                if(key_judge(ok))
                { 
                  STOP();  
                }
              }
              beep();
            }
       }
}



  


  