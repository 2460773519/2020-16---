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
 *                     /  _||| v|| �� |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|   ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. / 
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"". 
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |  
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /  
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=    ---='
 *          ^^^^^^^^^^^7��8�մ���^^^^^^^^^^^^^^^^^^^^^^^^^  �������͹���    
*/     //7��4

///car_zk=0 //ͣ��״̬
// car_zk=1 //�ܳ�״̬
// if(car_zk==3)      //���ﱻж�� ����
// car_zk=4 //�ȴ�ʮ�� 
// car_zk=5 //��̻ؼ�
// car_zk=6 //
// car_zk=7 //������
// 0�� 1��
// err_ft ���� A������
int main(void)  
{
  //int y=0,n,m;
    csh();//��ʼ��
   
    uart_putchar(UART_3,'G');// opmv ʶ������
//    systick_delay_ms(200);
//     uart_putchar(UART_3,'S');// opmv ʶ������
      while(1)        
      {        
          if(mt9v03x_finish_flag) 
          {
//            if(gpio_get(bo_4)==0&&beepds==0)  beepds=5;
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
              {    
                 lcd_showint16(75,0,(angle_ji));
                 lcd_showint16(75,1,(sum_mc));
                  lcd_showint16(75,3,(car_zk));
                  lcd_showint16(75,4,(s_p_mc));
                if(key_judge(ok))
                { 
                  STOP();  
                }
              }
              beep();
            }
       }
}



  


  