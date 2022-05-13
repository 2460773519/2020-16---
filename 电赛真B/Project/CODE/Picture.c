#include "Picture.h"

   // if(s_p==0&&stop==0&&yuan==0&&kaishi==0&&banma==0) out_stop();
//    if(stop==0&&sp_yaun_su==0) into_home();
//    if(stop==0&&sp_yaun_su==0&&yuan==0) sancha_bu();
//    if(stop==0&&banma==0&&sp_yaun_su==0) yuanhuan_bu(); 
////ͼ������ر���
float err_ft,dt_err_ft;
int LOW=27,low_yaun=-5,low_mov=0,err_ft_bu=0;//0-35
int add=0;
int HIGHT=20;
int White =210;
int Black =210;
int drath=8,drath_banma=32,sobel_yun=35;
int yidong=50;
int L_side[MT9V03X_H]={0},R_side[MT9V03X_H]={0};
int mid[MT9V03X_H]={0};
int  mc_js=0;
int fash_lanya=0;
int sancha_qian=25;
int a_bu[2],b_bu[2],bu_yes=1;
int err_ft_chan=0;
int r;
int yuan_1=0,yuan_2=0,yuan_3=0,yuan_4=0,yuan_5=0,yuan_6=0,yuan_7=0,yuan_8=0,yuan=0,yuan_L=0,yuan_R=0,yuan_x=0,yuan_low=0,yuan_time=0;
int S2_R=0,S2_L=0;//����
int i_angle=0;
int sancha_fx=0;
int banma=0;
int sz2[3]={0};
float qu_l,qu_r;
char s_cha_num = 2,s_cha_fx=1;  //�������  1 ���� 2 ���� 3 ���� 4 ����   
int sobel_sz[20];
int drath_piont=0;
int sancha_bu_time=8;
int into_yuan=82,out_yuan=75,in_huan_dian=40;
int gui_low=0;
//uint8 image_Gamma[MT9V03X_H][MT9V03X_W];
int g_GammaLUT[256];//ȫ�����飺����256��Ԫ�ص�gammaУ�����ұ�
int sobe_l[MT9V03X_H]={0},sobe_r[MT9V03X_H]={0};
int yuan_zhi_dian=20;
uint8 image_avg[MT9V03X_H][MT9V03X_W]={0};

void BuildTable(float fPrecompensation)
{
   int i;
   float f;
   for(i=0;i<256;i++)
   {
     f=(i+0.5F)/256;//��һ��
     f=(float)pow(f,1/2.2);
     g_GammaLUT[i]=(int)(f*256-0.5F);//����һ��
   }
 }

void get_image_Gamma()
{
  int i,j;
  for(i=0;i<MT9V03X_H;i++)
  {
    for(j=0;j<MT9V03X_W;j++)
    {
    //  image_Gamma[i][j]=g_GammaLUT[mt9v03x_image[i][j]];
      mt9v03x_image[i][j]=g_GammaLUT[mt9v03x_image[i][j]];
    }
  }
}

void otsuThreshold()
{
  int max[15]={0};
  int i,j;
  int sum1=0;
  int sum2=0;
  int  temp=0;
  for(i=54;i<59;i++)
  {
    for(j=L_side[i];j<R_side[i];j++)
    {
      temp++;
      
      sum1+=mt9v03x_image[i][j];
    }
  }
  sum2=sum1/temp;
  
  drath_piont=sum2;
//  Black=sum2+5+add;
//  White=sum2+5+add;
}





/************************************************/
/*              ��Ⱥ�                */
/************************************************/
int ddd,dd1,dd2;

int l_dra(uint8 *imgIn, int j)
{
  int dra;
  uint8 *next,*now;
  
  now=&*(imgIn);
  next=&*(imgIn-j);
  dd1=*now;
  dd2=*next;
  dra=100*(*now-*next)/(*now+*next);
  ddd=dra;
  
    return dra;
}
/************************************************/
int dde,de1,de2;

int r_dra(uint8 *imgIn, int j)
{

  int dra;
  uint8 *next,*now;
  
  now=&*(imgIn);
  next=&*(imgIn+j);
  de1=*now;
  de2=*next;

  dra=100*(*now-*next)/(*now+*next);
  dde=dra;
    return dra;
   
}

/*******************200-90***************************/ 
 //x        y  k= -10/100 y=-(sum-100)/10+5;
 //200  =  -5
 //100  =    5
                      

int get_setion_image(int y) //��ȡ�߽���Χ��ƽ��ֵ ��׼��ֵΪ8 +- 6
{
  int sum=0,m=0,i,j;
  float k;
  int temp[2];
  return k; 
}

void avg_image()
{
  int i,j,n,m;
  
    for(n=0;n<MT9V03X_H;n++)
    {
      for(m=0;m<MT9V03X_W;m++)
      {
        if(n==0||m==0||n==MT9V03X_H-1||m==MT9V03X_W-1)
        {
          image_avg[n][m]=mt9v03x_image[n][m];
        }
        else
        {
          image_avg[n][m]=
          (mt9v03x_image[n-1][m-1]+mt9v03x_image[n-1][m]+  mt9v03x_image[n-1][m+1]+
          mt9v03x_image[n][m-1]+mt9v03x_image[n][m]+mt9v03x_image[n][m+1]+
          mt9v03x_image[n+1][m-1]+mt9v03x_image[n+1][m]+mt9v03x_image[n+1][m+1])/9.0;
        }
      }
    }
    
     for(n=0;n<MT9V03X_H;n++)
    {
      for(m=0;m<MT9V03X_W;m++)
      {
        mt9v03x_image[n][m]=image_avg[n][m];
      }
    }
}


void find_side(uint8 p[ROW][COL])         //ROWͼ���������COLͼ�������
{
  int y=ROW-1;                            //�������һ�п�ʼ��
  int lside=COL/2,rside=COL/2; 
  int L_save,R_save;                      //�Ҷ�Ӧ�е���ʼ��
/****************************************��ͼ��������һ�еı߽�*********************************************/
    for(lside=COL/2-30;lside>5;lside--)    //��߽���������Ұ׺�����ĵ�
    {
      if(l_dra(&mt9v03x_image[y][lside],5)>drath) //�жϰ׺�����
      {
        L_side[y]=lside;                //�ҵ��õ�
        break;                          //�ҵ�֮������ѭ����ʼ������ĵ�
      }
      else
      {  
         L_side[y]=5;                   //���û�ҵ��Ͱ���߽�鵽�����
      }
    }
    
    for(rside=COL/2+30;rside<COL-5;rside++)    //�ұ߽���������Ұ׺�����ĵ�
    {
      if(r_dra(&mt9v03x_image[y][rside],5)>drath)
      {
        R_side[y]=rside;                //�ҵ��õ�
        break;                          //�ҵ�֮������ѭ����ʼ������ĵ�
      }
      else
      {  
         R_side[y]=154;                   //���û�ҵ��Ͱ��ұ߽�鵽���ұ�
      }
    }
    
    
    if(abs(R_side[y])-L_side[y]<=130) //�ҵ��������
    {
      if(R_side[y]<=120)
      {
           for(rside=COL/2+60;rside<COL-5;rside++)    //�ұ߽���������Ұ׺�����ĵ�
            {
              if(r_dra(&mt9v03x_image[y][rside],5)>drath)
              {
                R_side[y]=rside;                //�ҵ��õ�
                break;                          //�ҵ�֮������ѭ����ʼ������ĵ�
              }
              else
              {  
                 R_side[y]=154;                   //���û�ҵ��Ͱ��ұ߽�鵽���ұ�
              }
            }
      }
      if(L_side[y]<=39)
      {
      
       for(lside=COL/2-60;lside>5;lside--)    //��߽���������Ұ׺�����ĵ�
      {
        if(l_dra(&mt9v03x_image[y][lside],5)>drath) //�жϰ׺�����
        {
          L_side[y]=lside;                //�ҵ��õ�
          break;                          //�ҵ�֮������ѭ����ʼ������ĵ�
        }
        else
        {  
           L_side[y]=5;                   //���û�ҵ��Ͱ���߽�鵽�����
        }
      }
      }
    
 
      
    }
  

  
/****************************************��ͼ��ʣ���ж�Ӧ�ı߽�*********************************************/
  
  for(y=ROW-2;y>0;y--)                                    //��ʣ�µĵ�
  {
    ////////////////////////////////////��߽�/////////////////////////////////  
    if(L_side[y+1]<5) L_save=COL/2-50;                       //�����һ���ҵ��ı߽���ͼ�������ߣ�ʮ�ֵ�������Ǿʹ�ͼ���м��п�ʼ�ұ߽�
    else L_save=L_side[y+1]+8;                            //�������������ĵ� ��ʼ�о�Ϊ��һ���ҵ��ı߽�+8ȥ��
    
    for(lside=L_save;lside>0;lside--)
    {
     if(l_dra(&mt9v03x_image[y][lside],5)>drath)   //�׺�����
      {
        L_side[y]=lside;
        break;
      }
    }
    if(lside==0) L_side[y]=5;
    
  ////////////////////////////////////�ұ߽�/////////////////////////////////  
    
    if(R_side[y+1]>COL-6) R_save=COL/2+50;                       //�����һ���ҵ��ı߽���ͼ������ұߣ�ʮ�ֵ�������Ǿʹ�ͼ���м��п�ʼ�ұ߽�
    else R_save=R_side[y+1]-8;                            //�������������ĵ� ��ʼ�о�Ϊ��һ���ҵ��ı߽�-8ȥ��
    
    for(rside=R_save;rside<COL-1;rside++)
    {
     if(r_dra(&mt9v03x_image[y][rside],5)>drath)
      {
        R_side[y]=rside;
        break;
      }
    }
    if(rside==COL-1) R_side[y]=154;
  }
  /////////////////////////////////////////////////////////////
  if(stop==0)
  {
       if(car_zk==1||first_4_sizi==1)             on_shizi();    //�ܳ���س�ʱʶ��ʮ��
       if(car_zk==4)             juy_banma_4();
       if(car_zk==5&&shizi==0)             into_home();    //��̻ؼ�ʱ�жϰ�����
    //  sancha_bu();
    //  into_home();//������
  }
  
  for(y=ROW-1;y>0;y--)   //��ӳ�2������                         
  {
    mid[y]=( R_side[y]+L_side[y])/2;
  }
}


void on_shizi()
{
  int y,i;
  if(shizi==0)
  {
   // if(beepds==0) beepds=3;
    for(y=58;y>45;y--)
    {
       if(L_side[y]>=6||R_side[y]<=153) break;
    }
    if(y==45)
    { 
      //wuxian_data
      angle_ji=0;
      //beepds=5;
      sum_mc=0;

     if(first_sizi==0)
      {
        pche=1;
        first_sizi=1;
     // opmv_data=0x15;
        shizi=2;
        car_zk=10;
      //  opmv_data=0x11;
      //  beepds=25;
      //  car_zk=0;//ͣ��
      }
      else
      {
          if(car_zk==1) 
          {
            if(shuzi_num==1||shuzi_num==2) delay_time=200;  //1 ��  2�Լ���
            else  uart_putchar(UART_3,'S');                //�ȴ�����
            if(wuxian_data=='X') opmv_data=0x11;
            if(wuxian_data=='Y') opmv_data=0x12;
            if(wuxian_data=='U')
            {
              if(wuyu==0) opmv_data=0X15;
              if(wuyu==1) opmv_data=0x13;
              if(wuyu==2) opmv_data=0X11;
              wuyu++;
            }
             if(wuxian_data=='I')
            {
              if(wuyu==0) opmv_data=0X15;
              if(wuyu==1) opmv_data=0x13;
              if(wuyu==2) opmv_data=0X12;
              wuyu++;
            }
               if(wuxian_data=='O')
            {
              if(wuyu==0) opmv_data=0X15;
              if(wuyu==1) opmv_data=0x14;
              if(wuyu==2) opmv_data=0X11;
              wuyu++;
            }
             if(wuxian_data=='P')
            {
              if(wuyu==0) opmv_data=0X15;
              if(wuyu==1) opmv_data=0x14;
              if(wuyu==2) opmv_data=0X12;
              wuyu++;
            }
            
            car_zk=0;//ͣ��
            shizi=2;
            hope=0;
            no_ft=1;
          }
          if(car_zk==4) //�س���
          {
         //   beepds=5;
            shizi=2;
          }
          if(car_zk==5)
          {
            first_4_sizi=0;
            sum_mc=0;
             shizi=1;
             //beepds=20;
          }
          
      }
    }
  }
}


/*---------------------------------------------------
----------------------���---------------------------
//-----------------------------------------------------*///stop==0&&
void into_home()
{
 int n,m;
 int stop_Color=0;
   
   if(L_side[58]!=5&&R_side[58]!=154&&L_side[57]!=5&&R_side[57]!=154)
   {
 
  for(n=54;n<58;n++)
  {
    for(m=10;m<149;m=m+1)
    {
      if(abs(mt9v03x_image[n][m]-mt9v03x_image[n][m+1])>25)  stop_Color++;
    }
  }
    if(stop_Color>sobel_yun)
    {
      gpio_set(green,1);
      beepds=20;
      stop_mc_flash=1;
    //  STOP();
      //hope=0;
//      beepds=5;
    }
   }
}


void juy_banma_4()
{
  int m,n;
  int stop_Color=0;
  if(lu_sz[1]==0&&shizi!=2&&banma_4==1)   //�ж��ұ�����
  {
//    if(beepds==0) beepds=3;
    for(m=79;m<=140;m=m+5)
    {
      for(n=30;n<50;n++)
      {
        if(abs(mt9v03x_image[n][m]-mt9v03x_image[n+1][m])>23)  stop_Color++;
      }
    }
    
    if(stop_Color>=drath_banma)
    {
      banma_4=0;
      
        shizi=2;
        //beepds=20;
    }
  }
  
  if(lu_sz[1]==1&&shizi!=2&&banma_4==1)   //�ж��ұ�����
  {
//    if(beepds==0) beepds=3;
    for(m=79;m>=19;m=m-5)
    {
      for(n=30;n<50;n++)
      {
        if(abs(mt9v03x_image[n][m]-mt9v03x_image[n+1][m])>23)  stop_Color++;
      }
    }
    
    if(stop_Color>=drath_banma)
    {
      banma_4=0;
      
        shizi=2;
        //beepds=20;
    }
  }
}



float process_curvity(int x1, int y1, int x2, int y2, int x3, int y3)
{
    float K;
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    //����ķ��ű�ʾ����
    int q1 = (int)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    int AB = sqrt(q1);
    q1 = (int)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    int BC = sqrt(q1);
    q1 = (int)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    int AC = sqrt(q1);
    if (AB * BC * AC == 0)
    {
        K = 0;
    }
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}



float variance(int *array)
{
    int sum=0;
    int length = 40;
    float average=0;
    float var;
    for (int i = 0; i <= length;i++)
        {
            sum += *(array+i);//���
        }
        average = sum / length;//��ƽ��ֵ

        for (int i = 0; i <= length;i++)
        {
            var += pow(array[i]-average,2)/length;//�󷽲�
        }
        return  (var);
}


uint8 OTSU(uint8 *imgIn , uint8 Line , uint8 Line_end, uint8 Col_start , uint8 Col_end)
{
  int i,j,Pixe_Count=(Col_end-Col_start)*(Line_end-Line); //���ص�����
  uint8 MinValue,MaxValue;
  int PixelFront=0,PixelAfter=0; //ǰ�����������ص���
  float Per_Front=0,Per_After=0;//ǰ���ͱ������ص����
  float  ave_front=0,ave_after=0; //ǰ���������Ҷ�ƽ��ֵ
  uint32 Pixel_Value_Front=0,Pixel_Value_After=0; //ǰ���������Ҷ���ֵ  
  uint32 Pixel_Value_Total=0;//�Ҷ���ֵ
  int pixelCount[256]={0};
  float g,g_max=0; 
  uint8 Threshold=0;
  for (i = Line; i < Line_end; i++)
  {
    for (j = Col_start; j < Col_end; j++)
    {
        pixelCount[(int)imgIn[i * COL + j]]++;  //ͳ�ƻҶȼ���ÿ�����صĸ��� 
    }
  }
 
  for (MinValue = 0; MinValue < 255 && *(pixelCount+MinValue) == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
  for (MaxValue = 255; MaxValue > MinValue && *(pixelCount+MaxValue) == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ
  for (i = MinValue; i <= MaxValue; i++)
  {
      Pixel_Value_Total += pixelCount[i] * i;//�Ҷ���ֵ
  }  
  //---------���������䷽��
  for (i = MinValue; i < MaxValue; i++)  //i��Ϊ��ֵ��ȫֵ����
  {
    
      PixelFront += pixelCount[i];       //ͳ��ǰ������
      PixelAfter = Pixe_Count - PixelFront;  //ͳ�Ʊ�������
      
      Per_Front = (float)PixelFront/Pixe_Count;    //ǰ�����ذٷֱ�  w0
      Per_After = (float)PixelAfter/Pixe_Count;    //�������ذٷֱ�  w1  
      Pixel_Value_Front +=  pixelCount[i]*i;//ǰ���Ҷ���ֵ
      Pixel_Value_After = Pixel_Value_Total-Pixel_Value_Front; //�����Ҷ���ֵ
        
      ave_front = (float)Pixel_Value_Front/PixelFront;//ǰ��ƽ���Ҷ� u0
      ave_after = (float)Pixel_Value_After/PixelAfter; //����ƽ���Ҷ� u1
      
      g=Per_Front*Per_After*(ave_front-ave_after)*(ave_front-ave_after);
      if(g>g_max)
      {
        g_max=g; 
        Threshold=i;
      }
  }
  return Threshold;
}

uint32 m_sqrt(uint32 x)
{
  uint16 ans=0,p=0xffff;
  while(p!=0)
  {
    ans+=p;
    if(ans*ans>x)
    {
    ans-=p;
    }
    p=(uint16)(p/2);
  }
  return(ans);
}

int Sobel(uint8 *imgIn, int j)
{
  int x,y;
  uint16 grad;
  uint8 *up,*down;
  //����һ�Ծ�����зֱ�������x,y���� 
     up=&*(imgIn - 159);
     down=&*(imgIn + 159);
    //x�����ݶ�
        x=  *( up  + j+1)  //a2 imgIn - lineByte
        +2* *(imgIn+ j+1)  //a3
        +   *(down + j+1)  //a4
        -   *(up   + j-1)  //a0
        -2* *(imgIn+ j-1)  //a7
        -   *(down + j-1); //a6         
     //y�����ݶ�
        y=  *(up   + j-1)  //a0
        +2* *(up   +  j )  //a1
        +   *(up   + j+1)  //a2
        -   *(down + j-1)  //a6
        -2* *(down +  j )  //a5
        -   *(down + j+1); //a4
     grad=m_sqrt( (uint32)(x*x+y*y) );
   return grad;   
}

