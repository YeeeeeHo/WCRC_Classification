#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Circuit_Playground.h>

#include <Dynamixel.h>
#include <Pixy2.h>  //Pixy2카메라를 사용하기 위하여 headerfile을 불러오기
#include <DynamixelWorkbench.h> // Dynamixel을 이용하기 위해 headerfile 을 불러오기
//#include <OLLO.h> //ollo를 사용하기 위해 headerfile을 불러오기

#define DXL_BUS_SERIAL1 "1"           //Dynamixel on Serial1(USART1)  <-OpenCM9.04

#define BAUDRATE  1000000  //전송 속도 조절

#define DXL_ID1    1    ///모터 ID별 define
#define DXL_ID2    2
#define DXL_ID3    3
#define DXL_ID4    4
#define DXL_ID5    5
#define DXL_ID7    7
#define DXL_ID8    8
#define DXL_ID9    9
#define DXL_ID10   10


Pixy2 pixy;   // pixy2 카메라를 사용하기 위해 불러옴
OLLO myOLLO;  // 로봇 하단의 ollo dms 센서를 활용하기 위해
DynamixelWorkbench dxl_wb;  // motion 제어와 모터를 제어하기 위해
/////모션배열//////////////////////////////////
int32_t Init_1[5] = {494, 599, 252, 632, 512};  /////// 기본자세
int32_t grap_1[5] = {492, 669, 252, 632, 512};  ////// 아래찾기
int32_t grap_1_1[5] = {814, 669, 252, 632, 512}; /////// 아래잡기
int32_t grap_2[5] = {492, 489, 412, 662, 512};  //////// 위 찾기
int32_t grap_2_2[5] = {814, 489, 412, 654, 512};  ///////위 잡기
int32_t grap_2_3[5] = {814, 489, 322, 770, 512};  ///////// 위 잡기 2

int32_t grap_Init[5] = {814, 689, 252, 632, 512};   ///////////놓기 기본
int32_t placement_1[5] = {814, 709, 442, 442, 592};    /////////1번방
int32_t placement_1_1[5] = {492, 709, 442, 442, 592};    /////////1번방 놓기
int32_t placement_2[5] = {814, 709, 442, 442, 542};    /////////2번방
int32_t placement_2_2[5] = {492, 709, 442, 442, 542};    ///////2번방 놓기
int32_t placement_3[5] = {814, 709, 442, 442, 487};   //////3번방
int32_t placement_3_3[5] = {492, 709, 442, 442, 487};  //////////3번벙 놓기
int32_t placement_4[5] = {814, 709, 442, 442, 437};     ////////////4번방
int32_t placement_4_4[5] = {492, 709, 442, 442, 437}; ////////////4번방 놓기
int32_t placement_5[5] = {814, 689, 301, 572, 614};         /////////////5번방
int32_t placement_5_5[5] = {492, 689, 301, 572, 614};    ////////////5번방 놓기
int32_t placement_6[5] = {814, 689, 251, 632, 550};      /////////////6번방
int32_t placement_6_6[5] = {492, 689, 251, 632, 550};   //////////////6번방 놓기
int32_t placement_7[5] = {814, 689, 251, 632, 489};     /////////////7번방
int32_t placement_7_7[5] = {492, 689, 251, 632, 489};      //////////7번방 놓기
int32_t placement_8[5] = {814, 689, 301, 572, 424};         /////////8번방
int32_t placement_8_8[5] = {492, 689, 301, 572, 424};     ////////8번방 놓기

const uint8_t handler_index = 0;    /////syncWrite 할 때 전체 모터에 저 배열 값을 넣겠다 인거 같은데

int count = 1;      // 블록 순서
void setup()
{
 Serial.begin(115200);
  myOLLO.begin(1);
  myOLLO.begin(2);
  myOLLO.begin(3);
  myOLLO.begin(4);
  pixy.init();

  uint16_t model_number = 0;
  uint8_t dxl_id[9] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, DXL_ID7, DXL_ID8,DXL_ID9, DXL_ID10};

  dxl_wb.init(DXL_BUS_SERIAL1, BAUDRATE);
  dxl_wb.ping(dxl_id[0], &model_number);  
  dxl_wb.ping(dxl_id[1], &model_number);
  dxl_wb.ping(dxl_id[2], &model_number);
  dxl_wb.ping(dxl_id[3], &model_number);
  dxl_wb.ping(dxl_id[4], &model_number);
  dxl_wb.ping(dxl_id[5], &model_number);
  dxl_wb.ping(dxl_id[6], &model_number);
  dxl_wb.ping(dxl_id[7], &model_number);
  dxl_wb.ping(dxl_id[8], &model_number);
  dxl_wb.jointMode(dxl_id[0], 100, 32); //id에 따른 모터 동작모드 설정 - 관절모드
  dxl_wb.jointMode(dxl_id[1], 100, 32);
  dxl_wb.jointMode(dxl_id[2], 100, 32);
  dxl_wb.jointMode(dxl_id[3], 100, 32);
  dxl_wb.jointMode(dxl_id[4], 200, 32);
  dxl_wb.wheelMode(dxl_id[5]);  //id에 따른 모터 동작모드 설정 - 바퀴모드
  dxl_wb.wheelMode(dxl_id[6]);
  dxl_wb.wheelMode(dxl_id[7]);
  dxl_wb.wheelMode(dxl_id[8]); 

  dxl_wb.addSyncWriteHandler(dxl_id[5], "Goal_Position");
}

void loop()
{

  int i = 1;
  pixy.ccc.getBlocks();
  int ID1_Speed = 0 , ID2_Speed = 0, ID3_Speed = 0, ID4_Speed = 0;
  int R_dms1 = myOLLO.read(3); // 오른쪽 dms 센서 define 
  int L_dms4 = myOLLO.read(4);  // 왼쪽 dms 센서 define


  dxl_wb.syncWrite(handler_index, &grap_1[0]); // 기본자세 motion 실행
  delay(3000);

  myOLLO.write(2,1,1);// ollo LED 켜기 
  delay(200);
  myOLLO.write(2,0,1);// ollo LED 끄기
  delay(200);
  myOLLO.write(2,1,1);
  delay(200);
  myOLLO.write(2,0,1);
  delay(200);
  myOLLO.write(2,1,1);
  delay(200);
  myOLLO.write(2,0,1);
  delay(200);
  myOLLO.write(2,0,1);
  delay(200);
  myOLLO.write(2,1,1);
  delay(200);
  myOLLO.write(2,1,0);
  delay(200);
  myOLLO.write(2,1,1);
  delay(200);

   FF(100,100,-100,-100); // 전진
   delay(300);
   FF(150,150,-150,-150); // 전진
   delay(300);
   while (1)
   {
     int R_dms1 = myOLLO.read(3); 
     int L_dms4 = myOLLO.read(4);
     if (R_dms1> 680 && L_dms4 > 680)// dms 센서를 활용해 벽에 닿기 직전까지 전진
     {
      FF(150,150,-150,-150);
      delay(60);
      FF(100,100,-100,-100);
      delay(60);
      FF(0,0,0,0);
      delay(60);
      break;
     }
     else
     {
       FF(220,220,-200,-200);
     }
   }
    FF(-200,200,-200,250); // 왼 가기
    delay(1200);
    FF(0,0,0,0);
    delay(300);
  
    FF(100,100,-100,-100); // 앞으로 가기
    delay(300);
    
    FF(120,120,-120,-120);
    delay(300);
    
    FF(170,170,-150,-150);  
    delay(1700);
    
    FF(120,120,-120,-120);
    delay(300);
    
    FF(100,100,-100,-100);
    delay(100);
    
    FF(0,0,0,0);
    delay(60);

    dxl_wb.syncWrite(handler_index, &Init_1[0]); // 블록 detect 자세
    delay(2000);

   while(1) // 무한반복
   {
     while(i == 1)  // 순서를 정하기 위해 위에서 i = 1
     {
      pixy.ccc.getBlocks( 1, 1 << 0 );   /// 블럭 1번 Catch 2번 블록이면 2로 변환  //shift 연산자로 2진수로 되어있는 object detect 값을 넘겨 받기 0b0001
      if (pixy.ccc.numBlocks) {         ///만약 블록을 찾았다면
        if (pixy.ccc.blocks[0].m_signature == 1) {    /// 만약 블록의 signature가 1이라면
          if(pixy.ccc.blocks[0].m_x>152 && pixy.ccc.blocks[0].m_x<162)    // 블록을 중앙에 위치 시키기 위한
           {
             if (pixy.ccc.blocks[0].m_width < 95) // 블록 detect pixel 수 를 이용해 잡으러 앞으로 가기    
               {
                 FF(100,100,-100,-100);
               }
             if (pixy.ccc.blocks[0].m_width > 96) // 일정 개수 이상 detect 되었을 때 멈추고 큐브 잡기 잡기 거리를 조절 하는곳
             {
               FF(0,0,0,0);
               delay(100);
               step3();           // 블록 잡기 준비자세 motion 실행 & 회전 후 블럭 놓는 위치로 turn
               FF(0,0,0,0);
               delay(60);
                   while (1)
                   {
                     int R_dms1 = myOLLO.read(3);
                     int L_dms4 = myOLLO.read(4);
                     if (R_dms1> 725 || L_dms4 > 725) //밑의 센서를 이용해 블록 놓는 위치에 다가가기
                       {
                         FF(140,140,-100,-100); // 전진 & 각도 조절
                         delay(400); // 앞으로 가는 시간 조절 
                         FF(0,0,0,0);
                         delay(60);
                         break;
                       }
                     else
                       {
                         FF(100,100,-100,-100); // 직진
                       }
                   }
                 dxl_wb.syncWrite(handler_index, &placement_4[0]);   // 놓기 준비자세
                 delay(3000);
                 dxl_wb.syncWrite(handler_index, &placement_4_4[0]);  // 4번 놓기 
                 delay(3000);
                 step2();   // 대각선으로 후진
                 FF(0,0,0,0);
                 delay(730);
                 dxl_wb.syncWrite(handler_index, &Init_1[0]); // 기본자세
                 delay(3000);
                 count+=count;
                 i++;
             }
           }
          else if(pixy.ccc.blocks[0].m_x<134) // 로봇이 블록보다 왼쪽에 있을 경우 전진 조정
          {
            FF(-200,200,-200,200);
          }
          else if(pixy.ccc.blocks[0].m_x<153) // 로봇이 블록보다 많이 왼쪽에 있을 경우 전진 조정
          {
            FF(50,50,-100,-100);
          }
          else if(pixy.ccc.blocks[0].m_x>186) // 로봇이 블록보다 오른쪽에 있을 경우 전진 조정
          {
            FF(200,-200,200,-200);
          }
          else if(pixy.ccc.blocks[0].m_x>163) // 로봇이 블록보다 많이 오른쪽에 있을 경우 전진 조정
          {
            FF(100,100, -50, -50);

          }
        }
      }
    }
    while(i == 2) // 두번 째 물체를 잡기 위해 i = 2로 바뀐 모습
    {
      pixy.ccc.getBlocks( 1, 2 << 0 );  // 2번 signature 넘겨 받기 = 0b0010
      if (pixy.ccc.numBlocks) {
        if (pixy.ccc.blocks[0].m_signature == 2) {
          if(pixy.ccc.blocks[0].m_x>152 && pixy.ccc.blocks[0].m_x<162)
           {
             if (pixy.ccc.blocks[0].m_width < 95)
               {
                 FF(100,100,-100,-100);
               }
             if (pixy.ccc.blocks[0].m_width > 96)
             {
               FF(0,0,0,0);
               delay(100);
               step1();   // 위쪽 블록 detect자세
               FF(0,0,0,0);
               delay(60);
                   while (1)
                   {
                     int R_dms1 = myOLLO.read(3);
                     int L_dms4 = myOLLO.read(4);
                     if (R_dms1> 720 && L_dms4 > 720)
                       {
                         FF(140,140,-100,-100);
                         delay(140);
                         FF(0,0,0,0);
                         delay(60);

                         break;
                       }
                     else
                       {
                         FF(100,100,-100,-100);
                       }
                   }
                 dxl_wb.syncWrite(handler_index, &placement_1[0]);    //1번 위치 
                 delay(3000);
                 dxl_wb.syncWrite(handler_index, &placement_1_1[0]);    //1번위치 놓고 돌아오기
                 delay(3000);
                 step2(); // 대각선 후진
                 FF(0,0,0,0);
                 delay(730);
                 dxl_wb.syncWrite(handler_index, &Init_1[0]);
                 delay(3000);
                 count+=count;
                 i++;
             }
           }
          else if(pixy.ccc.blocks[0].m_x<134)
          {
            FF(-200,200,-200,200);
          }
          else if(pixy.ccc.blocks[0].m_x<153)
          {
            FF(50,50,-100,-100);
          }
          else if(pixy.ccc.blocks[0].m_x>186)
          {
            FF(200,-200,200,-200);
          }
          else if(pixy.ccc.blocks[0].m_x>163)
          {
            FF(100,100, -50, -50);
          }
        }
      }
   }
   while(i == 3)
   {
     pixy.ccc.getBlocks( 1, 4 << 0 ); //2진법이기 때문에 4 = 0b0100 
     if (pixy.ccc.numBlocks) {
         if (pixy.ccc.blocks[0].m_signature == 3) {
           if(pixy.ccc.blocks[0].m_x>152 && pixy.ccc.blocks[0].m_x<162)
            {
              if (pixy.ccc.blocks[0].m_width < 95)
                {
                  FF(100,100,-100,-100);
                }
              if (pixy.ccc.blocks[0].m_width > 96)
              {
                FF(0,0,0,0);
                delay(100);
                step1();  // 하단 블록 detect
                FF(0,0,0,0);
                delay(60);
                    while (1)
                    {
                      int R_dms1 = myOLLO.read(3);
                      int L_dms4 = myOLLO.read(4);
                      if (R_dms1> 720 && L_dms4 > 720)
                        {
                          FF(140,140,-100,-100);
                          delay(120);
                          FF(0,0,0,0);
                          delay(60);
                          break;
                        }
                      else
                        {
                          FF(100,100,-100,-100);
                        }
                    }
                  dxl_wb.syncWrite(handler_index, &placement_1[0]); // 1번 위치에 블록 내려놓기
                  delay(3000);
                  dxl_wb.syncWrite(handler_index, &placement_1_1[0]); 
                  delay(3000);
                  dxl_wb.syncWrite(handler_index, &grap_Init[0]);
                  delay(3000);
                  count+=count;
                  i++;
              }
            }
           else if(pixy.ccc.blocks[0].m_x<134)
           {
             FF(-200,200,-200,200);
           }
           else if(pixy.ccc.blocks[0].m_x<153)
           {
             FF(50,50,-100,-100);
           }
           else if(pixy.ccc.blocks[0].m_x>186)
           {
             FF(200,-200,200,-200);
           }
           else if(pixy.ccc.blocks[0].m_x>163)
           {
             FF(100,100, -50, -50);
           }
         }
       }
     }
    if (count >4)
      {
        FF(-200,200,-200,230);
        delay(3000);
             while (1){
               int R_dms1 = myOLLO.read(3);
               int L_dms4 = myOLLO.read(4);
               if (R_dms1 > 400 && L_dms4 > 400) {
                 break;
               }
               else{
                 FF(-200,200,-200,230);
               }
              }
        }
  }
exit(0);
}
//////////////////////////////// 함수 설정//////////////////////////////
 void FF(int ID1_Speed, int ID3_Speed, int ID2_Speed, int ID4_Speed) { 
   ID1_Speed = ID1_Speed +20;
   ID3_Speed = ID3_Speed +20;
   ID2_Speed = ID2_Speed - 20;
   ID4_Speed = ID4_Speed - 20;

   dxl_wb.goalSpeed(DXL_ID7, ID1_Speed);
   dxl_wb.goalSpeed(DXL_ID9, ID3_Speed);
   dxl_wb.goalSpeed(DXL_ID8, ID2_Speed);
   dxl_wb.goalSpeed(DXL_ID10, ID4_Speed);
 }
 void step1()
 {
   dxl_wb.syncWrite(handler_index, &grap_1[0]); 
   delay(3000);
   dxl_wb.syncWrite(handler_index, &grap_1_1[0]);
   delay(3000);
   dxl_wb.syncWrite(handler_index, &grap_Init[0]);
   delay(3000);
   //잡고 뒤로 후진
   FF(-200,-200,200,200);
   dela y(460);
   FF(0,0,0,0);
   delay(60);
   // 왼쪽  돌기 
   FF(-200,-200,-200,-200);
   delay(830);
 }
 void step3()
 {
   dxl_wb.syncWrite(handler_index, &grap_2[0]);
   delay(3000);
   dxl_wb.syncWrite(handler_index, &grap_2_2[0]);
   delay(3000);
   dxl_wb.syncWrite(handler_index, &grap_2_3[0]);
   delay(3000);
   //잡고 뒤로 후진
   FF(-200,-200,200,200);
   delay(460);
   FF(0,0,0,0);
   delay(60);
   // 왼쪽  돌기 
   FF(-200,-200,-200,-200);
   delay(850);
   FF(0,0,0,0);
   delay(60);
   dxl_wb.syncWrite(handler_index, &grap_Init[0]);
   delay(3000);

 
 }
 void step2()
 {
  FF(-300,0,0,300);
  delay(2000);

  FF(200,200,200,200);
  delay(830);
 }
