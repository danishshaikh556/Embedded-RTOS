//
//  
//  DANISH SHAIKH and NICK FONDA

#include <includes.h>

#define   RT_EDGE     0
#define   LT_EDGE     48

#define  TASK_STK_SIZE                128u

#define  TASK_Rotate_PRIO                                6u
#define  TASK_Move_PRIO  				 6u
#define  Taskthree_PRIO					 5u

#define MAX_RPM                 87
#define RIGHT_SIDE_SENSOR       SENSOR_A
#define LEFT_SIDE_SENSOR        SENSOR_A
#define RIGHT_SIDE_SENSOR_PORT  RIGHT_IR_SENSOR_A_PORT
#define RIGHT_SIDE_SENSOR_PIN   RIGHT_IR_SENSOR_A_PIN
#define LEFT_SIDE_SENSOR_PORT    LEFT_IR_SENSOR_A_PORT
#define LEFT_SIDE_SENSOR_PIN     LEFT_IR_SENSOR_A_PIN

//static  void  WheelSensorEnable (tSide);                                                           

static  OS_TCB       TaskRotateTCB;
static  OS_TCB       TaskMoveTCB;
static  OS_TCB       TaskthreeTCB;

static  CPU_STK       TaskRotateStk[TASK_STK_SIZE];
static  CPU_STK       TaskMoveStk[TASK_STK_SIZE];
static  CPU_STK       TaskthreeStk[TASK_STK_SIZE];

static  void   TaskRotate        (void  *p_arg);
static  void   TaskMove          (void  *p_arg);
static  void   Taskthree         (void  *p_arg);

static  const  tSide  AppRobotLeftSide  = LEFT_SIDE;
static  const  tSide  AppRobotRightSide = RIGHT_SIDE;

int   buttonpress=0;

int   left_pct = 60, right_pct = 60;                           ///////sppeds of motor
int   hlf_pct =30;
int   qtr_pct =15;
int   left_counts = 0;
int   right_counts = 0;
int   movement[2][9]; /*{{1,24},{2,-90},{1,24},{2,90},{1,24},{2,45},{1,12},{2,-45},{1,36}}*/

int   j=0;                                                //////to access the array between tasks
int   totalNoOfMovements=9;                              ///////////total no of movements ie no of elements in array

//
//  main 
//
main (void)
{
    OS_ERR  err;
    
    CPU_INT32U  clk_freq;
    CPU_INT32U  cnts;
    CPU_INT32U  ulPHYMR0;
    movement[0][0]=1;
     movement[0][1]=2;
     movement[0][2]=1;
     movement[0][3]=2;
     movement[0][4]=1;
     movement[0][5]=2;
     movement[0][6]=1;
     movement[0][7]=2;
     movement[0][8]=1;
     
     
     movement[1][0]=24;
     movement[1][1]=-90;
     movement[1][2]=24;
     movement[1][3]=90;
     movement[1][4]=24;
     movement[1][5]=45;
     movement[1][6]=12;
     movement[1][7]=-45;
     movement[1][8]=36;
    
    

    //BSP_WheelSensorEnable();
    BSP_IntDisAll();                                            // Disable all interrupts.     

    OSInit(&err);                                               // Init uC/OS-III.  

    OSTaskCreate((OS_TCB     *)& TaskRotateTCB,                // Create task one                       
                 (CPU_CHAR   *)"Task Rotate",
                 (OS_TASK_PTR )  TaskRotate,
                 (void       *) 0,
                 (OS_PRIO     ) TASK_Rotate_PRIO,
                 (CPU_STK    *)& TaskRotateStk[0],
                 (CPU_STK_SIZE) TASK_STK_SIZE / 10u,
                 (CPU_STK_SIZE) TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    
       OSTaskCreate((OS_TCB     *)& TaskMoveTCB,                // Create task two                       
                 (CPU_CHAR   *)"Task Move",
                 (OS_TASK_PTR )  TaskMove,
                 (void       *) 0,
                 (OS_PRIO     ) TASK_Move_PRIO,
                 (CPU_STK    *)& TaskMoveStk[0],
                 (CPU_STK_SIZE) TASK_STK_SIZE / 10u,
                 (CPU_STK_SIZE) TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);


	OSTaskCreate((OS_TCB     *)& TaskthreeTCB,                // Create task two                       
                 (CPU_CHAR   *)"Taskthree",
                 (OS_TASK_PTR )  Taskthree,
                 (void       *) 0,
                 (OS_PRIO     ) Taskthree_PRIO,
                 (CPU_STK    *)& TaskthreeStk[0],
                 (CPU_STK_SIZE) TASK_STK_SIZE / 10u,
                 (CPU_STK_SIZE) TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
       
 
        
 
    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);                  /* Enable and Reset the Ethernet Controller.            */
    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

    ulPHYMR0 = EthernetPHYRead(ETH_BASE, PHY_MR0);              /* Power Down PHY                                       */
    EthernetPHYWrite(ETH_BASE, PHY_MR0, ulPHYMR0 | PHY_MR0_PWRDN);
    SysCtlPeripheralDeepSleepDisable(SYSCTL_PERIPH_ETH);


    clk_freq = BSP_CPUClkFreq();                                /* Determine SysTick reference freq.                    */
    cnts     = clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */
    CPU_TS_TmrFreqSet(clk_freq);

     //BSP_WheelSensorEnable();
    // WheelSensorEnable(LEFT_SIDE);
    // WheelSensorEnable(RIGHT_SIDE);
     
     BSP_MotorsInit ();
     BSP_PushButtonsInit ();

    
     OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}

//
//  Task Rotate
//

static  void   TaskRotate (void  *p_arg)
{
     OS_ERR  err;

    // Display strings for holding counts for buttons, touch sensors
    
    CPU_INT08S         left_disp[14] = "Rotating(Deg)";
    //CPU_INT08S         left_disp2[13] = "Moving(inch)";
    CPU_INT08S          val_disp[3] = "00\0";
    BSP_DisplayClear();
    
    while (DEF_ON) {   
    
      while (buttonpress==0)
      {  
        if (!BSP_PushButtonGetStatus(1)) {
                      buttonpress=1;
    
      }
      }
        
      if(buttonpress==1 && movement[0][j]==2 && j<totalNoOfMovements)       ////Rotate Command given
      {
   
            if(movement[1][j]>0)                /////////////rotate in the positve  direction
            {
              
              BSP_MotorDir(AppRobotLeftSide, FORWARD);
              BSP_MotorSpeed(AppRobotLeftSide, 0);    
              BSP_MotorRun(AppRobotLeftSide);   
    
              BSP_MotorDir(AppRobotRightSide, REVERSE);
              BSP_MotorSpeed(AppRobotRightSide, 0);    
              BSP_MotorRun(AppRobotRightSide); 
              left_counts=0;
              right_counts=0;
                  while(left_counts<=((movement[1][j])/15))  //trying to translate movement to sensor counts?
                    {
                         BSP_DisplayClear();
                  //       val_disp[]=movement[1][j];
                  //      left_disp = "Rotating(Deg)";
                         BSP_DisplayStringDraw(val_disp, RT_EDGE, 0u);
                         BSP_DisplayStringDraw(left_disp, RT_EDGE+16, 0u);
                      BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                      BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                      OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                        while (left_counts==((movement[1][j])/15)&& right_counts!=left_counts)
                        {
                        BSP_MotorSpeed(AppRobotLeftSide, (0*256));  
                        BSP_MotorSpeed(AppRobotRightSide, (qtr_pct*256));
                      OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                        }
                      BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                      BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                      OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                        while (right_counts==((movement[1][j])/15)&& left_counts!=right_counts)
                        {
                        BSP_MotorSpeed(AppRobotLeftSide, (qtr_pct*256));  
                        BSP_MotorSpeed(AppRobotRightSide, (0*256));
                        OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                        }
                      BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                      BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                     OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                    }}
            else                               ///////////rotate in the negative direction
            {
              
              if (movement[1][j]<0)
              {
                
              
                 
              BSP_MotorDir(AppRobotLeftSide, REVERSE);
              BSP_MotorSpeed(AppRobotLeftSide, 0);    
              BSP_MotorRun(AppRobotLeftSide);   
    
              BSP_MotorDir(AppRobotRightSide, FORWARD);
              BSP_MotorSpeed(AppRobotRightSide, 0);    
              BSP_MotorRun(AppRobotRightSide);
              left_counts=0;
              right_counts=0;
                while(left_counts<=((movement[1][j]*(-1))/15))
                      {
                         BSP_DisplayClear();
                         //val_disp=(movement[1][j]*(-1));
                         //left_disp = "Rotating(Deg)";
                         BSP_DisplayStringDraw(val_disp, RT_EDGE, 0u);
                         BSP_DisplayStringDraw(left_disp, RT_EDGE+16, 0u);
                      BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                      BSP_MotorSpeed(AppRobotRightSide, (right_pct*256)); 
                      OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                      while (left_counts==((movement[1][j]*(-1))/15)&& right_counts!=left_counts)
                        {
                        BSP_MotorSpeed(AppRobotLeftSide, (0*256));  
                        BSP_MotorSpeed(AppRobotRightSide, (qtr_pct*256));
                        OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                        }
                      BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                      BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                      OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);   
                      while (right_counts==((movement[1][j]*(-1))/15)&& left_counts!=right_counts)
                        {
                        BSP_MotorSpeed(AppRobotLeftSide, (qtr_pct*256));  
                        BSP_MotorSpeed(AppRobotRightSide, (0*256));
                        OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                        }
                      BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                      BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                      OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);  
                      } 
                }
            }
                      

                   j=j+1;
                  
                   
                   if(j==totalNoOfMovements)                  ///condotion for halting completely
                  {
                  left_pct=0;
                  right_pct=0;
                  BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                  BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                  }          
                  
           
      }
       OSTimeDlyHMSM(0u, 0u, 0u, 5u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);         
      } 

       
    

    }

//
//    Task Two
//

static  void   TaskMove (void  *p_arg)
{
    OS_ERR  err;

   
    while (DEF_ON) {  /* Task body, always written as an infinite loop.       */


      if(buttonpress==1 && movement[0][j]==1 && j<totalNoOfMovements)      ///////checking if move fwd condition is satisfied
      {
         BSP_MotorDir(AppRobotLeftSide, FORWARD);
         BSP_MotorSpeed(AppRobotLeftSide, 0);    
          BSP_MotorRun(AppRobotLeftSide);   
    
          BSP_MotorDir(AppRobotRightSide, FORWARD);
          BSP_MotorSpeed(AppRobotRightSide, 0);    
          BSP_MotorRun(AppRobotRightSide); 
          left_counts=0;
          right_counts=0; 
            while( left_counts  <= ((movement[1][j])/(0.343611)))  
                  
                  //D=14/16 in
                  //cir = 3.14159*(14/16) = inches/revolution
                  //1 count= (3.14159*(14/16))/8 == 0.343611 inches/count 
            {
                          BSP_DisplayClear();
                         //val_disp=movement[1][j];
                         //left_disp2 = "Moving(inch)";
                         //BSP_DisplayStringDraw(val_disp, RT_EDGE, 0u);
                         //BSP_DisplayStringDraw(left_disp, RT_EDGE+16, 0u);
                BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
                while(left_counts>right_counts)
                { 
                  BSP_MotorSpeed(AppRobotLeftSide, ((hlf_pct+qtr_pct)*256)); 
                }
                while(left_counts<right_counts)
                { 
                  BSP_MotorSpeed(AppRobotRightSide, ((hlf_pct+qtr_pct)*256)); 
                }
            }

                j=j+1;
                  if(j==totalNoOfMovements)                  ///condotion for halting completely
                  {
                  left_pct=0;
                  right_pct=0;
                  BSP_MotorSpeed(AppRobotLeftSide, (left_pct*256));  
                  BSP_MotorSpeed(AppRobotRightSide, (right_pct*256));
}                 

      }       
   OSTimeDlyHMSM(0u, 0u, 0u, 50u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }}




///
/// Task three
///
static  void   Taskthree (void  *p_arg)
{       
    int setbit=0;
    int setbit2=0;
     OS_ERR  err;
 
    BSP_WheelSensorEnable();                                /* Enable wheel sensors.                                */

    
    while (DEF_ON) {                                            /* Task body, always written as an infinite loop.       */


      while(1)
      {
        if (GPIOPinRead(LEFT_SIDE_SENSOR_PORT, LEFT_SIDE_SENSOR_PIN)) {
            if(setbit==0) {
                setbit=1; }
            if(setbit==2) {
                    setbit=0;
                   left_counts++;   }
                          }
                                                                      
            if (GPIOPinRead(RIGHT_SIDE_SENSOR_PORT, RIGHT_SIDE_SENSOR_PIN)) {
            if(setbit2==0) {
                setbit2=1; }
            if(setbit2==2) {
                    setbit2=0;
		   right_counts++;
                          }
                                                                            }
      if (!GPIOPinRead(LEFT_SIDE_SENSOR_PORT, LEFT_SIDE_SENSOR_PIN)) {
          setbit =2;                                                 } 
      if (!GPIOPinRead(RIGHT_SIDE_SENSOR_PORT, RIGHT_SIDE_SENSOR_PIN)) {
          setbit2 =2;                                                  }
      
        
              OSTimeDlyHMSM(0u, 0u, 0u, 10u,
              OS_OPT_TIME_HMSM_STRICT,
              &err);
      }
    }}

//static void  WheelSensorEnable (tSide  eSide)
//{
 //   if (eSide == LEFT_SIDE) {
//
//        BSP_WheelSensorEnable();                                /* Enable wheel sensors.                                */
//
 //       BSP_WheelSensorIntEnable(LEFT_SIDE, LEFT_SIDE_SENSOR,   /* Enable wheel sensor interrupts.                      */
 //                                WheelSensorIntHandler);
//
//    } else {
//
//        BSP_WheelSensorEnable();                                /* Enable wheel sensors.                                */
//
//        BSP_WheelSensorIntEnable(RIGHT_SIDE, RIGHT_SIDE_SENSOR, /* Enable wheel sensor interrupts.                      */
//                                 WheelSensorIntHandler);
//    }
//}

