//控件1
//data
#define CHANGESYS "channel/widget_0/cmd/control"

Timer timer0(5, timer0_callback);//声明定时器，8ms一次中断
Timer timer1(8,timer1_callback);//声明定时器,10ms一次中断
struct PID//定义pid结构体
{
	float kp;
	float kd;
	float ki;
	float err=0;
	float err_last=0;
	float i=0;
	float out=0;
}
pid,pids;

int duty=0,ADC_value=0;
int setpoint=2068;//定义倒立点
int setpos=0;//定义旋转轴原点
int num=0,AA,BB,AA_flag,BB_flag,num_last;//定义编码器滤波标志位
void adc_avr(void);
void timer0_callback(void)//倒立环及电机控制
{
	pid.err=ADC_value-setpoint;
	pid.out=pid.kp*pid.err+pid.i*pid.ki+(pid.err-pid.err_last)*pid.kd;
	pid.err_last=pid.err;
	pid.i+=pid.err;
	
	duty=pid.out+pids.out;

 	if(duty<-4095)duty=-4095;
 	if(duty>4095)duty=4095;
 	if(ADC_value>setpoint+400||ADC_value<setpoint-400)duty=0;//设置倒立区阈值
 	if(duty<0)
	{
	    duty=-duty;
		analogWrite(A5,0,10000);		
		analogWrite(A6, duty,10000);
	}
	else if(duty>0)
	{
		analogWrite(A5, duty , 10000);		
		analogWrite(A6, 0 , 10000);
	}
	else 
	{
	    analogWrite(A5, 0 , 10000);		
		analogWrite(A6, 0 , 10000);
	}

}

void timer1_callback(void)//速度环
{
    pids.err=(num-num_last);
    num_last=num;
    pids.out=pids.kp*pids.err+pids.i*pids.ki;
    pids.i+=pids.err;
    if(pids.err<100)pids.i=0;
    else if(pids.out>4095||pids.out<-4095)pids.i=0;
}

void countA(void)//编码器滤波函数
{
    if(AA_flag==0)
	{
		if(digitalRead(D4)==0)
			num++;
		else num--;
		AA_flag=1;BB_flag=0;
	}

}
void countB(void)
{
if(BB_flag==0)
	{
		AA_flag=0;BB_flag=1;
	}


}

void pullsys()
{
    
}

void setup()
{
	pid.kp=200;//200;
	pid.kd=4000;//2000;
// 	Serial.begin(115200);
    pids.kp=-2000;
    pids.ki=0;
    pids.out=0;
 	SerialUSB.begin(115200);
	pinMode(A7,INPUT);
	pinMode(A5,OUTPUT);
	pinMode(A6,OUTPUT);
	pinMode(D7,OUTPUT);
	
	pinMode(D2,INPUT_PULLUP);//定义编码器io
	pinMode(D4,INPUT_PULLUP);
	
	analogWriteResolution(A5, 12);  
	analogWriteResolution(A6, 12); 
	analogWrite(A5, 0, 10000);
	analogWrite(A6, 0, 10000);
	timer0.start();
	timer1.start();
	attachInterrupt(D2,countA, FALLING);
	attachInterrupt(D4,countB, FALLING);
	
	IntoRobot.subscribe(CHANGESYS, NULL, pullsys);
}
void loop()
{
 	SerialUSB.println(ADC_value);
    adc_avr();
    SerialUSB.println(num);
    SerialUSB.println(duty);
    SerialUSB.println(pids.err);
    SerialUSB.println(pids.out);
 	SerialUSB.println();
    //publish
    //IntoRobot.publish(CHANGESYS, "pid.kp");
    //publish
    //IntoRobot.publish(CHANGESYS, (uint8_t *)tmp, strlen(tmp));
   // IntoRobot.publish(CHANGESYS, (uint8_t *)tmp, strlen(tmp));
}
void adc_avr(void)
{
    int i,sum=0;
    for(i=0;i<5;i++)
 	{
        sum+=analogRead(A7);
 	}
    ADC_value=sum/5.0;
}
