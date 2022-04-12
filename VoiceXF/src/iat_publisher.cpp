/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/
#include "iat_publisher.h"
 


using namespace std;


#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096


Pid_Typedef position;
Pid_Typedef anglar;
carNOW_Type myrobot = {0,0,0.0,0.0};

const char* Go ="Go";
const char* Back ="Back";
const char* Left ="Left";
const char* Right ="Right";
const char* Stop ="Stop";

char* Voice=NULL; //语音动作识别

double anglar_to_r(double anglar){
    return anglar * (0.0174532);
}
double r_to_anglar(double r){
    return r/(0.0174532);
}
class SubscribeAndPublish  
{  
public:  
  SubscribeAndPublish()  
    {  
        count = 0;
        position_now = 0;
        position_last = 0;
		position_x = 0;
		position_y = 0;
		Move_mode=0;

        yaw = 0;
        yaw_last = 0;
        //Topic you want to publish  
        cmd_pub= n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		//初始化PID
		position.P = 1.5f;
		position.I = 0;
		position.D = 0.5;
		position.ErrorMax = 1000.0f;
		position.IMax = 1000.0f;
		position.SetPoint = 0.0f;	
		position.OutMax = 0.7;

		anglar.P = 0.5f;
		anglar.I = 0;
		anglar.D = 2;
		anglar.ErrorMax = 1000.0f;
		anglar.IMax = 1000.0f;
		anglar.SetPoint = 0.0f;	
		anglar.OutMax = 3;	

    }  
    void PID_RUN();
    bool Voice_Dispose(void);
	void shuju_main_class(carNOW_Type *myrobot){
		position_last = position_now;
		yaw_last = yaw;

		position_now = myrobot->position_now;
		yaw = myrobot->yaw;
	}
private:  
    ros::NodeHandle n;   
    ros::Publisher cmd_pub;
    geometry_msgs::Twist Voice_contol;
    int count;
public:
    double position_now; 
    double position_last;
    double yaw;
    double yaw_last;

	double position_x;
	double position_y;

    short Move_mode;//移动方式
    Setpoint_Type Setpoint;//根据运动模式不同setpoint对象不同，go为米，back为度数

};//End of class SubscribeAndPublish  

char GetInput()
{
    //fd_set 为long型数组
    //其每个元素都能和打开的文件句柄建立联系
    fd_set rfds;
    struct timeval tv;
    char c = '\0';
 
    //将　rfds数组清零
    FD_ZERO(&rfds);
    //将rfds的第0位置为１，这样fd=1的文件描述符就添加到了rfds中
    //最初　rfds为00000000,添加后变为10000000
    FD_SET(0, &rfds);
    tv.tv_sec = 1;
    tv.tv_usec = 0; //设置等待超时时间
 
    //检测键盘是否有输入
    //由内核根据io状态修改rfds的内容，来判断执行了select的进程哪个句柄可读
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        c = getchar();
 
        return c;
    }
 
    //没有数据返回n
    return 'n';
}
 

int resultFlag=0;
int disposeFlag=0;

/* Upload User words */
int scanKeyboard()
{
  //  struct termios
  //    {
  //      tcflag_t c_iflag;		/* input mode flags */
  //      tcflag_t c_oflag;		/* output mode flags */
  //      tcflag_t c_cflag;		/* control mode flags */
  //      tcflag_t c_lflag;		/* local mode flags */
  //      cc_t c_line;			/* line discipline */
  //      cc_t c_cc[NCCS];		/* control characters */
  //      speed_t c_ispeed;		/* input speed */
  //      speed_t c_ospeed;		/* output speed */
  //  #define _HAVE_STRUCT_TERMIOS_C_ISPEED 1
  //  #define _HAVE_STRUCT_TERMIOS_C_OSPEED 1
  //    };
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
  new_settings = stored_settings;           //
  new_settings.c_lflag &= (~ICANON);        //
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(STDIN_FILENO,TCSANOW,&new_settings); //

  in = getchar();

  tcsetattr(STDIN_FILENO,TCSANOW,&stored_settings);
  return in;
}

static int upload_userwords()
{
	char*			userwords	=	NULL;
	size_t			len			=	0;
	size_t			read_len	=	0;
	FILE*			fp			=	NULL;
	int				ret			=	-1;

	fp = fopen("userwords.txt", "rb");
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n");
		goto upload_exit;
	}

	fseek(fp, 0, SEEK_END);
	len = ftell(fp); 
	fseek(fp, 0, SEEK_SET);  					
	
	userwords = (char*)malloc(len + 1);
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");
		goto upload_exit;
	}

	read_len = fread((void*)userwords, 1, len, fp); 
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");
		goto upload_exit;
	}
	userwords[len] = '\0';
	
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //ÉÏ´«ÓÃ»§´Ê±í
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
		goto upload_exit;
	}
	
upload_exit:
	if (NULL != fp)
	{
		fclose(fp);
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords);
		userwords = NULL;
	}
	
	return ret;
}


static void show_result(char *string, char is_over)
{
	printf("\rResult: [ %s ]", string);
	if(is_over)
		putchar('\n');
}

char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;
char *ros_result = NULL;

void on_result(char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		//show_result(g_result, is_last);
		ROS_INFO("result:[ %s ]\n",g_result);
		ros_result = result;
		resultFlag = 1;
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");
	else
		printf("\nRecognizer error %d\n", reason);
}

/* demo send audio data from a file */
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0;
	FILE*	f_pcm = NULL;
	char*	p_pcm = NULL;
	unsigned long	pcm_count = 0;
	unsigned long	pcm_size = 0;
	unsigned long	read_size = 0;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	if (NULL == audio_file)
		goto iat_exit;

	f_pcm = fopen(audio_file, "rb");
	if (NULL == f_pcm)
	{
		printf("\nopen [%s] failed! \n", audio_file);
		goto iat_exit;
	}

	fseek(f_pcm, 0, SEEK_END);
	pcm_size = ftell(f_pcm);
	fseek(f_pcm, 0, SEEK_SET);

	p_pcm = (char *)malloc(pcm_size);
	if (NULL == p_pcm)
	{
		printf("\nout of memory! \n");
		goto iat_exit;
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
	if (read_size != pcm_size)
	{
		printf("\nread [%s] error!\n", audio_file);
		goto iat_exit;
	}

	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode);
		goto iat_exit;
	}

	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode);
		goto iat_exit;
	}

	while (1)
	{
		unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
		int ret = 0;

		if (pcm_size < 2 * len)
			len = pcm_size;
		if (len <= 0)
			break;

		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

		if (0 != ret)
		{
			printf("\nwrite audio data failed! error code:%d\n", ret);
			goto iat_exit;
		}

		pcm_count += (long)len;
		pcm_size -= (long)len;		
	}

	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode);
		goto iat_exit;
	}

iat_exit:
	if (NULL != f_pcm)
	{
		fclose(f_pcm);
		f_pcm = NULL;
	}
	if (NULL != p_pcm)
	{
		free(p_pcm);
		p_pcm = NULL;
	}

	sr_stop_listening(&iat);
	sr_uninit(&iat);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(i++ < 6)
		sleep(1);
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}



void odom_Callback(const nav_msgs::Odometry msg)  
{   
	myrobot.position_x = msg.pose.pose.position.x;
	myrobot.position_y = msg.pose.pose.position.y;
	myrobot.yaw = tf::getYaw(msg.pose.pose.orientation);
	myrobot.yaw = r_to_anglar(myrobot.yaw);


	myrobot.position_now = myrobot.position_x*myrobot.position_x + myrobot.position_y*myrobot.position_y;
	myrobot.position_now = pow(myrobot.position_now,0.5);


	//ROS_INFO("Receive odom ");  
}

int main(int argc, char* argv[])
{
	// 初始化ROS
    ros::init(argc, argv, "voiceRecognition");
    ros::NodeHandle nh;
	SubscribeAndPublish Voice_class;
    system(STTY_US TTY_PATH);

    ros::Rate loop_rate(20);

	ros::Subscriber position_sub = nh.subscribe("/odom", 1, odom_Callback); 
	

    ROS_INFO("Sleeping...\n");
	char c=0;
	while(ros::ok()){
		ros::spinOnce();

		ROS_INFO("Plase press 's',you can start Voice input\n");


		c = get_char();
		ROS_INFO("You press Key:%c\n",c);
		if(c == 's'){
			// //语音库信息确认
			// int ret = MSP_SUCCESS;
			// int upload_on =	1; 
			// const char* login_params = "appid = 8eb8d75f, work_dir = .";
			// int aud_src = 0; 
			// const char* session_begin_params =
			// 	"sub = iat, domain = iat, language = zh_cn, "
			// 	"accent = mandarin, sample_rate = 16000, "
			// 	"result_type = plain, result_encoding = utf8";

			// ret = MSPLogin(NULL, NULL, login_params);
			// if (MSP_SUCCESS != ret)	{
			// 	printf("MSPLogin failed , Error code %d.\n",ret);
			// 	goto exit; // login fail, exit the program
			// }

			// //录取麦克风数据
			// demo_mic(session_begin_params);	
			// MSPLogout();

			resultFlag = 1;
			g_result = "Stop.";
		}
		if(c == 'w'){
			resultFlag = 1;
			g_result = "Go 50.";
		}
		if(c == 'a'){
			resultFlag = 1;
			g_result = "Left 40.";
		}
		if(c == 'd'){
			resultFlag = 1;
			g_result = "Right 40.";
		}
		if(c == 'x'){
			resultFlag = 1;
			g_result = "Back 50.";
		}
		
		// 语音识别发布话题
		if(resultFlag){
			resultFlag=0;
			Voice = g_result;
		}
		
		Voice_class.shuju_main_class(&myrobot);
		Voice_class.Voice_Dispose();
		Voice_class.PID_RUN();

        loop_rate.sleep();
	}




exit:
	MSPLogout(); // Logout...

	return 0;
}


/********************************************************
函数功能：声音内容处理函数
入口参数：无
出口参数：返回值
********************************************************/
bool SubscribeAndPublish::Voice_Dispose(void){
    

    const char *now = NULL;

    if((now=strstr(Voice,Go)) != NULL){
        Move_mode = Voice_AT::go;
        position.SetPoint = 0.01 *get_shuzi(now[3],now[4]);
        position_last = position_now;
        ROS_INFO("Dispose string is [go] ,setpoint if [%d],position_now is [%.3f]\n",Setpoint.setpoint,position_now-position_last);
    }
    else if((now=strstr(Voice,Back)) != NULL){
        Move_mode = Voice_AT::back;
       position.SetPoint = -0.01 *get_shuzi(now[5],now[6]);
        position_last = position_now;
        ROS_INFO("Dispose string is [back],setpoint is [%d]\n",Setpoint.setpoint);
    }
    else if((now=strstr(Voice,Left)) != NULL){
        Move_mode = Voice_AT::left;
        anglar.SetPoint = get_shuzi(now[5],now[6]);
        yaw_last = yaw;
        ROS_INFO("Dispose string is [left],setpoint is [%d]\n",Setpoint.setpoint);
    }
    else if((now=strstr(Voice,Right)) != NULL){
        Move_mode = Voice_AT::right;
        anglar.SetPoint = -1 * get_shuzi(now[6],now[7]);
        yaw_last = yaw;
        ROS_INFO("Dispose string is [right],setpoint is [%d]\n",Setpoint.setpoint);
    }
    else if((now=strstr(Voice,Stop)) != NULL){
        Move_mode = Voice_AT::stop;
        position.SetPoint = 0;
        anglar.SetPoint = 0;
        ROS_INFO("Dispose string is [stop]\n");
    }
    else {
        ROS_INFO("Dispose NULL is [%s]\n",now);
    }
    Voice = NULL;

}
/********************************************************
函数功能：pid
入口参数：无
出口参数：返回值
********************************************************/
void SubscribeAndPublish::PID_RUN(void){
    Voice_contol.linear.x = position_PID_Calc(&position,position_now-position_last);
    Voice_contol.angular.z = position_PID_Calc(&anglar ,yaw - yaw_last);

    
    if(Move_mode == Voice_AT::left || Move_mode == Voice_AT::right){
         ROS_INFO("PID anglaring setpoint[%.2f]  , now is [%.2f], yaw is[%.2f]\n",anglar.SetPoint,yaw - yaw_last,yaw);
    }
    if(Move_mode == Voice_AT::go || Move_mode == Voice_AT::back){
         ROS_INFO("PID linear setpoint[%.2f]  , now is [%.2f] , position is [%.2f]\n",position.SetPoint,position_now-position_last,position_now);
    }
    
    if(Move_mode == Voice_AT::stop){
         Voice_contol.linear.x=0;
         Voice_contol.angular.z=0;
    }
    
    yaw_last = yaw;
    position_last = position_now;

	//计算完毕发送速度
	cmd_pub.publish(Voice_contol);
}

int get_shuzi(char shiwei,char gewei){
    return ((int)shiwei - '0')*10 + ((int)gewei - '0');
}

	//只输出正负都会
float position_PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		if(P->PreError > -P->ErrorMax && P->PreError < P->ErrorMax)
		{
			P->SumError += P->PreError; 
		}
		
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
}