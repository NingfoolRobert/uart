#include <iostream>
#include <pthread.h>
#include<cstdio>      /*标准输入输出定义*/
#include<cstdlib>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<cerrno>      /*错误号定义*/
#include<cstring>
#include <mutex>

#define FALSE -1
#define TRUE 1
#define SIZE 100

using namespace std;

int UartOpen();
void UartClose(int fd);
int UartInit(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
void* Uart_Send(void* arg);
void* Uart_Recv(void* arg);

int main(int argc, char **argv)
{
    int fd;
    int err;
    fd=UartOpen();

    do
    {
        err = UartInit(fd,9600,0,8,1,'N');
        printf("fd=%d\t",fd);
        printf("err=%d\n",err);
        printf("Set Port Exactly!\n");
    }while(FALSE == err || FALSE == fd);

    int ret=0;
    pthread_t id1,id2;

    ret=pthread_create(&id1,NULL,Uart_Send,&fd);
    if(ret)
    {
        printf("create pthread error!\n");
        return -1;
    }

    ret=pthread_create(&id2,NULL,Uart_Recv,&fd);
    if(ret)
    {
        printf("create pthread error!\n");
        return  -1;
    }

    pthread_join(id1,NULL);
    pthread_join(id2,NULL);

}

int UartOpen()
{
    int fd;
    fd=open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
    printf("fd=%d\n",fd);
    if(fd==-1)
    {
        perror("Can't Open SerialPort");
    }
    return fd;
}

void UartClose(int fd)
{
    close(fd);
}

int UartSet(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int   i;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，
     * 并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。
     * 若调用成功，函数返回值为0，若调用失败，函数返回值为1.
     */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {
        case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; break;
        case 2:
            options.c_cflag |= CSTOPB; break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int UartInit(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err;
    //设置串口数据帧格式
    if (UartSet(fd,9600,0,8,1,'N') == FALSE)
    {
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

void* Uart_Send(void* arg)
{
    int *fd;
    fd=(int*) arg;
    while (1)
    {
        char str[SIZE] = {};
        cout << "input your data:" << endl;
        cin >> str;
        int data_len = 0;
        data_len = sizeof(str);
        write(*fd, str, data_len);
        printf("size of send_buf is %d\n", data_len);
        if (data_len > 0)
        {
            printf("send data is %s\n", str);
        }
        else
        {
            tcflush(*fd, TCOFLUSH);
            return (void *) FALSE;
        }
    }
}

void* Uart_Recv(void* arg)
{
        int* fd;
        fd=(int*) arg;
        while(1)
        {
            char rcv_buf[SIZE]={};
            int len=read(*fd,rcv_buf, sizeof(rcv_buf));
            string str=rcv_buf;
            if(len>0)
            {
                cout<<"rcv_buff[] is:"<<str<<endl;
            }
            else
            {
                continue;
            }
        }
}