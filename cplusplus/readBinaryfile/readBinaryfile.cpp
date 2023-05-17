// #include "stdafx.h"
#include <fstream>
#include <iostream>
#include<string.h>
using namespace std;

#define MV_LOG_DEBUG    printf(__VA_ARGS__)


typedef struct{

float x;
float y;
}tPointSimple;

typedef struct {
    int validFlag;
    tPointSimple p0;
    tPointSimple p1;
    tPointSimple p2;
    tPointSimple p3;
 }tSlotSpaceInfo;


#define saveFileName  "./test.txt"


int addInFile()
{
    ofstream outFile(saveFileName,ios::out|ios::binary);  //定义文件输出流   文件不存在时创建文件
    //对文件打开错误时的操作
    if(!outFile)
        {
        cout<<"The file open error!"<<endl;
        return 0;
        }
    else        //文件正常打开时，进行相应的处理
        {
        tSlotSpaceInfo *s = new tSlotSpaceInfo;
        cout<<"输入可用性";
        cin>>s->validFlag;
        cout<<"输入p0.x";
        cin>>s->p0.x;
        cout<<"输入p0.y";
        cin>>s->p0.y;

        cout<<"输入p1.x";
        cin>>s->p1.x;
        cout<<"输入p1.y";
        cin>>s->p1.y;

        cout<<"输入p2.x";
        cin>>s->p2.x;
        cout<<"输入p2.y";
        cin>>s->p2.y;

        cout<<"输入p3.x";
        cin>>s->p3.x;
        cout<<"输入p3.y";
        cin>>s->p3.y;

        outFile.write((char*)s,sizeof(tSlotSpaceInfo));   //文件输出流向文件中写入student信息
        }
    outFile.close();   //关闭输出流
    return 1;
}
int myReadFile(char * fileName)
{
    ifstream inFile(fileName,ios::in|ios::binary);   //文件输入流  将文件中的student信息读出到屏幕上
    //对文件打开错误时的操作
    if(!inFile)
    {
        cout<<"The inFile open error!"<<endl;
        return 0;
    }
    else
    {
    
        tSlotSpaceInfo *s = new tSlotSpaceInfo;
        inFile.read((char*)s,sizeof(tSlotSpaceInfo));
        cout<<"validFlag:"<<s->validFlag<<endl;
        cout<<"p0.x:"<<s->p0.x<<",po.y:"<<s->p0.y<<endl;
        cout<<"p1.x:"<<s->p1.x<<",p1.y:"<<s->p1.y<<endl;
        cout<<"p2.x:"<<s->p2.x<<",p2.y:"<<s->p2.y<<endl;
        cout<<"p3.x:"<<s->p3.x<<",p3.y:"<<s->p3.y<<endl;

    }
    inFile.close();       //关闭输入流
 }


int main(int argc,char **argv)
{
    cout<<"The main .............."<<endl;
    MV_LOG_DEBUG("123123123123\n");

    char fileName[100] = {0};
    if(argc<2)
    {
        cout<<"no input file"<<endl;
    }
    else
    {
        memcpy(fileName,argv[1],100);
        printf("fileName = %s\n",fileName);    
    }

    // addInFile();  //添加结构体
    myReadFile(fileName);  //读取结构体

    return 0;
}