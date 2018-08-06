#include <stdio.h>
#include <math.h>

#define PI 3.141592653589   //圆周率，12位小数   
#define N 8                 //傅里叶变换的点数   
#define M 3                 //蝶形运算的级数，N = 2^M   
typedef double ElemType;    //原始数据序列的数据类型,可以在这里设置  
  
typedef struct              //定义复数结构体   
{  
    ElemType real,imag;  
}complex;  
  
complex data[N];            //定义存储单元，原始数据与负数结果均使用之   
ElemType result[N];         //存储FFT后复数结果的模 

void ChangeSeat(complex *DataInput);
complex XX_complex(complex a, complex b);
void FFT(void);
void IFFT(void);
int FindInsertIndex(int *pDataArray, int iLen, int iData);
void BinaryInsertSort(int* pDataArray, int iDataNum);

