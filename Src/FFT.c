#include "FFT.h"

extern int globalCounter;

complex ChangeSeat(complex *DataInput)  
{  
    int nextValue,nextM,i,k,j=0;  
    complex temp;  
      
    nextValue=N/2;                  //变址运算，即把自然顺序变成倒位序，采用雷德算法  
    nextM=N-1;  
    for (i=0;i<nextM;i++)  
    {  
        if (i<j)                 //如果i<j,即进行变址  
        {  
            temp=DataInput[j];  
            DataInput[j]=DataInput[i];  
            DataInput[i]=temp;  
        }  
        k=nextValue;                //求j的下一个倒位序  
        while (k<=j)             //如果k<=j,表示j的最高位为1  
        {  
            j=j-k;                  //把最高位变成0  
            k=k/2;                  //k/2，比较次高位，依次类推，逐个比较，直到某个位为0  
        }  
        j=j+k;                      //把0改为1  
    }
    return temp;
}  

//复数乘法   
complex XX_complex(complex a, complex b)  
{  
    complex temp;  
      
    temp.real = a.real * b.real-a.imag*b.imag;  
    temp.imag = b.imag*a.real + a.imag*b.real;  
      
    return temp;  
}  
  
//FFT  
void FFT(complex data_in)  
{ 
    globalCounter = 0x0E;
    int L=0,B=0,J=0,K=0;  
    int step=0;  
    ElemType P=0,T=0;  
    complex W,Temp_XX;  
    //ElemType TempResult[N];  
      
    //ChangeSeat(data);  
    for(L=1; L<=M; L++)  
    {  
        B = 1<<(L-1);//B=2^(L-1)  
        for(J=0; J<=B-1; J++)  
        {  
            P = (1<<(M-L))*J;//P=2^(M-L) *J   
            step = 1<<L;//2^L  
            for(K=J; K<=N-1; K=K+step)  
            {  
                W.real =  cos(2*PI*P/N);  
                W.imag = -sin(2*PI*P/N);  
                  
                Temp_XX = XX_complex(data[K+B],W);  
                data[K+B].real = data[K].real - Temp_XX.real;  
                data[K+B].imag = data[K].imag - Temp_XX.imag;  
                  
                data[K].real = data[K].real + Temp_XX.real;  
                data[K].imag = data[K].imag + Temp_XX.imag;  
            }  
        }  
    }  
}  
void IFFT(complex data_in)  
{ 
    globalCounter = 0x0F;
    int L=0,B=0,J=0,K=0;  
    int step=0;  
    ElemType P=0,T=0;  
    complex W,Temp_XX;  
    //ElemType TempResult[N];  
      
    //ChangeSeat(data);  
    for(L=1; L<=M; L++)  
    {  
        B = 1<<(L-1);//B=2^(L-1)  
        for(J=0; J<=B-1; J++)  
        {  
            P = (1<<(M-L))*J;//P=2^(M-L) *J   
            step = 1<<L;//2^L  
            for(K=J; K<=N-1; K=K+step)  
            {  
                W.real =  cos(2*PI*P/N);  
                W.imag =  sin(2*PI*P/N);//逆运算，这里跟FFT符号相反   
                  
                Temp_XX = XX_complex(data[K+B],W);  
                data[K+B].real = data[K].real - Temp_XX.real;  
                data[K+B].imag = data[K].imag - Temp_XX.imag;  
                  
                data[K].real = data[K].real + Temp_XX.real;  
                data[K].imag = data[K].imag + Temp_XX.imag;  
            }  
        }  
    }  
}

int FindInsertIndex(int *pDataArray, int iLen, int iData)
{
	int iBegin = 0;
	int iEnd = iLen - 1;
	int index = -1;    //记录插入位置
	while (iBegin <= iEnd)
	{
		index = (iBegin + iEnd) / 2;
		if (pDataArray[index] > iData)
			iEnd = index - 1;
		else
			iBegin = index + 1; 
	}
	if (pDataArray[index] <= iData)
		index++;
	return index;
}

void BinaryInsertSort(int* pDataArray, int iDataNum)
{
	for (int i = 1; i < iDataNum; i++)    //从第2个数据开始插入
	{
		int index = FindInsertIndex(pDataArray, i, pDataArray[i]);    //二分寻找插入的位置
		
		if (i != index)    //插入位置不为i，才挪动、插入
		{
			int j = i;
			int temp = pDataArray[i];
			while (j > index)    //挪动位置
			{
				pDataArray[j] = pDataArray[j-1];
				j--;
			}
			pDataArray[j] = temp;    //插入
		}
	}
}

