#include<iostream>
using namespace std;
int main()
{
    FILE* fp;
    fp = fopen("first1000Black","r");
    int num1;
    char op;
    int addr;
    int num2;
    // while(fscanf(fp,"%d %c %x",&num1, &op, &addr)!=0)
    int startAddr;
    for(int i=0;i<1000;i++)
    {
        fscanf(fp,"%d %c %x",&num1, &op, &addr);
        if(i==0) startAddr=addr;
        if(op=='R')
        {
            fscanf(fp,"%x",&num2);
            printf("%d %c 0x%x %x\n",num1,op,startAddr+i*128,num2);
        }
        else
        {
            printf("%d %c 0x%x\n",num1,op,startAddr+i*128);
        }
    }
}