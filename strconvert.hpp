#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
//using namespace std;
void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen)
{
	int i = 0;
	for(i = 0; i < ucLen; i++)
	{
		//格式化输str,每unsigned char 转换字符占两位置%x写输%X写输
		sprintf(str + i * 2, "%02x", UnChar[i]);
	}
}
void convertStrToUnChar(char* str, unsigned char* UnChar)
{
	int i = strlen(str), j = 0, counter = 0;
	char c[2];
	unsigned int bytes[2];
 
	for (j = 0; j < i; j += 2) 
	{
		if(0 == j % 2)
		{
			c[0] = str[j];
			c[1] = str[j + 1];
			sscanf(c, "%02x" , &bytes[0]);
			UnChar[counter] = bytes[0];
//            printf("%02x ", UnChar[counter]);
			counter++;
		}
	}
	return;
}

//int main()
//{
//    string str="what123";
//    char ptr[10];
//    str.copy(ptr,5,0);
////    cout << ptr <<endl;
//    puts(ptr);

    
//    string y = "fuck999";
//    char ptr2[10];
//    y.copy(ptr2,5,0);
//    puts(ptr2);
    
//    string a = "wwww";
//    char *ptr3;
//    strcpy(ptr3,a.c_str());
//    puts(ptr3);
    
//    string x = "hello";
//    char *ptr1 ;
//    strcpy(ptr1, x.data());
//    puts(ptr1);
    
//    unsigned char src[6] = {0x12, 0x32,0x56,0x78,0x90,0xab};
//    char buffer[20];//维数定义些
//    convertUnCharToStr(buffer, src, 6);  
//    printf("%s\n", buffer);
    
//    unsigned char dst[6];
//    int len = strlen(buffer);
//    cout << len << endl;
//    convertStrToUnChar(buffer, dst);
//    int i = 0;
//    for(i = 0; i < 6; i++)
//    {
//        printf("%x ", dst[i]);
//    }
//    cout << endl;
    
//}
