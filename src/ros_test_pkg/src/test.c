#include <stdio.h>


int main()
{
    char * str = "abc";
    char x;
    x = '0'+2;

    str[0]=x;
    puts(str);
    return 0;
}