/* 
  
  testbanner - test linux banner output
  
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#define UTS_RELEASE        "4.1.0-rc41"
#define THS_RELEASE        "2015-05-19"

const char *cbanner_data[]  = {


                               //12345678901234567890123456789012345678901234567890123456789012345678901234567890
                                "88888 8   8 888888                              e        ea     eeea    \n"
                                "  8   8   8 8        e   e  eeee eeeee  eeeee   8 e     ee8     8  8    \n"
                                "  8e  8eee8 8eeeee   8   8  8    8   8  8   8   8888      8     8  8    \n"
                                "  88  88  8     88   8eee8e 8eee 8eee8e 8e  8     8       8     8 e8    \n"
                                "  88  88  8 e   88   88   8 88   88   8 88  8   8888    8888    8888    \n"
                                "  88  88  8 8eee88   88   8 88ee 88   8 88  8   8888 88 8888 88 8888 rc4\n",

                                "ths kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n",

                                "e   e                                 e        ea     eeea o8o  db  o8o \n"
                                "8   8   eeee eeeee  eeeee eeee e      8 e     ee8     8  8 o8888db8888o \n"
                                "8eee8e  8    8   8  8   8 8    8      8888      8     8  8     odbo     \n"
                                "88   8  8eee 8eee8e 8e  8 8eee 8e       8       8     8 e8 o88o db o88o \n"
                                "88   8  88   88   8 88  8 88   88     8888    8888    8888 oo   db   oo \n"
                                "88   8  88ee 88   8 88  8 88ee 88eee  8888 88 8888 88 8888 rc4  db      \n",

                                "ths kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n",

                                "/-----------------------------------------\\\n"
                                "|                  _/\\_                   |\n"
                                "|  moc            <o88o>             con  |\n"
                                "| oooooon          <88>        .moooooooo |\n"
                                "| oocoooooooon.    <88>    .moooooooooooo |\n"
                                "|  ooooooooooooon._d88b_.moooooooooooooo  |\n"
                                "|   (oooooooooooooo8888ooooooooooooooo)   |\n"
                                "|     (ooooooooooooo88ooooooooooooo)      |\n"
                                "|        oooooooooo8888oooooooooo         |\n"
                                "|               -ooo88ooo-                |\n"
                                "|          .mooooooY88Yoooooon.           |\n"
                                "|     __cooooooooo/ 88 \\oooooooooa,,      |\n"
                                "|   _ooooooooooooo: oo :8oooooooooooa,    |\n"
                                "|  /ooooooooooooo/  oo  \\oooooooooooo8\\   |\n"
                                "|  oooooooooooo     oo     oooooooooooo   |\n"
                                "|  \\oooooooo/       oo       \\oooooooo/   |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "|                   oo                    |\n"
                                "\\-----------------------------------------/\n",

                                "odonata kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n" 

                             };


const char *cbanner_1      = "\n"
                               //12345678901234567890123456789012345678901234567890123456789012345678901234567890
                                "88888 8   8 888888                              e        ea     eeea    \n"
                                "  8   8   8 8        e   e  eeee eeeee  eeeee   8 e     ee8     8  8    \n"
                                "  8e  8eee8 8eeeee   8   8  8    8   8  8   8   8888      8     8  8    \n"
                                "  88  88  8     88   8eee8e 8eee 8eee8e 8e  8     8       8     8 e8    \n"
                                "  88  88  8 e   88   88   8 88   88   8 88  8   8888    8888    8888    \n"
                                "  88  88  8 8eee88   88   8 88ee 88   8 88  8   8888 88 8888 88 8888 rc4\n"
                                "ths kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n",

const char *cbanner_2      = "\n"

                                "e   e                                 e        ea     eeea o8o  db  o8o \n"
                                "8   8   eeee eeeee  eeeee eeee e      8 e     ee8     8  8 o8888db8888o \n"
                                "8eee8e  8    8   8  8   8 8    8      8888      8     8  8     odbo     \n"
                                "88   8  8eee 8eee8e 8e  8 8eee 8e       8       8     8 e8 o88o db o88o \n"
                                "88   8  88   88   8 88  8 88   88     8888    8888    8888 oo   db   oo \n"
                                "88   8  88ee 88   8 88  8 88ee 88eee  8888 88 8888 88 8888 rc4  db      \n"
                                "ths kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n";


const char *cbanner_3      = "\n"
                               //12345678901234567890123456789012345678901234567890123456789012345678901234567890

                                "/------------------------------------------\\\n"
                                "|                   _/\\_                   |\n"
                                "|   moc            <o88o>             con  |\n"
                                "|  oooooon          <88>        .moooooooo |\n"
                                "|  oocoooooooon.    <88>    .moooooooooooo |\n"
                                "|   ooooooooooooon._d88b_.moooooooooooooo  |\n"
                                "|    (oooooooooooooo8888ooooooooooooooo)   |\n"
                                "|      (ooooooooooooo88ooooooooooooo)      |\n"
                                "|         oooooooooo8888oooooooooo         |\n"
                                "|                -ooo88ooo-                |\n"
                                "|           .mooooooY88Yoooooon.           |\n"
                                "|      __cooooooooo/ 88 \\oooooooooa,,      |\n"
                                "|    _ooooooooooooo: oo :8oooooooooooa,    |\n"
                                "|   /ooooooooooooo/  oo  \\oooooooooooo8\\   |\n"
                                "|   oooooooooooo     oo     oooooooooooo   |\n"
                                "|   \\oooooooo/       oo       \\oooooooo/   |\n"
                                "|                    oo                    |\n"
                                "|                    oo                    |\n"
                                "|                    oo                    |\n"
                                "|                    oo                    |\n"
                                "|                    oo                    |\n"
                                "\\------------------------------------------/\n"
                                "odonata kernel release " THS_RELEASE " / linux-" UTS_RELEASE "\n\n" ;




void showversion( void )
{
  printf("\n testbanner version 1.0.0 - test linux banner output\n");
  printf(" copyright (c) 2004 by <THS <tositrino@hanskult.de>\n\n");
}


int main(int argc , char **argv)
{
  
  showversion();
  
  printf("*****************\n");
  printf("*** banner #1 ***\n");
  printf("*****************\n");
  printf("%s",cbanner_1);

  printf("*****************\n");
  printf("*** banner #2 ***\n");
  printf("*****************\n");
  printf("%s",cbanner_2);

  printf("*****************\n");
  printf("*** banner #3 ***\n");
  printf("*****************\n");
  printf("%s",cbanner_3);

  exit(0);    
  
}
