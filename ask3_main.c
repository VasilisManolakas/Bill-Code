#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <assert.h>
#include <string.h>
#include "ask3.h"
int main (int argc, char ** argv){

    list *l = (list*)malloc(sizeof(list));
    assert (l!=NULL);
    initList(l);
    printMenu();
    int choice;
    do {
        fscanf(stdin,"%d",&choice);
        if (!(choice>=1 && choice <=7)){
            puts ("Wrong Input. Try Again.");
            clearbuf();
        }
    }while (choice<1 || choice >7);


    switch (choice){
        case 1:

        case 2:

        case 3:

        case 4:

        case 5:

        case 6:

        case 7:

        default:

    }
    return (0);
}
