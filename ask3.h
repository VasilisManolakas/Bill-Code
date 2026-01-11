#include <stdio.h>
#include <stdlib.h>
#pragma once

typedef struct {
    char* rn;
    char * sn;
    int sem;
}student;

typedef struct node{
    int data;
    struct node *next;
    struct node *prev;
    student *s;
}node;

typedef struct list{
    node * head;
    node * tail;
    int size;
}list;

int addNode (list *, int );
void printMenu();
void initList(list *);
int insertFront(list*,  student*);
node* searchNodeForward(list*,char*);
node* searchNodeBackwards(list*,int);
int deleteNode(list*, int);
int alterNodeList(list*);
int traverseListBackwards(list*);
void clearbuf();
list * createListwithStudents(list *l, int);
