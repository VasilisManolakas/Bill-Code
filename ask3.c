#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ask3.h"

void printMenu(){
    puts("1. Add student:");
    puts("2. Delete Student:");
    puts("3. Search for student based on registry number:");
    puts("4. Search for student(s) based on student name(s)");
    puts("5. Alter student information:");
    puts("6. Create a list of students that study on specified semester:");
    puts("7. Print all students in specified semester:");
    puts("Input choice:");
    return;
}

void initList(list*l){
    l->head = NULL;
    l->size = 0;
    l->tail = NULL;
    return;
}

int insertFront (list *l, student *st){
    node *new_node = (node*) malloc(sizeof(node));
    if (new_node == NULL )
        return 1;
    new_node->next = NULL;
    new_node->prev = l->head;
    if (l->head == NULL)
        l->tail=new_node;
    else
        l->head->prev = new_node;
    // Insert struct elements
    strcpy(new_node->s->rn,st->rn);
    strcpy(new_node->s->sn,st->sn);
    new_node->s->sem = st->sem;
    //new_node->s-> = d;
    l->head = new_node;
    l->size++;
    return 0;
}

node* searchNodeForward(list *l, char * reg){
    node *traverser = l->head;
    if (traverser == NULL){
        puts("List is Empty.");
        return NULL;
    }
    while (traverser){
        if (!strcmp(traverser->s->rn, reg)){
            puts("Data found.");
            printf("%s\t %s\t %d\t ", l->head->s->rn, l->head->s->sn, l->head->s->sem);
            //Other code
            return traverser;
        }
        traverser = traverser->next;
    }
    puts("Data not found.");
    return NULL;
}

void alterNodeForward (list *l, char * reg)
{
    node * tmp = searchNodeForward(l,reg);
    if (!tmp) return;


}
node * searchNodeBackwards (list *l, int d)
{
    node *traverser = l-> tail;
    if (traverser == NULL) {
        puts("List is empty.");
        return NULL;
    }
    while (traverser){
        if (traverser->data == d){
            puts("Data found.");
            return traverser;
        }
        traverser = traverser->prev;
    }
    puts("Data not found.");
    return NULL;
}
//Done
int deleteNode(list *l, int d)
{
    node * test = searchNodeBackwards(l,d);
    if (test == NULL)
        return 1;
    if (test->prev == NULL)
        l->head = test->next;
    else
        test->prev->next = test->next;
    if (test->next == NULL)
        l->tail = test->prev;
    else
        test->next->prev = test->prev;
    free(test);
    return 0;
}

int AlterNodeList(list* l, student *st){
    node* traverser = (node*)malloc(sizeof(node));
    if (traverser == NULL)
        return 1;
    traverser = l->head;
    while (traverser != NULL){
        traverser = traverser->next;
    }
    return 0;
}

int traverseListBackwards(list *l){
    node * traverser = (node*)malloc(sizeof(node));
    if (traverser == NULL)
        return 1;
    traverser = l->tail;
    while (traverser){
        traverser = traverser->prev;
    }
    return 0;
}

void clearbuf(void){
    char ch;
    while ((ch = getchar()) != '\n' && ch != EOF);
    return;
}

list *createListwithStudents(list *l, int seme){
    node * traverser = (node*)malloc(sizeof(int));
    if (traverser == NULL)
        return NULL;
    traverser = l->head;
    list *ls;
    initList(ls);
    while (traverser)
    {
        if (traverser->s->sem == seme)
            insertFront(ls,traverser->s);
        traverser = traverser->next;
    }
return ls;
}
