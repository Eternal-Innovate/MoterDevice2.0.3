#ifndef MTXCLASS
#define MTXCLASS

typedef enum {
    DOUBLE = 0,
    SINGLE = 1,
    INT32 = 2,
} DataType;

typedef struct{
    void *data;          
    int *dims;           
    int ndims;           
    DataType type;       
    int is_complex;   
		
} mxArray;

void disp(mxArray *arr);

#endif