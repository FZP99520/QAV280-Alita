#ifndef __device_H
#define __device_H


typedef enum
{
    E_DevCali_Null,
    E_DevCali_Req,
    E_DevCali_Doing,
    E_DevCali_Finished,
    E_DevCali_Err
}E_DevCali_Status_TypeDef;

typedef enum
{
    E_DevInit_Status_Null,
    E_DevInit_Status_Fail,
    E_DevInit_Status_Done,
    E_DevInit_Status_Err
}E_DevInit_Status_TypeDef;

typedef enum
{
    E_DevReadWrite_Ret_Fail,
    E_DevReadWrite_Ret_Ok
}E_DevReadWrite_Ret_TypeDef;

typedef enum
{
    E_DevUpdate_Null,
    E_DevUpdate_Fail,
    E_DevUpdate_Doing,
    E_DevUpdate_Ok
}E_DevUpdate_Ret_TypeDef;

#endif

