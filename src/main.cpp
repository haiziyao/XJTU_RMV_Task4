#include "CameraParams.h"
#include "MvCameraControl.h"
#include <cstring>
#include <iostream>
//参考资料https://blog.csdn.net/qq_45445740/article/details/124420388
/*
下面主要实现的内容有:查找设备->创建句柄->开启设备(参数获取，参数设置)->开启取流(创建线程，图像获取)->停止取流(销毁线程)->关闭设备->销毁句柄
*/
int main(){
    unsigned int nTLayerType = MV_GIGE_DEVICE | MV_USB_DEVICE ;
    MV_CC_DEVICE_INFO_LIST m_stDevList ;
    int nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    if(MV_OK != nRet){
        std::cout<<"err : EnumDevices fail;设备查找出现错误，错误码"<<nRet<<std::endl;
    }
    if (m_stDevList.nDeviceNum == 0){
        std::cout<<"no camera found!"<<std::endl;
        return -1;
    }
    int nDeviceIndex = 0;
    void*  m_handle = NULL;
    MV_CC_DEVICE_INFO m_stDevInfo ;
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
    nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
    if (MV_OK != nRet){
        printf("error: CreateHandle fail [%x]\n", nRet);
        return -1;
    }

    nRet = MV_CC_OpenDevice(m_handle);
    if (MV_OK != nRet){
        printf("error: OpenDevice fail [%x]\n", nRet);
        return -1;
    }
    nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet){
        printf("error: StartGrabbing fail [%x]\n", nRet);
        return -1;
    }

    MVCC_INTVALUE stIntvalue ;
    nRet = MV_CC_GetIntValue(m_handle, "PayloadSize", &stIntvalue);
    if (MV_OK != nRet){
        printf("error: GetIntValue fail [%x]\n", nRet);
        return -1;
    }
    int nBufSize = stIntvalue.nCurValue;
    unsigned int    nTestFrameSize = 0; 
    unsigned char*  pFrameBuf = NULL;    
    pFrameBuf = (unsigned char*)malloc(nBufSize);  
    MV_FRAME_OUT_INFO_EX stInfo;
    memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
     while(1)
    {
        
        nRet = MV_CC_GetOneFrameTimeout(m_handle, pFrameBuf, nBufSize, &stInfo, 1000);
        if (MV_OK != nRet)
        {
            
        }
        else
        {
            //...图像数据处理
            nTestFrameSize++;
        }
    }

    //...其他处理
    
    nRet = MV_CC_StopGrabbing(m_handle);
    if (MV_OK != nRet){
        printf("error: StopGrabbing fail [%x]\n", nRet);
        return -1;
    }
    nRet = MV_CC_CloseDevice(m_handle);
    if (MV_OK != nRet){
        printf("error: CloseDevice fail [%x]\n", nRet);
        return -1;
    }
    nRet = MV_CC_DestroyHandle(m_handle);
    if (MV_OK != nRet){
        printf("error: DestroyHandle fail [%x]\n", nRet);
        return -1;
    }
}