#include "mbed.h"
#include "xPinMap.h"

#ifdef FEATURE_USBSERIAL

/* Copyright (c) 2017 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* TARGET NOT STM does not support this HAL */
#ifndef TARGET_STM
#define USBSTM_HAL_UNSUPPORTED
#endif

/* STM32F4 family without USB_STM_HAL use another HAL */
#if defined(TARGET_STM) && defined(TARGET_STM32F4) && !defined(USB_STM_HAL)
#define USBSTM_HAL_UNSUPPORTED
#endif

#ifndef USBSTM_HAL_UNSUPPORTED
#include "xUSBHAL.h"
#include "pinmap.h"

#include "xUSBHAL_STM32.h"

/* mbed endpoint definition to hal definition */
#define EP_ADDR(ep) (((ep) >> 1)|((ep) & 1) << 7)

/* from hal definition to mbed definition */
#define ADDR_EPIN(ep) (((ep) << 1) | 1)
#define ADDR_EPOUT(ep) (((ep) << 1))

/*  this call at device reception completion on a Out Enpoint  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    uint8_t endpoint = ADDR_EPOUT(epnum);
    priv->epComplete[endpoint] = 1;
    /* -2 endpoint 0 In out are not in call back list */
    if (epnum) {
        bool (xUSBHAL::*func)(void) = priv->epCallback[endpoint-2];
        (obj->*func)();
    } else {
        void (xUSBHAL::*func)(void) = priv->ep0_out;
        (obj->*func)();
    }
}

/*  this is call at device transmission completion on In endpoint */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    uint8_t endpoint = ADDR_EPIN(epnum);
    priv->epComplete[endpoint] = 1;
    /*  -2 endpoint 0 In out are not in call back list */
    if (epnum) {
        bool (xUSBHAL::*func)(void) = priv->epCallback[endpoint-2];
        (obj->*func)();
    } else {
        void (xUSBHAL::*func)(void) = priv->ep0_in;
        (obj->*func)();
    }
}
/*  This is call at device set up reception  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    void (xUSBHAL::*func)(void)=priv->ep0_setup;
    void (xUSBHAL::*func1)(void)=priv->ep0_read;
    (obj->*func)();
    (obj->*func1)();
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    void (xUSBHAL::*func)(unsigned int suspended) = priv->suspend_change;
    (obj->*func)(1);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    void (xUSBHAL::*func)(unsigned int suspended) = priv->suspend_change;
    (obj->*func)(0);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    void (xUSBHAL::*func)(unsigned int suspended) = priv->connect_change;
    (obj->*func)(1);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    void (xUSBHAL::*func)(unsigned int suspended) = priv->connect_change;
    (obj->*func)(0);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    xUSBHAL_Private_t *priv=((xUSBHAL_Private_t *)(hpcd->pData));
    xUSBHAL *obj= priv->inst;
    unsigned int i;
    for(i=0;i<hpcd->Init.dev_endpoints;i++) {
        priv->epComplete[2*i]=0;
        HAL_PCD_EP_Close(hpcd,EP_ADDR(2*i));
        HAL_PCD_EP_Flush(hpcd,EP_ADDR(2*i));
        priv->epComplete[2*i+1]=0;
        HAL_PCD_EP_Close(hpcd,EP_ADDR(2*i+1));
        HAL_PCD_EP_Flush(hpcd,EP_ADDR(2*i+1));

    }
    void (xUSBHAL::*func)(void)=priv->bus_reset;
    bool (xUSBHAL::*ep_realise)(uint8_t endpoint, uint32_t maxPacket, uint32_t flags) = priv->ep_realise;
    (obj->*func)();
    (obj->*ep_realise)(EP0IN, MAX_PACKET_SIZE_EP0,0);
    (obj->*ep_realise)(EP0OUT, MAX_PACKET_SIZE_EP0,0);
}


/* hal pcd handler , used for STM32 HAL PCD Layer */

uint32_t xUSBHAL::endpointReadcore(uint8_t endpoint, uint8_t *buffer) {
    return 0;
}

xUSBHAL::~xUSBHAL(void) {
	xUSBHAL_Private_t *HALPriv = (xUSBHAL_Private_t *)(hpcd.pData);
	HAL_PCD_DeInit(&hpcd);
	delete HALPriv;
}

void xUSBHAL::connect(void) {
    NVIC_EnableIRQ(USBHAL_IRQn);
}

void xUSBHAL::disconnect(void) {
    NVIC_DisableIRQ(USBHAL_IRQn);
}

void xUSBHAL::configureDevice(void) {
    // Not needed
}

void xUSBHAL::unconfigureDevice(void) {
    // Not needed
}

void xUSBHAL::setAddress(uint8_t address) {
	HAL_PCD_SetAddress(&hpcd, address);
    EP0write(0, 0);
}

bool xUSBHAL::realiseEndpoint(uint8_t endpoint, uint32_t maxPacket, uint32_t flags) {
    uint32_t epIndex = EP_ADDR(endpoint);
    uint32_t type = 0;
    uint32_t len;
    HAL_StatusTypeDef ret;
    switch (endpoint) {
    case EP0IN:
    case EP0OUT:
        type =  0;
        break;
    case EPISO_IN:
    case EPISO_OUT:
        type = 1;
        break;
    case EPBULK_IN:
    case EPBULK_OUT:
        type =  2;
        break;
    case EPINT_IN:
    case EPINT_OUT:
        type =  3;
        break;
    }
    if (maxPacket > MAXTRANSFER_SIZE) return false;
    if (epIndex & 0x80) {
        len = HAL_PCDEx_GetTxFiFo(&hpcd,epIndex & 0x7f);
        MBED_ASSERT(len >= maxPacket);
        UNUSED(len);
    }
    ret = HAL_PCD_EP_Open(&hpcd, epIndex,  maxPacket, type);
    MBED_ASSERT(ret!=HAL_BUSY);
    return (ret == HAL_OK) ? true:false;
}

// read setup packet
void xUSBHAL::EP0setup(uint8_t *buffer) {
    memcpy(buffer,  hpcd.Setup, MAX_PACKET_SIZE_SETUP);
	memset(hpcd.Setup,0,MAX_PACKET_SIZE_SETUP);
}

void xUSBHAL::EP0readStage(void) {
}

void xUSBHAL::EP0read(void) {
     xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)hpcd.pData;
	 uint32_t epIndex = EP_ADDR(EP0OUT);
	 uint8_t *pBuf = (uint8_t *)HALPriv->pBufRx0;
	 HAL_StatusTypeDef ret;
	 HALPriv->epComplete[EP0OUT] = 2;
	 ret = HAL_PCD_EP_Receive(&hpcd, epIndex, pBuf, MAX_PACKET_SIZE_EP0 );
	 MBED_ASSERT(ret!=HAL_BUSY);
	UNUSED(ret);
}

uint32_t xUSBHAL::EP0getReadResult(uint8_t *buffer) {
    xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)hpcd.pData;
    uint32_t length = (uint32_t) HAL_PCD_EP_GetRxCount(&hpcd, 0);
	HALPriv->epComplete[EP0OUT] = 0;
    if (length) {
        uint8_t  *buff = (uint8_t *)HALPriv->pBufRx0;
        memcpy(buffer, buff, length);
    }
    return length;
}

void xUSBHAL::EP0write(uint8_t *buffer, uint32_t size) {
    /*  check that endpoint maximum size is not exceeding TX fifo */
    MBED_ASSERT(hpcd.IN_ep[0].maxpacket >= size);
    endpointWrite(EP0IN, buffer, size);
}

void xUSBHAL::EP0getWriteResult(void) {

}

void xUSBHAL::EP0stall(void) {
    stallEndpoint(EP0IN);
}

EP_STATUS xUSBHAL::endpointRead(uint8_t endpoint, uint32_t maximumSize) {
    xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)(hpcd.pData);
    uint32_t epIndex = EP_ADDR(endpoint);
    uint8_t* pBuf = (uint8_t *)HALPriv->pBufRx;
	HAL_StatusTypeDef ret;
    // clean reception end flag before requesting reception
    HALPriv->epComplete[endpoint] = 2;
    ret = HAL_PCD_EP_Receive(&hpcd, epIndex, pBuf, maximumSize);
	MBED_ASSERT(ret!=HAL_BUSY);
	UNUSED(ret);
    return EP_PENDING;
}

EP_STATUS xUSBHAL::endpointReadResult(uint8_t endpoint, uint8_t * buffer, uint32_t *bytesRead) {
    xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)(hpcd.pData);
	if (HALPriv->epComplete[endpoint]==0) {
		/*  no reception possible !!! */
		bytesRead = 0;
        return EP_COMPLETED;
    }else if ((HALPriv->epComplete[endpoint]!=1))
	return EP_PENDING;
    uint32_t epIndex = EP_ADDR(endpoint);
    uint8_t  *buff = (uint8_t *)HALPriv->pBufRx;
    uint32_t length = (uint32_t) HAL_PCD_EP_GetRxCount(&hpcd, epIndex);
    memcpy(buffer, buff, length);
    *bytesRead = length;
	HALPriv->epComplete[endpoint]= 0;
    return EP_COMPLETED;
}

EP_STATUS xUSBHAL::endpointWrite(uint8_t endpoint, uint8_t *data, uint32_t size) {
	xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)(hpcd.pData);
    uint32_t epIndex = EP_ADDR(endpoint);
    HAL_StatusTypeDef ret;
    // clean transmission end flag before requesting transmission
    HALPriv->epComplete[endpoint] = 2;
    ret = HAL_PCD_EP_Transmit(&hpcd, epIndex, data, size);
	MBED_ASSERT(ret!=HAL_BUSY);
    // update the status
    if (ret != HAL_OK) return EP_INVALID; 
    // fix me return is too simple
    return EP_PENDING;
}

EP_STATUS xUSBHAL::endpointWriteResult(uint8_t endpoint) {
	xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)(hpcd.pData);
    if (HALPriv->epComplete[endpoint] == 1)
        return EP_COMPLETED;
    return EP_PENDING;
}

void xUSBHAL::stallEndpoint(uint8_t endpoint) {
    xUSBHAL_Private_t *HALPriv =  (xUSBHAL_Private_t *)(hpcd.pData);
	HAL_StatusTypeDef ret;
	HALPriv->epComplete[endpoint] = 0;
	ret = HAL_PCD_EP_SetStall(&hpcd, EP_ADDR(endpoint));
	MBED_ASSERT(ret!=HAL_BUSY);
	UNUSED(ret);
}

void xUSBHAL::unstallEndpoint(uint8_t endpoint) {
	HAL_StatusTypeDef ret;
    ret = HAL_PCD_EP_ClrStall(&hpcd, EP_ADDR(endpoint));
	MBED_ASSERT(ret!=HAL_BUSY);
	UNUSED(ret);
}

bool xUSBHAL::getEndpointStallState(uint8_t endpoint) {
    return false;
}

void xUSBHAL::remoteWakeup(void) {
}

void xUSBHAL::_usbisr(void) {
    instance->usbisr();
}

void xUSBHAL::usbisr(void) {
    HAL_PCD_IRQHandler(&instance->hpcd);
}

#endif

#endif // FEATURE_USBSERIAL
