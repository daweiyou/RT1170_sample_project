/*
 * Copyright 2019-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "littlevgl_support.h"
#include "lvgl.h"
#if defined(SDK_OS_FREE_RTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif
#include "board.h"

#include "fsl_gpio.h"
#include "fsl_cache.h"
#include "fsl_debug_console.h"

#include "fsl_gt911.h"
#include "fsl_pxp.h"
#if LV_USE_GPU && LV_USE_GPU_NXP_PXP
#include "src/lv_gpu/lv_gpu_nxp_pxp.h"
#include "src/lv_gpu/lv_gpu_nxp_pxp_osa.h"
#endif

#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
#include "vg_lite.h"
#include "vg_lite_platform.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Cache line size. */
#ifndef FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#define FSL_FEATURE_L2CACHE_LINESIZE_BYTE 0
#endif
#ifndef FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#define FSL_FEATURE_L1DCACHE_LINESIZE_BYTE 0
#endif

#if (FSL_FEATURE_L2CACHE_LINESIZE_BYTE > FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#else
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#endif

#if (DEMO_CACHE_LINE_SIZE > FRAME_BUFFER_ALIGN)
#define DEMO_FB_ALIGN DEMO_CACHE_LINE_SIZE
#else
#define DEMO_FB_ALIGN FRAME_BUFFER_ALIGN
#endif

#if (LV_ATTRIBUTE_MEM_ALIGN_SIZE > DEMO_FB_ALIGN)
#undef DEMO_FB_ALIGN
#define DEMO_FB_ALIGN LV_ATTRIBUTE_MEM_ALIGN_SIZE
#endif

#define DEMO_FB_SIZE \
    (((DEMO_BUFFER_WIDTH * DEMO_BUFFER_HEIGHT * LCD_FB_BYTE_PER_PIXEL) + DEMO_FB_ALIGN - 1) & ~(DEMO_FB_ALIGN - 1))

#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
#define VG_LITE_MAX_CONTIGUOUS_SIZE 0x200000
#define VG_LITE_COMMAND_BUFFER_SIZE (256 << 10)
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DEMO_FlushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

#if LV_USE_GPU
static void DEMO_CleanInvalidateCache(lv_disp_drv_t *disp_drv);
#endif

static void DEMO_InitTouch(void);

//static bool DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data);
static void DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data);

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer);

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp);

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode);

#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
static status_t BOARD_PrepareVGLiteController(void);

static status_t BOARD_InitVGliteClock(void);
#endif /* LV_USE_GPU_NXP_VG_LITE */

/*******************************************************************************
 * Variables
 ******************************************************************************/
SDK_ALIGN(static uint8_t s_frameBuffer[2][DEMO_FB_SIZE], DEMO_FB_ALIGN);

//dawei: s_final_frameBuffer will be output to LCD
SDK_ALIGN(static uint8_t s_final_frameBuffer[2][DEMO_FB_SIZE], DEMO_FB_ALIGN);


#if defined(SDK_OS_FREE_RTOS)
static SemaphoreHandle_t s_transferDone;
#else
static volatile bool s_transferDone;
#endif

static gt911_handle_t s_touchHandle;
static const gt911_config_t s_touchConfig = {
    .I2C_SendFunc     = BOARD_MIPIPanelTouch_I2C_Send,
    .I2C_ReceiveFunc  = BOARD_MIPIPanelTouch_I2C_Receive,
    .pullResetPinFunc = BOARD_PullMIPIPanelTouchResetPin,
    .intPinFunc       = BOARD_ConfigMIPIPanelTouchIntPin,
    .timeDelayMsFunc  = VIDEO_DelayMs,
    .touchPointNum    = 1,
    .i2cAddrMode      = kGT911_I2cAddrMode0,
    .intTrigMode      = kGT911_IntRisingEdge,
};
static int s_touchResolutionX;
static int s_touchResolutionY;

#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
static uint32_t registerMemBase = 0x41800000;
static uint32_t gpu_mem_base    = 0x0;

/*
 * In case custom VGLite memory parameters are used, the application needs to
 * allocate and publish the VGLite heap base, its size and the size of the
 * command buffer(s) using the following global variables:
 */
extern void *vglite_heap_base;
extern uint32_t vglite_heap_size;
extern uint32_t vglite_cmd_buff_size;

#if (CUSTOM_VGLITE_MEMORY_CONFIG == 0)
/* VGLite driver heap */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t vglite_contiguous_mem[VG_LITE_MAX_CONTIGUOUS_SIZE], 64);

void *vglite_heap_base        = &vglite_contiguous_mem;
uint32_t vglite_heap_size     = VG_LITE_MAX_CONTIGUOUS_SIZE;
uint32_t vglite_cmd_buff_size = VG_LITE_COMMAND_BUFFER_SIZE;
#endif /* CUSTOM_VGLITE_MEMORY_CONFIG */

#endif /* LV_USE_GPU_NXP_VG_LITE */

static uint8_t dispbuffidx = 1;

/*******************************************************************************
 * Code
 ******************************************************************************/
#define DEMO_PXP PXP

void * switch_disp_buffer()
{
	dispbuffidx = (dispbuffidx+1)%2;
	return (void *)s_final_frameBuffer[dispbuffidx];
}

void* get_cur_disp_buffer()
{
	return (void *)s_final_frameBuffer[dispbuffidx];
}

#define ROTATE_OUTBUFF 1
#define ROTATE_PSBUFF 0

#if ROTATE_OUTBUFF
static void DEMO_InitPxp(void)
{
	/*
	 * Configure the PXP for rotate and scale.
	 */
	PXP_Init(DEMO_PXP);

	PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0U);

    //set position in outbufer, because final the buffer is rotated, so width=1280, Height = 720
	PXP_SetProcessSurfacePosition(DEMO_PXP, 0U, 0U, DEMO_PANEL_HEIGHT - 1U, DEMO_PANEL_WIDTH - 1U);

	/* Disable AS. */
	PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

	PXP_EnableCsc1(DEMO_PXP, false);
}

static void* process_pxp_convert(void *inbuf)
{
	void *lcdFrameAddr;

	pxp_ps_buffer_config_t psBufferConfig = {
		.pixelFormat = kPXP_PsPixelFormatRGB565, /* Note: This is 32-bit per pixel */
		.swapByte	 = false,
		.bufferAddrU = 0U,
		.bufferAddrV = 0U,
		//dawei
		.pitchBytes  = (DEMO_BUFFER_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL), //1280*2bytes for H*W = 1280*720
	};

	/* Output config. */
	pxp_output_buffer_config_t outputBufferConfig = {
		.pixelFormat	= kPXP_OutputPixelFormatRGB565,
		.interlacedMode = kPXP_OutputProgressive,
		.buffer1Addr	= 0U,
		
		.pitchBytes 	=  (DEMO_PANEL_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL), //

		.width	= DEMO_PANEL_HEIGHT, //dawei: output buffer will be rotated, so widht=1280, Height=720
		.height = DEMO_PANEL_WIDTH,
	};

	PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0);
	/* Rotate and scale the camera input to fit display output. */

	/* The PS rotate and scale could not work at the same time, so rotate the output. */
	PXP_SetRotateConfig(DEMO_PXP, kPXP_RotateOutputBuffer, kPXP_Rotate90, kPXP_FlipDisable);

	//dawei: 1280*720 do not need scale
	//PXP_SetProcessSurfaceScaler(DEMO_PXP, DEMO_BUFFER_WIDTH, DEMO_BUFFER_HEIGHT, DEMO_PANEL_HEIGHT, DEMO_PANEL_WIDTH);


	/* Convert the LVGL buffer to LCD buffer. */
	psBufferConfig.bufferAddr = (uint32_t)inbuf;
	PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &psBufferConfig);

	lcdFrameAddr				   = switch_disp_buffer();
	outputBufferConfig.buffer0Addr = (uint32_t)lcdFrameAddr;
	PXP_SetOutputBufferConfig(DEMO_PXP, &outputBufferConfig);

	PXP_Start(DEMO_PXP);

	/* Wait for PXP process complete. */
	while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP)))
	{
	}
	PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);


	return lcdFrameAddr;
}
#endif

#if ROTATE_PSBUFF
static void DEMO_InitPxp(void)
{
	/*
	 * Configure the PXP for rotate and scale.
	 */
	PXP_Init(DEMO_PXP);

	PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0U);

    
	PXP_SetProcessSurfacePosition(DEMO_PXP, 0U, 0U, DEMO_PANEL_WIDTH - 1U, DEMO_PANEL_HEIGHT - 1U);

	/* Disable AS. */
	PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

	PXP_EnableCsc1(DEMO_PXP, false);
}

static void* process_pxp_convert(void *inbuf)
{
	void *lcdFrameAddr;

	pxp_ps_buffer_config_t psBufferConfig = {
		.pixelFormat = kPXP_PsPixelFormatRGB565, /* Note: This is 32-bit per pixel */
		.swapByte	 = false,
		.bufferAddrU = 0U,
		.bufferAddrV = 0U,
		//dawei
		.pitchBytes  = (DEMO_BUFFER_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL), //1280*2bytes for H*W = 1280*720
	};

	/* Output config. */
	pxp_output_buffer_config_t outputBufferConfig = {
		.pixelFormat	= kPXP_OutputPixelFormatRGB565,
		.interlacedMode = kPXP_OutputProgressive,
		.buffer1Addr	= 0U,
		
		//dawei: lcd output buffer is W720, H1280
		.pitchBytes 	=  (DEMO_PANEL_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL), //

		.width	= DEMO_PANEL_WIDTH, //dawei: output buffer is rotated, so widht=1280, Height=720
		.height = DEMO_PANEL_HEIGHT,
	};

	PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0);
	/* Rotate and scale the camera input to fit display output. */

	/* Rotate PS buffer */
	PXP_SetRotateConfig(DEMO_PXP, kPXP_RotateProcessSurface, kPXP_Rotate90, kPXP_FlipDisable);

	//dawei: 1280*720 do not need scale
	//PXP_SetProcessSurfaceScaler(DEMO_PXP, DEMO_BUFFER_WIDTH, DEMO_BUFFER_HEIGHT, DEMO_PANEL_HEIGHT, DEMO_PANEL_WIDTH);

	/* Convert the LVGL buffer to LCD buffer. */
	psBufferConfig.bufferAddr = (uint32_t)inbuf;
	PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &psBufferConfig);

	lcdFrameAddr				   = switch_disp_buffer();
	outputBufferConfig.buffer0Addr = (uint32_t)lcdFrameAddr;
	PXP_SetOutputBufferConfig(DEMO_PXP, &outputBufferConfig);

	PXP_Start(DEMO_PXP);

	/* Wait for PXP process complete. */
	while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP)))
	{
	}
	PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);


	return lcdFrameAddr;
}
#endif



void lv_port_pre_init(void)
{
}

void lv_port_disp_init(void)
{
    //static lv_disp_buf_t disp_buf;
    static lv_disp_draw_buf_t disp_buf;
    
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
    //dawei
    memset(s_final_frameBuffer, 0, sizeof(s_final_frameBuffer));
    
    //lv_disp_buf_init(&disp_buf, s_frameBuffer[0], s_frameBuffer[1], LV_HOR_RES_MAX * LV_VER_RES_MAX);
    lv_disp_draw_buf_init(&disp_buf, s_frameBuffer[0], s_frameBuffer[1],  LV_HOR_RES_MAX * LV_VER_RES_MAX);   /*Initialize the display buffer*/

    
    status_t status;
    dc_fb_info_t fbInfo;

#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
    /* Initialize GPU. */
    BOARD_PrepareVGLiteController();
#endif

    /*-------------------------
     * Initialize your display
     * -----------------------*/
    BOARD_PrepareDisplayController();

    status = g_dc.ops->init(&g_dc);
    if (kStatus_Success != status)
    {
        assert(0);
    }

    g_dc.ops->getLayerDefaultConfig(&g_dc, 0, &fbInfo);
    fbInfo.pixelFormat = DEMO_BUFFER_PIXEL_FORMAT;

    //Dawei: FB buffer is same as LCD panel
    fbInfo.width       = DEMO_PANEL_WIDTH;//1280
    fbInfo.height      = DEMO_PANEL_HEIGHT;//720

    fbInfo.startX      = DEMO_BUFFER_START_X;
    fbInfo.startY      = DEMO_BUFFER_START_Y;
    
    //dawei
    fbInfo.strideBytes = DEMO_BUFFER_BYTE_PER_PIXEL*fbInfo.width;
    
    g_dc.ops->setLayerConfig(&g_dc, 0, &fbInfo);

    g_dc.ops->setCallback(&g_dc, 0, DEMO_BufferSwitchOffCallback, NULL);

#if defined(SDK_OS_FREE_RTOS)
    s_transferDone = xSemaphoreCreateBinary();
    if (NULL == s_transferDone)
    {
        PRINTF("Frame semaphore create failed\r\n");
        assert(0);
    }
#else
    s_transferDone = false;
#endif

    /* littlevgl starts render in frame buffer 0, so show frame buffer 1 first. */
    //g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)s_frameBuffer[1]);
    
    //dawei change this buffer to s_final_frameBuffer
    //g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)s_final_frameBuffer[1]);
	g_dc.ops->setFrameBuffer(&g_dc, 0, get_cur_disp_buffer());

    /* Wait for frame buffer sent to display controller video memory. */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0)
    {
#if defined(SDK_OS_FREE_RTOS)
        if (xSemaphoreTake(s_transferDone, portMAX_DELAY) != pdTRUE)
        {
            PRINTF("Wait semaphore error: s_transferDone\r\n");
            assert(0);
        }
#else
        while (false == s_transferDone)
        {
        }
#endif
    }

    g_dc.ops->enableLayer(&g_dc, 0);

    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;      /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv); /*Basic initialization*/
    disp_drv.full_refresh = 1;

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = DEMO_FlushDisplay;

#if LV_USE_GPU
    disp_drv.clean_dcache_cb = DEMO_CleanInvalidateCache;
#endif

    /*Set a display buffer*/
    //disp_drv.buffer = &disp_buf;
    
    disp_drv.draw_buf = &disp_buf;
    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);

#if LV_USE_GPU && LV_USE_GPU_NXP_PXP
    lv_gpu_nxp_pxp_init(&pxp_default_cfg);
#endif

    DEMO_InitPxp();//dawei: init PXP here

#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
    if (vg_lite_init(64, 64) != VG_LITE_SUCCESS) {
        PRINTF("VGLite init error. STOP.");
        vg_lite_close();
        assert(0);
    }
#endif
}

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer)
{
#if defined(SDK_OS_FREE_RTOS)
    BaseType_t taskAwake = pdFALSE;

    xSemaphoreGiveFromISR(s_transferDone, &taskAwake);
    portYIELD_FROM_ISR(taskAwake);
#else
    s_transferDone = true;
#endif
}

#if LV_USE_GPU
static void DEMO_CleanInvalidateCache(lv_disp_drv_t *disp_drv)
{
#if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCache();
#else
    SCB_CleanInvalidateDCache();
#endif
}
#endif

static void DEMO_FlushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    void * pxp_out_p = NULL;
    
    static uint8_t test = 0;
    static uint8_t test1 = 0;
    
#if !defined(SDK_OS_FREE_RTOS)
    s_transferDone = false;
#endif

    DCACHE_CleanInvalidateByRange((uint32_t)color_p, DEMO_FB_SIZE);

    //switch another FB buffer in LVGL
    //g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)color_p);

    //Dawei: here do PXP transform from LVGL landscape buffer to portrial buffer
    pxp_out_p = process_pxp_convert((void *)color_p);
    
    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)pxp_out_p);


#if defined(SDK_OS_FREE_RTOS)
    if (xSemaphoreTake(s_transferDone, portMAX_DELAY) == pdTRUE)
    {
        /* IMPORTANT!!!
         * Inform the graphics library that you are ready with the flushing*/
        lv_disp_flush_ready(disp_drv);
    }
    else
    {
        PRINTF("Display flush failed\r\n");
        assert(0);
    }
#else
    while (false == s_transferDone)
    {
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
#endif
}

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad */
    DEMO_InitTouch();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = DEMO_ReadTouch;
    lv_indev_drv_register(&indev_drv);
}

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 0);
    }
}

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode)
{
    if (mode == kGT911_IntPinInput)
    {
        BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR &= ~(1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
    }
    else
    {
        if (mode == kGT911_IntPinPullDown)
        {
            GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 0);
        }
        else
        {
            GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 1);
        }

        BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR |= (1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
    }
}

/*Initialize your touchpad*/
static void DEMO_InitTouch(void)
{
    status_t status;

    const gpio_pin_config_t resetPinConfig = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0, .interruptMode = kGPIO_NoIntmode};
    GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, &resetPinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, &resetPinConfig);

    status = GT911_Init(&s_touchHandle, &s_touchConfig);

    if (kStatus_Success != status)
    {
        PRINTF("Touch IC initialization failed\r\n");
        assert(false);
    }

    GT911_GetResolution(&s_touchHandle, &s_touchResolutionX, &s_touchResolutionY);//x=1280, y=720
}

/* Will be called by the library to read the touchpad */
#if 0
static bool DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static int touch_x = 0;
    static int touch_y = 0;

    if (kStatus_Success == GT911_GetSingleTouch(&s_touchHandle, &touch_x, &touch_y))
    {
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    
    /*Set the last pressed coordinates*/
    //data->point.x = touch_x * DEMO_PANEL_WIDTH / s_touchResolutionX;
    //data->point.y = touch_y * DEMO_PANEL_HEIGHT / s_touchResolutionY;

    //dawei change the coordination of touch since buffer is rotated 90бу
    data->point.x = (touch_y)* DEMO_PANEL_HEIGHT / s_touchResolutionY;
    data->point.y = (720 - touch_x) * DEMO_PANEL_WIDTH / s_touchResolutionX;

    PRINTF("%d %d %d %d\r\n",touch_x,touch_y,data->point.x,data->point.y);
    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}
#else
static void DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static int touch_x = 0;
    static int touch_y = 0;

    if (kStatus_Success == GT911_GetSingleTouch(&s_touchHandle, &touch_x, &touch_y))
    {
        data->state = LV_INDEV_STATE_PR;
        PRINTF("%d %d %d %d\r\n",touch_x,touch_y,data->point.x,data->point.y);
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    
    /*Set the last pressed coordinates*/
    //data->point.x = touch_x * DEMO_PANEL_WIDTH / s_touchResolutionX;
    //data->point.y = touch_y * DEMO_PANEL_HEIGHT / s_touchResolutionY;

    //dawei change the coordination of touch since buffer is rotated 90бу
    data->point.x = (touch_y)* DEMO_PANEL_HEIGHT / s_touchResolutionY;
    data->point.y = (720 - touch_x) * DEMO_PANEL_WIDTH / s_touchResolutionX;


    /*Return `false` because we are not buffering and no more data to read*/
    return;
}
#endif
#if LV_USE_GPU && LV_USE_GPU_NXP_VG_LITE
void GPU2D_IRQHandler(void)
{
    vg_lite_IRQHandler();
}

static status_t BOARD_InitVGliteClock(void)
{
    const clock_root_config_t gc355ClockConfig = {
        .clockOff = false,
        .mux      = kCLOCK_GC355_ClockRoot_MuxVideoPllOut,
        .div      = 2,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Gc355, &gc355ClockConfig);

    CLOCK_GetRootClockFreq(kCLOCK_Root_Gc355);

    CLOCK_EnableClock(kCLOCK_Gpu2d);

    NVIC_SetPriority(GPU2D_IRQn, 3);

    EnableIRQ(GPU2D_IRQn);

    return kStatus_Success;
}

static status_t BOARD_PrepareVGLiteController(void)
{
    status_t status;

    status = BOARD_InitVGliteClock();

    if (kStatus_Success != status)
    {
        return status;
    }

    vg_lite_init_mem(registerMemBase, gpu_mem_base, vglite_heap_base, vglite_heap_size);

    vg_lite_set_command_buffer_size(vglite_cmd_buff_size);

    return kStatus_Success;
}
#endif /* LV_USE_GPU_NXP_VG_LITE */



