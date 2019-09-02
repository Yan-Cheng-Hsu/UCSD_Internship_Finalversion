/// \addtogroup module_scif_driver_setup
//@{
#include "scif.h"
#include "scif_framework.h"
#undef DEVICE_FAMILY_PATH
#if defined(DEVICE_FAMILY)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/DEVICE_FAMILY/x>
#elif defined(DeviceFamily_CC26X0)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc26x0/x>
#elif defined(DeviceFamily_CC26X0R2)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc26x0r2/x>
#elif defined(DeviceFamily_CC13X0)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc13x0/x>
#else
    #define DEVICE_FAMILY_PATH(x) <x>
#endif
#include DEVICE_FAMILY_PATH(inc/hw_types.h)
#include DEVICE_FAMILY_PATH(inc/hw_memmap.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_event.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_rtc.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_wuc.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_sce.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_smph.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_evctl.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_aiodio.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_timer.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_wuc.h)
#include DEVICE_FAMILY_PATH(inc/hw_event.h)
#include DEVICE_FAMILY_PATH(inc/hw_ints.h)
#include DEVICE_FAMILY_PATH(inc/hw_ioc.h)
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection(void);
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x1408, 0x040C, 0x1408, 0x042C, 0x1408, 0x0447, 0x1408, 0x044D, 0x4436, 0x2437, 0xAEFE, 0xADB7, 0x6442, 0x7000, 0x7C6F, 0x6876, 
    /*0x0020*/ 0x0069, 0x1425, 0x6877, 0x006B, 0x1425, 0x6878, 0x006D, 0x1425, 0x786F, 0xF801, 0xFA02, 0xBEF2, 0x7874, 0x6876, 0xFD0E, 0x6878, 
    /*0x0040*/ 0xED92, 0xFD06, 0x7C74, 0x642D, 0x0450, 0x786F, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 0x6630, 0x6542, 0x0000, 0x1874, 
    /*0x0060*/ 0x9D88, 0x9C01, 0xB60D, 0x1067, 0xAF19, 0xAA00, 0xB609, 0xA8FF, 0xAF39, 0xBE06, 0x0C6F, 0x886B, 0x8F08, 0xFD47, 0x9DB7, 0x086F, 
    /*0x0080*/ 0x8801, 0x8A02, 0xBEEC, 0x262F, 0xAEFE, 0x4630, 0x0450, 0x5527, 0x6642, 0x0000, 0x0C6F, 0x140B, 0x0450, 0x6742, 0x03FF, 0x0C71, 
    /*0x00A0*/ 0x7870, 0x6871, 0xED37, 0xB605, 0x0000, 0x0C70, 0x7C75, 0x652D, 0x0C71, 0x7871, 0x6872, 0xFD0E, 0xF801, 0xE92B, 0xFD0E, 0xBE01, 
    /*0x00C0*/ 0x6436, 0xBDB7, 0x241A, 0xA6FE, 0xADB7, 0x641A, 0xADB7, 0x0000, 0x0000, 0x0086, 0x0192, 0x0108, 0x0197, 0x0191, 0x01B6, 0x0000, 
    /*0x00E0*/ 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0001, 0x0000, 0x0000, 0x0000, 
    /*0x0100*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x6000, 0x15BD, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7000, 0x6881, 0x15D4, 
    /*0x0120*/ 0x6C81, 0x0881, 0x8A00, 0xBE0C, 0x6881, 0x15B7, 0x6C81, 0x7091, 0x6881, 0x15D4, 0x6C81, 0x7001, 0x6881, 0x15F2, 0x6C81, 0x7C82, 
    /*0x0140*/ 0x6881, 0x15B7, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7001, 0x6881, 0x15D4, 0x6C81, 0x0881, 0x8A00, 0xBE04, 0x7000, 0x6881, 
    /*0x0160*/ 0x15D4, 0x6C81, 0x6881, 0x15B7, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7010, 0x6881, 0x15D4, 0x6C81, 0x0881, 0x8A00, 0xBE04, 
    /*0x0180*/ 0x787A, 0x6881, 0x15D4, 0x6C81, 0x6881, 0x15B7, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7012, 0x6881, 0x15D4, 0x6C81, 0x0881, 
    /*0x01A0*/ 0x8A00, 0xBE04, 0x7003, 0x6881, 0x15D4, 0x6C81, 0x6881, 0x15B7, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7011, 0x6881, 0x15D4, 
    /*0x01C0*/ 0x6C81, 0x0881, 0x8A00, 0xBE04, 0x7020, 0x6881, 0x15D4, 0x6C81, 0x6881, 0x15B7, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7011, 
    /*0x01E0*/ 0x6881, 0x15D4, 0x6C81, 0x0881, 0x8A00, 0xBE0C, 0x6881, 0x15B7, 0x6C81, 0x7091, 0x6881, 0x15D4, 0x6C81, 0x7001, 0x6881, 0x15F2, 
    /*0x0200*/ 0x6C81, 0x7C83, 0x6881, 0x1611, 0x6C81, 0x087B, 0x0C67, 0xADB7, 0x6000, 0x15BD, 0x6C81, 0x7090, 0x6881, 0x15D4, 0x6C81, 0x7011, 
    /*0x0220*/ 0x6881, 0x15D4, 0x6C81, 0x0879, 0x8A00, 0xBE2E, 0x0884, 0x8801, 0x0C84, 0x787F, 0xF220, 0x6881, 0x15D4, 0x6C81, 0x0880, 0x8A00, 
    /*0x0240*/ 0xBE10, 0x087F, 0x8A00, 0xBE04, 0x087F, 0x8802, 0x0C7F, 0x052B, 0x087F, 0x8801, 0x0C7F, 0x087F, 0x8A0D, 0xBE02, 0x0001, 0x0C80, 
    /*0x0260*/ 0x0543, 0x0880, 0x8A01, 0xBE0F, 0x087F, 0x8A02, 0xBE04, 0x087F, 0x88FE, 0x0C7F, 0x053E, 0x087F, 0x88FF, 0x0C7F, 0x087F, 0x8A00, 
    /*0x0280*/ 0xBE02, 0x0000, 0x0C80, 0x055A, 0x0879, 0x8A01, 0xBE13, 0x087E, 0x8A00, 0xBE03, 0x000D, 0x0C7F, 0x0552, 0x087E, 0x8A01, 0xBE02, 
    /*0x02A0*/ 0x0000, 0x0C7F, 0x087E, 0x8801, 0x0C7E, 0x787F, 0xF220, 0x6881, 0x15D4, 0x6C81, 0x6881, 0x15B7, 0x6C81, 0x7090, 0x6881, 0x15D4, 
    /*0x02C0*/ 0x6C81, 0x7011, 0x6881, 0x15D4, 0x6C81, 0x0881, 0x8A00, 0xBE0C, 0x6881, 0x15B7, 0x6C81, 0x7091, 0x6881, 0x15D4, 0x6C81, 0x7001, 
    /*0x02E0*/ 0x6881, 0x15F2, 0x6C81, 0x7C83, 0x6881, 0x1611, 0x6C81, 0x0884, 0x8A1E, 0xBE03, 0x0870, 0x8201, 0x0C70, 0x087E, 0x8A02, 0xBE05, 
    /*0x0300*/ 0x0000, 0x0C7E, 0x0870, 0x8201, 0x0C70, 0x0879, 0x8A00, 0xBE03, 0x087B, 0x0C67, 0x0590, 0x0879, 0x8A01, 0xBE02, 0x087C, 0x0C67, 
    /*0x0320*/ 0xADB7, 0xADB7, 0x7003, 0x161C, 0x0001, 0x0C68, 0xADB7, 0x7009, 0x1462, 0xFB4D, 0x8609, 0x7101, 0x6431, 0x2531, 0xA6FE, 0xFB00, 
    /*0x0340*/ 0x7078, 0xFB54, 0x7018, 0xFB4C, 0x7003, 0xFB4C, 0xFD47, 0xFB4C, 0x1465, 0x6403, 0x001F, 0x8B2C, 0xFDB1, 0x8902, 0x0C85, 0x1629, 
    /*0x0360*/ 0x0870, 0x8202, 0x0C70, 0x0001, 0x0C68, 0xADB7, 0xADB7, 0x53C6, 0x15C2, 0x670F, 0x15C5, 0x53C6, 0x15C2, 0x460E, 0x53CA, 0x15C2, 
    /*0x0380*/ 0x470F, 0xADB7, 0xD802, 0xDEFE, 0xADB7, 0x539C, 0x2713, 0xAE0B, 0x2713, 0xAE09, 0x2713, 0xAE07, 0x2713, 0xAE05, 0x2713, 0xAE03, 
    /*0x03A0*/ 0xD801, 0xBEF4, 0xE202, 0xADB7, 0xEA00, 0xBE1B, 0xB50E, 0xFDA1, 0x8601, 0xFC00, 0xB602, 0x660E, 0x8E02, 0x460E, 0xFD47, 0x53CC, 
    /*0x03C0*/ 0x15C2, 0x670F, 0x15C5, 0x53CF, 0x15C2, 0x470F, 0x660E, 0x53C6, 0x15C2, 0x670F, 0x15C5, 0x53CC, 0x15C2, 0x2612, 0xA601, 0xE201, 
    /*0x03E0*/ 0x470F, 0xADB7, 0xEA00, 0xBE1C, 0xB50B, 0x660E, 0x53C7, 0x15C2, 0x670F, 0x15C5, 0x53D1, 0x15C2, 0xFDA1, 0x2612, 0xA601, 0xF201, 
    /*0x0400*/ 0x470F, 0x8601, 0xFC00, 0xB602, 0x660E, 0x8E02, 0x460E, 0xFD47, 0x53CB, 0x15C2, 0x670F, 0x15C5, 0x53CF, 0x15C2, 0x470F, 0xF0FF, 
    /*0x0420*/ 0xADB7, 0x460E, 0x53C6, 0x15C2, 0x670F, 0x15C5, 0x53CF, 0x15C2, 0x660E, 0x53C7, 0x15C2, 0xADB7, 0xF007, 0x1462, 0x86FF, 0x63F8, 
    /*0x0440*/ 0xEB51, 0x8680, 0x6000, 0xED8F, 0xEB49, 0xFD47, 0xEB49, 0x1465, 0xADB7, 0x1462, 0x7079, 0xFB55, 0x71FB, 0xFB54, 0xFD47, 0xFB54, 
    /*0x0460*/ 0x1465, 0x4431, 0x4400, 0xADB7
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    120, 116, 112, 108, 104, 100, 96, 92, 28, 24, 20, 16, 12, 8, 4, 0
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:20] Data structure size (number of 16-bit words)
  * - [19:12] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [11:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x005010F2, 0x00000000, 0x00000000, 0x007010FC, // cvTask
    0x00000000, 0x00000000, 0x0010110A, 0x00000000  // sampleADC
};




// No run-time logging task data structure signatures needed in this project




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Initilializes task resource hardware dependencies
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifTaskResourceInit(void) {
    scifInitIo(11, AUXIOMODE_OPEN_DRAIN_WITH_INPUT, -1, 1);
    HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[11]) |= IOC_IOCFG0_HYST_EN_M;
    scifInitIo(2, AUXIOMODE_OPEN_DRAIN_WITH_INPUT, -1, 1);
    HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[2]) |= IOC_IOCFG0_HYST_EN_M;
    scifInitIo(3, AUXIOMODE_ANALOG, -1, 0);
} // scifTaskResourceInit




/** \brief Uninitilializes task resource hardware dependencies
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifTaskResourceUninit(void) {
    scifUninitIo(11, -1);
    scifUninitIo(2, -1);
    scifUninitIo(3, -1);
} // scifTaskResourceUninit




/** \brief Re-initializes I/O pins used by the specified tasks
  *
  * It is possible to stop a Sensor Controller task and let the System CPU borrow and operate its I/O
  * pins. For example, the Sensor Controller can operate an SPI interface in one application state while
  * the System CPU with SSI operates the SPI interface in another application state.
  *
  * This function must be called before \ref scifExecuteTasksOnceNbl() or \ref scifStartTasksNbl() if
  * I/O pins belonging to Sensor Controller tasks have been borrowed System CPU peripherals.
  *
  * \param[in]      bvTaskIds
  *     Bit-vector of task IDs for the task I/Os to be re-initialized
  */
void scifReinitTaskIo(uint32_t bvTaskIds) {
    if (bvTaskIds & (1 << SCIF_CV_TASK_TASK_ID)) {
        scifReinitIo(11, -1, 0);
        HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[11]) |= IOC_IOCFG0_HYST_EN_M;
        scifReinitIo(2, -1, 0);
        HWREG((IOC_BASE + IOC_O_IOCFG0) + pAuxIoIndexToMcuIocfgOffsetLut[2]) |= IOC_IOCFG0_HYST_EN_M;
    }
    if (bvTaskIds & (1 << SCIF_SAMPLE_ADC_TASK_ID)) {
        scifReinitIo(3, -1, 0);
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E00DE,
    (volatile SCIF_TASK_CTRL_T*) 0x400E00E8,
    (volatile uint16_t*) 0x400E00CE,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    scifTaskResourceInit,
    scifTaskResourceUninit,
    (volatile uint16_t*) NULL,
    (volatile uint16_t*) NULL,
    NULL
};




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first at the specified value of the RTC
  * and then periodically at the specified interval. The application must call this function after
  * calling \ref scifInit().
  *
  * The application must ensure that:
  * - \a tickStart is not in the past (prefer using \ref scifStartRtcTicksNow() to avoid this)
  * - \a tickPeriod is not too short. RTC ticks will be skipped silently if the Sensor Controller does
  *   not complete its tasks within a single tick interval.
  *
  * \param[in]      tickStart
  *     RTC value when the first tick is generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP) = tickStart;
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC) = tickPeriod;
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) |= AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M;
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_RTC_CH2);
} // scifStartRtcTicks




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first after approximately 128 us and
  * then periodically at the specified interval. The application must call this function after calling
  * \ref scifInit().
  *
  * The application must ensure that \a tickPeriod is not too short. RTC ticks will be skipped silently
  * if the Sensor Controller does not complete its tasks within a single tick interval.
  *
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicksNow(uint32_t tickPeriod) {
    uint32_t key, sec, subsec;
    key = scifOsalEnterCriticalSection();
    sec = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
    subsec = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
    scifStartRtcTicks(((sec << 16) | (subsec >> 16)) + 8, tickPeriod);
    scifOsalLeaveCriticalSection(key);
} // scifStartRtcTicksNow




/** \brief Stops RTC-based task scheduling tick generation
  *
  * The application must call this function before calling \ref scifUninit().
  */
void scifStopRtcTicks(void) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M);
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_NONE);
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);
} // scifStopRtcTicks


//@}


// Generated by DESKTOP-2MAQ27I at 2019-08-29 15:32:51.881
