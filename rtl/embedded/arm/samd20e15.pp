unit samd20e15;
interface
{$PACKRECORDS C}
{$GOTO ON}

type
  TIRQn_Enum = (
    NonMaskableInt_IRQn = -14,         //   2 Non Maskable Interrupt                 
    HardFault_IRQn = -13,              //   3 Cortex-M0+ Hard Fault Interrupt        
    SVCall_IRQn = -5,                  //  11 Cortex-M0+ SV Call Interrupt           
    PendSV_IRQn = -2,                  //  14 Cortex-M0+ Pend SV Interrupt           
    SysTick_IRQn = -1,                 //  15 Cortex-M0+ System Tick Interrupt       
    PM_IRQn     = 0,                   //   0 SAMD20E15 Power Manager (PM) 
    SYSCTRL_IRQn = 1,                  //   1 SAMD20E15 System Control (SYSCTRL) 
    WDT_IRQn    = 2,                   //   2 SAMD20E15 Watchdog Timer (WDT) 
    RTC_IRQn    = 3,                   //   3 SAMD20E15 Real-Time Counter (RTC) 
    EIC_IRQn    = 4,                   //   4 SAMD20E15 External Interrupt Controller (EIC) 
    NVMCTRL_IRQn = 5,                  //   5 SAMD20E15 Non-Volatile Memory Controller (NVMCTRL) 
    EVSYS_IRQn  = 6,                   //   6 SAMD20E15 Event System Interface (EVSYS) 
    SERCOM0_IRQn = 7,                  //   7 SAMD20E15 Serial Communication Interface 0 (SERCOM0) 
    SERCOM1_IRQn = 8,                  //   8 SAMD20E15 Serial Communication Interface 1 (SERCOM1) 
    SERCOM2_IRQn = 9,                  //   9 SAMD20E15 Serial Communication Interface 2 (SERCOM2) 
    SERCOM3_IRQn = 10,                 //  10 SAMD20E15 Serial Communication Interface 3 (SERCOM3) 
    TC0_IRQn    = 13,                  //  13 SAMD20E15 Basic Timer Counter 0 (TC0) 
    TC1_IRQn    = 14,                  //  14 SAMD20E15 Basic Timer Counter 1 (TC1) 
    TC2_IRQn    = 15,                  //  15 SAMD20E15 Basic Timer Counter 2 (TC2) 
    TC3_IRQn    = 16,                  //  16 SAMD20E15 Basic Timer Counter 3 (TC3) 
    TC4_IRQn    = 17,                  //  17 SAMD20E15 Basic Timer Counter 4 (TC4) 
    TC5_IRQn    = 18,                  //  18 SAMD20E15 Basic Timer Counter 5 (TC5) 
    ADC_IRQn    = 21,                  //  21 SAMD20E15 Analog Digital Converter (ADC) 
    AC_IRQn     = 22,                  //  22 SAMD20E15 Analog Comparators (AC) 
    DAC_IRQn    = 23,                  //  23 SAMD20E15 Digital Analog Converter (DAC) 
    PTC_IRQn    = 24                   //  24 SAMD20E15 Peripheral Touch Controller (PTC) 
  );

  TAc_Registers = record
    CTRLA       : byte;                // Control A 
    CTRLB       : byte;                // Control B 
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    Reserved1   : array[0..0] of byte;
    STATUSA     : byte;                // Status A 
    STATUSB     : byte;                // Status B 
    STATUSC     : byte;                // Status C 
    Reserved2   : array[0..0] of byte;
    WINCTRL     : byte;                // Window Control 
    Reserved3   : array[0..2] of byte;
    COMPCTRL    : array[0..1] of longword; // Comparator Control n 
    Reserved4   : array[0..7] of byte;
    SCALER      : array[0..1] of byte; // Scaler n 
  end;

  TAdc_Registers = record
    CTRLA       : byte;                // Control A 
    REFCTRL     : byte;                // Reference Control 
    AVGCTRL     : byte;                // Average Control 
    SAMPCTRL    : byte;                // Sampling Time Control 
    CTRLB       : word;                // Control B 
    Reserved1   : array[0..1] of byte;
    WINCTRL     : byte;                // Window Monitor Control 
    Reserved2   : array[0..2] of byte;
    SWTRIG      : byte;                // Software Trigger 
    Reserved3   : array[0..2] of byte;
    INPUTCTRL   : longword;            // Inputs Control 
    EVCTRL      : byte;                // Event Control 
    Reserved4   : array[0..0] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    RESULT      : word;                // Result 
    WINLT       : word;                // Window Monitor Lower Threshold 
    Reserved5   : array[0..1] of byte;
    WINUT       : word;                // Window Monitor Upper Threshold 
    Reserved6   : array[0..1] of byte;
    GAINCORR    : word;                // Gain Correction 
    OFFSETCORR  : word;                // Offset Correction 
    CALIB       : word;                // Calibration 
    DBGCTRL     : byte;                // Debug Control 
  end;

  TDac_Registers = record
    CTRLA       : byte;                // Control A 
    CTRLB       : byte;                // Control B 
    EVCTRL      : byte;                // Event Control 
    Reserved1   : array[0..0] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    DATA        : word;                // Data 
    Reserved2   : array[0..1] of byte;
    DATABUF     : word;                // Data Buffer 
  end;

  TDsu_Registers = record
    CTRL        : byte;                // Control 
    STATUSA     : byte;                // Status A 
    STATUSB     : byte;                // Status B 
    Reserved1   : array[0..0] of byte;
    ADDR        : longword;            // Address 
    LENGTH      : longword;            // Length 
    DATA        : longword;            // Data 
    DCC         : array[0..1] of longword; // Debug Communication Channel n 
    DID         : longword;            // Device Identification 
    Reserved2   : array[0..211] of byte;
    DCFG        : array[0..1] of longword; // Device Configuration 
    Reserved3   : array[0..3847] of byte;
    ENTRY       : array[0..1] of longword; // Coresight ROM Table Entry n 
    &END        : longword;            // Coresight ROM Table End 
    Reserved4   : array[0..4031] of byte;
    MEMTYPE     : longword;            // Coresight ROM Table Memory Type 
    PID4        : longword;            // Peripheral Identification 4 
    PID5        : longword;            // Peripheral Identification 5 
    PID6        : longword;            // Peripheral Identification 6 
    PID7        : longword;            // Peripheral Identification 7 
    PID0        : longword;            // Peripheral Identification 0 
    PID1        : longword;            // Peripheral Identification 1 
    PID2        : longword;            // Peripheral Identification 2 
    PID3        : longword;            // Peripheral Identification 3 
    CID0        : longword;            // Component Identification 0 
    CID1        : longword;            // Component Identification 1 
    CID2        : longword;            // Component Identification 2 
    CID3        : longword;            // Component Identification 3 
  end;

  TEic_Registers = record
    CTRL        : byte;                // Control 
    STATUS      : byte;                // Status 
    NMICTRL     : byte;                // Non-Maskable Interrupt Control 
    NMIFLAG     : byte;                // Non-Maskable Interrupt Flag Status and Clear 
    EVCTRL      : longword;            // Event Control 
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    WAKEUP      : longword;            // Wake-Up Enable 
    CONFIG      : array[0..1] of longword; // Configuration n 
  end;

  TEvsys_Registers = record
    CTRL        : byte;                // Control 
    Reserved1   : array[0..2] of byte;
    CHANNEL     : longword;            // Channel 
    USER        : word;                // User Multiplexer 
    Reserved2   : array[0..1] of byte;
    CHSTATUS    : longword;            // Channel Status 
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
  end;

  TGclk_Registers = record
    CTRL        : byte;                // Control 
    STATUS      : byte;                // Status 
    CLKCTRL     : word;                // Generic Clock Control 
    GENCTRL     : longword;            // Generic Clock Generator Control 
    GENDIV      : longword;            // Generic Clock Generator Division 
  end;

  TNvmctrl_Registers = record
    CTRLA       : word;                // Control A 
    Reserved1   : array[0..1] of byte;
    CTRLB       : longword;            // Control B 
    PARAM       : longword;            // NVM Parameter 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    Reserved2   : array[0..2] of byte;
    INTENSET    : byte;                // Interrupt Enable Set 
    Reserved3   : array[0..2] of byte;
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    Reserved4   : array[0..2] of byte;
    STATUS      : word;                // Status 
    Reserved5   : array[0..1] of byte;
    ADDR        : longword;            // Address 
    LOCK        : word;                // Lock Section 
  end;

  TPac_Registers = record
    WPCLR       : longword;            // Write Protection Clear 
    WPSET       : longword;            // Write Protection Set 
  end;

  TPm_Registers = record
    CTRL        : byte;                // Control 
    SLEEP       : byte;                // Sleep Mode 
    Reserved1   : array[0..5] of byte;
    CPUSEL      : byte;                // CPU Clock Select 
    APBASEL     : byte;                // APBA Clock Select 
    APBBSEL     : byte;                // APBB Clock Select 
    APBCSEL     : byte;                // APBC Clock Select 
    Reserved2   : array[0..7] of byte;
    AHBMASK     : longword;            // AHB Mask 
    APBAMASK    : longword;            // APBA Mask 
    APBBMASK    : longword;            // APBB Mask 
    APBCMASK    : longword;            // APBC Mask 
    Reserved3   : array[0..15] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    Reserved4   : array[0..0] of byte;
    RCAUSE      : byte;                // Reset Cause 
  end;

  TPortGroup_Registers = record
    DIR         : longword;            // Data Direction 
    DIRCLR      : longword;            // Data Direction Clear 
    DIRSET      : longword;            // Data Direction Set 
    DIRTGL      : longword;            // Data Direction Toggle 
    OUT         : longword;            // Data Output Value 
    OUTCLR      : longword;            // Data Output Value Clear 
    OUTSET      : longword;            // Data Output Value Set 
    OUTTGL      : longword;            // Data Output Value Toggle 
    &IN         : longword;            // Data Input Value 
    CTRL        : longword;            // Control 
    WRCONFIG    : longword;            // Write Configuration 
    Reserved1   : array[0..3] of byte;
    PMUX        : array[0..15] of byte; // Peripheral Multiplexing n 
    PINCFG      : array[0..31] of byte; // Pin Configuration n 
    Reserved2   : array[0..31] of byte;
  end;

  TPort_Registers = record
    Group       : array[0..1] of TPortGroup_Registers; // PortGroup groups [GROUPS] 
  end;

  TRtcMode2Alarm_Registers = record
    ALARM       : longword;            // MODE2_ALARM Alarm n Value 
    MASK        : byte;                // MODE2_ALARM Alarm n Mask 
    Reserved1   : array[0..2] of byte;
  end;

  TRtcMode0_Registers = record
    CTRL        : word;                // MODE0 Control 
    READREQ     : word;                // Read Request 
    EVCTRL      : word;                // MODE0 Event Control 
    INTENCLR    : byte;                // MODE0 Interrupt Enable Clear 
    INTENSET    : byte;                // MODE0 Interrupt Enable Set 
    INTFLAG     : byte;                // MODE0 Interrupt Flag Status and Clear 
    Reserved1   : array[0..0] of byte;
    STATUS      : byte;                // Status 
    DBGCTRL     : byte;                // Debug Control 
    FREQCORR    : byte;                // Frequency Correction 
    Reserved2   : array[0..2] of byte;
    COUNT       : longword;            // MODE0 Counter Value 
    Reserved3   : array[0..3] of byte;
    COMP        : array[0..0] of longword; // MODE0 Compare n Value 
  end;

  TRtcMode1_Registers = record
    CTRL        : word;                // MODE1 Control 
    READREQ     : word;                // Read Request 
    EVCTRL      : word;                // MODE1 Event Control 
    INTENCLR    : byte;                // MODE1 Interrupt Enable Clear 
    INTENSET    : byte;                // MODE1 Interrupt Enable Set 
    INTFLAG     : byte;                // MODE1 Interrupt Flag Status and Clear 
    Reserved1   : array[0..0] of byte;
    STATUS      : byte;                // Status 
    DBGCTRL     : byte;                // Debug Control 
    FREQCORR    : byte;                // Frequency Correction 
    Reserved2   : array[0..2] of byte;
    COUNT       : word;                // MODE1 Counter Value 
    Reserved3   : array[0..1] of byte;
    PER         : word;                // MODE1 Counter Period 
    Reserved4   : array[0..1] of byte;
    COMP        : array[0..1] of word; // MODE1 Compare n Value 
  end;

  TRtcMode2_Registers = record
    CTRL        : word;                // MODE2 Control 
    READREQ     : word;                // Read Request 
    EVCTRL      : word;                // MODE2 Event Control 
    INTENCLR    : byte;                // MODE2 Interrupt Enable Clear 
    INTENSET    : byte;                // MODE2 Interrupt Enable Set 
    INTFLAG     : byte;                // MODE2 Interrupt Flag Status and Clear 
    Reserved1   : array[0..0] of byte;
    STATUS      : byte;                // Status 
    DBGCTRL     : byte;                // Debug Control 
    FREQCORR    : byte;                // Frequency Correction 
    Reserved2   : array[0..2] of byte;
    CLOCK       : longword;            // MODE2 Clock Value 
    Reserved3   : array[0..3] of byte;
    Mode2Alarm  : array[0..0] of TRtcMode2Alarm_Registers; // RtcMode2Alarm groups [ALARM_NUM] 
  end;

  TSercomI2cm_Registers = record
    CTRLA       : longword;            // I2CM Control A 
    CTRLB       : longword;            // I2CM Control B 
    DBGCTRL     : byte;                // I2CM Debug Control 
    Reserved1   : array[0..0] of byte;
    BAUD        : word;                // I2CM Baud Rate 
    INTENCLR    : byte;                // I2CM Interrupt Enable Clear 
    INTENSET    : byte;                // I2CM Interrupt Enable Set 
    INTFLAG     : byte;                // I2CM Interrupt Flag Status and Clear 
    Reserved2   : array[0..0] of byte;
    STATUS      : word;                // I2CM Status 
    Reserved3   : array[0..1] of byte;
    ADDR        : byte;                // I2CM Address 
    Reserved4   : array[0..2] of byte;
    DATA        : byte;                // I2CM Data 
  end;

  TSercomI2cs_Registers = record
    CTRLA       : longword;            // I2CS Control A 
    CTRLB       : longword;            // I2CS Control B 
    Reserved1   : array[0..3] of byte;
    INTENCLR    : byte;                // I2CS Interrupt Enable Clear 
    INTENSET    : byte;                // I2CS Interrupt Enable Set 
    INTFLAG     : byte;                // I2CS Interrupt Flag Status and Clear 
    Reserved2   : array[0..0] of byte;
    STATUS      : word;                // I2CS Status 
    Reserved3   : array[0..1] of byte;
    ADDR        : longword;            // I2CS Address 
    DATA        : byte;                // I2CS Data 
  end;

  TSercomSpi_Registers = record
    CTRLA       : longword;            // SPI Control A 
    CTRLB       : longword;            // SPI Control B 
    DBGCTRL     : byte;                // SPI Debug Control 
    Reserved1   : array[0..0] of byte;
    BAUD        : byte;                // SPI Baud Rate 
    Reserved2   : array[0..0] of byte;
    INTENCLR    : byte;                // SPI Interrupt Enable Clear 
    INTENSET    : byte;                // SPI Interrupt Enable Set 
    INTFLAG     : byte;                // SPI Interrupt Flag Status and Clear 
    Reserved3   : array[0..0] of byte;
    STATUS      : word;                // SPI Status 
    Reserved4   : array[0..1] of byte;
    ADDR        : longword;            // SPI Address 
    DATA        : word;                // SPI Data 
  end;

  TSercomUsart_Registers = record
    CTRLA       : longword;            // USART Control A 
    CTRLB       : longword;            // USART Control B 
    DBGCTRL     : byte;                // USART Debug Control 
    Reserved1   : array[0..0] of byte;
    BAUD        : word;                // USART Baud 
    INTENCLR    : byte;                // USART Interrupt Enable Clear 
    INTENSET    : byte;                // USART Interrupt Enable Set 
    INTFLAG     : byte;                // USART Interrupt Flag Status and Clear 
    Reserved2   : array[0..0] of byte;
    STATUS      : word;                // USART Status 
    Reserved3   : array[0..5] of byte;
    DATA        : word;                // USART Data 
  end;

  TSysctrl_Registers = record
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    PCLKSR      : longword;            // Power and Clocks Status 
    XOSC        : word;                // XOSC Control 
    Reserved1   : array[0..1] of byte;
    XOSC32K     : word;                // XOSC32K Control 
    Reserved2   : array[0..1] of byte;
    OSC32K      : longword;            // OSC32K Control 
    OSCULP32K   : byte;                // OSCULP32K Control 
    Reserved3   : array[0..2] of byte;
    OSC8M       : longword;            // OSC8M Control A 
    DFLLCTRL    : word;                // DFLL Config 
    Reserved4   : array[0..1] of byte;
    DFLLVAL     : longword;            // DFLL Calibration Value 
    DFLLMUL     : longword;            // DFLL Multiplier 
    DFLLSYNC    : byte;                // DFLL Synchronization 
    Reserved5   : array[0..2] of byte;
    BOD33       : longword;            // 3.3V Brown-Out Detector (BOD33) Control 
    Reserved6   : array[0..3] of byte;
    VREG        : word;                // VREG Control 
    Reserved7   : array[0..1] of byte;
    VREF        : longword;            // VREF Control A 
  end;

  TTcCount8_Registers = record
    CTRLA       : word;                // Control A 
    READREQ     : word;                // Read Request 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    CTRLC       : byte;                // Control C 
    Reserved1   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
    Reserved2   : array[0..0] of byte;
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    COUNT       : byte;                // COUNT8 Counter Value 
    Reserved3   : array[0..2] of byte;
    PER         : byte;                // COUNT8 Period Value 
    Reserved4   : array[0..2] of byte;
    CC          : array[0..1] of byte; // COUNT8 Compare/Capture 
  end;

  TTcCount16_Registers = record
    CTRLA       : word;                // Control A 
    READREQ     : word;                // Read Request 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    CTRLC       : byte;                // Control C 
    Reserved1   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
    Reserved2   : array[0..0] of byte;
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    COUNT       : word;                // COUNT16 Counter Value 
    Reserved3   : array[0..5] of byte;
    CC          : array[0..1] of word; // COUNT16 Compare/Capture 
  end;

  TTcCount32_Registers = record
    CTRLA       : word;                // Control A 
    READREQ     : word;                // Read Request 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    CTRLC       : byte;                // Control C 
    Reserved1   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
    Reserved2   : array[0..0] of byte;
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    COUNT       : longword;            // COUNT32 Counter Value 
    Reserved3   : array[0..3] of byte;
    CC          : array[0..1] of longword; // COUNT32 Compare/Capture 
  end;

  TWdt_Registers = record
    CTRL        : byte;                // Control 
    CONFIG      : byte;                // Configuration 
    EWCTRL      : byte;                // Early Warning Interrupt Control 
    Reserved1   : array[0..0] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    CLEAR       : byte;                // Clear 
  end;

  TRtc_Registers = record
    case byte of
      0: ( MODE0 : TRtcMode0_Registers );
      1: ( MODE1 : TRtcMode1_Registers );
      2: ( MODE2 : TRtcMode2_Registers );
  end;

  TSercom_Registers = record
    case byte of
      0: ( I2CM : TSercomI2cm_Registers );
      1: ( I2CS : TSercomI2cs_Registers );
      2: ( SPI : TSercomSpi_Registers );
      3: ( USART : TSercomUsart_Registers );
  end;

  TTc_Registers = record
    case byte of
      0: ( COUNT8 : TTcCount8_Registers );
      1: ( COUNT16 : TTcCount16_Registers );
      2: ( COUNT32 : TTcCount32_Registers );
  end;

const
  AC_BASE       = $42004400;
  ADC_BASE      = $42004000;
  DAC_BASE      = $42004800;
  DSU_BASE      = $41002000;
  EIC_BASE      = $40001800;
  EVSYS_BASE    = $42000400;
  GCLK_BASE     = $40000C00;
  NVMCTRL_BASE  = $41004000;
  PAC0_BASE     = $40000000;
  PAC1_BASE     = $41000000;
  PAC2_BASE     = $42000000;
  PM_BASE       = $40000400;
  PORT_BASE     = $41004400;
  PORT_IOBUS_BASE= $60000000;
  RTC_BASE      = $40001400;
  SERCOM0_BASE  = $42000800;
  SERCOM1_BASE  = $42000C00;
  SERCOM2_BASE  = $42001000;
  SERCOM3_BASE  = $42001400;
  SYSCTRL_BASE  = $40000800;
  TC0_BASE      = $42002000;
  TC1_BASE      = $42002400;
  TC2_BASE      = $42002800;
  TC3_BASE      = $42002C00;
  TC4_BASE      = $42003000;
  TC5_BASE      = $42003400;
  WDT_BASE      = $40001000;

var
  AC            : TAc_Registers absolute AC_BASE;
  ADC           : TAdc_Registers absolute ADC_BASE;
  DAC           : TDac_Registers absolute DAC_BASE;
  DSU           : TDsu_Registers absolute DSU_BASE;
  EIC           : TEic_Registers absolute EIC_BASE;
  EVSYS         : TEvsys_Registers absolute EVSYS_BASE;
  GCLK          : TGclk_Registers absolute GCLK_BASE;
  NVMCTRL       : TNvmctrl_Registers absolute NVMCTRL_BASE;
  PAC0          : TPac_Registers absolute PAC0_BASE;
  PAC1          : TPac_Registers absolute PAC1_BASE;
  PAC2          : TPac_Registers absolute PAC2_BASE;
  PM            : TPm_Registers absolute PM_BASE;
  PORT          : TPort_Registers absolute PORT_BASE;
  PORT_IOBUS    : TPort_Registers absolute PORT_IOBUS_BASE;
  RTC           : TRtc_Registers absolute RTC_BASE;
  SERCOM0       : TSercom_Registers absolute SERCOM0_BASE;
  SERCOM1       : TSercom_Registers absolute SERCOM1_BASE;
  SERCOM2       : TSercom_Registers absolute SERCOM2_BASE;
  SERCOM3       : TSercom_Registers absolute SERCOM3_BASE;
  SYSCTRL       : TSysctrl_Registers absolute SYSCTRL_BASE;
  TC0           : TTc_Registers absolute TC0_BASE;
  TC1           : TTc_Registers absolute TC1_BASE;
  TC2           : TTc_Registers absolute TC2_BASE;
  TC3           : TTc_Registers absolute TC3_BASE;
  TC4           : TTc_Registers absolute TC4_BASE;
  TC5           : TTc_Registers absolute TC5_BASE;
  WDT           : TWdt_Registers absolute WDT_BASE;

implementation

procedure NonMaskableInt_interrupt; external name 'NonMaskableInt_interrupt';
procedure HardFault_interrupt; external name 'HardFault_interrupt';
procedure SVCall_interrupt; external name 'SVCall_interrupt';
procedure PendSV_interrupt; external name 'PendSV_interrupt';
procedure SysTick_interrupt; external name 'SysTick_interrupt';
procedure PM_interrupt; external name 'PM_interrupt';
procedure SYSCTRL_interrupt; external name 'SYSCTRL_interrupt';
procedure WDT_interrupt; external name 'WDT_interrupt';
procedure RTC_interrupt; external name 'RTC_interrupt';
procedure EIC_interrupt; external name 'EIC_interrupt';
procedure NVMCTRL_interrupt; external name 'NVMCTRL_interrupt';
procedure EVSYS_interrupt; external name 'EVSYS_interrupt';
procedure SERCOM0_interrupt; external name 'SERCOM0_interrupt';
procedure SERCOM1_interrupt; external name 'SERCOM1_interrupt';
procedure SERCOM2_interrupt; external name 'SERCOM2_interrupt';
procedure SERCOM3_interrupt; external name 'SERCOM3_interrupt';
procedure TC0_interrupt; external name 'TC0_interrupt';
procedure TC1_interrupt; external name 'TC1_interrupt';
procedure TC2_interrupt; external name 'TC2_interrupt';
procedure TC3_interrupt; external name 'TC3_interrupt';
procedure TC4_interrupt; external name 'TC4_interrupt';
procedure TC5_interrupt; external name 'TC5_interrupt';
procedure ADC_interrupt; external name 'ADC_interrupt';
procedure AC_interrupt; external name 'AC_interrupt';
procedure DAC_interrupt; external name 'DAC_interrupt';
procedure PTC_interrupt; external name 'PTC_interrupt';


{$i cortexm0_start.inc}

procedure Vectors; assembler; nostackframe;
label interrupt_vectors;
asm
  .section ".init.interrupt_vectors"
  interrupt_vectors:
  .long _stack_top
  .long Startup
  .long NonMaskableInt_interrupt
  .long HardFault_interrupt
  .long 0
  .long 0
  .long 0
  .long 0
  .long 0
  .long 0
  .long 0
  .long SVCall_interrupt
  .long 0
  .long 0
  .long PendSV_interrupt
  .long SysTick_interrupt
  .long PM_interrupt
  .long SYSCTRL_interrupt
  .long WDT_interrupt
  .long RTC_interrupt
  .long EIC_interrupt
  .long NVMCTRL_interrupt
  .long EVSYS_interrupt
  .long SERCOM0_interrupt
  .long SERCOM1_interrupt
  .long SERCOM2_interrupt
  .long SERCOM3_interrupt
  .long 0
  .long 0
  .long TC0_interrupt
  .long TC1_interrupt
  .long TC2_interrupt
  .long TC3_interrupt
  .long TC4_interrupt
  .long TC5_interrupt
  .long 0
  .long 0
  .long ADC_interrupt
  .long AC_interrupt
  .long DAC_interrupt
  .long PTC_interrupt

  .weak NonMaskableInt_interrupt
  .weak HardFault_interrupt
  .weak SVCall_interrupt
  .weak PendSV_interrupt
  .weak SysTick_interrupt
  .weak PM_interrupt
  .weak SYSCTRL_interrupt
  .weak WDT_interrupt
  .weak RTC_interrupt
  .weak EIC_interrupt
  .weak NVMCTRL_interrupt
  .weak EVSYS_interrupt
  .weak SERCOM0_interrupt
  .weak SERCOM1_interrupt
  .weak SERCOM2_interrupt
  .weak SERCOM3_interrupt
  .weak TC0_interrupt
  .weak TC1_interrupt
  .weak TC2_interrupt
  .weak TC3_interrupt
  .weak TC4_interrupt
  .weak TC5_interrupt
  .weak ADC_interrupt
  .weak AC_interrupt
  .weak DAC_interrupt
  .weak PTC_interrupt

  .set NonMaskableInt_interrupt, Haltproc
  .set HardFault_interrupt, Haltproc
  .set SVCall_interrupt, Haltproc
  .set PendSV_interrupt, Haltproc
  .set SysTick_interrupt, Haltproc
  .set PM_interrupt, Haltproc
  .set SYSCTRL_interrupt, Haltproc
  .set WDT_interrupt, Haltproc
  .set RTC_interrupt, Haltproc
  .set EIC_interrupt, Haltproc
  .set NVMCTRL_interrupt, Haltproc
  .set EVSYS_interrupt, Haltproc
  .set SERCOM0_interrupt, Haltproc
  .set SERCOM1_interrupt, Haltproc
  .set SERCOM2_interrupt, Haltproc
  .set SERCOM3_interrupt, Haltproc
  .set TC0_interrupt, Haltproc
  .set TC1_interrupt, Haltproc
  .set TC2_interrupt, Haltproc
  .set TC3_interrupt, Haltproc
  .set TC4_interrupt, Haltproc
  .set TC5_interrupt, Haltproc
  .set ADC_interrupt, Haltproc
  .set AC_interrupt, Haltproc
  .set DAC_interrupt, Haltproc
  .set PTC_interrupt, Haltproc

  .text
  end;
end.
