unit samd11d14as;
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
    PM_IRQn     = 0,                   //   0 SAMD11D14AS Power Manager (PM) 
    SYSCTRL_IRQn = 1,                  //   1 SAMD11D14AS System Control (SYSCTRL) 
    WDT_IRQn    = 2,                   //   2 SAMD11D14AS Watchdog Timer (WDT) 
    RTC_IRQn    = 3,                   //   3 SAMD11D14AS Real-Time Counter (RTC) 
    EIC_IRQn    = 4,                   //   4 SAMD11D14AS External Interrupt Controller (EIC) 
    NVMCTRL_IRQn = 5,                  //   5 SAMD11D14AS Non-Volatile Memory Controller (NVMCTRL) 
    DMAC_IRQn   = 6,                   //   6 SAMD11D14AS Direct Memory Access Controller (DMAC) 
    USB_IRQn    = 7,                   //   7 SAMD11D14AS Universal Serial Bus (USB) 
    EVSYS_IRQn  = 8,                   //   8 SAMD11D14AS Event System Interface (EVSYS) 
    SERCOM0_IRQn = 9,                  //   9 SAMD11D14AS Serial Communication Interface 0 (SERCOM0) 
    SERCOM1_IRQn = 10,                 //  10 SAMD11D14AS Serial Communication Interface 1 (SERCOM1) 
    SERCOM2_IRQn = 11,                 //  11 SAMD11D14AS Serial Communication Interface 2 (SERCOM2) 
    TCC0_IRQn   = 12,                  //  12 SAMD11D14AS Timer Counter Control (TCC0) 
    TC1_IRQn    = 13,                  //  13 SAMD11D14AS Basic Timer Counter 1 (TC1) 
    TC2_IRQn    = 14,                  //  14 SAMD11D14AS Basic Timer Counter 2 (TC2) 
    ADC_IRQn    = 15,                  //  15 SAMD11D14AS Analog Digital Converter (ADC) 
    AC_IRQn     = 16,                  //  16 SAMD11D14AS Analog Comparators (AC) 
    DAC_IRQn    = 17,                  //  17 SAMD11D14AS Digital Analog Converter (DAC) 
    PTC_IRQn    = 18                   //  18 SAMD11D14AS Peripheral Touch Controller (PTC) 
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
    INPUTCTRL   : longword;            // Input Control 
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

  TDmac_Registers = record
    CTRL        : word;                // Control 
    CRCCTRL     : word;                // CRC Control 
    CRCDATAIN   : longword;            // CRC Data Input 
    CRCCHKSUM   : longword;            // CRC Checksum 
    CRCSTATUS   : byte;                // CRC Status 
    DBGCTRL     : byte;                // Debug Control 
    QOSCTRL     : byte;                // QOS Control 
    Reserved1   : array[0..0] of byte;
    SWTRIGCTRL  : longword;            // Software Trigger Control 
    PRICTRL0    : longword;            // Priority Control 0 
    Reserved2   : array[0..7] of byte;
    INTPEND     : word;                // Interrupt Pending 
    Reserved3   : array[0..1] of byte;
    INTSTATUS   : longword;            // Interrupt Status 
    BUSYCH      : longword;            // Busy Channels 
    PENDCH      : longword;            // Pending Channels 
    ACTIVE      : longword;            // Active Channel and Levels 
    BASEADDR    : longword;            // Descriptor Memory Section Base Address 
    WRBADDR     : longword;            // Write-Back Memory Section Base Address 
    Reserved4   : array[0..2] of byte;
    CHID        : byte;                // Channel ID 
    CHCTRLA     : byte;                // Channel Control A 
    Reserved5   : array[0..2] of byte;
    CHCTRLB     : longword;            // Channel Control B 
    Reserved6   : array[0..3] of byte;
    CHINTENCLR  : byte;                // Channel Interrupt Enable Clear 
    CHINTENSET  : byte;                // Channel Interrupt Enable Set 
    CHINTFLAG   : byte;                // Channel Interrupt Flag Status and Clear 
    CHSTATUS    : byte;                // Channel Status 
  end;

  TDmacDescriptor_Registers = record
    BTCTRL      : word;                // Block Transfer Control 
    BTCNT       : word;                // Block Transfer Count 
    SRCADDR     : longword;            // Transfer Source Address 
    DSTADDR     : longword;            // Transfer Destination Address 
    DESCADDR    : longword;            // Next Descriptor Address 
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
    CONFIG      : array[0..0] of longword; // Configuration n 
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

  THmatrixbPrs_Registers = record
    PRAS        : longword;            // Priority A for Slave 
    PRBS        : longword;            // Priority B for Slave 
  end;

  THmatrixb_Registers = record
    Reserved1   : array[0..127] of byte;
    Prs         : array[0..15] of THmatrixbPrs_Registers; // HmatrixbPrs groups 
    Reserved2   : array[0..15] of byte;
    SFR         : array[0..15] of longword; // Special Function 
  end;

  TMtb_Registers = record
    POSITION    : longword;            // MTB Position 
    MASTER      : longword;            // MTB Master 
    FLOW        : longword;            // MTB Flow 
    BASE        : longword;            // MTB Base 
    Reserved1   : array[0..3823] of byte;
    ITCTRL      : longword;            // MTB Integration Mode Control 
    Reserved2   : array[0..155] of byte;
    CLAIMSET    : longword;            // MTB Claim Set 
    CLAIMCLR    : longword;            // MTB Claim Clear 
    Reserved3   : array[0..7] of byte;
    LOCKACCESS  : longword;            // MTB Lock Access 
    LOCKSTATUS  : longword;            // MTB Lock Status 
    AUTHSTATUS  : longword;            // MTB Authentication Status 
    DEVARCH     : longword;            // MTB Device Architecture 
    Reserved4   : array[0..7] of byte;
    DEVID       : longword;            // MTB Device Configuration 
    DEVTYPE     : longword;            // MTB Device Type 
    PID4        : longword;            // CoreSight 
    PID5        : longword;            // CoreSight 
    PID6        : longword;            // CoreSight 
    PID7        : longword;            // CoreSight 
    PID0        : longword;            // CoreSight 
    PID1        : longword;            // CoreSight 
    PID2        : longword;            // CoreSight 
    PID3        : longword;            // CoreSight 
    CID0        : longword;            // CoreSight 
    CID1        : longword;            // CoreSight 
    CID2        : longword;            // CoreSight 
    CID3        : longword;            // CoreSight 
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
    EXTCTRL     : byte;                // External Reset Controller 
    Reserved1   : array[0..4] of byte;
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
    Group       : array[0..0] of TPortGroup_Registers; // PortGroup groups [GROUPS] 
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
    Reserved1   : array[0..3] of byte;
    BAUD        : longword;            // I2CM Baud Rate 
    Reserved2   : array[0..3] of byte;
    INTENCLR    : byte;                // I2CM Interrupt Enable Clear 
    Reserved3   : array[0..0] of byte;
    INTENSET    : byte;                // I2CM Interrupt Enable Set 
    Reserved4   : array[0..0] of byte;
    INTFLAG     : byte;                // I2CM Interrupt Flag Status and Clear 
    Reserved5   : array[0..0] of byte;
    STATUS      : word;                // I2CM Status 
    SYNCBUSY    : longword;            // I2CM Syncbusy 
    Reserved6   : array[0..3] of byte;
    ADDR        : longword;            // I2CM Address 
    DATA        : byte;                // I2CM Data 
    Reserved7   : array[0..6] of byte;
    DBGCTRL     : byte;                // I2CM Debug Control 
  end;

  TSercomI2cs_Registers = record
    CTRLA       : longword;            // I2CS Control A 
    CTRLB       : longword;            // I2CS Control B 
    Reserved1   : array[0..11] of byte;
    INTENCLR    : byte;                // I2CS Interrupt Enable Clear 
    Reserved2   : array[0..0] of byte;
    INTENSET    : byte;                // I2CS Interrupt Enable Set 
    Reserved3   : array[0..0] of byte;
    INTFLAG     : byte;                // I2CS Interrupt Flag Status and Clear 
    Reserved4   : array[0..0] of byte;
    STATUS      : word;                // I2CS Status 
    SYNCBUSY    : longword;            // I2CS Syncbusy 
    Reserved5   : array[0..3] of byte;
    ADDR        : longword;            // I2CS Address 
    DATA        : byte;                // I2CS Data 
  end;

  TSercomSpi_Registers = record
    CTRLA       : longword;            // SPI Control A 
    CTRLB       : longword;            // SPI Control B 
    Reserved1   : array[0..3] of byte;
    BAUD        : byte;                // SPI Baud Rate 
    Reserved2   : array[0..6] of byte;
    INTENCLR    : byte;                // SPI Interrupt Enable Clear 
    Reserved3   : array[0..0] of byte;
    INTENSET    : byte;                // SPI Interrupt Enable Set 
    Reserved4   : array[0..0] of byte;
    INTFLAG     : byte;                // SPI Interrupt Flag Status and Clear 
    Reserved5   : array[0..0] of byte;
    STATUS      : word;                // SPI Status 
    SYNCBUSY    : longword;            // SPI Syncbusy 
    Reserved6   : array[0..3] of byte;
    ADDR        : longword;            // SPI Address 
    DATA        : longword;            // SPI Data 
    Reserved7   : array[0..3] of byte;
    DBGCTRL     : byte;                // SPI Debug Control 
  end;

  TSercomUsart_Registers = record
    CTRLA       : longword;            // USART Control A 
    CTRLB       : longword;            // USART Control B 
    Reserved1   : array[0..3] of byte;
    BAUD        : word;                // USART Baud Rate 
    RXPL        : byte;                // USART Receive Pulse Length 
    Reserved2   : array[0..4] of byte;
    INTENCLR    : byte;                // USART Interrupt Enable Clear 
    Reserved3   : array[0..0] of byte;
    INTENSET    : byte;                // USART Interrupt Enable Set 
    Reserved4   : array[0..0] of byte;
    INTFLAG     : byte;                // USART Interrupt Flag Status and Clear 
    Reserved5   : array[0..0] of byte;
    STATUS      : word;                // USART Status 
    SYNCBUSY    : longword;            // USART Syncbusy 
    Reserved6   : array[0..7] of byte;
    DATA        : word;                // USART Data 
    Reserved7   : array[0..5] of byte;
    DBGCTRL     : byte;                // USART Debug Control 
  end;

  TSysctrl_Registers = record
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    PCLKSR      : longword;            // Power and Clocks Status 
    XOSC        : word;                // External Multipurpose Crystal Oscillator (XOSC) Control 
    Reserved1   : array[0..1] of byte;
    XOSC32K     : word;                // 32kHz External Crystal Oscillator (XOSC32K) Control 
    Reserved2   : array[0..1] of byte;
    OSC32K      : longword;            // 32kHz Internal Oscillator (OSC32K) Control 
    OSCULP32K   : byte;                // 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control 
    Reserved3   : array[0..2] of byte;
    OSC8M       : longword;            // 8MHz Internal Oscillator (OSC8M) Control 
    DFLLCTRL    : word;                // DFLL48M Control 
    Reserved4   : array[0..1] of byte;
    DFLLVAL     : longword;            // DFLL48M Value 
    DFLLMUL     : longword;            // DFLL48M Multiplier 
    DFLLSYNC    : byte;                // DFLL48M Synchronization 
    Reserved5   : array[0..2] of byte;
    BOD33       : longword;            // 3.3V Brown-Out Detector (BOD33) Control 
    Reserved6   : array[0..7] of byte;
    VREF        : longword;            // Voltage References System (VREF) Control 
    DPLLCTRLA   : byte;                // DPLL Control A 
    Reserved7   : array[0..2] of byte;
    DPLLRATIO   : longword;            // DPLL Ratio Control 
    DPLLCTRLB   : longword;            // DPLL Control B 
    DPLLSTATUS  : byte;                // DPLL Status 
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

  TTcc_Registers = record
    CTRLA       : longword;            // Control A 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    Reserved1   : array[0..1] of byte;
    SYNCBUSY    : longword;            // Synchronization Busy 
    FCTRLA      : longword;            // Recoverable Fault A Configuration 
    FCTRLB      : longword;            // Recoverable Fault B Configuration 
    WEXCTRL     : longword;            // Waveform Extension Configuration 
    DRVCTRL     : longword;            // Driver Control 
    Reserved2   : array[0..1] of byte;
    DBGCTRL     : byte;                // Debug Control 
    Reserved3   : array[0..0] of byte;
    EVCTRL      : longword;            // Event Control 
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    STATUS      : longword;            // Status 
    COUNT       : longword;            // Count 
    PATT        : word;                // Pattern 
    Reserved4   : array[0..1] of byte;
    WAVE        : longword;            // Waveform Control 
    PER         : longword;            // Period 
    CC          : array[0..3] of longword; // Compare and Capture 
    Reserved5   : array[0..15] of byte;
    PATTB       : word;                // Pattern Buffer 
    Reserved6   : array[0..1] of byte;
    WAVEB       : longword;            // Waveform Control Buffer 
    PERB        : longword;            // Period Buffer 
    CCB         : array[0..3] of longword; // Compare and Capture Buffer 
  end;

  TUsbDeviceDescBank_Registers = record
    ADDR        : longword;            // DEVICE_DESC_BANK Endpoint Bank, Adress of Data Buffer 
    PCKSIZE     : longword;            // DEVICE_DESC_BANK Endpoint Bank, Packet Size 
    EXTREG      : word;                // DEVICE_DESC_BANK Endpoint Bank, Extended 
    STATUS_BK   : byte;                // DEVICE_DESC_BANK Enpoint Bank, Status of Bank 
    Reserved1   : array[0..4] of byte;
  end;

  TUsbDeviceEndpoint_Registers = record
    EPCFG       : byte;                // DEVICE_ENDPOINT End Point Configuration 
    Reserved1   : array[0..2] of byte;
    EPSTATUSCLR : byte;                // DEVICE_ENDPOINT End Point Pipe Status Clear 
    EPSTATUSSET : byte;                // DEVICE_ENDPOINT End Point Pipe Status Set 
    EPSTATUS    : byte;                // DEVICE_ENDPOINT End Point Pipe Status 
    EPINTFLAG   : byte;                // DEVICE_ENDPOINT End Point Interrupt Flag 
    EPINTENCLR  : byte;                // DEVICE_ENDPOINT End Point Interrupt Clear Flag 
    EPINTENSET  : byte;                // DEVICE_ENDPOINT End Point Interrupt Set Flag 
    Reserved2   : array[0..21] of byte;
  end;

  TUsbDevice_Registers = record
    CTRLA       : byte;                // Control A 
    Reserved1   : array[0..0] of byte;
    SYNCBUSY    : byte;                // Synchronization Busy 
    QOSCTRL     : byte;                // USB Quality Of Service 
    Reserved2   : array[0..3] of byte;
    CTRLB       : word;                // DEVICE Control B 
    DADD        : byte;                // DEVICE Device Address 
    Reserved3   : array[0..0] of byte;
    STATUS      : byte;                // DEVICE Status 
    FSMSTATUS   : byte;                // Finite State Machine Status 
    Reserved4   : array[0..1] of byte;
    FNUM        : word;                // DEVICE Device Frame Number 
    Reserved5   : array[0..1] of byte;
    INTENCLR    : word;                // DEVICE Device Interrupt Enable Clear 
    Reserved6   : array[0..1] of byte;
    INTENSET    : word;                // DEVICE Device Interrupt Enable Set 
    Reserved7   : array[0..1] of byte;
    INTFLAG     : word;                // DEVICE Device Interrupt Flag 
    Reserved8   : array[0..1] of byte;
    EPINTSMRY   : word;                // DEVICE End Point Interrupt Summary 
    Reserved9   : array[0..1] of byte;
    DESCADD     : longword;            // Descriptor Address 
    PADCAL      : word;                // USB PAD Calibration 
    Reserved10  : array[0..213] of byte;
    DeviceEndpoint : array[0..7] of TUsbDeviceEndpoint_Registers; // UsbDeviceEndpoint groups [EPT_NUM] 
  end;

  TUsbDeviceDescriptor_Registers = record
    DeviceDescBank : array[0..1] of TUsbDeviceDescBank_Registers; // UsbDeviceDescBank groups 
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

  TUsb_Registers = record
    case byte of
      0: ( DEVICE : TUsbDevice_Registers );
  end;

const
  AC_BASE       = $42002400;
  ADC_BASE      = $42002000;
  DAC_BASE      = $42002800;
  DMAC_BASE     = $41004800;
  DSU_BASE      = $41002000;
  EIC_BASE      = $40001800;
  EVSYS_BASE    = $42000400;
  GCLK_BASE     = $40000C00;
  SBMATRIX_BASE = $41007000;
  MTB_BASE      = $41006000;
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
  SYSCTRL_BASE  = $40000800;
  TC1_BASE      = $42001800;
  TC2_BASE      = $42001C00;
  TCC0_BASE     = $42001400;
  USB_BASE      = $41005000;
  WDT_BASE      = $40001000;

var
  AC            : TAc_Registers absolute AC_BASE;
  ADC           : TAdc_Registers absolute ADC_BASE;
  DAC           : TDac_Registers absolute DAC_BASE;
  DMAC          : TDmac_Registers absolute DMAC_BASE;
  DSU           : TDsu_Registers absolute DSU_BASE;
  EIC           : TEic_Registers absolute EIC_BASE;
  EVSYS         : TEvsys_Registers absolute EVSYS_BASE;
  GCLK          : TGclk_Registers absolute GCLK_BASE;
  SBMATRIX      : THmatrixb_Registers absolute SBMATRIX_BASE;
  MTB           : TMtb_Registers absolute MTB_BASE;
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
  SYSCTRL       : TSysctrl_Registers absolute SYSCTRL_BASE;
  TC1           : TTc_Registers absolute TC1_BASE;
  TC2           : TTc_Registers absolute TC2_BASE;
  TCC0          : TTcc_Registers absolute TCC0_BASE;
  USB           : TUsb_Registers absolute USB_BASE;
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
procedure DMAC_interrupt; external name 'DMAC_interrupt';
procedure USB_interrupt; external name 'USB_interrupt';
procedure EVSYS_interrupt; external name 'EVSYS_interrupt';
procedure SERCOM0_interrupt; external name 'SERCOM0_interrupt';
procedure SERCOM1_interrupt; external name 'SERCOM1_interrupt';
procedure SERCOM2_interrupt; external name 'SERCOM2_interrupt';
procedure TCC0_interrupt; external name 'TCC0_interrupt';
procedure TC1_interrupt; external name 'TC1_interrupt';
procedure TC2_interrupt; external name 'TC2_interrupt';
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
  .long DMAC_interrupt
  .long USB_interrupt
  .long EVSYS_interrupt
  .long SERCOM0_interrupt
  .long SERCOM1_interrupt
  .long SERCOM2_interrupt
  .long TCC0_interrupt
  .long TC1_interrupt
  .long TC2_interrupt
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
  .weak DMAC_interrupt
  .weak USB_interrupt
  .weak EVSYS_interrupt
  .weak SERCOM0_interrupt
  .weak SERCOM1_interrupt
  .weak SERCOM2_interrupt
  .weak TCC0_interrupt
  .weak TC1_interrupt
  .weak TC2_interrupt
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
  .set DMAC_interrupt, Haltproc
  .set USB_interrupt, Haltproc
  .set EVSYS_interrupt, Haltproc
  .set SERCOM0_interrupt, Haltproc
  .set SERCOM1_interrupt, Haltproc
  .set SERCOM2_interrupt, Haltproc
  .set TCC0_interrupt, Haltproc
  .set TC1_interrupt, Haltproc
  .set TC2_interrupt, Haltproc
  .set ADC_interrupt, Haltproc
  .set AC_interrupt, Haltproc
  .set DAC_interrupt, Haltproc
  .set PTC_interrupt, Haltproc

  .text
  end;
end.
