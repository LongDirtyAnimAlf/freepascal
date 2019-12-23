unit samc21j17au;
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
    SYSTEM_IRQn = 0,                   //   0 SAMC21J17AU System Interrupts 
    WDT_IRQn    = 1,                   //   1 SAMC21J17AU Watchdog Timer (WDT) 
    RTC_IRQn    = 2,                   //   2 SAMC21J17AU Real-Time Counter (RTC) 
    EIC_IRQn    = 3,                   //   3 SAMC21J17AU External Interrupt Controller (EIC) 
    FREQM_IRQn  = 4,                   //   4 SAMC21J17AU Frequency Meter (FREQM) 
    TSENS_IRQn  = 5,                   //   5 SAMC21J17AU Temperature Sensor (TSENS) 
    NVMCTRL_IRQn = 6,                  //   6 SAMC21J17AU Non-Volatile Memory Controller (NVMCTRL) 
    DMAC_IRQn   = 7,                   //   7 SAMC21J17AU Direct Memory Access Controller (DMAC) 
    EVSYS_IRQn  = 8,                   //   8 SAMC21J17AU Event System Interface (EVSYS) 
    SERCOM0_IRQn = 9,                  //   9 SAMC21J17AU Serial Communication Interface 0 (SERCOM0) 
    SERCOM1_IRQn = 10,                 //  10 SAMC21J17AU Serial Communication Interface 1 (SERCOM1) 
    SERCOM2_IRQn = 11,                 //  11 SAMC21J17AU Serial Communication Interface 2 (SERCOM2) 
    SERCOM3_IRQn = 12,                 //  12 SAMC21J17AU Serial Communication Interface 3 (SERCOM3) 
    SERCOM4_IRQn = 13,                 //  13 SAMC21J17AU Serial Communication Interface 4 (SERCOM4) 
    SERCOM5_IRQn = 14,                 //  14 SAMC21J17AU Serial Communication Interface 5 (SERCOM5) 
    CAN0_IRQn   = 15,                  //  15 SAMC21J17AU Control Area Network 0 (CAN0) 
    CAN1_IRQn   = 16,                  //  16 SAMC21J17AU Control Area Network 1 (CAN1) 
    TCC0_IRQn   = 17,                  //  17 SAMC21J17AU Timer Counter Control 0 (TCC0) 
    TCC1_IRQn   = 18,                  //  18 SAMC21J17AU Timer Counter Control 1 (TCC1) 
    TCC2_IRQn   = 19,                  //  19 SAMC21J17AU Timer Counter Control 2 (TCC2) 
    TC0_IRQn    = 20,                  //  20 SAMC21J17AU Basic Timer Counter 0 (TC0) 
    TC1_IRQn    = 21,                  //  21 SAMC21J17AU Basic Timer Counter 1 (TC1) 
    TC2_IRQn    = 22,                  //  22 SAMC21J17AU Basic Timer Counter 2 (TC2) 
    TC3_IRQn    = 23,                  //  23 SAMC21J17AU Basic Timer Counter 3 (TC3) 
    TC4_IRQn    = 24,                  //  24 SAMC21J17AU Basic Timer Counter 4 (TC4) 
    ADC0_IRQn   = 25,                  //  25 SAMC21J17AU Analog Digital Converter 0 (ADC0) 
    ADC1_IRQn   = 26,                  //  26 SAMC21J17AU Analog Digital Converter 1 (ADC1) 
    AC_IRQn     = 27,                  //  27 SAMC21J17AU Analog Comparators (AC) 
    DAC_IRQn    = 28,                  //  28 SAMC21J17AU Digital Analog Converter (DAC) 
    SDADC_IRQn  = 29,                  //  29 SAMC21J17AU Sigma-Delta Analog Digital Converter (SDADC) 
    PTC_IRQn    = 30                   //  30 SAMC21J17AU Peripheral Touch Controller (PTC) 
  );

  TMPU_Type_Registers = record
  end;

  TARM_MPU_Region_t_Registers = record
  end;

  TAc_Registers = record
    CTRLA       : byte;                // Control A 
    CTRLB       : byte;                // Control B 
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUSA     : byte;                // Status A 
    STATUSB     : byte;                // Status B 
    DBGCTRL     : byte;                // Debug Control 
    WINCTRL     : byte;                // Window Control 
    Reserved1   : array[0..0] of byte;
    SCALER      : array[0..3] of byte; // Scaler n 
    COMPCTRL    : array[0..3] of longword; // Comparator Control n 
    SYNCBUSY    : longword;            // Synchronization Busy 
  end;

  TAdc_Registers = record
    CTRLA       : byte;                // Control A 
    CTRLB       : byte;                // Control B 
    REFCTRL     : byte;                // Reference Control 
    EVCTRL      : byte;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    SEQSTATUS   : byte;                // Sequence Status 
    INPUTCTRL   : word;                // Input Control 
    CTRLC       : word;                // Control C 
    AVGCTRL     : byte;                // Average Control 
    SAMPCTRL    : byte;                // Sample Time Control 
    WINLT       : word;                // Window Monitor Lower Threshold 
    WINUT       : word;                // Window Monitor Upper Threshold 
    GAINCORR    : word;                // Gain Correction 
    OFFSETCORR  : word;                // Offset Correction 
    Reserved1   : array[0..1] of byte;
    SWTRIG      : byte;                // Software Trigger 
    Reserved2   : array[0..2] of byte;
    DBGCTRL     : byte;                // Debug Control 
    Reserved3   : array[0..2] of byte;
    SYNCBUSY    : word;                // Synchronization Busy 
    Reserved4   : array[0..1] of byte;
    RESULT      : word;                // Result 
    Reserved5   : array[0..1] of byte;
    SEQCTRL     : longword;            // Sequence Control 
    CALIB       : word;                // Calibration 
  end;

  TCan_Registers = record
    CREL        : longword;            // Core Release 
    ENDN        : longword;            // Endian 
    MRCFG       : longword;            // Message RAM Configuration 
    DBTP        : longword;            // Fast Bit Timing and Prescaler 
    TEST        : longword;            // Test 
    RWD         : longword;            // RAM Watchdog 
    CCCR        : longword;            // CC Control 
    NBTP        : longword;            // Nominal Bit Timing and Prescaler 
    TSCC        : longword;            // Timestamp Counter Configuration 
    TSCV        : longword;            // Timestamp Counter Value 
    TOCC        : longword;            // Timeout Counter Configuration 
    TOCV        : longword;            // Timeout Counter Value 
    Reserved1   : array[0..15] of byte;
    ECR         : longword;            // Error Counter 
    PSR         : longword;            // Protocol Status 
    TDCR        : longword;            // Extended ID Filter Configuration 
    Reserved2   : array[0..3] of byte;
    IR          : longword;            // Interrupt 
    IE          : longword;            // Interrupt Enable 
    ILS         : longword;            // Interrupt Line Select 
    ILE         : longword;            // Interrupt Line Enable 
    Reserved3   : array[0..31] of byte;
    GFC         : longword;            // Global Filter Configuration 
    SIDFC       : longword;            // Standard ID Filter Configuration 
    XIDFC       : longword;            // Extended ID Filter Configuration 
    Reserved4   : array[0..3] of byte;
    XIDAM       : longword;            // Extended ID AND Mask 
    HPMS        : longword;            // High Priority Message Status 
    NDAT1       : longword;            // New Data 1 
    NDAT2       : longword;            // New Data 2 
    RXF0C       : longword;            // Rx FIFO 0 Configuration 
    RXF0S       : longword;            // Rx FIFO 0 Status 
    RXF0A       : longword;            // Rx FIFO 0 Acknowledge 
    RXBC        : longword;            // Rx Buffer Configuration 
    RXF1C       : longword;            // Rx FIFO 1 Configuration 
    RXF1S       : longword;            // Rx FIFO 1 Status 
    RXF1A       : longword;            // Rx FIFO 1 Acknowledge 
    RXESC       : longword;            // Rx Buffer / FIFO Element Size Configuration 
    TXBC        : longword;            // Tx Buffer Configuration 
    TXFQS       : longword;            // Tx FIFO / Queue Status 
    TXESC       : longword;            // Tx Buffer Element Size Configuration 
    TXBRP       : longword;            // Tx Buffer Request Pending 
    TXBAR       : longword;            // Tx Buffer Add Request 
    TXBCR       : longword;            // Tx Buffer Cancellation Request 
    TXBTO       : longword;            // Tx Buffer Transmission Occurred 
    TXBCF       : longword;            // Tx Buffer Cancellation Finished 
    TXBTIE      : longword;            // Tx Buffer Transmission Interrupt Enable 
    TXBCIE      : longword;            // Tx Buffer Cancellation Finished Interrupt Enable 
    Reserved5   : array[0..7] of byte;
    TXEFC       : longword;            // Tx Event FIFO Configuration 
    TXEFS       : longword;            // Tx Event FIFO Status 
    TXEFA       : longword;            // Tx Event FIFO Acknowledge 
  end;

  TCanMramRxbe_Registers = record
    RXBE_0      : longword;            // Rx Buffer Element 0 
    RXBE_1      : longword;            // Rx Buffer Element 1 
    RXBE_DATA   : array[0..15] of longword; // Rx Buffer Element Data 
  end;

  TCanMramRxf0e_Registers = record
    RXF0E_0     : longword;            // Rx FIFO 0 Element 0 
    RXF0E_1     : longword;            // Rx FIFO 0 Element 1 
    RXF0E_DATA  : array[0..15] of longword; // Rx FIFO 0 Element Data 
  end;

  TCanMramRxf1e_Registers = record
    RXF1E_0     : longword;            // Rx FIFO 1 Element 0 
    RXF1E_1     : longword;            // Rx FIFO 1 Element 1 
    RXF1E_DATA  : array[0..15] of longword; // Rx FIFO 1 Element Data 
  end;

  TCanMramSidfe_Registers = record
    SIDFE_0     : longword;            // Standard Message ID Filter Element 
  end;

  TCanMramTxbe_Registers = record
    TXBE_0      : longword;            // Tx Buffer Element 0 
    TXBE_1      : longword;            // Tx Buffer Element 1 
    TXBE_DATA   : array[0..15] of longword; // Tx Buffer Element Data 
  end;

  TCanMramTxefe_Registers = record
    TXEFE_0     : longword;            // Tx Event FIFO Element 0 
    TXEFE_1     : longword;            // Tx Event FIFO Element 1 
  end;

  TCanMramXifde_Registers = record
    XIDFE_0     : longword;            // Extended Message ID Filter Element 0 
    XIDFE_1     : longword;            // Extended Message ID Filter Element 1 
  end;

  TCcl_Registers = record
    CTRL        : byte;                // Control 
    Reserved1   : array[0..2] of byte;
    SEQCTRL     : array[0..1] of byte; // SEQ Control x 
    Reserved2   : array[0..1] of byte;
    LUTCTRL     : array[0..3] of longword; // LUT Control x 
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
    Reserved3   : array[0..1] of byte;
    SYNCBUSY    : longword;            // Synchronization Busy 
    DBGCTRL     : byte;                // Debug Control 
  end;

  TDivas_Registers = record
    CTRLA       : byte;                // Control 
    Reserved1   : array[0..2] of byte;
    STATUS      : byte;                // Status 
    Reserved2   : array[0..2] of byte;
    DIVIDEND    : longword;            // Dividend 
    DIVISOR     : longword;            // Divisor 
    RESULT      : longword;            // Result 
    REM         : longword;            // Remainder 
    SQRNUM      : longword;            // Square Root Input 
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
    SRCADDR     : longword;            // Block Transfer Source Address 
    DSTADDR     : longword;            // Block Transfer Destination Address 
    DESCADDR    : longword;            // Next Descriptor Address 
  end;

  TDsu_Registers = record
    CTRL        : byte;                // Control 
    STATUSA     : byte;                // Status A 
    STATUSB     : byte;                // Status B 
    STATUSC     : byte;                // Status C 
    ADDR        : longword;            // Address 
    LENGTH      : longword;            // Length 
    DATA        : longword;            // Data 
    DCC         : array[0..1] of longword; // Debug Communication Channel n 
    DID         : longword;            // Device Identification 
    Reserved1   : array[0..211] of byte;
    DCFG        : array[0..1] of longword; // Device Configuration 
    Reserved2   : array[0..3847] of byte;
    ENTRY0      : longword;            // CoreSight ROM Table Entry 0 
    ENTRY1      : longword;            // CoreSight ROM Table Entry 1 
    &END        : longword;            // CoreSight ROM Table End 
    Reserved3   : array[0..4031] of byte;
    MEMTYPE     : longword;            // CoreSight ROM Table Memory Type 
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
    CTRLA       : byte;                // Control 
    NMICTRL     : byte;                // NMI Control 
    NMIFLAG     : word;                // NMI Interrupt Flag 
    SYNCBUSY    : longword;            // Syncbusy register 
    EVCTRL      : longword;            // Event Control 
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    ASYNCH      : longword;            // EIC Asynchronous edge Detection Enable 
    CONFIG      : array[0..1] of longword; // Configuration n 
  end;

  TEvsys_Registers = record
    CTRLA       : byte;                // Control 
    Reserved1   : array[0..10] of byte;
    CHSTATUS    : longword;            // Channel Status 
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    SWEVT       : longword;            // Software Event 
    CHANNEL     : array[0..11] of longword; // Channel n 
    Reserved2   : array[0..47] of byte;
    USER        : array[0..46] of longword; // User Multiplexer n 
  end;

  TFreqm_Registers = record
    CTRLA       : byte;                // Control A Register 
    CTRLB       : byte;                // Control B Register 
    CFGA        : word;                // Config A register 
    Reserved1   : array[0..3] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear Register 
    INTENSET    : byte;                // Interrupt Enable Set Register 
    INTFLAG     : byte;                // Interrupt Flag Register 
    STATUS      : byte;                // Status Register 
    SYNCBUSY    : longword;            // Synchronization Busy Register 
    VALUE       : longword;            // Count Value Register 
  end;

  TGclk_Registers = record
    CTRLA       : byte;                // Control 
    Reserved1   : array[0..2] of byte;
    SYNCBUSY    : longword;            // Synchronization Busy 
    Reserved2   : array[0..23] of byte;
    GENCTRL     : array[0..8] of longword; // Generic Clock Generator Control 
    Reserved3   : array[0..59] of byte;
    PCHCTRL     : array[0..40] of longword; // Peripheral Clock Control 
  end;

  THmatrixbPrs_Registers = record
    PRAS        : longword;            // Priority A for Slave 
    PRBS        : longword;            // Priority B for Slave 
  end;

  THmatrixb_Registers = record
    MCFG        : array[0..15] of longword; // Master Configuration 
    SCFG        : array[0..15] of longword; // Slave Configuration 
    Prs         : array[0..3] of THmatrixbPrs_Registers; // HmatrixbPrs groups [CLK_AHB_ID] 
    Reserved1   : array[0..95] of byte;
    MRCR        : longword;            // Master Remap Control 
    Reserved2   : array[0..11] of byte;
    SFR         : array[0..15] of longword; // Special Function 
  end;

  TMclk_Registers = record
    Reserved1   : array[0..0] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    CPUDIV      : byte;                // CPU Clock Division 
    Reserved2   : array[0..10] of byte;
    AHBMASK     : longword;            // AHB Mask 
    APBAMASK    : longword;            // APBA Mask 
    APBBMASK    : longword;            // APBB Mask 
    APBCMASK    : longword;            // APBC Mask 
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
    Reserved6   : array[0..5] of byte;
    PBLDATA0    : longword;            // Page Buffer Load Data 0 
    PBLDATA1    : longword;            // Page Buffer Load Data 1 
  end;

  TOscctrl_Registers = record
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    STATUS      : longword;            // Power and Clocks Status 
    XOSCCTRL    : word;                // External Multipurpose Crystal Oscillator (XOSC) Control 
    CFDPRESC    : byte;                // Clock Failure Detector Prescaler 
    EVCTRL      : byte;                // Event Control 
    OSC48MCTRL  : byte;                // 48MHz Internal Oscillator (OSC48M) Control 
    OSC48MDIV   : byte;                // OSC48M Divider 
    OSC48MSTUP  : byte;                // OSC48M Startup Time 
    Reserved1   : array[0..0] of byte;
    OSC48MSYNCBUSY : longword;         // OSC48M Synchronization Busy 
    DPLLCTRLA   : byte;                // DPLL Control 
    Reserved2   : array[0..2] of byte;
    DPLLRATIO   : longword;            // DPLL Ratio Control 
    DPLLCTRLB   : longword;            // Digital Core Configuration 
    DPLLPRESC   : byte;                // DPLL Prescaler 
    Reserved3   : array[0..2] of byte;
    DPLLSYNCBUSY : byte;               // DPLL Synchronization Busy 
    Reserved4   : array[0..2] of byte;
    DPLLSTATUS  : byte;                // DPLL Status 
    Reserved5   : array[0..6] of byte;
    CAL48M      : longword;            // 48MHz Oscillator Calibration 
  end;

  TOsc32kctrl_Registers = record
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    STATUS      : longword;            // Power and Clocks Status 
    RTCCTRL     : longword;            // Clock selection 
    XOSC32K     : word;                // 32kHz External Crystal Oscillator (XOSC32K) Control 
    CFDCTRL     : byte;                // Clock Failure Detector Control 
    EVCTRL      : byte;                // Event Control 
    OSC32K      : longword;            // 32kHz Internal Oscillator (OSC32K) Control 
    OSCULP32K   : longword;            // 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control 
  end;

  TPac_Registers = record
    WRCTRL      : longword;            // Write control 
    EVCTRL      : byte;                // Event control 
    Reserved1   : array[0..2] of byte;
    INTENCLR    : byte;                // Interrupt enable clear 
    INTENSET    : byte;                // Interrupt enable set 
    Reserved2   : array[0..5] of byte;
    INTFLAGAHB  : longword;            // Bridge interrupt flag status 
    INTFLAGA    : longword;            // Peripheral interrupt flag status - Bridge A 
    INTFLAGB    : longword;            // Peripheral interrupt flag status - Bridge B 
    INTFLAGC    : longword;            // Peripheral interrupt flag status - Bridge C 
    Reserved3   : array[0..19] of byte;
    STATUSA     : longword;            // Peripheral write protection status - Bridge A 
    STATUSB     : longword;            // Peripheral write protection status - Bridge B 
    STATUSC     : longword;            // Peripheral write protection status - Bridge C 
  end;

  TPm_Registers = record
    Reserved1   : array[0..0] of byte;
    SLEEPCFG    : byte;                // Sleep Configuration 
    Reserved2   : array[0..5] of byte;
    STDBYCFG    : word;                // Standby Configuration 
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
    EVCTRL      : longword;            // Event Input Control 
    PMUX        : array[0..15] of byte; // Peripheral Multiplexing n 
    PINCFG      : array[0..31] of byte; // Pin Configuration n 
    Reserved1   : array[0..31] of byte;
  end;

  TPort_Registers = record
    Group       : array[0..1] of TPortGroup_Registers; // PortGroup groups [GROUPS] 
  end;

  TRstc_Registers = record
    RCAUSE      : byte;                // Reset Cause 
  end;

  TRtcMode2Alarm_Registers = record
    ALARM       : longword;            // MODE2_ALARM Alarm n Value 
    MASK        : byte;                // MODE2_ALARM Alarm n Mask 
    Reserved1   : array[0..2] of byte;
  end;

  TRtcMode0_Registers = record
    CTRLA       : word;                // MODE0 Control A 
    Reserved1   : array[0..1] of byte;
    EVCTRL      : longword;            // MODE0 Event Control 
    INTENCLR    : word;                // MODE0 Interrupt Enable Clear 
    INTENSET    : word;                // MODE0 Interrupt Enable Set 
    INTFLAG     : word;                // MODE0 Interrupt Flag Status and Clear 
    DBGCTRL     : byte;                // Debug Control 
    Reserved2   : array[0..0] of byte;
    SYNCBUSY    : longword;            // MODE0 Synchronization Busy Status 
    FREQCORR    : byte;                // Frequency Correction 
    Reserved3   : array[0..2] of byte;
    COUNT       : longword;            // MODE0 Counter Value 
    Reserved4   : array[0..3] of byte;
    COMP        : array[0..0] of longword; // MODE0 Compare n Value 
  end;

  TRtcMode1_Registers = record
    CTRLA       : word;                // MODE1 Control A 
    Reserved1   : array[0..1] of byte;
    EVCTRL      : longword;            // MODE1 Event Control 
    INTENCLR    : word;                // MODE1 Interrupt Enable Clear 
    INTENSET    : word;                // MODE1 Interrupt Enable Set 
    INTFLAG     : word;                // MODE1 Interrupt Flag Status and Clear 
    DBGCTRL     : byte;                // Debug Control 
    Reserved2   : array[0..0] of byte;
    SYNCBUSY    : longword;            // MODE1 Synchronization Busy Status 
    FREQCORR    : byte;                // Frequency Correction 
    Reserved3   : array[0..2] of byte;
    COUNT       : word;                // MODE1 Counter Value 
    Reserved4   : array[0..1] of byte;
    PER         : word;                // MODE1 Counter Period 
    Reserved5   : array[0..1] of byte;
    COMP        : array[0..1] of word; // MODE1 Compare n Value 
  end;

  TRtcMode2_Registers = record
    CTRLA       : word;                // MODE2 Control A 
    Reserved1   : array[0..1] of byte;
    EVCTRL      : longword;            // MODE2 Event Control 
    INTENCLR    : word;                // MODE2 Interrupt Enable Clear 
    INTENSET    : word;                // MODE2 Interrupt Enable Set 
    INTFLAG     : word;                // MODE2 Interrupt Flag Status and Clear 
    DBGCTRL     : byte;                // Debug Control 
    Reserved2   : array[0..0] of byte;
    SYNCBUSY    : longword;            // MODE2 Synchronization Busy Status 
    FREQCORR    : byte;                // Frequency Correction 
    Reserved3   : array[0..2] of byte;
    CLOCK       : longword;            // MODE2 Clock Value 
    Reserved4   : array[0..3] of byte;
    Mode2Alarm  : array[0..0] of TRtcMode2Alarm_Registers; // RtcMode2Alarm groups [ALARM_NUM] 
  end;

  TSdadc_Registers = record
    CTRLA       : byte;                // Control A 
    REFCTRL     : byte;                // Reference Control 
    CTRLB       : word;                // Control B 
    EVCTRL      : byte;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    SEQSTATUS   : byte;                // Sequence Status 
    INPUTCTRL   : byte;                // Input Control 
    CTRLC       : byte;                // Control C 
    WINCTRL     : byte;                // Window Monitor Control 
    WINLT       : longword;            // Window Monitor Lower Threshold 
    WINUT       : longword;            // Window Monitor Upper Threshold 
    OFFSETCORR  : longword;            // Offset Correction 
    GAINCORR    : word;                // Gain Correction 
    SHIFTCORR   : byte;                // Shift Correction 
    Reserved1   : array[0..0] of byte;
    SWTRIG      : byte;                // Software Trigger 
    Reserved2   : array[0..2] of byte;
    SYNCBUSY    : longword;            // Synchronization Busy 
    RESULT      : longword;            // Result 
    SEQCTRL     : byte;                // Sequence Control 
    Reserved3   : array[0..2] of byte;
    ANACTRL     : byte;                // Analog Control 
    Reserved4   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
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
    SYNCBUSY    : longword;            // I2CM Synchronization Busy 
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
    SYNCBUSY    : longword;            // I2CS Synchronization Busy 
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
    SYNCBUSY    : longword;            // SPI Synchronization Busy 
    Reserved6   : array[0..3] of byte;
    ADDR        : longword;            // SPI Address 
    DATA        : longword;            // SPI Data 
    Reserved7   : array[0..3] of byte;
    DBGCTRL     : byte;                // SPI Debug Control 
  end;

  TSercomUsart_Registers = record
    CTRLA       : longword;            // USART Control A 
    CTRLB       : longword;            // USART Control B 
    CTRLC       : longword;            // USART Control C 
    BAUD        : word;                // USART Baud Rate 
    RXPL        : byte;                // USART Receive Pulse Length 
    Reserved1   : array[0..4] of byte;
    INTENCLR    : byte;                // USART Interrupt Enable Clear 
    Reserved2   : array[0..0] of byte;
    INTENSET    : byte;                // USART Interrupt Enable Set 
    Reserved3   : array[0..0] of byte;
    INTFLAG     : byte;                // USART Interrupt Flag Status and Clear 
    Reserved4   : array[0..0] of byte;
    STATUS      : word;                // USART Status 
    SYNCBUSY    : longword;            // USART Synchronization Busy 
    Reserved5   : array[0..7] of byte;
    DATA        : word;                // USART Data 
    Reserved6   : array[0..5] of byte;
    DBGCTRL     : byte;                // USART Debug Control 
  end;

  TSupc_Registers = record
    INTENCLR    : longword;            // Interrupt Enable Clear 
    INTENSET    : longword;            // Interrupt Enable Set 
    INTFLAG     : longword;            // Interrupt Flag Status and Clear 
    STATUS      : longword;            // Power and Clocks Status 
    BODVDD      : longword;            // BODVDD Control 
    BODCORE     : longword;            // BODCORE Control 
    VREG        : longword;            // VREG Control 
    VREF        : longword;            // VREF Control 
  end;

  TTalCtis_Registers = record
    CTICTRLA    : byte;                // Cross-Trigger Interface n Control A 
    CTIMASK     : byte;                // Cross-Trigger Interface n Mask 
  end;

  TTal_Registers = record
    CTRLA       : byte;                // Control A 
    Reserved1   : array[0..2] of byte;
    RSTCTRL     : byte;                // Reset Control 
    EXTCTRL     : byte;                // External Break Control 
    EVCTRL      : byte;                // Event Control 
    Reserved2   : array[0..0] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    GLOBMASK    : byte;                // Global Break Requests Mask 
    HALT        : byte;                // Debug Halt Request 
    RESTART     : byte;                // Debug Restart Request 
    BRKSTATUS   : word;                // Break Request Status 
    Ctis        : array[0..2] of TTalCtis_Registers; // TalCtis groups [CTI_NUM] 
    Reserved3   : array[0..9] of byte;
    INTSTATUS   : array[0..30] of byte; // Interrupt n Status 
    Reserved4   : array[0..32] of byte;
    IRQTRIG     : word;                // Interrupt Trigger 
    Reserved5   : array[0..1] of byte;
    CPUIRQS     : array[0..0] of longword; // Interrupt Status for CPU n 
  end;

  TTcCount8_Registers = record
    CTRLA       : longword;            // Control A 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    WAVE        : byte;                // Waveform Generation Control 
    DRVCTRL     : byte;                // Control C 
    Reserved1   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
    SYNCBUSY    : longword;            // Synchronization Status 
    COUNT       : byte;                // COUNT8 Count 
    Reserved2   : array[0..5] of byte;
    PER         : byte;                // COUNT8 Period 
    CC          : array[0..1] of byte; // COUNT8 Compare and Capture 
    Reserved3   : array[0..16] of byte;
    PERBUF      : byte;                // COUNT8 Period Buffer 
    CCBUF       : array[0..1] of byte; // COUNT8 Compare and Capture Buffer 
  end;

  TTcCount16_Registers = record
    CTRLA       : longword;            // Control A 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    WAVE        : byte;                // Waveform Generation Control 
    DRVCTRL     : byte;                // Control C 
    Reserved1   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
    SYNCBUSY    : longword;            // Synchronization Status 
    COUNT       : word;                // COUNT16 Count 
    Reserved2   : array[0..5] of byte;
    CC          : array[0..1] of word; // COUNT16 Compare and Capture 
    Reserved3   : array[0..15] of byte;
    CCBUF       : array[0..1] of word; // COUNT16 Compare and Capture Buffer 
  end;

  TTcCount32_Registers = record
    CTRLA       : longword;            // Control A 
    CTRLBCLR    : byte;                // Control B Clear 
    CTRLBSET    : byte;                // Control B Set 
    EVCTRL      : word;                // Event Control 
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    STATUS      : byte;                // Status 
    WAVE        : byte;                // Waveform Generation Control 
    DRVCTRL     : byte;                // Control C 
    Reserved1   : array[0..0] of byte;
    DBGCTRL     : byte;                // Debug Control 
    SYNCBUSY    : longword;            // Synchronization Status 
    COUNT       : longword;            // COUNT32 Count 
    Reserved2   : array[0..3] of byte;
    CC          : array[0..1] of longword; // COUNT32 Compare and Capture 
    Reserved3   : array[0..11] of byte;
    CCBUF       : array[0..1] of longword; // COUNT32 Compare and Capture Buffer 
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
    PATTBUF     : word;                // Pattern Buffer 
    Reserved6   : array[0..5] of byte;
    PERBUF      : longword;            // Period Buffer 
    CCBUF       : array[0..3] of longword; // Compare and Capture Buffer 
  end;

  TTsens_Registers = record
    CTRLA       : byte;                // Control A Register 
    CTRLB       : byte;                // Control B Register 
    CTRLC       : byte;                // Control C Register 
    EVCTRL      : byte;                // Event Control Register 
    INTENCLR    : byte;                // Interrupt Enable Clear Register 
    INTENSET    : byte;                // Interrupt Enable Set Register 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear Register 
    STATUS      : byte;                // Status Register 
    SYNCBUSY    : longword;            // Synchronization Busy Register 
    VALUE       : longword;            // Value Register 
    WINLT       : longword;            // Window Monitor Lower Threshold Register 
    WINUT       : longword;            // Window Monitor Upper Threshold Register 
    GAIN        : longword;            // Gain Register 
    OFFSET      : longword;            // Offset Register 
    CAL         : longword;            // Calibration Register 
    DBGCTRL     : byte;                // Debug Control Register 
  end;

  TWdt_Registers = record
    CTRLA       : byte;                // Control 
    CONFIG      : byte;                // Configuration 
    EWCTRL      : byte;                // Early Warning Interrupt Control 
    Reserved1   : array[0..0] of byte;
    INTENCLR    : byte;                // Interrupt Enable Clear 
    INTENSET    : byte;                // Interrupt Enable Set 
    INTFLAG     : byte;                // Interrupt Flag Status and Clear 
    Reserved2   : array[0..0] of byte;
    SYNCBUSY    : longword;            // Synchronization Busy 
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
  AC_BASE       = $42005000;
  ADC0_BASE     = $42004400;
  ADC1_BASE     = $42004800;
  CAN0_BASE     = $42001C00;
  CAN1_BASE     = $42002000;
  CCL_BASE      = $42005C00;
  DAC_BASE      = $42005400;
  DIVAS_BASE    = $48000000;
  DIVAS_IOBUS_BASE= $60000200;
  DMAC_BASE     = $41006000;
  DSU_BASE      = $41002000;
  EIC_BASE      = $40002800;
  EVSYS_BASE    = $42000000;
  FREQM_BASE    = $40002C00;
  GCLK_BASE     = $40001C00;
  HMATRIXHS_BASE= $4100A000;
  MCLK_BASE     = $40000800;
  MTB_BASE      = $41008000;
  NVMCTRL_BASE  = $41004000;
  OSCCTRL_BASE  = $40001000;
  OSC32KCTRL_BASE= $40001400;
  PAC_BASE      = $40000000;
  PM_BASE       = $40000400;
  PORT_BASE     = $41000000;
  PORT_IOBUS_BASE= $60000000;
  RSTC_BASE     = $40000C00;
  RTC_BASE      = $40002400;
  SDADC_BASE    = $42004C00;
  SERCOM0_BASE  = $42000400;
  SERCOM1_BASE  = $42000800;
  SERCOM2_BASE  = $42000C00;
  SERCOM3_BASE  = $42001000;
  SERCOM4_BASE  = $42001400;
  SERCOM5_BASE  = $42001800;
  SUPC_BASE     = $40001800;
  TAL_BASE      = $42006000;
  TC0_BASE      = $42003000;
  TC1_BASE      = $42003400;
  TC2_BASE      = $42003800;
  TC3_BASE      = $42003C00;
  TC4_BASE      = $42004000;
  TCC0_BASE     = $42002400;
  TCC1_BASE     = $42002800;
  TCC2_BASE     = $42002C00;
  TSENS_BASE    = $40003000;
  WDT_BASE      = $40002000;

var
  AC            : TAc_Registers absolute AC_BASE;
  ADC0          : TAdc_Registers absolute ADC0_BASE;
  ADC1          : TAdc_Registers absolute ADC1_BASE;
  CAN0          : TCan_Registers absolute CAN0_BASE;
  CAN1          : TCan_Registers absolute CAN1_BASE;
  CCL           : TCcl_Registers absolute CCL_BASE;
  DAC           : TDac_Registers absolute DAC_BASE;
  DIVAS         : TDivas_Registers absolute DIVAS_BASE;
  DIVAS_IOBUS   : TDivas_Registers absolute DIVAS_IOBUS_BASE;
  DMAC          : TDmac_Registers absolute DMAC_BASE;
  DSU           : TDsu_Registers absolute DSU_BASE;
  EIC           : TEic_Registers absolute EIC_BASE;
  EVSYS         : TEvsys_Registers absolute EVSYS_BASE;
  FREQM         : TFreqm_Registers absolute FREQM_BASE;
  GCLK          : TGclk_Registers absolute GCLK_BASE;
  HMATRIXHS     : THmatrixb_Registers absolute HMATRIXHS_BASE;
  MCLK          : TMclk_Registers absolute MCLK_BASE;
  MTB           : TMtb_Registers absolute MTB_BASE;
  NVMCTRL       : TNvmctrl_Registers absolute NVMCTRL_BASE;
  OSCCTRL       : TOscctrl_Registers absolute OSCCTRL_BASE;
  OSC32KCTRL    : TOsc32kctrl_Registers absolute OSC32KCTRL_BASE;
  PAC           : TPac_Registers absolute PAC_BASE;
  PM            : TPm_Registers absolute PM_BASE;
  PORT          : TPort_Registers absolute PORT_BASE;
  PORT_IOBUS    : TPort_Registers absolute PORT_IOBUS_BASE;
  RSTC          : TRstc_Registers absolute RSTC_BASE;
  RTC           : TRtc_Registers absolute RTC_BASE;
  SDADC         : TSdadc_Registers absolute SDADC_BASE;
  SERCOM0       : TSercom_Registers absolute SERCOM0_BASE;
  SERCOM1       : TSercom_Registers absolute SERCOM1_BASE;
  SERCOM2       : TSercom_Registers absolute SERCOM2_BASE;
  SERCOM3       : TSercom_Registers absolute SERCOM3_BASE;
  SERCOM4       : TSercom_Registers absolute SERCOM4_BASE;
  SERCOM5       : TSercom_Registers absolute SERCOM5_BASE;
  SUPC          : TSupc_Registers absolute SUPC_BASE;
  TAL           : TTal_Registers absolute TAL_BASE;
  TC0           : TTc_Registers absolute TC0_BASE;
  TC1           : TTc_Registers absolute TC1_BASE;
  TC2           : TTc_Registers absolute TC2_BASE;
  TC3           : TTc_Registers absolute TC3_BASE;
  TC4           : TTc_Registers absolute TC4_BASE;
  TCC0          : TTcc_Registers absolute TCC0_BASE;
  TCC1          : TTcc_Registers absolute TCC1_BASE;
  TCC2          : TTcc_Registers absolute TCC2_BASE;
  TSENS         : TTsens_Registers absolute TSENS_BASE;
  WDT           : TWdt_Registers absolute WDT_BASE;

implementation

procedure NonMaskableInt_interrupt; external name 'NonMaskableInt_interrupt';
procedure HardFault_interrupt; external name 'HardFault_interrupt';
procedure SVCall_interrupt; external name 'SVCall_interrupt';
procedure PendSV_interrupt; external name 'PendSV_interrupt';
procedure SysTick_interrupt; external name 'SysTick_interrupt';
procedure SYSTEM_interrupt; external name 'SYSTEM_interrupt';
procedure WDT_interrupt; external name 'WDT_interrupt';
procedure RTC_interrupt; external name 'RTC_interrupt';
procedure EIC_interrupt; external name 'EIC_interrupt';
procedure FREQM_interrupt; external name 'FREQM_interrupt';
procedure TSENS_interrupt; external name 'TSENS_interrupt';
procedure NVMCTRL_interrupt; external name 'NVMCTRL_interrupt';
procedure DMAC_interrupt; external name 'DMAC_interrupt';
procedure EVSYS_interrupt; external name 'EVSYS_interrupt';
procedure SERCOM0_interrupt; external name 'SERCOM0_interrupt';
procedure SERCOM1_interrupt; external name 'SERCOM1_interrupt';
procedure SERCOM2_interrupt; external name 'SERCOM2_interrupt';
procedure SERCOM3_interrupt; external name 'SERCOM3_interrupt';
procedure SERCOM4_interrupt; external name 'SERCOM4_interrupt';
procedure SERCOM5_interrupt; external name 'SERCOM5_interrupt';
procedure CAN0_interrupt; external name 'CAN0_interrupt';
procedure CAN1_interrupt; external name 'CAN1_interrupt';
procedure TCC0_interrupt; external name 'TCC0_interrupt';
procedure TCC1_interrupt; external name 'TCC1_interrupt';
procedure TCC2_interrupt; external name 'TCC2_interrupt';
procedure TC0_interrupt; external name 'TC0_interrupt';
procedure TC1_interrupt; external name 'TC1_interrupt';
procedure TC2_interrupt; external name 'TC2_interrupt';
procedure TC3_interrupt; external name 'TC3_interrupt';
procedure TC4_interrupt; external name 'TC4_interrupt';
procedure ADC0_interrupt; external name 'ADC0_interrupt';
procedure ADC1_interrupt; external name 'ADC1_interrupt';
procedure AC_interrupt; external name 'AC_interrupt';
procedure DAC_interrupt; external name 'DAC_interrupt';
procedure SDADC_interrupt; external name 'SDADC_interrupt';
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
  .long SYSTEM_interrupt
  .long WDT_interrupt
  .long RTC_interrupt
  .long EIC_interrupt
  .long FREQM_interrupt
  .long TSENS_interrupt
  .long NVMCTRL_interrupt
  .long DMAC_interrupt
  .long EVSYS_interrupt
  .long SERCOM0_interrupt
  .long SERCOM1_interrupt
  .long SERCOM2_interrupt
  .long SERCOM3_interrupt
  .long SERCOM4_interrupt
  .long SERCOM5_interrupt
  .long CAN0_interrupt
  .long CAN1_interrupt
  .long TCC0_interrupt
  .long TCC1_interrupt
  .long TCC2_interrupt
  .long TC0_interrupt
  .long TC1_interrupt
  .long TC2_interrupt
  .long TC3_interrupt
  .long TC4_interrupt
  .long ADC0_interrupt
  .long ADC1_interrupt
  .long AC_interrupt
  .long DAC_interrupt
  .long SDADC_interrupt
  .long PTC_interrupt

  .weak NonMaskableInt_interrupt
  .weak HardFault_interrupt
  .weak SVCall_interrupt
  .weak PendSV_interrupt
  .weak SysTick_interrupt
  .weak SYSTEM_interrupt
  .weak WDT_interrupt
  .weak RTC_interrupt
  .weak EIC_interrupt
  .weak FREQM_interrupt
  .weak TSENS_interrupt
  .weak NVMCTRL_interrupt
  .weak DMAC_interrupt
  .weak EVSYS_interrupt
  .weak SERCOM0_interrupt
  .weak SERCOM1_interrupt
  .weak SERCOM2_interrupt
  .weak SERCOM3_interrupt
  .weak SERCOM4_interrupt
  .weak SERCOM5_interrupt
  .weak CAN0_interrupt
  .weak CAN1_interrupt
  .weak TCC0_interrupt
  .weak TCC1_interrupt
  .weak TCC2_interrupt
  .weak TC0_interrupt
  .weak TC1_interrupt
  .weak TC2_interrupt
  .weak TC3_interrupt
  .weak TC4_interrupt
  .weak ADC0_interrupt
  .weak ADC1_interrupt
  .weak AC_interrupt
  .weak DAC_interrupt
  .weak SDADC_interrupt
  .weak PTC_interrupt

  .set NonMaskableInt_interrupt, Haltproc
  .set HardFault_interrupt, Haltproc
  .set SVCall_interrupt, Haltproc
  .set PendSV_interrupt, Haltproc
  .set SysTick_interrupt, Haltproc
  .set SYSTEM_interrupt, Haltproc
  .set WDT_interrupt, Haltproc
  .set RTC_interrupt, Haltproc
  .set EIC_interrupt, Haltproc
  .set FREQM_interrupt, Haltproc
  .set TSENS_interrupt, Haltproc
  .set NVMCTRL_interrupt, Haltproc
  .set DMAC_interrupt, Haltproc
  .set EVSYS_interrupt, Haltproc
  .set SERCOM0_interrupt, Haltproc
  .set SERCOM1_interrupt, Haltproc
  .set SERCOM2_interrupt, Haltproc
  .set SERCOM3_interrupt, Haltproc
  .set SERCOM4_interrupt, Haltproc
  .set SERCOM5_interrupt, Haltproc
  .set CAN0_interrupt, Haltproc
  .set CAN1_interrupt, Haltproc
  .set TCC0_interrupt, Haltproc
  .set TCC1_interrupt, Haltproc
  .set TCC2_interrupt, Haltproc
  .set TC0_interrupt, Haltproc
  .set TC1_interrupt, Haltproc
  .set TC2_interrupt, Haltproc
  .set TC3_interrupt, Haltproc
  .set TC4_interrupt, Haltproc
  .set ADC0_interrupt, Haltproc
  .set ADC1_interrupt, Haltproc
  .set AC_interrupt, Haltproc
  .set DAC_interrupt, Haltproc
  .set SDADC_interrupt, Haltproc
  .set PTC_interrupt, Haltproc

  .text
  end;
end.
