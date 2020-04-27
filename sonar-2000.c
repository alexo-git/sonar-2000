// ===========================================================================
// =                      Sonar                                              =
// = V1.00            11.06.99 First version                                 =
// = V1.01            14.06.99 Optimized speed, change step_up conv.         =
// = V1.02            11.06.99 Added CRC8 to comm. protocol                  =
// = V1.03            13.09.99 corrected Send_pulses() to "send" 0 pulses    =
// = V1.04            19.12.99 fix seriouse bug in stepper control           =
//                             see "one_step()"                              =
// = V1.05            04.02.00 added new command "TEST_MODE" for stand-alone =
// =                           testing                                       =
// ===========================================================================
// =              Alex O, Israel 1999                                        =
// ===========================================================================
#include <16C76.h>        // define sfr - regiser for 16c73
#include <usart14.h>      // define usart for 16c73
#include <DELAY14.H>      // define delay procedures for 14 - bit program mem

#define CLOCK_20MHz
//#define SENSOR_200KHZ
//#define STDL_TESTER_MODE

#define abs16(a)   ((a) >= 0  ? (a) : (0-a))

#ifdef CLOCK_20MHz
  #define Delay_Us(T)      Delay_Us_20MHz(T)
  #define Delay_Ms(T)      Delay_Ms_20MHz(T)
  #define T0_DELAY         16      // for ~10 KHz sampling (20 MHz)
  #define CHG_TIMEOUT      40      // parrots ;-)     (2 sec)
  #define UART_DIV         10      // 115200 bod!
#else
  #define Delay_Us(T)      Delay_Us_20MHz(T/2)
  #define Delay_Ms(T)      Delay_Ms_20MHz(T/2)
  #define T0_DELAY         143     // for 10.04 KHz sampling (10 MHz)
  #define CHG_TIMEOUT      20      // parrots ;-)     (2 sec)
  #define UART_DIV         10      // 57600 :-(
#endif

#define MAX_AD_SAMPLE       1300    // 
#define REF_CURR            8       // ~ 100 mv;
#define MAX_ER_REF          2       // max 2 steps differential reference error
#define MIN_FREE_ZONE       180     // minimal worked zone (steps)
#define FIX_STEP_TIME       8       // time on one steps (8)

// analog parts common pin
#define tx_inp              PORTB.2  // after change this, see send_pulses()!!
#define sw_up               PORTC.2
#define rx_out              PORTA.0
#define f_back              PORTA.2

// compass pins
#define EOC_pin             PORTC.5
#define RESET_pin           PORTB.3
#define PC_pin              PORTC.0
#define SCLK_pin            PORTC.3
#define SDI_pin             PORTC.4

// stepper motor pins
#define phase_A             PORTB.7
#define phase_B             PORTB.6
#define phase_C             PORTB.5
#define phase_D             PORTB.4
#define mot_enable          PORTC.1
#define stepper_mon         PORTA.1
#define ref_sens            PORTB.0

// RS485 interface direct control pin
#define IF_dir              PORTB.1


// power control constants for stepper motor
#define FULL_POWER          0xFF
#define HOLD_POWER          0x2F
#define NO_POWER            0x00
#define STEP_FORWARD        0x00
#define STEP_BACKWARD       0x01

const ON  = 1;
const OFF = 0;

const TRUE  =  1;
const FALSE =  0;


bits FLAGS;
#define start_receive       FLAGS.0     // Receiver and sampling ISR started
#define burst_mode          FLAGS.1     // burst measurement mode on

bits GSTATUS;
#define comp_fail           GSTATUS.0 // compass data fail
#define conv_fail           GSTATUS.1 // step_up converter fail
#define m_ref_fail          GSTATUS.2 // stepper motor ref. zone fail 
#define m_ref_zone          GSTATUS.3 // stepper motor in ref. zone 
#define g_internal_err      GSTATUS.5 // internal error detected
#define bad_cmd             GSTATUS.6 // bad command received

bits UART_CONFIG;


#define SIGNATURE           0xAA

#define REINIT_CMD          0x01
#define GET_COMPASS_CMD     0x02
#define GO_REF_CMD          0x03
#define DO_STEPS_CMD        0x04
#define MEASURE_CMD         0x05
#define MEASURE_BURST_CMD   0x06
#define GET_POS_CMD         0x07
#define SET_STEPTIME_CMD    0x08
// ....
#define TEST_MODE           0x0F

#define EXT_RESERVED0       0x10
#define EXT_RESERVED1       0x11
#define EXT_RESERVED2       0x12
#define EXT_RESERVED3       0x13

#define BAD_CMD             0xFF

//     -------------------------------------------------------------------
//        Definition structure incoming and outgoing pack for
//        communicatin link
//     -------------------------------------------------------------------
struct  incoming_pack {             
//   unsigned char     sig_in;           // see SIGNATURE
// all next bytes has 8-th bits zero!
   unsigned char     cmd;               
   // see XXX_CMD

   unsigned long     c_sm_data;        // NO LONG! see trn_x2x() !   
   // 0..7 bits - radar angle (in steps); if 0 rotate not need; 
   // 8,9,10,11 - power in stepper motor coil AFTER action;
   // 12th bit - direction 0 - FWRD, 1-BCWRD; 

   unsigned char     c_charge_level;    
   // transmitter power level (0..127)

   unsigned char     c_pulse_count;
   // count transmitter pulses in packet (0..127)

   unsigned long     c_max_sample;     // Obsolet, see trn_x2x() !
   // needed sample count; (0..16384)
};

#define SIZE_INCOMPACK   7      // without signature byte !!


//     -------------------------------------------------------------------

struct  outgoing_pack {
// unsigned char     sig_out;

   unsigned long     X_field;    // or  sensor_pos 
   unsigned long     Y_field;    // or  stepper_pos 

   // current angle from compass
   unsigned char     g_status;          
   unsigned char     g_CRC;
   // see GSTATUS
};

#define SIZE_OUTGPACK    6      // without signature byte !!

//     -------------------------------------------------------------------

struct incoming_pack c_param;           // incoming control packet
struct outgoing_pack r_param;           // outgoing result  packet

//     -------------------------------------------------------------------


const char phase_pattern[4] = {                 //      ABCDtime
          0b10100000,
          0b01100000,
          0b01010000,
          0b10010000
};


//     -------------------------------------------------------------------
//     ---- work cells
signed char    phase_state;       // current phase state in stepper motor coils
unsigned char  sm_direct;
unsigned char  sm_power;          // current power for stepper motor
unsigned long  X_axs_field;       // magnetometer field - X axis
unsigned long  Y_axs_field;       // magnetometer field - Y axis
unsigned long  sample_cnt;        // sample counter

unsigned char  charge_level;      // transmitter power level (0..127)
unsigned char  cur_chrg_level;    // current charge level (updated in __INT)
unsigned char  pulse_count;       // count transmitter pulses in packet (0..127)
unsigned long  max_sample;        // required sample count; (0..16384)
unsigned char  crc;               // CRC8 value
unsigned char  stepper_pos;       // current position of stepper motor
unsigned char  sensor_pos;        // position of stepper motor when in ref. zone
unsigned char  step_delay;        // delay for single step


//     -------------------------------------------------------------------
//     ----                      temporary variables
//     -------------------------------------------------------------------
unsigned char sav_power;
char          ctmp;
unsigned char uctmp;
unsigned char uctmp1;
long          ltmp;
long          ltmp1;
unsigned long ultmp;
char j;                                 // temporary for ADC subroutines
unsigned long rv;

/*================================================================
* Name: long trn_p2l() / long trn_l2p() / 
*
* Description:  translate long variable to packed data
*
*================================================================*/

  unsigned long trn_p2l(unsigned long par) {
     rv  = par & 0x007F;
     rv |= (par & 0x7F00) >> 1;
     return rv;
  }


  unsigned long trn_l2p(unsigned long par) {
     rv =  par  & 0x007F;
     rv |= ((par & 0x3F80) << 1);
     return rv;
  }

/*================================================================
* Name: upd_crc8(unsigned char data) 
*
* Description:  calculated and updated 8 bit polynominal crc
*
*================================================================*/

  void upd_crc8(unsigned char data)
  {
    unsigned char i,f;

    data = data & 0xff;

    for (i = 0; i < 8; ++i)
    {
      f    = 1 & (data ^ crc);
      crc  = crc  >> 1;
      data = data >> 1;
      if (f) crc = crc ^ 0x8c;
    }
  }



/*================================================================
* Name: set_time_out
*
* Description:  Started TMR1; time in 0.81 ms tick, used
*               for get time-outs
*================================================================*/
  void set_time_out(unsigned char time){
    TMR1L = 0;
    TMR1H = 0xFF-time;
    PIR1.TMR1IF = 0;
  }


/*================================================================
* Name: __INT
*
* Description:  Interrupt from TMR0
*
*================================================================*/
    
unsigned char iuctmp,iuctmp1;                   // use only for ISR __INT !

void __INT(void)
{

   INTCON.GIE = 0;    // -- Disable int 
   #asm 
      movwf   temp_WREG           ;save copy w register
      swapf   STATUS,w            ;affects no STATUS bits : Only way OUT to save STATUS Reg ?????
      clrf    STATUS
      movwf   temp_STATUS         ;save status 
      movf    PCLATH,w            ;
      movwf   temp_PCLATH         ;save copy of PCLATH
      clrf    PCLATH
      bcf     STATUS,IRP
      movf    FSR,w               ;
      movwf   temp_FSR            ;save copy of FSR

      movf    __WImage,w          ;
      movwf   temp_WImage         ;save copy of __WImage 
   #endasm
   if (PIE1.TMR2IE && PIR1.TMR2IF) {
    PIR1.TMR2IF = 0;
    NOP();
    ADCON0.CHS0 = 0;                   
    ADCON0.CHS1 = 1;
    ADCON0.CHS2 = 0;
     //--- delay ADC ---
    for(j = 0; j < 6; j++);
    PIR1.ADIF = 0;
    ADCON0.GO = 1;
    while(ADCON0.GO);
    PIR1.ADIF = 0;

    NOP();
    cur_chrg_level = ADRES;
    iuctmp = CCPR1L;
    if (cur_chrg_level < charge_level)
      { if (iuctmp < 0x20) iuctmp++;}
    else  
      { if (iuctmp > 0x00) iuctmp--;}
    CCPR1L = iuctmp;
   }

   // if interrupt from timer0
   else if (INTCON.T0IE && INTCON.T0IF) {
     INTCON.T0IF = 0;          // -- Clear TMR0 int flag
     TMR0 = T0_DELAY;          // Load to TMR0 std const

     if (sample_cnt > 0) {
       sample_cnt--;

        //--- delay ADC ---
       for(j = 0; j < 6; j++);
       PIR1.ADIF = 0;
       ADCON0.GO = 1;
       while(ADCON0.GO);
       PIR1.ADIF = 0;

        //--- transmit byte to line  
       while (!TXSTA.TRMT);     // Wait until buffer is empty
      iuctmp = ADRES;   // TX data
      TXREG = iuctmp;
      
      //--- count CRC
       upd_crc8(iuctmp);

    
     } else start_receive = 0;
   }

   // if an interrupt from INT0 - we are in the opto sensor reference zone 
   // flag <m_ref_zone> cleared in one_step() function after each steps
   else if (INTCON.INTF && INTCON.INTE) {
     INTCON.INTF = 0;
     m_ref_zone = ON;
   }
   else {
    g_internal_err = 1;
    NOP();
    NOP();
   }
   
   #asm
      bcf     STATUS,RP0          ;ensure bank is set to 0
      movf    temp_WImage,w       ;
      movwf   __WImage            ;restore __WImage
      
      movf    temp_PCLATH,w       ;
      movwf   PCLATH              ;restore PCLATH
      swapf   temp_STATUS,w       ;restore old STATUS nibbles 
      movwf   STATUS              ;restore status and bank information
      swapf   temp_WREG,f         ;
      swapf   temp_WREG,w         ;restore working register
      
   #endasm
}

/*================================================================
* Name: stepper_power()
*
* Description:  set power level for stepper motor 
*               
*================================================================*/
  void stepper_power()
  {
    if (sm_power==FULL_POWER) {
      CCP2CON = 0x00;
      mot_enable = 1;
    } else
    if (sm_power==NO_POWER)   {
      CCP2CON = 0x00;
      mot_enable = 0;
    } else {
      CCP2CON = 0x0F;
      CCPR2L = sm_power;       
    }
  }

/*================================================================
* Name: receive_echo()
*
* Description:  starting TMR0 interrupt to sample freq 10 KHz;
*               see __INT0   
*================================================================*/
  void receive_echo()
  {
    sav_power = sm_power;
    sm_power = FULL_POWER;
    stepper_power();
    
    CCP1CON = 0x00;		// Disable step-up converter
    PIE1.TMR2IE = 0;            
    PIR1.TMR2IF = 0;
    sw_up = 0;

    ADCON0.CHS0 = 0;                   
    ADCON0.CHS1 = 0;
    ADCON0.CHS2 = 0;
    
    IF_dir = 1;		        // setting RS485 driver to output
    sample_cnt = max_sample;
    start_receive = 1;		// signal for __INT0 to start convertion
    INTCON.T0IE = 1;
    while (start_receive) {};    // wait until __INT0 sampled and received echo-signal
    INTCON.T0IE = 0;             // TMR0 interrupt disable
    sm_power = sav_power;
    stepper_power();
    PIE1.TMR2IE = 1;            // Enable step-up converter
    T2CON.TMR2ON = 1;
    CCP1CON = 0x0F;
  }


/*================================================================
* Name: charge_tx()
*
* Description:  charge output capacitors to "charge_level" voltage
*
*================================================================*/
 void charge_tx()
 {
   set_time_out(255);                 // set timeout period
   uctmp = 0;
   ultmp = 0;
   while (1==1)
   {
     ltmp += cur_chrg_level;          // simple filtering
     ltmp /= 2;
     ltmp1 = ltmp-charge_level;

     if (ltmp1 > 15) 
       tx_inp = 1;
     else
       tx_inp = 0;
     
     if (abs16(ltmp1) < 5) {
         conv_fail = 0; 
         tx_inp = 0;
         return;
     }

     if (PIR1.TMR1IF){
         if (++uctmp > CHG_TIMEOUT) {
           conv_fail = 1; 
           tx_inp = 0;
           return;
         } 
         else  
           set_time_out(255);       // set timeout period   
     }
   }
 }


/*================================================================
* Name: send_pulses()
*
* Description:  make transmitter 200 Khz "pulse_count" pulses
*               Different version for different sys. clock
*================================================================*/

#ifdef SENSOR_200KHZ
                    
#ifdef CLOCK_20MHz

  void send_pulses()
  {
   unsigned char cnt;
   cnt = pulse_count;
   if (cnt == 0) return;
   INTCON.GIE  = 0;                // disable  all interrupt
   #asm
        BCF     03,5
        MOVF    cnt,w
        MOVWF   __WImage
DLUS10M 
        NOP             
        NOP             

        NOP                        // add to 20 Mhz 
        NOP             
        NOP             
        NOP             
        NOP             
        NOP             
        NOP             

        BSF     06,2
        NOP             
        NOP             
        NOP             
        NOP            
        NOP             

        NOP                        // add to 20 MHz
        NOP            
        NOP             
        NOP            
        NOP             
        NOP             

        BCF     06,2
        DECFSZ  __WImage 
        GOTO    DLUS10M 
   #endasm
   INTCON.GIE  = 1;          // enable  all interrupt
  }

#else

  void send_pulses()
  {
   unsigned char cnt;
   cnt = pulse_count;
   if (cnt == 0) return;
   INTCON.GIE  = 0;          // disable  all interrupt
   #asm
        BCF     03,5
        MOVF    cnt,w
        MOVWF   __WImage
DLUS10M 
        NOP             
        NOP             

        BSF     06,2
        NOP             
        NOP             
        NOP             
        NOP            
        NOP             

        BCF     06,2
        DECFSZ  __WImage 
        GOTO    DLUS10M 
   #endasm
   INTCON.GIE  = 1;          // enable  all interrupt
  }

#endif


#else  // 40KHZ

  void send_pulses()
  {
   unsigned char cnt;

   INTCON.GIE  = 0;                // disable  all interrupt
   for (cnt=0; cnt < pulse_count; cnt++)
   {
     tx_inp = 1;
     NOP();
     NOP();
     NOP();
     NOP();
     Delay_Us(10);
     tx_inp = 0;
     Delay_Us(8);
     NOP();
   }
   INTCON.GIE  = 1;                // disable  all interrupt

  }
#endif


/*================================================================
* Name: one_step()
*
* Description:  make one step to stepper motor without delay
*               
*================================================================*/
  void one_step()
  {

    PORTB &= 0x0F;
    if (sm_direct==STEP_FORWARD) {
      if (++phase_state == 4) phase_state = 0; 
      PORTB |= phase_pattern[phase_state];
      stepper_pos++;
    }
    else {
      if (--phase_state == -1) phase_state = 3; 
      PORTB |= phase_pattern[phase_state];
      stepper_pos--;
    }

    if (ref_sens==1)  
      m_ref_zone = OFF;
    else 
      sensor_pos = stepper_pos;
  }


/*================================================================
* Name: void do_steps(unsigned long sm_data)
*
* Description:  decode sm_data and make steps to stepper motor
*  sm_data format :
*    0..7 bits - radar angle (in steps); if 0 rotation is not need; 
*    8,9,10,11 - power in stepper motor coil AFTER action;
*    12th bit - direction 0 - FWRD, 1-BCWRD; 
*               
*================================================================*/
  void do_steps(unsigned long sm_data)
  {
    if ((sm_data & 0x1000)==0) 
      sm_direct = STEP_FORWARD;
    else  
      sm_direct = STEP_BACKWARD;
      
    sm_power = FULL_POWER;        // restore power
    stepper_power();

    uctmp1 = sm_data & 0x00FF;
    if (uctmp1 > 0)
      for (uctmp=0; uctmp < uctmp1; uctmp++) {
        one_step();
        Delay_Ms(step_delay);
      }

    sm_power = ((sm_data & 0x0F00) >> 6);        
    stepper_power();              // set idle currency in coil 

  }



/*================================================================
* Name: unsigned char search_ref_lo()
*
* Description:  look up for reference point; 
*               return count steps before reference zone
*================================================================*/
  unsigned char search_ref_lo() {
    sm_power = FULL_POWER;      // restore power
    stepper_power();

    for (uctmp1=0; uctmp1<205; uctmp1++)
    {
      one_step();
      Delay_Ms(FIX_STEP_TIME);
      if (m_ref_zone==ON)
      {
        sm_power = HOLD_POWER;      // restore power
        stepper_power();
        return uctmp1;
      }
    }

    sm_power = HOLD_POWER;      // restore power
    stepper_power();
    m_ref_fail = ON;
    return 0xFF;
  }


/*================================================================
* Name: search_ref()
*
* Description:  look up for reference point
*               
*================================================================*/
  void search_ref() {
    stepper_pos = 0;
    sensor_pos  = 0;
    r_param.X_field = 0;

    ultmp = (STEP_FORWARD << 12) | ((FULL_POWER << 8) & 0x0F00) | 25;
    do_steps(ultmp);
     
    ltmp = 0;
    sm_direct = STEP_FORWARD;
    m_ref_fail = OFF;
    uctmp1 = search_ref_lo();
    NOP();
    if (m_ref_fail == ON)
    {
      sm_direct = STEP_BACKWARD;
      m_ref_fail = OFF;
      uctmp1 = search_ref_lo();
      if (m_ref_fail == ON)
      {
         r_param.X_field = 1;
         return;
      }
    }

    sm_direct = STEP_BACKWARD;
    m_ref_fail = OFF;
    uctmp1 = search_ref_lo();
    if (m_ref_fail == ON)
    {
      r_param.X_field = 2;
      return;
    }

    if (uctmp1 < 5)
    {
      sm_direct = STEP_BACKWARD;
      m_ref_fail = OFF;
      uctmp1 = search_ref_lo();
    }

    ultmp = uctmp1;
    ltmp  = ultmp;
    r_param.Y_field = ultmp;
    ltmp -= 200;

    if (abs16(ltmp) > MAX_ER_REF)
    {
      r_param.X_field = 3;
      m_ref_fail=ON;
      return;
    }
    
    stepper_pos = 0;
    sensor_pos  = 0;
    r_param.X_field = sensor_pos;
    r_param.Y_field = stepper_pos;
  }


/*================================================================
* Name: init_var()
*
* Description:  Initialize all variables 
*               
*================================================================*/
  void init_var()
  {
    GSTATUS = 0;
    charge_level = 0;       // 11 volt
    pulse_count = 10;
    max_sample = MAX_AD_SAMPLE;
    phase_state = 0;
    sm_direct = STEP_FORWARD;
    burst_mode = OFF;
    stepper_pos = 0;
    step_delay=FIX_STEP_TIME;
  }

/*================================================================
* Name: init_hard()
*
* Description:  Initialize all hardware 
*               
*================================================================*/
  void init_hard()
  {
  // Initialize ports
   OPTION = 0b00000000;   // prescaler x/2 (1.25 Mhz on 10Mhz Fosc) 

   TMR0 = T0_DELAY;       // Load to TMR0 std const
   INTCON.T0IF = 0;       // TMR0 interrupt flag clear
   INTCON.T0IE = 0;       // TMR0 interrupt disable
   INTCON.GIE = 0;        // global interrupt disable

   PIE1 = 0x00;
   PIE2 = 0x00;

   TRISA = 0b11111111;     // 
   TRISB = 0b00000001;     // RESET - out
   TRISC = 0b10110000;     // SCL-in,P/C-out,SDO-in,TX-out,rx-in
                        
   PORTA = 0xFF;
   PORTB = 0x00;
   PORTC = 0xFF;
   tx_inp     = 0;          // transmitter to passive state
   sw_up      = 0;          // step-up converter MOSFET t-r off
   mot_enable = 0;          // stepper motor driver disabled

  // ---- Init ADC ----

   ADCON0 = 0b10000001;     // Fosc/32 , ch 0 select 
                            // Tad = 3,2us(10 Mhz) 
   ADCON1 = 0;              //  8 channels + Vdd  (000)

  
   // ---- Init USART to connect with PC ----
   
   UART_CONFIG = USART_ASYNCH_MODE & 
                 USART_HI_SPEED;
                 
   OpenUSART(UART_CONFIG, UART_DIV);
   PIE1.RCIE   = 0;         // disable UART interrupt
   PIE1.TXIE   = 0;

   INTCON.T0IE = 0;         // TMR0 interrupt disable
   INTCON.PEIE = 1;         // Enable Peripheral INT
   OPTION.INTEDG = 0;	    // Interrupt on falling edge of RB0/INT pin
   INTCON.INTE = 1;         // Enable  RB0/INT0 pin interrupt
   INTCON.GIE  = 1;         // global interrupt enable

   IF_dir = 0;		    // setting RS485 driver to input

   // ---- Init TMR1 to use time-out soft detector ----
   PIE1.TMR1IE = 0;         // TMR1 interrupt disable
   T1CON = 0b00110101;      // prescale Fosc:32, timer on,no syn (312.5Khz)

   // ---- Init TMR2 to PWM  ----
   PR2 = 0x3F;        // 78.125 KHz on 20 MHz
   T2CON.TMR2ON = 1;
   T2CON.T2CKPS0 = 0;       // prescale TMR2 1:1
   T2CON.T2CKPS1 = 0;
   
   T2CON.TOUTPS0 = 1;       // 4882 KHz sample step-up converter;
   T2CON.TOUTPS1 = 1;
   T2CON.TOUTPS2 = 1;
   T2CON.TOUTPS3 = 1;
   PIE1.TMR2IE = 1;
   
   CCP2CON = 0x0F;

   CCPR1L = 0x10;
   CCP1CON = 0x0F;

   // ---- stepper motor off
   sm_power = NO_POWER;
   stepper_power();
   

   // initialize pins and reset compass
   PC_pin = 1;
   SCLK_pin = 1;
   Delay_Ms(10);
   RESET_pin = 0;           
   Delay_Ms(30);
   RESET_pin = 1;
   
   // ---- Init stepper motor position
   search_ref();
  }

/****************************************************************
* Function Name: W_USART                                        *
* Return Value: void                                            *
* Parameters:  data : TXREG value                               *
* Description: This routine sends the 8 bit TX data.            *
****************************************************************/
void W_USART(unsigned char data)
{
 while (!TXSTA.TRMT); // Wait until buffer is empty
 TXREG = data;   // TX data
}

  
/*================================================================
* Name: WaitCommand()
*
* Description:  wait and receive command from communication
*               interface
*================================================================*/
void WaitCommand() {
   while (!TXSTA.TRMT);     // Wait until hardware transmit buffer is empty
   IF_dir = 0;		    // setting RS485 driver to input
RESTART_RCV:
   for (uctmp = 0; uctmp < SIZE_INCOMPACK ; uctmp++)
   {
     while(!PIR1.RCIF);             // Wait for data ready
     if (RCSTA.FERR || RCSTA.OERR){
        NOP();
        RCSTA.CREN = 0;
        NOP();
        RCSTA.CREN = 1;
     }
     uctmp1 = RCREG;             // Store RX data
     if (uctmp1 == SIGNATURE) goto RESTART_RCV;
     *(&c_param + uctmp) = uctmp1;
   }   
   Delay_Us(200);           // safe guard interval
   IF_dir = 1;		    // setting RS485 driver to output
   
}

/*================================================================
* Name: SendStatus()
*
* Description:  send radar status and command result
*               to communication interface 
*================================================================*/
void SendStatus() 
{
   IF_dir = 1;		    // setting RS485 driver to output
   r_param.g_status = GSTATUS;
   upd_crc8(SIGNATURE);
   for (uctmp = 0; uctmp < (SIZE_OUTGPACK-1); uctmp++) {
     uctmp1 = *(&r_param + uctmp);
      upd_crc8(uctmp1);
   }
   r_param.g_CRC = crc; 

   W_USART(SIGNATURE);
   for (uctmp = 0; uctmp < SIZE_OUTGPACK; uctmp++) {
     while (!TXSTA.TRMT);                   // Wait until buffer is empty
     TXREG = *(&r_param + uctmp);           // TX data
   }
}

  unsigned char sim_pass = 0;
  char sim_dir = 0;

void main() {
   

   init_var();
   init_hard();


   while(1) {
#ifdef STDL_TESTER_MODE
     c_param.cmd  = TEST_MODE;
#else
     WaitCommand();         // wait command
#endif
     crc = 0x00;
     switch (c_param.cmd){
       case REINIT_CMD:             // reinit all variables, hardware and
          init_var();               // setting sonar head in zero position
          init_hard();
          break;
       case GET_COMPASS_CMD:        // read data from compass and sent it to communication link
          break;
       case MEASURE_BURST_CMD:      // do steps with paramereter from <c_param.c_sm_data> and
          burst_mode = ON;          // make meashurement cycle.
          ultmp = trn_p2l(c_param.c_sm_data);
          do_steps(ultmp);
       case MEASURE_CMD:
          charge_level = c_param.c_charge_level * 2;
          pulse_count = c_param.c_pulse_count;
          max_sample = trn_p2l(c_param.c_max_sample);
          charge_tx();
          send_pulses();
          receive_echo();
          r_param.X_field = sensor_pos;
          r_param.Y_field = stepper_pos;
          burst_mode = OFF;         // clear burst-mode flag (not used now)
          break;
       case GO_REF_CMD:             // set motor to starting position
          search_ref();
          break;
       case DO_STEPS_CMD:           // do rotate stepper motor with parameters in <c_param.c_sm_data>
          ultmp = trn_p2l(c_param.c_sm_data);
          do_steps(ultmp);
          break;
       case GET_POS_CMD:
          r_param.X_field = sensor_pos;
          r_param.Y_field = stepper_pos;
          break;
       case SET_STEPTIME_CMD:
          step_delay = c_param.c_charge_level;
          break;
       case TEST_MODE:
          switch (sim_dir){
            case STEP_FORWARD  :
               c_param.c_sm_data = trn_l2p(0x0D01);
               if (++sim_pass > 199) sim_dir = STEP_BACKWARD;
               break;
            case STEP_BACKWARD :
               c_param.c_sm_data = trn_l2p(0x1D01);
               if (--sim_pass < 1) sim_dir = STEP_FORWARD;
               break;
          }
          burst_mode = ON;          // make meashurement cycle.
          ultmp = trn_p2l(c_param.c_sm_data);
          do_steps(ultmp);

#ifdef STDL_TESTER_MODE
          charge_level = 150;
          pulse_count  = 30; 
          max_sample   = 500;
#endif
          charge_tx();
          send_pulses();
          receive_echo();
          r_param.X_field = sensor_pos;
          r_param.Y_field = stepper_pos;
          burst_mode = OFF;         // clear burst-mode flag (not used now)
          break;

       case EXT_RESERVED0:
       case EXT_RESERVED1:
       case EXT_RESERVED2:       
       case EXT_RESERVED3:       
          continue;

       default:
         bad_cmd = TRUE;
     }
     SendStatus();
     NOP();
     NOP();
   }
} // end main


#include <usart14.lib>

