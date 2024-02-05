/*
 * Copyright 2020 Claude Schwarz
 * Copyright 2020 Niklas Ekström - rewrite in Verilog
 * Copyright 2024 David Sharp - Lots of added comments and an attempt to increase compatibility.
*/


// Revision History
// v1.0.0 : 2024-01-21 : Added lots of comments to understand what this is all doing. No changes. 
			
/* Basically (as I understand it). This takes 16 data lines and 24 (32) address lines from the 68k and multiplexes them to/from the Pi over 16 GPIO pins.
*  There are 3 output pins from the CPLD to the Pi which communicate IPL, reset status and transmission in progress.
*  There is a Clock pin from the Pi (this is one of the functions of the Pi where you can define a clock) which everything is clocked from.
*  Finally there are 4 control pins from the Pi. One for Read, one for Write and two for what part of the cycle we're in (ie what the 16 pins are currently doing).
*  The four states of the cyle are defined as
* REG_DATA - Read or write data depending on the status of the PI_RD or PI_WR pins
* REG_ADDR_LO - Select the A1-A15 range. Note that A0 is an oddity that doesn't go past the CPLD as it's only used in the 68k to select lower or upper data strobes. As we're faking a 68k though we need it.
* REG_ADDR_HI - Select the A16-A23 (technically A16-A31 but 24-31 don't exist on a 68k!) range.
* REG_STATUS - This is a 16bit "status" register however all it contains is INIT and RESET. It could do much more!
* In addtion the last byte of REG_ADDR_HI is utilised to pass the R/W bit (and in the ST version the FC status).

*  There are four flip-flops (latches) and two buffers (line drivers).
*  The two buffers are there to allow 5V CMOS logic to connect to the 3.6V I/O on the CPLD.
*  The four flip-flops take in data/address on their input and then push it out on the CLK.
*  There are separate flip-flops for Data in (RD) and out (WR) but not for Address. This is because the processor always drives the address. It "activates" a device by selecting the appropriate address and then
*  reading from or writing the appropriate data.
*/




module pistorm(


    // Pi connected pins. 
    output reg      PI_TXN_IN_PROGRESS, // GPIO0 - Labelled in the schematic as AUX0     - Pi pin 27 / ID_SD
    output reg      PI_IPL_ZERO,        // GPIO1 - Labelled in the schematic as AUX1     - Pi pin 28 / ID_SC
    input   [1:0]   PI_A,       // GPIO[3..2]    - Labelled in the schematic as SA[1..2] - Pi pins (5 / SCL1, I2C), (3 / SDA1, I2C)
    input           PI_CLK,     // GPIO4         - Labelled in the schematic as PICLK    - Pi pin 7 / GPIO_GCLK
    output reg      PI_RESET,   // GPIO5         - Labelled in the schematic as SA0      - Pi pin 29 
    input           PI_RD,      // GPIO6         - Labelled in the schematic as SOE      - Pi pin 31
    input           PI_WR,      // GPIO7         - Labelled in the schematic as SWE      - Pi pin 26 / SPI_CE1_N
    inout   [15:0]  PI_D,       // GPIO[23..8]   - Labelled in the schematic as SD[0..15]  

    // SN74LVC16374DGGR connected pins
/* 16-BIT EDGE-TRIGGERED D-TYPE FLIP-FLOP WITH 3-STATE OUTPUTS
*       https://www.ti.com/lit/ds/symlink/sn74lvc16374.pdf
*	The SN74LVC16374 is particularly suitable for implementing buffer registers, I/O ports, bidirectional bus drivers, and working registers.
*	It can be used as two 8-bit flip-flops or one 16-bit flip-flop.  On the positive transition of the clock (CLK) input, the Q outputs of the flip-flop take on the logic levels set up at the data (D) inputs.
*	low logic levels) or a high-impedance state. In the high-impedance state, the outputs neither load nor drive the
*	bus lines significantly. The high-impedance state and the increased drive provide the capability to drive bus lines
*	without need for interface or pullup components.
*	OE does not affect internal operations of the flip-flop. Old data can be retained or new data can be entered while
*	the outputs are in the high-impedance state.
*	To ensure the high-impedance state during power up or power down, OE should be tied to VCC through a pullup
*	resistor; the minimum value of the resistor is determined by the current-sinking capability of the driver.
*	Active bus-hold circuitry is provided to hold unused or floating data inputs at a valid logic level.
*/

// In PiStorm the SN74LVC1367 is used to multiplex Data and Address lines into the Pi. Although only 24bit wide address bus for the A500 inplementation, U6B has 8 bits free for a full 32bit wide address bus. 
// Unfortunately these cannot be used as that last byte is used to send R/W (and FC for the ST). This is probably only a problem for the A4000 as that's Z3 address space.


    output reg      LTCH_A_0,   //               - Labelled in the schematic as ADDR_LTCH_0 - This is the CLK on U2A, first 8 bits of Address bus.
    output reg      LTCH_A_8,   //               - Labelled in the schematic as ADDR_LTCH_8 - This is the CLK on U2B, second 8 bits of Address bus
    output reg      LTCH_A_16,  //               - Labelled in the schematic as ADDR_LTCH_16 - This is the CLK on U6A, third 8 bits of Address bus
    output reg      LTCH_A_24,  //               - Labelled in the schematic as ADDR_LTCH_24 - This is the CLK on U6B, forth 8 bits of Address bus (not used on A500/2000). 
    output reg      LTCH_A_OE_n,  //             - Labelled in the schematic as ADDR_OE - This is the output enable (OE) on U2 & U6
    output reg      LTCH_D_RD_U,  //             - Labelled in the schematic as DATA_RD_LTCH_HB - This is the CLK on U3B, second 8 bits of Data bus
    output reg      LTCH_D_RD_L,  //             - Labelled in the schematic as DATA_RD_LTCH_LB - This is the CLK on U3A, first 8 bits of Data Bus
    output reg      LTCH_D_RD_OE_n,  //          - Labelled in the schematic as DATA_RD_OE - This is the output enable (OE) on U3 
    output reg      LTCH_D_WR_U,  //             - Labelled in the schematic as DATA_WR_LTCH_HB - This is the CLK on U5B, second 8 bits of Data bus
    output reg      LTCH_D_WR_L,  //             - Labelled in the schematic as DATA_WR_LTCH_LB - This is the CLK on U5A, first 8 bits of Data bus
    output reg      LTCH_D_WR_OE_n,  //          - Labelled in the schematic as DATA_WE_OE - This is the output enable (OE) on U5

/*
* Theory of operations (AIUI)
*
* U3 is a latch/flip-flop to transfer data to the Pi. Data (D0-D15) is passed through when DATA_RD_OE is set on the clocks (DATA_RD_LTCH_LB and DATA_RD_LTCH_HB for D0-D7 and D8-D15 respectively).
* U5 is a latch/flip-flop. to transfer data from the Pi to the Amiga. Data (D0-D15) is passed through when DATA_WR_OE is set on the clocks (DATA_WR_LTCH_LB and DATA_WR_LTCH_HB for D0-D7 and D8-D15 respectively).
* U2 is a latch/flip-flop. to transfer address to the Amiga. Address (A1-A15) is passed through when ADDR_OE is set on the clocks (ADDR_LTCH_0 and ADDR_LTCH_8 for A1-A7 and A8-A15 respectively).
* U6 is a latch/flip-flop. to transfer address to the Amiga. Address (A16-A31 [only A23 for the A500/2000]) is passed through when ADDR_OE is set on the clocks (ADDR_LTCH_16 and ADDR_LTCH_24 for A16-23 and A24-A31 respectively).
 */



// Direct connected inputs - TBA move these round

    input           M68K_CLK, // CLK in the schematic (sometimes 7MHz on Commodore schematics). Processor clock - Can also be generated by /C1 XNOR /C3. Note that although directly connected to the CPLD (which is a 3.6V device) this doesn't exceed voltage limits. Pin 62 on U4

// SN74CBTD3384PWLE connected pins
/*
TBA - Insert description of SN74CBTD3384PWLE (its some kind of line driver / buffer that allows 5/3.3v
*/

    output  reg [2:0] M68K_FC, // FC0-FC2 in the schematic. Processor status outputs which can be used to determine the internal state of the 68k any time /AS is asserted. Pins 22,21 & 14 on U1.
    output reg      M68K_AS_n, // AS in the schematic. Address Strobe - Falling edge indicates addresses are valid. Pin 14 on U7.
    output reg      M68K_UDS_n, // UDS in the schematic. Upper Data Strobe. Pin  17 on U7.
    output reg      M68K_LDS_n, // LDS in the schematic. Lower Data Strobe. Pin 18 on U7.
    output reg      M68K_RW,    // R/W in the schematic. Read Write output - high for read (or internal cycle), low for a write cycle. Pin 21 on U7.

    input           M68K_DTACK_n, // DTACK in the schematic. Data Transfer ACKnowledge. Pin 22 on U7.
    input           M68K_BERR_n,  // BERR in the schematic (sometimes BEER on Commodore schematics). Bus ERRor. Pin 8 on U1.

    input           M68K_VPA_n, //Valid Peripheral Address - Input to the 68k indicating the address has selected a 6800 or 6502 style peripheral.
    output reg      M68K_E, // E in the schematic. 68k generated E clock used for 6800 family peripherals or 6502 peripherals. Required for CIAs. - Always generated regardless of the state of the bus or coprocessor...
                            /* This is somewhat of a problem because two E-Clocks generated at the same time causes a problem. There are multiple ways round this. 
			    * The A1200 & A600 generate the E-clock from Gayle. It's not connected on the processor. Gayle however also deals with 6800 devices (ie the CIA) so we can ignore it totally
			    * The A2630 *either* generates it *or* leaves it to come from the onboard 68k depending if it's an A2000 or a B2000. It needs to sync.
			    * TBA we should do the same!
			    */

    output reg      M68K_VMA_n, //VMA in the schematic. Valid Memory Address - Output from the 68k indicating a valid address for 6800 style peripheral devices in response to a /VPA input. Pin 3 on U1.

    input   [2:0]   M68K_IPL_n, //LPL0-IPL2 in the schematic. Encoded System Interupts (Interupt Priority Level). Pins 11, 17 & 18 on U1.

    inout           M68K_RESET_n, // RESET in the schematic (sometimes RST on some schematics). System Reset - Bidirectonal. Pin 3 on U7.
    inout           M68K_HALT_n, // HALT in the schematic.  System HALT - Bidirectional - TBA - Should tri-state bus when asserted. For a complete system reset the 68000 looks for both to be asserted. Pin 4 on U7.

    input           M68K_BR_n,  // BR in the schematic. Bus Request - TBA - Input to the 68k to request mastership of the local bus. Pin 7 on Y7.
    output reg      M68K_BG_n, // BG in the schematic.  Bus Grant - TBA - Asserted in response to a /BR to indicate that the 68k will fully relinquish the bus at the end of this cycle. Pin 11 on U7.
    input           M68K_BGACK_n, // BGACK on schematic. Bus Grant ACKnowledge - TBA - Any device that receives a bus grant from the 6800 should assert this signal as long as DMA continues. Pin 8 on U7.
// Direct connected inputs - TBA move these round

    input           M68K_C1, //C1 Clock. 3.58MHz synced to the falling edge of the 7.16MHz system clock. Also called /CCK. Pin 99 on U4
    input           M68K_C3, //C3 Clock. 3.58MHz synced to the rising edge of the 7.16MHz system clock. Also called /CCKQ. Pin 100 on U4
    input           CLK_SEL //Select if to generate clock from C1/C3 or use provided clock. Pin 51 on U4

  );

  wire c200m = PI_CLK; //define clock from the RPi
  reg [2:0] c7m_sync; //define variable c7m_sync (3 bits)
//  wire c7m = M68K_CLK;
  wire c7m = c7m_sync[2]; //define variable as equal to the register.
  wire c1c3_clk = !(M68K_C1 ^ M68K_C3); // Generate 7MHz clock from C3 XNOR C1. In theory we could also do this in a "sidecar" version for the A500 which would avoid board mods on < Rev8

  // 

  localparam REG_DATA = 2'd0; // define REG_DATA as 2 bit wide decimal 0
  localparam REG_ADDR_LO = 2'd1; // define REG_ADDR_LO as 2 bit wide decimal 1
  localparam REG_ADDR_HI = 2'd2; // define REG_ADDR_HI as 2 bit wide decimal 2
  localparam REG_STATUS = 2'd3; // define REG_STATUS as two bit wide decimal 3

  // Setup - initial block runs once. 

  initial begin
    PI_TXN_IN_PROGRESS <= 1'b0; // Initialise PI_TXN_IN_PROGRESS TO 0. This tells the PI what we're doing
    PI_IPL_ZERO <= 1'b0; // TBA this passes IPL0 to the Pi - Initialise IPL0 to 0. 

    PI_RESET <= 1'b0; // Initialise RESET state to 0

    M68K_FC <= 3'd0; // Initialise FC0-FC2 to 000 - TBA this should change but doesn't. ST pulls this in REG_ADDR_HI bits 15:13.

    M68K_RW <= 1'b1; // Initialise R/W to 1

    M68K_E <= 1'b0; // Initialise E clock to 0
    M68K_VMA_n <= 1'b1; // Initialise VMA to 1

    M68K_BG_n <= 1'b1; // Initialise BG to 1.  
  end

  reg [1:0] rd_sync; // define rd_sync variable (2 bits)
  reg [1:0] wr_sync; // define wr_sync variable (2 bits)


// On the rising edge of c200m (the PI clock) 

  always @(posedge c200m) begin // on every PI bus cycle update these registers with the current status of PI_RD or PI_WR
    rd_sync <= {rd_sync[0], PI_RD};
    wr_sync <= {wr_sync[0], PI_WR};
  end

  wire rd_rising = !rd_sync[1] && rd_sync[0]; // define some wires to detect if PI_RD is rising or falling
  wire wr_rising = !wr_sync[1] && wr_sync[0]; // define some wires to detect if PI_WR is rising or falling

  reg [15:0] data_out; // define data_out variable (16 bits). This is data that gets passed back to the Pi
  assign PI_D = PI_A == REG_STATUS && PI_RD ? data_out : 16'bz; // If we're on "REG_STATUS" mode *AND* PI_RD then set PI_D to be data_out otherwise set high impedance. This lets us transfer data back to the Pi...

// On the rising edge of c200m (the PI clock) 
  always @(posedge c200m) begin
    if (rd_rising && PI_A == REG_STATUS) begin
      data_out <= {ipl, 13'd0}; // Set the data back to the Pi to contain IPL0-3 and 13 bits of nothing...ST firmware also passes reset here and uses the reset pin for BERR. It seems a waste not to use this more...
    end
  end

  reg [15:0] status;
  wire reset_out = !status[1]; // Pick up the reset from the status passed in the REG_STATUS part of the cycle.

  assign M68K_RESET_n = reset_out ? 1'b0 : 1'bz; // Do a RESET if called for from the Pi.
  assign M68K_HALT_n = reset_out ? 1'b0 : 1'bz; // A full reset also needs a HALT so we also do that.

  reg op_req = 1'b0; // Do we do stuff?
  reg op_rw = 1'b1; // Is this a write or read cycle?
  reg op_uds_n = 1'b1; // Is this UDS?
  reg op_lds_n = 1'b1; // Is this LDS?

//  On any change this passes data to the latch (the @(*) is shorthand for "all nets and variables that are read by the statement")
// I had problems getting my head round this section below so I wrote some code to run in the simulator which generated the following truth table.
// TBA - Check this out further. 
// Why is LTCH_D_RD_OE_n enabled when on REG_STATUS?
// Is this a timing thing?

/*

PI_A
00 = REG_DATA
01 = REG_ADDR_LO
10 = REG_ADDR_HI
11 = REG_STATUS

## Grouped by PI_WR and PI_WR

PI_A    PI_WR   PI_RD   Non-Zero (all others zero)
00      0       0       LTCH_D_RD_OE_n
01      0       0       LTCH_D_RD_OE_n
10      0       0       LTCH_D_RD_OE_n
11      0       0       LTCH_D_RD_OE_n

00      0       1
01      0       1       LTCH_D_RD_OE_n
10      0       1       LTCH_D_RD_OE_n
11      0       1       LTCH_D_RD_OE_n

00      1       0       LTCH_D_WR_U,    LTCH_D_WR_L,    LTCH_D_RD_OE_n
01      1       0       LTCH_A_0,       LTCH_A_8,       LTCH_D_RD_OE_n
10      1       0       LTCH_A_16,      LTCH_A_24,      LTCH_D_RD_OE_n
11      1       0                                       LTCH_D_RD_OE_n

00      1       1       LTCH_D_WR_U,    LTCH_D_WR_L
01      1       1       LTCH_A_0,       LTCH_A_8,       LTCH_D_RD_OE_n
10      1       1       LTCH_A_16,      LTCH_A_24,      LTCH_D_RD_OE_n
11      1       1       LTCH_D_RD_OE_n


## Grouped by PI_

PI_A    PI_WR   PI_RD   Non-Zero (all others zero)
REG_DATA #############################################################
00      0       0       LTCH_D_RD_OE_n
00      0       1
00      1       0       LTCH_D_WR_U,    LTCH_D_WR_L,    LTCH_D_RD_OE_n
00      1       1       LTCH_D_WR_U,    LTCH_D_WR_L
REG_ADDR_LO ##########################################################
01      0       0                                       LTCH_D_RD_OE_n
01      0       1                                       LTCH_D_RD_OE_n
01      1       0       LTCH_A_0,       LTCH_A_8,       LTCH_D_RD_OE_n
01      1       1       LTCH_A_0,       LTCH_A_8,       LTCH_D_RD_OE_n
REG_ADDR_HI ##########################################################
10      0       0                                       LTCH_D_RD_OE_n
10      0       1                                       LTCH_D_RD_OE_n
10      1       0       LTCH_A_16,      LTCH_A_24,      LTCH_D_RD_OE_n
10      1       1       LTCH_A_16,      LTCH_A_24,      LTCH_D_RD_OE_n
REG_STATUS ###########################################################
11      0       0       LTCH_D_RD_OE_n
11      0       1       LTCH_D_RD_OE_n
11      1       0       LTCH_D_RD_OE_n
11      1       1       LTCH_D_RD_OE_n
*/



  always @(*) begin
    LTCH_D_WR_U <= PI_A == REG_DATA && PI_WR;
    LTCH_D_WR_L <= PI_A == REG_DATA && PI_WR;

    LTCH_A_0 <= PI_A == REG_ADDR_LO && PI_WR;
    LTCH_A_8 <= PI_A == REG_ADDR_LO && PI_WR;

    LTCH_A_16 <= PI_A == REG_ADDR_HI && PI_WR;
    LTCH_A_24 <= PI_A == REG_ADDR_HI && PI_WR;

    LTCH_D_RD_OE_n <= !(PI_A == REG_DATA && PI_RD);
  end

  reg a0; //a0 is normally only used internally to the 68k to indicate upper or lower of course we're faking a 68k so we need it.


// On the rising edge of c200m (the PI clock) sync to on-board clock which is either generated from C1/C3 or provided.

  always @(posedge c200m) begin
    c7m_sync <= {c7m_sync[1:0], (CLK_SEL?M68K_CLK:c1c3_clk)};
  end

  wire c7m_rising = !c7m_sync[2] && c7m_sync[1];
  wire c7m_falling = c7m_sync[2] && !c7m_sync[1];

  reg [2:0] ipl; //define ipl variable (3 bits)
  reg [2:0] ipl_1; // define ipl_1 variable (3 bits)
  reg [2:0] ipl_2; // define ipl_2 variable (3 bits)

//  On the rising edge of c200m (the PI clock) we mess about with IPL. TBA need to work out how this works. 
  always @(posedge c200m) begin
    if (c7m_falling) begin
      ipl_1 <= ~M68K_IPL_n;
      ipl_2 <= ipl_1;
    end

    if (ipl_2 == ipl_1)
      ipl <= ipl_2;

    PI_IPL_ZERO <= ipl == 3'd0;
  end

//  On the rising edge of c200m (the PI clock) we pass reset to the Pi if it has been asserted from the Amiga side.

  always @(posedge c200m) begin
    PI_RESET <= reset_out ? 1'b1 : M68K_RESET_n;
  end

  reg [3:0] e_counter = 4'd0; // define e_counter variable (4 bits) and set to 0

// On the falling edge of c7m (the Amiga clock) we count up the e_counter, reseting when it gets to 9 (1001). This isn't the same behavour as the real 68k as there is no guarantee that the E-Clock will be syncronised to the system clock.
// This also means it doesn't *matter* if it's syncronised or not and why we can get away with an external clock...but need to think about VMA/VPA if we do so (ie take E as an input). 

  always @(negedge c7m) begin
    if (e_counter == 4'd9)
      e_counter <= 4'd0;
    else
      e_counter <= e_counter + 4'd1;
  end

// On the falling edge of c7m (the Amiga clock) flip the E clock on/off depending on the counter. This gives a E clock which should be six clocks low, four clocks high..
// NOTE - The logic is the wrong way round in the A500/A2000 Technical reference manual!. Should be (according to Motarola) 6 high, 4 low but the TRM gives it as 4 high, 6 low.
// TBA - need to make this variable depending on if A500/2000 or B2000


  always @(negedge c7m) begin
    if (e_counter == 4'd9)
      M68K_E <= 1'b0;
    else if (e_counter == 4'd5)
      M68K_E <= 1'b1;
  end

// There are 8 68000 State Machine states, S0 through to S7
// See https://www.nxp.com/docs/en/reference-manual/MC68000UM.pdf page 5-3

  reg [2:0] state = 3'd0; // Set variable stat (3 bits) to 000 - 
  reg [2:0] PI_TXN_IN_PROGRESS_delay; // define variable PI_TX_IN_PROGRESS_delay (3 bits).

// On the rising edge of c200m (Pi clock) do stuff 

  always @(posedge c200m) begin 

// So, if PI_WR is "rising" then we look at PI_A and do stuff

    if (wr_rising) begin 
      case (PI_A)
        REG_ADDR_LO: begin // PI_A is 01
          a0 <= PI_D[0]; // pick up a0 as a special case.
          PI_TXN_IN_PROGRESS <= 1'b1; // Tell the PI we're doing stuff
        end
        REG_ADDR_HI: begin // PI_A is 10
          op_req <= 1'b1; // We need to do stuff so set op required
          op_rw <= PI_D[9]; // Is this a write or read operation
          op_uds_n <= PI_D[8] ? a0 : 1'b0; // Set UDS based off a0
          op_lds_n <= PI_D[8] ? !a0 : 1'b0; // Set LDS based off a0
        end
        REG_STATUS: begin // PI_A is 11
          status <= PI_D; // Grab the status from the PI. Of which only INIT and RESET is used. 
        end
      endcase
    end

// Loop through the eight 68000 machine states... of which there can actually be 10 or more with inserted wait states...
// See A500 Plus Service Manual page 2-4 for example of the added wait states.
// Normally the bus cycle terminates in S7 except when BERR is asserted in the absence of DTACK. In that case, the bus cycle terminates one clock cycle later in S9.
// Also need to have wait/delay states if no BERR *OR* DTACK. The ST version passes BERR to the PI via the reset line. 
// TBA - This section deals with all three possible states (Read, Write or Read-Modify-Write) in one loop. As such it's a little bit hard to follow. 

    case (state)
/* STATE 0 
*
* Read  - The read cycle starts in state 0 (S0). The processor places valid function codes on FC0–FC2 and drives R/W high to identify a read cycle.
* Write - The write cycle starts in S0. The processor places valid function codes on FC2–FC0 and drives R/W high (if a preceding write cycle has left R/W low).
* Read-Modify-Write - Same as the read cycle.
* DS Note - This appears to be nonsensical as it can't be both high for read & write. What I think it means however is regardless of which cycle we always *start* with R/W high. 
*/
// TBA - no function codes! As these relate however to internal functions that we need from the Pi they need to get out to the CPLD. 

/* FC2  FC1  FC0 	Address Space Type
*  Low  Low  Low 	(Undefined, Reserved)
*  Low  Low  High 	User Data
*  Low  High Low 	User Program
*  Low  High High 	(Undefined, Reserved)
*  High Low  Low 	(Undefined, Reserved)
*  High Low  High 	Supervisor Data
*  High High Low 	Supervisor Program
*  High High High 	CPU Space
*/

// TBA - The more I look at this the more I think I'm going to rewrite the state machine. Claude said that this version was the experimental version that overwrote the full state machine version.

      3'd0: begin // 68K State S0 
        M68K_RW <= 1'b1; // S7 -> S0 - Set R/W High to indicate a read cycle. 
//        if (c7m_falling) begin
//          if (op_req) begin
//TBA - Fairly certain this should be 3'd1 not 2'd1! TBA - change to 4 bits.
            state <= 2'd1; // Change to state S1
//TBA - Should set valid function codes on FC0-FC2 (see above and previous reference to MC68000 manual page 4-3
//          end
//        end
      end

// STATE 1 - Entering state 1 (S1), the processor drives a valid address on the address bus. So I think this is done elsewhere which makes it hard to follow logically.
// I think actually this is an extension of state 0 
// We seem to change to the next state on clock rising which is correct. 
// TBA - I think this is wrong too.

      3'd1: begin // 68K State S1 - TBA 4 bits!
        if (op_req) begin
          if(c7m_rising) begin
            state <= 3'd2; // Change to state S2 - TBA 4 bits!
          end
        end
      end
/* STATE 2 
* Read - On the rising edge of state 2 (S2), the processor asserts AS* and LDS*, or DS.
*      - According to the A500/A2000 Technical reference manual it asserts LDS* or UDS* but only for reads (writes are delayed until S4). See page 47 and then 48 for diagrams. 
* Write - On the rising edge of S2, the processor asserts AS and drives R/W low.
*/

      3'd2: begin // 68K State S2 - TBA 4 bits!
        M68K_RW <= op_rw; // S1 -> S2 
        LTCH_D_WR_OE_n <= op_rw;
        LTCH_A_OE_n <= 1'b0;
        M68K_AS_n <= 1'b0;
        M68K_UDS_n <= op_rw ? op_uds_n : 1'b1; // Set UDS to op_rw if op_uds_n is high
        M68K_LDS_n <= op_rw ? op_lds_n : 1'b1; // Set LDS to op_rw if op_lds_n is high


// TBA - the manual states on the rising edge rather than the falling edge - need to check this out. Are we actually changing states late? 
        if (c7m_falling) begin
          M68K_UDS_n <= op_uds_n;
          M68K_LDS_n <= op_lds_n;
          state <= 3'd3; // Change to state S3
        end
      end

// STATE 3 - During state 3 (S3), no bus signals are altered.
// DS - This is where I'm slightly confused about why this code does this here. I think actually this should be State S4 (I mean it works so it's not wrong). 
// TBA - Consider fix to clarify states... or leave because it works. 

      3'd3: begin // 68K State S3
        op_req <= 1'b0;
        if(c7m_rising) begin
          if (!M68K_DTACK_n || (!M68K_VMA_n && e_counter == 4'd8)) begin // If DTACK isn't asserted *OR* (VMA isn't asserted *AND* e_counter is 8 [of 0-9] TBA may need to change this if E-Clock removed to sync...
            state <= 3'd4; // Change to state S4
            PI_TXN_IN_PROGRESS_delay[2:0] <= 3'b111; // Set PI_TXN_IN_PROGRESS_delay to 111 - TBA Why? Need to work out this bit.
          end
          else begin
            if (!M68K_VPA_n && e_counter == 4'd2) begin // Otherwise if VPA isn't asserted *AND* e_counter is 2 [of 0-9] set VMA.
              M68K_VMA_n <= 1'b0;
            end
          end
        end
      end


// STATE 4 During state 4 (S4), the processor waits for a cycle termination signal (DTACK or BERR) or VPA, an M6800 peripheral signal. When VPA is asserted during S4, the cycle becomes a peripheral cycle (refer to Appendix B M6800 Peripheral Interface). If neither termination signal is asserted before the falling edge at the end of S4, the processor inserts wait states (full clock cycles) until either DTACK or BERR is asserted.

      3'd4: begin // 68K State S4
        PI_TXN_IN_PROGRESS_delay <= {PI_TXN_IN_PROGRESS_delay[1:0],1'b0}; // set PI_TXN_PROGRESS_delay bit 0 to 0
        PI_TXN_IN_PROGRESS <= PI_TXN_IN_PROGRESS_delay[2]; // Set PI_TXN_PROGRESS to PI_TXN_IN_PROGRESS_delay bit 2  
        LTCH_D_RD_U <= 1'b1; // Set the latches to read upper
        LTCH_D_RD_L <= 1'b1; // Set the latches to read lower
        if (c7m_falling) begin
          state <= 3'd5; // Change to state S5
          PI_TXN_IN_PROGRESS <= 1'b0; // set PI_TXN_IN_PROGRESS to 0
        end
      end

// STATE 5 During state 5 (S5), no bus signals are altered.
// DS - This is what I would expect S3 to be like. 


      3'd5: begin // 68K State S5
        LTCH_D_RD_U <= 1'b0; // Unset latches to read upper
        LTCH_D_RD_L <= 1'b0; // Unset latches to read lower
        if (c7m_rising) begin
          state <= 3'd6; // Change to state S6 - 
        end
      end

// STATE 6 During state 6 (S6), data from the device is driven onto the data bus.
       
      3'd6: begin // 68K State S6
        if (c7m_falling) begin
          M68K_VMA_n <= 1'b1; // Set VMA to 1
          state <= 3'd7; // Change to state S7
        end
      end
      
// STATE 7 On the falling edge of the clock entering state 7 (S7), the processor latches data from the addressed device and negates AS and LDS, or DS. At  the rising edge of S7, the processor places the address bus in the highimpedance state. The device negates DTACK or BERR at this time.
      3'd7: begin // 68K State S7
        LTCH_D_WR_OE_n <= 1'b1; // Set latches to enable write
        LTCH_A_OE_n <= 1'b1; // Set latches to enable address
        M68K_AS_n <= 1'b1; // Unset AS* 
        M68K_UDS_n <= 1'b1; //Unset UDS
        M68K_LDS_n <= 1'b1; //Unset LDS
//        if(c7m_rising) begin
//          M68K_RW <= 1'b1; // S7 -> S0
// TBA This should detect if BERR is asserted in the absence of DTACK and terminate one clock cycle later in S9. In which case we don't jump to S0!
          state <= 3'd0; // Change to state S0
//        end
      end
    endcase
  end

endmodule
