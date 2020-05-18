// SPI SLAVE // 2010.11.24 
// pin assigment for VERIMAX 
// byte_data_received included to i/o and port declaration. Pin-assigned to the 8 leds 
// spi lines assigned to conector 
// TODO: use structure of 'step' project to use internal oscillator for clock 
// 2010.11.29 // multi-module structure like 'step' project 
// project name must be the same with the name of the TOP-LEVEL-MODULE (...spi5)...QUARTUS HOWTO... 
// UFM block is now used for 'clk' no external needed 
// 22% of device used 
// TODO: TEST IT WITH SPI INPUT 
// 2010.12.02 
// Single row female header added to the handmade interface board 
// SPI pinout changed accordingly 
// Now can be breadboarded with mbed 
// tested ok 
// quadrature encoder module added and tested 
// expanded to 32bit 
// verimax on board 8leds display bits [15:8] 
// 2010.12.03 
// maximum frecuency of internal oscilator used (osc instead clk for clocking) 
// mbed code updated: SPI frequency increased from 100KHz to 1MHz 
// tested with 120 cpr, HRPG-ASCA#56 AVAGO encoder. 
// 480 pulses counted for one rotation 
// ****** It works in x4 mode :) ********* 
// Diplay format changed to display decimal position 
// Display frequency reduced to 30 cycles (30x10ms) 
// 2^32 is decimal 2 147 450 879 -> 2 milion+ pulses 
// LCD 2x16 added and tested 
// TODO: INDEX counter 16bit , INDEX resetable pulse counter 16 bit ----> 3 counters 64 bits 2x32bit transactions, 
// Interrupt lines when counters reset deactivated in next transaction 
// Get the register values from SPI if an Input is active (mbed saves them in NV-Memory---> absolute mode simulated) 
// Simulated absolute mode works if encoder is not moved during power-off 
// This is ***** EXTRA FUNCTIONALITY ****** 
// 
// 2011.01.03 
// Worked with AS5304 magnetic encoder, 25um(0.025mm) per count


//TOP LEVEL MODULE ------------------------------------------------------------------------------------------------ 
module spi5(SCK, MOSI, MISO, SSEL, LED, byte_data_received, quadA, quadB,index, position); 
// input SCK, SSEL, MOSI; 
// SPI input signals output MISO; 
// SPI output signal output LED; output [7:0] byte_data_received; 
// Received Byte output [31:0] position;

input quadA, quadB,index;

// To use the UFM block for clk wire osc,clk; wire oscena; assign oscena=1'b1;

/* Instantiation of ufmclk module. */ 		
ufmclk  ufmclk1(
       .oscena (oscena),
       .osc  (osc));
/* Instantiation of clk divider module.*/
divider d1(
       .osc  (osc),
       .clk  (clk));
/* Instantiation  of spi module.*/
spi spi(
       .osc (osc),
       .SCK (SCK),
       .MOSI (MOSI),
       .MISO (MISO),
       .SSEL (SSEL),
       .LED (LED),
.position (position), .byte_data_received (byte_data_received) );

/* Instantiation  of qdec module.*/
qdec q1(
       .osc (osc),
       .quadA (quadA),
       .quadB (quadB),
       .position (position),
       .A (A),
       .B (B),
.DIR (DIR), .index (index) );

endmodule //END OF TOP LEVEL MODULE --------------------------------------------------------------------------------------


module spi(osc, SCK, MOSI, MISO, SSEL, LED, byte_data_received,position); input osc; input SCK, SSEL, MOSI; output MISO; output LED; output [7:0] byte_data_received; input [31:0] position;

// sync SCK to the FPGA clock using a 3-bits shift register reg [2:0] SCKr; always @(posedge osc) SCKr <= {SCKr[1:0], SCK}; wire SCK_risingedge = (SCKr[2:1]==2'b01); 
// now we can detect SCK rising edges wire SCK_fallingedge = (SCKr[2:1]==2'b10); 
// and falling edges

// sync SSSEL reg [2:0] SSELr; always @(posedge osc) SSELr <= {SSELr[1:0], SSEL}; wire SSEL_active = ~SSELr[1]; 
// SSEL is active low wire SSEL_startmessage = (SSELr[2:1]==2'b10);
 // message starts at falling edge wire SSEL_endmessage = (SSELr[2:1]==2'b01); 
 // message stops at rising edge

// sync SMOSI reg [1:0] MOSIr; always @(posedge osc) MOSIr <= {MOSIr[0], MOSI}; wire MOSI_data = MOSIr[1];

// Receiving data from the SPI bus ----------------------------------------------------------- reg [2:0] bitcnt; 
// we handle SPI in 8-bits format, so we need a 3 bits counter to count the bits as they come in reg byte_received; 
// high when a byte has been received reg [7:0] byte_data_received; reg MISO; reg [31:0] shifter;

always @(posedge osc)

begin

 if(~SSEL_active)
 begin
   bitcnt <= 3'b000;
shifter = position[31:0];

 end
 else
 if(SCK_risingedge)
 begin
   bitcnt <= bitcnt + 3'b001;
   //byte_data_received <= {byte_data_received[6:0], MOSI_data};  
   // implement a shift-left register (since we receive the data MSB first)
byte_data_received[7:0] = position[15:8]; shifter = shifter <<1; MISO <= shifter[31];

 end
end always @(posedge osc) byte_received <= SSEL_active && SCK_risingedge && (bitcnt==3'b111); 
// we use the LSB of the data received to control an LED reg LED; always @(posedge osc) if(byte_received) LED <= byte_data_received[0];


endmodule


/*******************************************************************************************

  Module ufmclk is the module 
  which gets the clock from the oscillator 
******************************************************************************************/
`timescale 1 ps / 1 ps //synopsys translate_on module ufmclk_altufm_osc_7p3 ( osc, oscena) /* synthesis synthesis_clearbox=1 */; output osc; input oscena;

wire wire_maxii_ufm_block1_osc;

maxii_ufm maxii_ufm_block1 ( .arclk(1'b0), .ardin(1'b0), .arshft(1'b0), .bgpbusy(), .busy(), .drclk(1'b0), .drdout(), .drshft(1'b0), .osc(wire_maxii_ufm_block1_osc), .oscena(oscena) `ifdef FORMAL_VERIFICATION `else 
// synopsys translate_off `endif , .drdin(1'b0), .erase(1'b0), .program(1'b0) `ifdef FORMAL_VERIFICATION `else 
// synopsys translate_on `endif // synopsys translate_off , .ctrl_bgpbusy(), .devclrn(), .devpor(), .sbdin(), .sbdout() 
// synopsys translate_on ); defparam maxii_ufm_block1.address_width = 9, maxii_ufm_block1.osc_sim_setting = 180000, maxii_ufm_block1.lpm_type = "maxii_ufm"; assign osc = wire_maxii_ufm_block1_osc; endmodule //ufmclk_altufm_osc_7p3 //VALID FILE


// synopsys translate_off `timescale 1 ps / 1 ps // synopsys translate_on module ufmclk ( oscena, osc)/* synthesis synthesis_clearbox = 1 */;

input oscena; output osc;

wire sub_wire0; wire osc = sub_wire0;

ufmclk_altufm_osc_7p3 ufmclk_altufm_osc_7p3_component ( .oscena (oscena), .osc (sub_wire0));

endmodule


/**************************************************************************************************

Module divider divides the clock from ufmclk to produce
a clock suitable to drive the motor controller
*************************************************************************************************/
module divider (osc, clk); input osc;

  	output clk;
  	reg clk;
     reg [16:0] count;
always @( posedge osc)
     begin
        count = count + 1;
        //clk = count[16];
clk = count[0];

     end
endmodule

//---------------------------------------------------quadrature decoder

module qdec(osc, quadA, quadB, position,A,B,DIR,index);

input osc, quadA, quadB,index; output [31:0] position; output A,B,DIR;

reg [2:0] quadA_delayed, quadB_delayed; reg A,B,DIR; reg [31:0] position;

always @(posedge osc) quadA_delayed <= {quadA_delayed[1:0], quadA}; always @(posedge osc) quadB_delayed <= {quadB_delayed[1:0], quadB};

wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2]; wire count_direction = quadA_delayed[1] ^ quadB_delayed[2];


//----------->always @(posedge quadA) position <= position+1; //always @(posedge quadB) cnt[4] <= 0;

always @(posedge osc) begin

    //position = position+1;
 if(count_enable)
 begin
//position = position+1;

  if(count_direction) position<=position+1; else position<=position-1;
A <= quadA_delayed[2];

   B <= quadB_delayed[2];
   DIR <= count_direction;
 end
end

endmodule

//********************************** END OF PROGRAM.**********************************************
