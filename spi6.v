// spi6.v

// SPI SLAVE 
// 2010.11.24 
// pin assigment for VERIMAX 
// byte_data_received included to i/o and port declaration. Pin-assigned to the 8 leds 
// spi lines assigned to conector 
// TODO: use structure of 'step' project to use internal oscillator for clock 
// 2010.11.29 
// multi-module structure like 'step' project 
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

//********************************************************************************************************************* 
// 2011.04.26 
// Project spi5 copied to spi6, succesfully compiled 
// DONE: Add second encoder (qdec) module 
// 2nd encoder module created 
// byte_data_received **WAS** included to i/o and port declaration(spi5). Pin-assigned to the 8 leds. **NOW REMOVED** 
// Compiled succesfully. SPI now has to transmit the 32bits of the new encoder 
// Module spi updated to transmit 2x32bit counters [=64 bits], = 4x16bit SPI transactions. Compiled succesfully 
// TODO: Reduce PINS used, ASSIGN the new PINS

// 2011.04.27 
// position and position2 use 64 PINS! 
// As position,position2 are used only for inter-module comunication and not needed as PINS---> 
// position, position2, removed from TOP LEVEL MODULE (**mark1**)---> 
// --->PINS reduced to 11, Total logic elements 45 / 240 ( 19 % ) from 48% before. 
// TODO: CHECK if still works 
// 4 LEDS for 1st Encoder, and 4 LEDS for 2nd Encoder---> To know if it counts, without Osciloscope or SPI connection 
// shifter[63:0] now updated at SSEL-faling-edge instead SSEL-low 
// Pin-Assignment same with project spi5 
// TODO: pin assignment for new encoder

// 2011.07.14 
// Third encoder added 
// qdec2 module not used. Instead 2 more instantiations of qdec module are used for second and third encoder 
// 36% used from 19% before 
// Registers increased from 64 to 96 bits (3 x 32Bit counters, one for each encoder 
// SPI Master has to know how many bits to get 
// PINS assigned for 2nd,3d encoder but just to check compilation. 
// TODO: To assign 2nd,3d encoder pins at proper PINS for VERIMAX



//TOP LEVEL MODULE ------------------------------------------------------------------------------------------------ 
//**mark1** module spi6(SCK, MOSI, MISO, SSEL, LED, quadA, quadB,index,position, quad2A, quad2B,index2,position2); 
module spi6(SCK, MOSI, MISO, SSEL, LEDS, quadA, quadB,index, quad2A, quad2B,index2, quad3A, quad3B,index3); input SCK, SSEL, MOSI; 
// SPI input signals output MISO; 
// SPI output signal output [7:0] LEDS; 
// Received Byte

// 1st encoder input quadA, quadB,index; 
//**mark1** output [31:0] position;

// 2nd encoder input quad2A, quad2B,index2; 
//**mark1** output [31:0] position2;

// 3nd encoder input quad3A, quad3B,index3; 
//**mark1** output [31:0] position3;

// To use the UFM block for clk wire osc,clk; wire oscena; assign oscena=1'b1; 
//--------------------------

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
       .LEDS (LEDS),
.position (position), .position2 (position2) );

/* Instantiation  of qdec module.*/
qdec q1(
       .osc (osc),
       .quadA (quadA),
       .quadB (quadB),
       .position (position),
       .A (A),
       .B (B),
.DIR (DIR), .index (index) );

/* Instantiation  of qdec2 module.*/
// This is not the best way
// To try to create a second instance of qdec--->DONE
/*qdec2 q2(
       .osc (osc),
       .quad2A (quad2A),
       .quad2B (quad2B),
       .position2 (position2),
       .A2 (A2),
       .B2 (B2),
.DIR2 (DIR2), .index2 (index2) );

/
// 2011.07.14 Second Instantiation of qdec module.

qdec q2(
       .osc (osc),
       .quadA (quad2A),
       .quadB (quad2B),
       .position (position2),
       .A (A2),
       .B (B2),
.DIR (DIR2), .index (index2) );

// 2011.07.14 THIRD Instantiation of qdec module.

qdec q3(
       .osc (osc),
       .quadA (quad3A),
       .quadB (quad3B),
       .position (position3),
       .A (A3),
       .B (B3),
.DIR (DIR3), .index (index3) );


endmodule //END OF TOP LEVEL MODULE --------------------------------------------------------------------------------------


module spi(osc, SCK, MOSI, MISO, SSEL, LEDS, position, position2, position3); input osc; input SCK, SSEL, MOSI; output MISO; output [7:0] LEDS; input [31:0] position; input [31:0] position2; input [31:0] position3;

// sync SCK to the FPGA clock using a 3-bits shift register reg [2:0] SCKr; always @(posedge osc) SCKr <= {SCKr[1:0], SCK}; wire SCK_risingedge = (SCKr[2:1]==2'b01); 
// now we can detect SCK rising edges wire SCK_fallingedge = (SCKr[2:1]==2'b10); 
// and falling edges

// sync SSEL reg [2:0] SSELr; always @(posedge osc) SSELr <= {SSELr[1:0], SSEL}; wire SSEL_active = ~SSELr[1]; 
// SSEL is active low wire SSEL_startmessage = (SSELr[2:1]==2'b10); 
// message starts at falling edge wire SSEL_endmessage = (SSELr[2:1]==2'b01); 
// message stops at rising edge

// sync MOSI reg [1:0] MOSIr; always @(posedge osc) MOSIr <= {MOSIr[0], MOSI}; wire MOSI_data = MOSIr[1];

// Receive Registers reg [3:0] bitcnt; 
// SPI in 16-bits format, so 4 bits counter to count the bits as they come in reg SPI_TRANSACTION_DONE; 
// high when a (16bit) SPI transaction is completed

// Transmit Registers reg [7:0] LEDS; reg MISO; 
// changed from 63 to 96 for the third encoder reg [95:0] shifter;


always @(posedge osc) begin //if(~SSEL_active)

 if(SSEL_startmessage)
 begin										// IF SSEL falling edge
   shifter[95:64] = position[31:0];		// Added for third encoder
shifter[63:32] = position2[31:0]; 
// Transfer position-counters to shifter shifter[31:00] = position3[31:0];
 // A T T E N T I O N : position counters are sampled to shifter[63:0] at...SSEL falling edge

 end
 else										// IF SSEL IS ACTIVE
 if(SCK_risingedge)						// AND IF SCK RISING-EDGE
 begin
LEDS[3:0] <= position[11:8]; 
// ENCODER1 TO LEDS 3:0 LEDS[7:4] <= position2[11:8]; 
// ENCODER2 TO LEDS 7:4 shifter = shifter <<1; 
// Highest Bit will exit to SPI first, *** SPI master has to know how many bits to get *** MISO <= shifter[95]; 
// SingleBit REGISTER MISO gets the [95] BIT of REGISTER shifter <---THIS AT PIN MISO, IS THE DATA

 end
end


endmodule

//------------------------------------------1st---------quadrature decoder------------------- 
module qdec(osc, quadA, quadB, position,A,B,DIR,index); input osc, quadA, quadB,index; output [31:0] position; output A,B,DIR; reg [2:0] quadA_delayed, quadB_delayed; reg A,B,DIR; reg [31:0] position;

always @(posedge osc) quadA_delayed <= {quadA_delayed[1:0], quadA}; always @(posedge osc) quadB_delayed <= {quadB_delayed[1:0], quadB}; wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2]; wire count_direction = quadA_delayed[1] ^ quadB_delayed[2]; always @(posedge osc) begin

 if(count_enable)
 begin
  if(count_direction) position<=position+1; else position<=position-1;
A <= quadA_delayed[2];

   B <= quadB_delayed[2];
   DIR <= count_direction;
 end
end endmodule


// 2011.07 a second instantiation of qdec module used instead [module qdeq2] /* 
//----------------------------------------2nd-----------quadrature decoder------------------ 
module qdec2(osc, quad2A, quad2B, position2,A2,B2,DIR2,index2); input osc, quad2A, quad2B,index2; output [31:0] position2; output A2,B2,DIR2; reg [2:0] quad2A_delayed, quad2B_delayed; reg A2,B2,DIR2; reg [31:0] position2; always @(posedge osc) quad2A_delayed <= {quad2A_delayed[1:0], quad2A}; always @(posedge osc) quad2B_delayed <= {quad2B_delayed[1:0], quad2B}; wire count_enable2 = quad2A_delayed[1] ^ quad2A_delayed[2] ^ quad2B_delayed[1] ^ quad2B_delayed[2]; wire count_direction2 = quad2A_delayed[1] ^ quad2B_delayed[2]; always @(posedge osc) begin

 if(count_enable2)
 begin
  if(count_direction2) position2<=position2+1; else position2 <= position2-1;
A2 <= quad2A_delayed[2];

   B2 <= quad2B_delayed[2];
   DIR2 <= count_direction2;
 end
end endmodule

/
/*******************************************************************************************

  Module ufmclk is the module 
  which gets the clock from the oscillator 
******************************************************************************************/
`timescale 1 ps / 1 ps //synopsys translate_on module ufmclk_altufm_osc_7p3 ( osc, oscena) /* synthesis synthesis_clearbox=1 */; output osc; input oscena;

wire wire_maxii_ufm_block1_osc;

maxii_ufm maxii_ufm_block1 ( .arclk(1'b0), .ardin(1'b0), .arshft(1'b0), .bgpbusy(), .busy(), .drclk(1'b0), .drdout(), .drshft(1'b0), .osc(wire_maxii_ufm_block1_osc), .oscena(oscena) `ifdef FORMAL_VERIFICATION `else // synopsys translate_off `endif , .drdin(1'b0), .erase(1'b0), .program(1'b0) `ifdef FORMAL_VERIFICATION `else // synopsys translate_on `endif // synopsys translate_off , .ctrl_bgpbusy(), .devclrn(), .devpor(), .sbdin(), .sbdout() // synopsys translate_on ); defparam maxii_ufm_block1.address_width = 9, maxii_ufm_block1.osc_sim_setting = 180000, maxii_ufm_block1.lpm_type = "maxii_ufm"; assign osc = wire_maxii_ufm_block1_osc; endmodule //ufmclk_altufm_osc_7p3 //VALID FILE


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


//********************************** END OF PROGRAM.**********************************************
