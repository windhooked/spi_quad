SPI spi(p5, p6, p7); // mosi, miso, sclk DigitalOut cs(p8);

TextLCD lcd(p10, p12, p15, p16, p29, p30, TextLCD::LCD20x4); // rs, e, d4-d7

int counter;

// default baud rate is 9600 Serial pc(USBTX, USBRX); // tx, rx

int main() {

   // Setup the spi for 8 bit data, high steady state clock,
   // second edge capture, with a 1MHz clock rate
   spi.format(16,3);
   spi.frequency(1000000);
   lcd.cls();
   lcd.locate(0,0);
   lcd.printf("ENCODER X4\n");
   
   while(1) 
       {
           cs = 0;                                     // start verimax spi transaction
           int rbyte =  spi.write(0x55);               // get 16 bits from verimax
           int rbyte2 = spi.write(0x55);               // get 16 more bits from verimax                     
           cs = 1;                                     // terminate verimax spi transaction
           wait_ms(10);                                // simulate 10ms read cycle 
           
           counter += 1;                               // display the position  
           if(counter == 30)                           // every 300ms
               { 
                counter =0;       
                pc.printf("%d\n",(rbyte2+rbyte*65535));// display position to terminal (com port via USB) 
                lcd.locate(0,1);
                lcd.printf("pulse %10d ", (rbyte2+rbyte*65535) )  ;
                                                       // display position to LCD                 
               }
        } 
} 
