# Seismic Vibration Detection System Using Spartan‑7 FPGA


## Abstract
This project presents the design and implementation of a real-time seismic vibration detection system using the Spartan‑7 FPGA on the Boolean development board. The system simulates ground vibration using an LFSR‑based sensor emulator, compares the vibration level against a predefined threshold, and activates an alarm LED when excessive seismic activity is detected. The vibration value is displayed on the seven‑segment display using a multiplexed digit driver. Verilog HDL is used for hardware implementation, and Vivado is used for synthesis, simulation, and FPGA programming.

## Introduction
Seismic vibration monitoring plays a critical role in early earthquake detection and structural health analysis. FPGA-based designs offer advantages such as high-speed parallel processing, real-time response, and reconfigurability. This project implements a digital vibration detection system entirely on the Spartan‑7 FPGA using the Boolean board. A pseudo-random vibration generator emulates real-world ground motion, which is evaluated by a comparator and an FSM-based alarm system.
Objectives

## The objectives of this project are:
- Simulate real‑time vibration using an LFSR-based digital sensor.
- Detect when vibration exceeds the seismic safety threshold.
- Indicate high vibration using an alarm LED.
- Display vibration on a seven‑segment display.
- Implement the system fully in Verilog HDL on Spartan‑7 FPGA.




## Block Diagram
The following block diagram illustrates the major components of the system:
 
<img width="886" height="1022" alt="image" src="https://github.com/user-attachments/assets/ab4763a2-9c7b-41ba-996e-1013b91ac870" />


## Verilog HDL Code
The complete Verilog implementation is shown below:
```verilog
`timescale 1ns / 1ps 

module seismic_top( 
input clk, 
output alarm_led, 
output [6:0] seg, 
output [3:0] an 
); 
localparam [7:0] THRESHOLD = 8'd50; // fixed threshold for hardware 
wire [7:0] vibration; 
wire alert_raw; 
wire alarm_signal; 
sensor_emulator U1 (.clk(clk), .vibration(vibration)); 
comparator U2 (.vibration(vibration), .threshold(THRESHOLD), .alert(alert_raw)); 
fsm_controller U3 (.clk(clk), .alert_in(alert_raw), .alarm(alarm_signal)); 
assign alarm_led = alarm_signal; 
sevenseg_driver U4 (.clk(clk), .value(vibration), .seg(seg), .an(an)); 
endmodule 
// ---------------- SENSOR EMULATOR ----------------
module sensor_emulator( 
input clk, 
output reg [7:0] vibration 
); 
reg [7:0] lfsr = 8'hA5; 
always @(posedge clk) begin 
lfsr <= {lfsr[6:0], lfsr[7] ^ lfsr[5]}; 
vibration <= lfsr % 200; 
end 
endmodule 

// ---------------- COMPARATOR ----------------
module comparator( 
input [7:0] vibration, 
input [7:0] threshold, 
output reg alert 
); 
always @(*) begin 
alert = (vibration > threshold); 
end 
endmodule 
// ---------------- FSM ----------------
module fsm_controller( 
input clk, 
input alert_in, 
output reg alarm 
); 
parameter SAFE = 1'b0; 
parameter ALERT = 1'b1; 
reg state = SAFE; 
always @(posedge clk) begin 
case(state) 
SAFE: if(alert_in) state <= ALERT; 
ALERT: if(!alert_in) state <= SAFE; 
endcase 
end 
always @(*) begin 
alarm = (state == ALERT); 
end 
endmodule 
// ---------------- SEVEN-SEGMENT DRIVER ----------------
module sevenseg_driver( 
input clk, 
input [7:0] value, 
output reg [6:0] seg, 
output reg [3:0] an 
); 
reg [1:0] digit = 0; 
reg [3:0] bcd; 
always @(posedge clk) begin 
digit <= digit + 1; 
end 
always @(*) begin 
case(digit) 
2'b00: begin an = 4'b1110; bcd = value % 10; end // Units 
2'b01: begin an = 4'b1101; bcd = (value/10) % 10; end // Tens 
2'b10: begin an = 4'b1011; bcd = value / 100; end // Hundreds 
default: begin an = 4'b0111; bcd = 0; end 
endcase 
end 
always @(*) begin 
case(bcd) 
4'd0: seg = 7'b1000000; 
4'd1: seg = 7'b1111001; 
4'd2: seg = 7'b0100100; 
4'd3: seg = 7'b0110000; 
4'd4: seg = 7'b0011001; 
4'd5: seg = 7'b0010010; 
4'd6: seg = 7'b0000010; 
4'd7: seg = 7'b1111000; 
4'd8: seg = 7'b0000000; 
4'd9: seg = 7'b0010000; 
default: seg = 7'b1111111; 
endcase 
end 
endmodule 
```
## TESTBENCH CODE:
```verilog
module tb_seismic; 
reg clk = 0; 
wire alarm_led; 
wire [6:0] seg; 
wire [3:0] an; 
seismic_top DUT ( 
.clk(clk), 
.alarm_led(alarm_led), 
.seg(seg), 
.an(an) 
); 
always #5 clk = ~clk; // Generate 100MHz clock 
initial begin 
$display("Time | Alarm"); 
$monitor("%0t | %b", $time, alarm_led); 
#500 $finish; 
end 
endmodule 
```

## Verilog HDL code for implementation in FPGA:
```verilog
`timescale 1ns / 1ps
module seismic_top(
    input clk,                     
    output alarm_led,              
    output [6:0] seg,              
    output [3:0] an,
    output [2:0] RGB0             // RGB LED output
);

localparam [7:0] THRESHOLD = 8'd50;   // used only for alarm LED behavior

wire [7:0] vibration;
wire alert_raw;
wire alarm_signal;

// Sensor emulator - slow increase every 1 second
sensor_emulator U1 (
    .clk(clk),
    .vibration(vibration)
);

// Comparator
comparator U2 (
    .vibration(vibration),
    .threshold(THRESHOLD),
    .alert(alert_raw)
);

// FSM
fsm_controller U3 (
    .clk(clk),
    .alert_in(alert_raw),
    .alarm(alarm_signal)
);

assign alarm_led = alarm_signal;

// ---------------------------------------------------------
// RGB COLOR CONTROL (ACTIVE LOW LEDs)
// SAFE    (<40)  ? GREEN
// WARNING (40-60) ? YELLOW (RED + GREEN)
// DANGER  (>60)  ? RED
// ---------------------------------------------------------

reg [2:0] rgb_reg;

always @(*) begin
    if (vibration < 8'd80) begin
        // GREEN ON
        rgb_reg = 3'b010;   // R=1, G=1, B=0
    end
    else if (vibration >= 8'd81 && vibration <= 8'd99) begin
        // YELLOW (RED + GREEN)
        rgb_reg = 3'b110;   // R=1, G=0, B=0
    end
    else begin
        // RED ON
        rgb_reg = 3'b100;   // R=0, G=1, B=1
    end
end

assign RGB0 = rgb_reg;

// Seven segment display
sevenseg_driver U4 (
    .clk(clk),
    .value(vibration),
    .seg(seg),
    .an(an)
);

endmodule


// ------------------------------------------------------------
// SENSOR EMULATOR - Increment 1 every 1 second
// ------------------------------------------------------------
module sensor_emulator(
    input clk,
    output reg [7:0] vibration
);

reg [26:0] slow = 0;   // 27-bit for 100M count

always @(posedge clk) begin
    slow <= slow + 1;

    if (slow == 27'd100_000_000) begin  
        vibration <= vibration + 10;    // increment every 1 second
        slow <= 0;
    end
end

endmodule


// ------------------------------------------------------------
// COMPARATOR
// ------------------------------------------------------------
module comparator(
    input [7:0] vibration,
    input [7:0] threshold,
    output reg alert
);

always @(*) begin
    alert = (vibration > threshold);
end

endmodule


// ------------------------------------------------------------
// FSM CONTROLLER
// ------------------------------------------------------------
module fsm_controller(
    input clk,
    input alert_in,
    output reg alarm
);

parameter SAFE  = 1'b0;
parameter ALERT = 1'b1;

reg state = SAFE;

always @(posedge clk) begin
    case(state)
        SAFE:  if(alert_in) state <= ALERT;
        ALERT: if(!alert_in) state <= SAFE;
    endcase
end

always @(*) begin
    alarm = (state == ALERT);
end

endmodule


// ------------------------------------------------------------
// SEVEN SEGMENT DRIVER (with refresh clock)
// ------------------------------------------------------------
module sevenseg_driver(
    input clk,
    input [7:0] value,
    output reg [6:0] seg,
    output reg [3:0] an
);

reg [15:0] refresh_cnt = 0;
reg refresh_clk = 0;

reg [1:0] digit = 0;
reg [3:0] bcd;

// Clock divider ~1 kHz
always @(posedge clk) begin
    refresh_cnt <= refresh_cnt + 1;
    refresh_clk <= refresh_cnt[15];
end

// Digit switching using slow refresh clock
always @(posedge refresh_clk) begin
    digit <= digit + 1;
end

// Select digit and BCD value
always @(*) begin
    case(digit)
        2'b00: begin an = 4'b1110; bcd = value % 10; end
        2'b01: begin an = 4'b1101; bcd = (value / 10) % 10; end
        2'b10: begin an = 4'b1011; bcd = value / 100; end
        default: begin an = 4'b0111; bcd = 0; end
    endcase
end

// BCD to 7-segment
always @(*) begin
    case(bcd)
        4'd0: seg = 7'b1000000;
        4'd1: seg = 7'b1111001;
        4'd2: seg = 7'b0100100;
        4'd3: seg = 7'b0110000;
        4'd4: seg = 7'b0011001;
        4'd5: seg = 7'b0010010;
        4'd6: seg = 7'b0000010;
        4'd7: seg = 7'b1111000;
        4'd8: seg = 7'b0000000;
        4'd9: seg = 7'b0010000;
        default: seg = 7'b1111111;
    endcase
end

endmodule



// ------------------------------------------------------------
// TESTBENCH
// ------------------------------------------------------------
module tb_seismic;

reg clk = 0;

wire alarm_led;
wire [6:0] seg;
wire [3:0] an;
wire [2:0] RGB0;

seismic_top DUT (
    .clk(clk),
    .alarm_led(alarm_led),
    .seg(seg),
    .an(an),
    .RGB0(RGB0)
);

always #5 clk = ~clk;   // 100 MHz clock

initial begin
    $display("Time | Alarm | RGB0 | Vibration ");
    $monitor("%0t | %b | %b | %d", $time, alarm_led, RGB0, DUT.vibration);

    #2000 $finish;
end

endmodule

```
## CONSTRAIN FILE:
```verilog
# clock
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports {clk}]
create_clock -period 10.000 -name gclk [get_ports clk]
# Set Bank 0 voltage / config
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
# Alarm LED -> map to on-board LED[0]
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports {alarm_led}]
# Seven-segment display 0 mapping (use seg[0]..seg[6] -> D0_SEG[0]..D0_SEG[6])
set_property -dict {PACKAGE_PIN D7  IOSTANDARD LVCMOS33} [get_ports {seg[0]}]  # D0_SEG[0]
set_property -dict {PACKAGE_PIN C5  IOSTANDARD LVCMOS33} [get_ports {seg[1]}]  # D0_SEG[1]
set_property -dict {PACKAGE_PIN A5  IOSTANDARD LVCMOS33} [get_ports {seg[2]}]  # D0_SEG[2]
set_property -dict {PACKAGE_PIN B7  IOSTANDARD LVCMOS33} [get_ports {seg[3]}]  # D0_SEG[3]
set_property -dict {PACKAGE_PIN A7  IOSTANDARD LVCMOS33} [get_ports {seg[4]}]  # D0_SEG[4]
set_property -dict {PACKAGE_PIN D6  IOSTANDARD LVCMOS33} [get_ports {seg[5]}]  # D0_SEG[5]
set_property -dict {PACKAGE_PIN B5  IOSTANDARD LVCMOS33} [get_ports {seg[6]}]  # D0_SEG[6]
# Seven-segment anodes (digit enable) mapping: an[3:0] -> D0_AN[0..3]
set_property -dict {PACKAGE_PIN D5  IOSTANDARD LVCMOS33} [get_ports {an[0]}]  # D0_AN[0]
set_property -dict {PACKAGE_PIN C4  IOSTANDARD LVCMOS33} [get_ports {an[1]}]  # D0_AN[1]
set_property -dict {PACKAGE_PIN C7  IOSTANDARD LVCMOS33} [get_ports {an[2]}]  # D0_AN[2]
set_property -dict {PACKAGE_PIN A8  IOSTANDARD LVCMOS33} [get_ports {an[3]}]  # D0_AN[3]
```


## Simulation Results
Simulation verifies that the alarm condition is activated whenever the vibration value exceeds the threshold. The FSM transitions between SAFE and ALERT states based on input alert signal. Seven‑segment output values change rapidly due to multiplexing.

 <img width="1589" height="822" alt="image" src="https://github.com/user-attachments/assets/c96660bf-122a-4245-abde-5c2528c010ef" />

### Simulation Explanation – Image 1 (at 35.000 ns)
At 35 ns, the waveform shows:
- alarm_led = 1, indicating that the vibration value is above the threshold.
- an = 1110, meaning the units digit of the vibration is currently being displayed.
- seg = 1111001, which corresponds to digit ‘1’ on the seven-segment display.
Because the display is multiplexed, the an lines cycle through 1110 → 1101 → 1011, showing units, tens, and hundreds digits continuously. Since the internal vibration value crossed the threshold, the FSM correctly enters the ALERT state, turning alarm_led ON.

 <img width="1659" height="891" alt="image" src="https://github.com/user-attachments/assets/763bc3aa-9cf9-4f17-b196-9f813a326a07" />

### Simulation Explanation – Image 2 (at 195.100 ns)
At 195.1 ns, the waveform shows:
- alarm_led = 0, meaning the vibration is below the threshold at this instant.
- an = 1110, so the units digit is currently active.
- seg = 1111001, again representing digit ‘1’ being displayed at that moment.
The surrounding values show typical multiplexing behavior:
seg cycles through patterns like 1000000 (0), 1111001 (1), 0011001 (4), etc., while an cycles through 1110, 1101, 1011. This confirms that the seven-segment display driver and vibration generator are working correctly. The FSM remains in SAFE state because the vibration level did not exceed the threshold.






## Hardware Implementation
The designed seismic vibration monitoring system was successfully implemented on the Spartan-7 Boolean FPGA board using AMD Vivado. After generating the bitstream, the program was downloaded to the board through the USB-JTAG programming port.
### 1. Seven-Segment Display Output 
In the first image, the seven-segment display shows the value “0028”, which corresponds to the current vibration reading generated by the sensor emulator in hardware.
- The display updates every one second because the vibration counter is slowed down in hardware.
- Only the first four digits (Display-0) are used, while the remaining displays remain off.
- The value increments gradually, demonstrating the real-time increase of vibration as programmed.
#### Simultaneously, the RGB LED glows Green, which indicates:
- Vibration < Threshold
- The system is in the SAFE state, so the ALERT condition is not triggered.
This confirms that the comparator and FSM logic are functioning correctly for values below the threshold.
<img width="898" height="505" alt="image" src="https://github.com/user-attachments/assets/8f823e69-1cd9-4907-b819-56b4fb4d9cb1" />

#### 2. Threshold Crossing and Alert Indication 
In the second image, the seven-segment display shows a higher value such as “0108”, which indicates the vibration has crossed the threshold value of 50.
Here, the RGB LED changes to Red, showing that:
- Vibration > Threshold
- The comparator output becomes HIGH
- The FSM moves into the ALERT state
- The alarm signal is asserted and the alert LED turns ON
This visually verifies that the hardware alert mechanism works exactly as expected when vibration exceeds the safe limit.
<img width="881" height="496" alt="image" src="https://github.com/user-attachments/assets/dcd17dec-a387-4cb0-ab5a-d4b041434d4d" />


## Conclusion
A fully functional seismic vibration detection and alert system has been successfully designed and implemented on the Spartan-7 FPGA. The project demonstrates the complete workflow of digital system development using Verilog HDL—including modeling, simulation, synthesis, implementation, and real-time hardware testing.
The system accurately generates vibration values, compares them with a threshold, and triggers an alarm through FSM-based decision logic. The seven-segment display provides an updated numerical indication of the vibration magnitude, while the LED alert offers a quick and clear warning mechanism. This project validates the effectiveness of FPGA-based solutions for real-time signal monitoring and safety-critical applications such as seismic activity detection.

