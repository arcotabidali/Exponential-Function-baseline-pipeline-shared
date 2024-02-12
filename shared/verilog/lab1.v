module lab1 #
(
	parameter WIDTHIN = 16,		// Input format is Q2.14 (2 integer bits + 14 fractional bits = 16 bits)
	parameter WIDTHOUT = 32,	// Intermediate/Output format is Q7.25 (7 integer bits + 25 fractional bits = 32 bits)
	// Taylor coefficients for the first five terms in Q2.14 format
	parameter [WIDTHIN-1:0] A0 = 16'b01_00000000000000, // a0 = 1
	parameter [WIDTHIN-1:0] A1 = 16'b01_00000000000000, // a1 = 1
	parameter [WIDTHIN-1:0] A2 = 16'b00_10000000000000, // a2 = 1/2
	parameter [WIDTHIN-1:0] A3 = 16'b00_00101010101010, // a3 = 1/6
	parameter [WIDTHIN-1:0] A4 = 16'b00_00001010101010, // a4 = 1/24
	parameter [WIDTHIN-1:0] A5 = 16'b00_00000010001000,  // a5 = 1/120
	parameter [WIDTHOUT-1:0] extended_A5 = {5'b0, A5, 11'b0}  // we want to use only 32x16 multiplier so adapting to it.
	
)
(
	input clk,
	input reset,	
	
	input i_valid,
	input i_ready,
	output o_valid,
	output o_ready,
	
	input [WIDTHIN-1:0] i_x,
	output [WIDTHOUT-1:0] o_y
);
//Output value could overflow (32-bit output, and 16-bit inputs multiplied
//together repeatedly).  Don't worry about that -- assume that only the bottom
//32 bits are of interest, and keep them.
reg [WIDTHIN-1:0] x;	// Register to hold input X
reg [WIDTHOUT-1:0] y_intermediate; // Register to hold intermediate output
reg [WIDTHOUT-1:0] y_Q;	// Register to hold output Y

reg valid_mul_mux_out_for_mul0_add0_Q; // multiplier's mux's output is valid that for the current state mul0_add0
reg valid_mul_mux_out_for_mul1_add1_Q; // multiplier's mux's output is valid that for the current state mul1_add1
reg valid_mul_mux_out_for_mul2_add2_Q; // similar to above commment...
reg valid_mul_mux_out_for_mul3_add3_Q;
reg valid_mul_mux_out_for_mul4_add4_Q;


reg valid_Q2;		// Output of register y is valid


// signal for enabling sequential circuit elements
reg enable;


// Signals for computing the y output
wire [WIDTHOUT-1:0] mul_out; // multiplier's output
wire [WIDTHOUT-1:0] add_out; // adder's output
wire [WIDTHOUT-1:0] y_D;

//multiplier's mux
reg mul_mux_select; // select of mux that feeds input to the only multiplier
wire[WIDTHOUT-1:0] mul_mux_out; //output of mux that feeds input to the only multiplier

//adder's mux
reg [2:0] add_mux_select; // select of mux that feeds input to the only adder
wire[WIDTHIN-1:0] add_mux_out; // output fo mux that feeds input to only adder


//multiplexer for multiplier
mux_2to1  #(.WIDTHOUT(WIDTHOUT)) mult_mux (.A5(extended_A5), .y_intermediate(y_intermediate), .sel(mul_mux_select), .out(mul_mux_out));
//multiplexer for adder
mux_4to1  #(.WIDTHIN(WIDTHIN)) add_mux (.A0(A0), .A1(A1), .A2(A2), .A3(A3), .A4(A4), .sel(add_mux_select), .out(add_mux_out));

// compute y value
mult32x16 Mult (.i_dataa(mul_mux_out), .i_datab(x), .o_res(mul_out));
addr32p16 Addr (.i_dataa(mul_out), 	.i_datab(add_mux_out), 	.o_res(add_out));

//assign output of adder to y_D
assign y_D = add_out;


//Finate state machine //by default the initial state is standby, where we are waiting for valid input to perform operations
reg [2:0] state;
parameter mul0_add0=3'b000; // we will peform first multiplication and first addition of tylor expansion together in one cycle in this state
parameter mul1_add1=3'b001; // we will peform second multiplication and second  of tylor expansion together in one cycle in this state
parameter mul2_add2=3'b010; // ...
parameter mul3_add3=3'b011; // ...
parameter mul4_add4=3'b100; // ...
parameter standby=3'b101;   // deffualt initial state or state in which we wait for vaild input

//we will be performing mul0 add0 in once cycle, mul1 add1 in next cycle in so on.

initial
	begin
		state <= standby;
	end

always @(state)
	begin
		case ((state))
			standby: // do nothing reset the selects of respective MUXs
				begin
					mul_mux_select <= 1'b0; 
					add_mux_select <= 3'b000;
				end
			mul0_add0: // we entered mul0_add0 state, feed the appropriate inputs to muliplier and adder through select
				begin
					mul_mux_select <= 1'b0;
					add_mux_select <= 3'b000;
				end
			mul1_add1: // similar to above comment..
				begin
					mul_mux_select <= 1'b1;
					add_mux_select <= 3'b001;
				end
			mul2_add2: // ...
				begin
					mul_mux_select <= 1'b1;
					add_mux_select <= 3'b010;
				end
			mul3_add3: // ...
				begin				
					mul_mux_select <= 1'b1;
					add_mux_select <= 3'b011;
				end
			mul4_add4: // ...
				begin
					mul_mux_select <= 1'b1;
					add_mux_select <= 3'b100;
				end
			default: // defualt we will reset selects of the respective MUXs
				begin
					mul_mux_select <= 1'b0;
					add_mux_select <= 3'b000;
				end
		endcase
	end

// Combinational logic
always @* begin
	// signal for enable
	enable = i_ready;	
end

// Infer the registers
always @ (posedge clk or posedge reset) begin
	if (reset) begin
		valid_mul_mux_out_for_mul0_add0_Q <= 1'b0; //reset validity flags
		valid_mul_mux_out_for_mul1_add1_Q <= 1'b0;
		valid_mul_mux_out_for_mul2_add2_Q <= 1'b0;
		valid_mul_mux_out_for_mul3_add3_Q <= 1'b0;
		valid_mul_mux_out_for_mul4_add4_Q <= 1'b0;		
		valid_Q2 <= 1'b0;
		state = standby;
		x <= 0;
		y_intermediate <= 0;
		y_Q <= 0;
	end else if (enable) begin
		// propagate the valid value
		valid_mul_mux_out_for_mul0_add0_Q <= i_valid; 
		valid_mul_mux_out_for_mul1_add1_Q <= valid_mul_mux_out_for_mul0_add0_Q;
		valid_mul_mux_out_for_mul2_add2_Q <= valid_mul_mux_out_for_mul1_add1_Q;
		valid_mul_mux_out_for_mul3_add3_Q <= valid_mul_mux_out_for_mul2_add2_Q;
		valid_mul_mux_out_for_mul4_add4_Q <= valid_mul_mux_out_for_mul3_add3_Q;
		valid_Q2 <= valid_mul_mux_out_for_mul4_add4_Q;
		
		// read in new x only if we are ready to take new inputs
		if(o_ready)
			x <= i_x;

		// transiton for our state
		begin
			case (state)
				standby:
					if(i_valid) // got new input to process!
						state = mul0_add0;
				mul0_add0:
					if(valid_mul_mux_out_for_mul0_add0_Q) // move to next state as we have valid output from previous state.
						state = mul1_add1;
				mul1_add1:
					if(valid_mul_mux_out_for_mul1_add1_Q) // same as above comment
						state = mul2_add2;
				mul2_add2:
					if(valid_mul_mux_out_for_mul2_add2_Q) // ...
						state = mul3_add3;
				mul3_add3:
					if(valid_mul_mux_out_for_mul3_add3_Q) // ...
						state = mul4_add4;
				mul4_add4:
					if(valid_mul_mux_out_for_mul4_add4_Q) //there two options after mul4_add4 since we finished processing,  
						if(i_valid)						  //if there  i_valid input go to mul0_add0 immediately										
							state = mul0_add0;			  //else stay in standby waiting for i_valid input.
						else
							state = standby;
				default:
					state = standby;
			endcase
		end
		
		//output computed intermediate y value
		y_intermediate <= y_D;
		
		// output computed y value, this will become valid only after mul4_add4 state
		y_Q <= y_D;
	end
end

// assign outputs
assign o_y = y_Q;
// ready for inputs as long as receiver is ready for outputs and we going to finish processing in current cycle (so that we can start processing new input in very next cycle) or in standby*/
assign o_ready = i_ready && (state==mul4_add4 || state==standby);   		
// the output is valid as long as the corresponding input was valid and 
//	the receiver is ready. If the receiver isn't ready, the computed output
//	will still remain on the register outputs and the circuit will resume
//  normal operation with the receiver is ready again (i_ready is high)*/
assign o_valid = valid_Q2 & i_ready;	

endmodule

/*******************************************************************************************/

// Multiplier module for all the remaining 32x16 multiplications
module mult32x16 (
	input  [31:0] i_dataa,
	input  [15:0] i_datab,
	output [31:0] o_res
);

reg [47:0] result;

always @ (*) begin
	result = i_dataa * i_datab;
end

// The result of Q7.25 x Q2.14 is in the Q9.39 format. Therefore we need to change it
// to the Q7.25 format specified in the assignment by selecting the appropriate bits
// (i.e. dropping the most-significant 2 bits and least-significant 14 bits).
assign o_res = result[45:14];

endmodule

/*******************************************************************************************/

// Adder module for all the 32b+16b addition operations 
module addr32p16 (
	input [31:0] i_dataa,
	input [15:0] i_datab,
	output [31:0] o_res
);

// The 16-bit Q2.14 input needs to be aligned with the 32-bit Q7.25 input by zero padding
assign o_res = i_dataa + {5'b00000, i_datab, 11'b00000000000};

endmodule

/*******************************************************************************************/
module mux_4to1 		#(parameter WIDTHIN = 16)
						(input [WIDTHIN-1:0] A0,                 // WIDTHIN-bit input called A0 
						 input [WIDTHIN-1:0] A1,                 // WIDTHIN-bit input called A1  
						 input [WIDTHIN-1:0] A2,                 // WIDTHIN-bit input called A2  
						 input [WIDTHIN-1:0] A3,                 // WIDTHIN-bit input called A3  
						 input [WIDTHIN-1:0] A4,                 // WIDTHIN-bit input called A4						  
						 input [2:0] sel,               		 // input sel used to select between A1,A2,A3,A4 
						 output [WIDTHIN-1:0] out);              // 4-bit output based on input sel  
	// weh sel[2]  is 0, (sel[1] ? (sel[0] ? A1 : A2) : (sel[0] ? A3 : A4)) is selected else A0.
 	// When sel[1] is 0, (sel[0]? A3:A4) is selected and sel[1] is 1, (sel[0] ? A1:A2) is taken  
	// If sel[0] is 0, A4 is sent to output, else A3 and if sel[0] is 0, A2 is sent to output, else A1  
	assign out = sel[2] ? A0 : (sel[1] ? (sel[0] ? A1 : A2) : (sel[0] ? A3 : A4));  
	
endmodule  

/*******************************************************************************************/

module mux_2to1 		#(parameter WIDTHOUT = 32)
						(input [WIDTHOUT-1:0] y_intermediate,                 // WIDTHIN-bit input called y_D  
						 input [WIDTHOUT-1:0] A5,                  // WIDTHIN-bit input called A5                               
						 input sel,               				  // input sel used to select between x,A5
						 output [WIDTHOUT-1:0] out);               // WIDTHIN-bit output based on input sel  
	
	// When sel is 0, A5 is selected else if sel is 1, y_intermediate is selected	 
	assign out = sel ? y_intermediate : A5;  
	
endmodule  

/*******************************************************************************************/




