module RegisterFile(
    input clk,
    input RegWrite, RegWrite2,
    input [3:0] Rd, Rs, Rt,
    input [31:0] BusW, BusW2,
    output reg [31:0] BusA, BusB
	//output [31:0] R14_debug
);

    reg signed [31:0] Registers[0:15];

    // Initialize PC register (R15)
    initial begin
        Registers[15] = 32'h00000001;
    end

    // WRITE at posedge
    always @(posedge clk) begin
        if (RegWrite2)
            Registers[15] <= BusW2;

        if (RegWrite && Rd != 4'd15)
            Registers[Rd] <= BusW;
    end

    // READ at negedge (after write has occurred)
	always @(negedge clk) begin
		
	    if (RegWrite && Rs == Rd)     
	        BusA <= BusW;          
	    else
	        BusA <= Registers[Rs];      
	
	    if (RegWrite && Rt == Rd)         
	        BusB <= BusW;
	    else
	        BusB <= Registers[Rt];
	end	 
	
	//assign R14_debug = Registers[14];

endmodule
