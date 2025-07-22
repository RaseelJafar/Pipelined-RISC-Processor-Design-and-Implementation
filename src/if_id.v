module IF_ID (
    input clk,
    input reset,
	input stall,
	input kill1,
    input [31:0] PC_in,
    input [31:0] instruction_in,
    output reg [31:0] PC_out,
    output reg [31:0] instruction_out
);	

	parameter NOP = 32'b0;   

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC_out <= 32'hFFFFFFFF;
            instruction_out <= 32'hxxxxxxxx;
        end 
		else if (!stall) begin	 
            PC_out <= PC_in;  
			if (kill1)	 //disIR
            	instruction_out <=NOP;	
			else
				instruction_out <= instruction_in;	
        end	
		//in stall the instruction out and PC is the same	
    end	
			
endmodule
