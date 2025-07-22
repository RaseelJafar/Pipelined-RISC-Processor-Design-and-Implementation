//ALU MODULE
 
module ALU(
    input [31:0] A, B,
    input [2:0] ALUOp, 
    output reg signed [31:0] ALU_result,
    output reg Zero, Negative, Positive   //flags
 
);
    always @(*) begin
        case(ALUOp)
            3'b000: ALU_result = A | B;    // OR
            3'b001: ALU_result = A + B;    // ADD
            3'b010: ALU_result = A - B;    // SUB
            3'b011: begin             // CMP
                if (A == B) ALU_result = 0;
                else if (A < B) ALU_result = -1;
                else ALU_result = 1;
            end
			//3'b100: ALU_result = A - 0;    // SUB_0 
  
            default: ALU_result = 0;
        endcase
        Zero = (ALU_result == 0);  
    	Negative = (ALU_result < 0);
        Positive = (ALU_result > 0);
 
    end
endmodule    
 