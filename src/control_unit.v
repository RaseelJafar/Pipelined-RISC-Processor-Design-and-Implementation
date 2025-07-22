//CONTROL UNIT
module control_unit( 
    input [5:0] opcode,
	input stall,
	input [1:0]ForwardA,
	input [1:0]ForwardB,
	input Exception,
    output reg  RegWr, ExtOp,
    output reg [2:0] ALUOp,
    output reg MemRd, MemWr,
    output reg WBdata ,
	output reg  [1:0]RegDst,
	output reg  [1:0]RegReadB,
	output reg Data_write,
	output reg [1:0]ALUSrc
);
 
always @(*) 
begin
        // Default values
        RegDst = 0; RegWr = 0; ExtOp = 0;RegReadB = 0;
        ALUOp = 3'b000; ALUSrc = 0;
        MemRd = 0; MemWr = 0; WBdata = 0; Data_write=0;
		
		if (Exception ==1)
			begin
				RegDst = 0; 
				RegWr = 0; 
				ExtOp = 0;
				RegReadB = 0;
        		ALUOp = 3'b000; 
				ALUSrc = 0;
				MemRd = 0; 
				MemWr = 0; 
				WBdata = 0; 
				Data_write=0;
			end
		
		else if (stall) begin 
			//stall control signal for all zero 
				
			if ((ForwardA==1 || ForwardB==1) && (opcode == 6'd8))
			begin
					//rs==rd in LDW
				    
			end
			else if(opcode == 6'd8) begin  
				//round 1 signals 
				    RegDst = 0;
		            ALUSrc = 1;
		            WBdata = 1;
		            RegWr = 1;
		            MemRd = 1;
		            ALUOp = 3'b001;
		            ExtOp = 1;
				
			end		
			
			if(opcode == 6'd9) begin
				  //round 1 signals
				    RegDst = 1;
		            ALUSrc = 1;
		            MemWr = 1;
		            ALUOp = 3'b001;
		            ExtOp = 1;
				   RegReadB = 1;
				
			end		
			
		end else 
		begin
	        case (opcode) 
	 
	            6'd0: 
				begin // OR
	                RegDst = 0; RegWr = 1; ALUOp = 3'b000;
	            end
	            6'd1: 
				begin // ADD
	                RegDst = 0; RegWr = 1; ALUOp = 3'b001;
	            end
	            6'd2: 
				begin // SUB
	                RegDst = 0; RegWr = 1; ALUOp = 3'b010;
	            end
	            6'd3: 
				begin // CMP
	                RegDst = 0; RegWr = 1; ALUOp = 3'b011;
	            end
	            6'd4: 
				begin // ORI
	                ALUSrc = 1; RegWr = 1; ALUOp = 3'b000; ExtOp=0;
	            end
	            6'd5: 
				begin // ADDI
	                ALUSrc = 1; RegWr = 1; ALUOp = 3'b001; ExtOp=1; //RegReadB = 10; „„ﬂ‰ ›Ì Œ··
	            end
	            6'd6: 
				begin // LW
	                RegDst = 0; ALUSrc = 1; WBdata = 1; RegWr = 1; MemRd = 1; ALUOp = 3'b001;ExtOp=1;
	            end
	            6'd7: 
				begin // SW
	                 ALUSrc = 1; MemWr = 1; ALUOp = 3'b001; ExtOp=1; RegReadB = 1;
	            end
	 
				6'd8:  //LDW
				begin  
					//round 2 signals
		            RegDst = 1;
		            ALUSrc = 2;
		            WBdata = 1;
		            RegWr = 1;
		            MemRd = 1;
		            ALUOp = 3'b001;
		            ExtOp = 1;
		        end	 
				
	 			
	            6'd9: 
				begin // SWD 
					//round
		            RegDst = 2;
		            ALUSrc = 2;
		            MemWr = 1;
		            ALUOp = 3'b001;
		            ExtOp = 1;
					RegReadB = 2;
	            end	
	 
	            6'd10,6'd11, 6'd12: 
				begin // BZ, BGZ, BLZ
	                ExtOp = 1;
	            end
	            6'd13, 6'd14:
				begin // JR, J
	                ExtOp = 1;
	            end
				6'd15: 
				begin // CLL
	                RegDst = 2; RegWr = 1; Data_write=1; ExtOp = 1;
	            end
	        endcase
			end
    end
endmodule    
 
 
//IMM
module immediate_extender(
    input [13:0] imm_input,
    input Mostbit,
    output [31:0] imm_output
);
assign imm_output = (Mostbit == 1'b0) ? {18'b0, imm_input} :
                        {{18{imm_input[13]}}, imm_input};
endmodule
 