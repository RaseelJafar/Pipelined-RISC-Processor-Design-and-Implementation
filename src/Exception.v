module Exception(
	input [5:0]opcode,
	input [3:0] IF_ID_RegDest,
	output reg exception
	);
	
always @ (*) 
	begin
		  exception =1'b0 ;
		if ((opcode == 6'd8 || opcode == 6'd9) && IF_ID_RegDest[0] == 1'b1) begin  // if Rd is odd 
			exception = 1'b1;
		end
	end
endmodule

		
