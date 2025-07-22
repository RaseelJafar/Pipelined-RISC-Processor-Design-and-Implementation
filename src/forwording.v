module forwarding_unit(	
	input ID_EX_MemRd,
	input [31:0]IF_ID_PC,
	input [31:0]ID_EX_PC,
	input [5:0] opcode,
    input [3:0] ID_EX_Rs,
    input [3:0] ID_EX_Rt,
	input [3:0] ID_EX_RegRd,
    input [3:0] EX_MEM_RegRd,
    input [3:0] MEM_WB_RegRd,
	input ID_EX_RegWrite,
    input EX_MEM_RegWrite,
    input MEM_WB_RegWrite,
	input Datawrite,
	input Exception,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB,
	output reg stall
);
    always @(*) begin
        // Default
        ForwardA = 2'b00;
        ForwardB = 2'b00; 
		stall = 1'b0;

        if (ID_EX_RegWrite && (ID_EX_RegRd != 0) && (ID_EX_RegRd == ID_EX_Rs) )
            ForwardA = 2'b01; 			
		else if (EX_MEM_RegWrite && (EX_MEM_RegRd != 0) &&(EX_MEM_RegRd == ID_EX_Rs))
            ForwardA = 2'b10;
		else if (MEM_WB_RegWrite && (MEM_WB_RegRd != 0) &&(MEM_WB_RegRd == ID_EX_Rs) && Datawrite==0)
            ForwardA = 2'b11;
		else  
			ForwardA = 2'b00;
			
			
        if (ID_EX_RegWrite && (ID_EX_RegRd != 0) && (ID_EX_RegRd == ID_EX_Rt))
            ForwardB = 2'b01;
		else if(EX_MEM_RegWrite && (EX_MEM_RegRd != 0) &&(EX_MEM_RegRd  == ID_EX_Rt))
			ForwardB = 2'b10;
		else if (MEM_WB_RegWrite && (MEM_WB_RegRd != 0) &&(MEM_WB_RegRd == ID_EX_Rt) && Datawrite == 0)
            ForwardB = 2'b11;
		else  
			ForwardB = 2'b00;
			
	
			
		if ((ID_EX_MemRd == 1) && (ForwardA==1 || ForwardB==1))	begin
			stall = 1'b1;											
			end
		//if( ((opcode == 6'd8) && (IF_ID_PC != ID_EX_PC)) || ((opcode == 6'd9) && (IF_ID_PC != ID_EX_PC) )  ) begin  
			
		if ((((opcode == 6'd8) && (IF_ID_PC != ID_EX_PC)) || ((opcode == 6'd9) && (IF_ID_PC != ID_EX_PC)) ) && Exception == 0 ) begin
				stall = 1'b1; 
		
		end	
		
    end
endmodule
