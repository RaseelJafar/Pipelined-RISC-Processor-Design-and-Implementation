module PC_ctrl (
    input [5:0] OPcode,
    input zero,
    input positive,
    input negative,
	output reg kill1,
    output reg [1:0] PCSrc
);
    always @(*) begin
        kill1 = 0; 	  //not taken
		PCSrc = 2'b00; // Default: normal PC+4

        case (OPcode)
            6'b001010: if (zero)   begin  PCSrc = 2'b10; kill1 =1; end // BZ
            6'b001011: if (positive) begin PCSrc = 2'b10; kill1 =1; end// BGZ
            6'b001100: if (negative) begin PCSrc = 2'b10; kill1=1; end// BLZ
            6'b001101:            begin  PCSrc = 2'b01; kill1=1; end// JR
            6'b001110:            begin  PCSrc = 2'b10; kill1=1; end // J
            6'b001111:            begin  PCSrc = 2'b10; kill1=1; end // CLL

            default: PCSrc = 2'b00;
        endcase
    end
endmodule	  



module Program_Counter ( 
	input [31:0] IF_ID_PC,
	input [31:0] BTA,
    input clk, 
	input stall,
    input reset,
    input [1:0] PCSrc,
    input signed [31:0] imm_ext,
    input [31:0] reg_addr,
    output reg [31:0] PC
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 0;
        end 
	else if (!stall) begin
            case (PCSrc)
                2'b00: PC <= PC + 4;
                2'b01: PC <= reg_addr;
                2'b10: PC <= IF_ID_PC +(imm_ext <<2);
                2'b11: PC <= PC; // Stall
            endcase
	end
    end
endmodule

