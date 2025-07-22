module ID_EX (
    input clk,
    input reset,
    // Control signals
    input [2:0] ALUOp_in,
    input [1:0] ALUSrc_in,
    input       MemRead_in,
    input       MemWrite_in,
    input       WBdata_in,
    input       RegWrite_in,
    input       Data_write_in,

    // Data
    input [31:0] BusA_in,
    input [31:0] BusB_in,
    input [31:0] imm_ext_in,
	//register dest
    input [3:0] Rd_in,
	input [31:0] PC,
	input [31:0] BTA_in,

    output reg [1:0] RegDst_out,
    output reg [2:0] ALUOp_out,
    output reg [1:0] ALUSrc_out,
    output reg       MemRead_out,
    output reg       MemWrite_out,
    output reg       WBdata_out,
    output reg       RegWrite_out,
    output reg       Data_write_out,

    output reg [31:0] BusA_out,
    output reg [31:0] BusB_out,
    output reg [31:0] imm_ext_out,
    output reg [3:0] Rd_out,
	output reg [31:0] PC_out,
	output reg [31:0] BTA_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            {RegDst_out, ALUOp_out, ALUSrc_out, MemRead_out, MemWrite_out,
             WBdata_out, RegWrite_out, Data_write_out} <= 0;

            {BusA_out, BusB_out, imm_ext_out} <= 0;
            {Rd_out,BTA_out} <= 0; 
			PC_out <= 32'hFFFFFFFF;
        end else begin
            //RegDst_out     <= RegDst_in;
            ALUOp_out      <= ALUOp_in;
            ALUSrc_out     <= ALUSrc_in;
            MemRead_out    <= MemRead_in;
            MemWrite_out   <= MemWrite_in;
            WBdata_out     <= WBdata_in;
            RegWrite_out   <= RegWrite_in;
            Data_write_out <= Data_write_in;

            BusA_out <= BusA_in;
            BusB_out <= BusB_in;
            imm_ext_out <= imm_ext_in;
            Rd_out <= Rd_in;
			PC_out <= PC ;
			BTA_out<=BTA_in; 
        end
    end

endmodule
