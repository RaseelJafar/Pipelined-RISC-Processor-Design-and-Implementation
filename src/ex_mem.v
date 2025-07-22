module EX_MEM (
    input clk,
    input reset,

    // Control
    input MemRead_in,
    input MemWrite_in,
    input WBdata_in,
    input RegWrite_in,
    input Data_write_in,

    // Data
    input [31:0] ALU_result_in,
    input [31:0] BusB_in,
    input [3:0] dest_reg_in,
	input [31:0] PC,

    output reg MemRead_out,
    output reg MemWrite_out,
    output reg WBdata_out,
    output reg RegWrite_out,
    output reg Data_write_out,

    output reg [31:0] ALU_result_out,
    output reg [31:0] BusB_out,
    output reg [3:0] dest_reg_out,
	output reg [31:0] PC_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            {MemRead_out, MemWrite_out, WBdata_out, RegWrite_out, Data_write_out} <= 0;
            ALU_result_out <= 0;
            BusB_out <= 0;
            dest_reg_out <= 0;
			PC_out <=32'hFFFFFFFF ;
        end else begin
            MemRead_out <= MemRead_in;
            MemWrite_out <= MemWrite_in;
            WBdata_out <= WBdata_in;
            RegWrite_out <= RegWrite_in;
            Data_write_out <= Data_write_in;

            ALU_result_out <= ALU_result_in;
            BusB_out <= BusB_in;
            dest_reg_out <= dest_reg_in;
			PC_out <= PC ;
        end
    end

endmodule
