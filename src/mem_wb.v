module MEM_WB (
    input clk,
    input reset,

    //input WBdata_in,
    input RegWrite_in,
    input Data_write_in,

    //input [31:0] ALU_result_in,
    input [31:0] Data_in,
    input [3:0] dest_reg_in,
	input [31:0] PC,

    //output reg WBdata_out,
    output reg RegWrite_out,
    output reg Data_write_out,

    //output reg [31:0] ALU_result_out,
    output reg [31:0] Data_out,
    output reg [3:0] dest_reg_out,
	output reg [31:0] PC_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            {RegWrite_out, Data_write_out} <= 0;
            //ALU_result_out <= 0;
            Data_out <= 0;
            dest_reg_out <= 0;
			PC_out<=32'hFFFFFFFF;
        end else begin
           // WBdata_out <= WBdata_in;
            RegWrite_out <= RegWrite_in;
            Data_write_out <= Data_write_in;

            //ALU_result_out <= ALU_result_in;
            Data_out <= Data_in;
            dest_reg_out <= dest_reg_in;
			PC_out<=PC;
			
        end
    end

endmodule
