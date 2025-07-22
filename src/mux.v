module mux32 #(parameter N = 2) (  
    input  wire [3:0] in [(1<<N)-1:0],
    input  wire [N-1:0] sel,
    output wire [3:0] out
);
    assign out = in[sel];
endmodule
 
module mux4x1_4bit (
    input  wire [3:0] in0,
    input  wire [3:0] in1,
    input  wire [3:0] in2,
    input  wire [3:0] in3,
    input  wire [1:0] sel,
    output reg  [3:0] out
);
 
    always @(*) begin
        case (sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
        endcase
    end
 
endmodule 
 



module mux4x1_32bit (
    input  [31:0] in0,
    input  [31:0] in1,
    input  [31:0] in2,
    input  [31:0] in3,
    input  [1:0]  sel,
    output reg [31:0] out
);

    always @(*) begin
        case (sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
            default: out = 32'b0;
        endcase
    end

endmodule
