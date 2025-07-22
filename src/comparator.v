module Comparator (
    input  [31:0] A,
    input  [31:0] B,
    output reg zero,
    output reg positive,
    output reg negative
);	


	always @(*) begin  
			
        if (A == B) begin
            zero     = 1;
            positive = 0;
            negative = 0;
        end
        else if ($signed(A) > $signed(B)) begin
            zero     = 0;
            positive = 1;
            negative = 0;
        end
        else begin
            zero     = 0;
            positive = 0;
            negative = 1;
        end
    end

endmodule
