// data_memory
module data_memory (
    input  clk,
    input  [31:0] address,
    input  [31:0] write_data,
    input  mem_write,
    input  mem_read,
    output reg [31:0] read_data
);
 
    reg [31:0] DataMem [0:1023];  // 4KB RAM
 	integer  i;
    // Read data (combinational)
    always @(*) begin
        if (mem_read) begin
            read_data = DataMem[address[31:0]];	 
			$monitor("Time=%0t | mem_read | Address=0x%h | Data=0x%h", $time, address, read_data) ; 
			$display("----------- Data Memory Snapshot -----------");
            for (i = 0; i < 16; i = i + 1) begin
                $display("DataMem[%0d] = 0x%h", i, DataMem[i]);
            end
            $display("--------------------------------------------"); 
			
		end	
        
    end
 
    // Write data (synchronous)
    always @(posedge clk) begin
        if (mem_write) begin
            DataMem[address[31:0]] <= write_data; 
			$monitor("Time=%0t | MEMWRITE | Address=0x%h | Data=0x%h", $time, address, write_data) ; 
			$display("----------- Data Memory Snapshot -----------");
            for (i = 0; i < 16; i = i + 1) begin
                $display("DataMem[%0d] = 0x%h", i, DataMem[i]);
            end
            $display("--------------------------------------------");	
		end
    end
   
endmodule  
 
 
