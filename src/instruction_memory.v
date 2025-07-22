// instruction_memory.v
module instruction_memory(
    input wire [31:0] address,      //PC
    output reg [31:0] instruction   //instruction
);
 
    reg [31:0] memory [0:1023]; // 4KB instruction memory (1024 word)
 
    always @(*) begin
        instruction = memory[address[31:2]]; // word-addressed
    end
 
    initial begin
        integer i;
        for (i = 0; i < 1024; i = i + 1)
            memory[i] = 32'hFFFFFFFF;
 			
		//inst format 	6'opcode | 4'Rd | 4'Rs | 4'Rt | 14'Imm
		
		
        // Sample program  1
		// Read & Write in same cycle ----- Forwording 	
		/*
    	memory[0]  = {6'b000101, 4'd1, 4'd0, 4'd0, 14'd5};    // ADDI R1, R0, 5	    
    	memory[1]  = {6'b000101, 4'd2, 4'd0, 4'd0, 14'd10};   // ADDI R2, R0, 10
    	memory[2]  = {6'b000001, 4'd3, 4'd1, 4'd2, 14'd0};    // ADD R3, R1, R2	 
    	memory[3]  = {6'b000010, 4'd4, 4'd2, 4'd1, 14'd0};    // SUB R4, R2, R1
    	memory[4]  = {6'b000000, 4'd5, 4'd1, 4'd2, 14'd0};    // OR R5, R1, R2
    	memory[5]  = {6'b000100, 4'd6, 4'd1, 4'd0, 14'd3};    // ORI R6, R1, 3
   	memory[6]  = {6'b000011, 4'd7, 4'd1, 4'd2, 14'd0};    // CMP R7, R1, R2
    	memory[7]  = {6'b000011, 4'd8, 4'd2, 4'd2, 14'd0};    // CMP R8, R2, R2
    	memory[8]  = {6'b000011, 4'd9, 4'd2, 4'd1, 14'd0};    // CMP R9, R2, R1	
		*/
		
		
		// Example program with memory load/store and ALU ops
		// Sample program  2  
		// LWD & SWD AND Rd is odd 
		 /*
	    memory[0]  = {6'b000101, 4'd1, 4'd0, 4'd0, 14'd8};     // ADDI R1, R0, 8      ; R1 = 8
	    memory[1]  = {6'b000101, 4'd2, 4'd0, 4'd0, 14'd20};    // ADDI R2, R0, 20     ; R2 = 20	  
		memory[2]  = {6'b001001, 4'd8, 4'd1, 4'd0, 14'd4};     // SDW R8, 4(R1)       ; MEM[R1 + 4] = R8    MEM[5+R1] = R9
	    memory[3]  = {6'b001001, 4'd6, 4'd0, 4'd0, 14'd0};     // SDW R6, 0(R0)       ; MEM[0] = R6			MEM[1+R0] = R7
	    memory[4]  = {6'b001000, 4'd3, 4'd0, 4'd0, 14'd0};     // LDW R3, 0(R0)       ;	Exception
	    memory[5]  = {6'b001000, 4'd4, 4'd1, 4'd0, 14'd4};     // LDW R4, 4(R1)       ; R4 = MEM[8+4]   R5 = MEM[R1+5]
	    memory[6]  = {6'b000001, 4'd5, 4'd2, 4'd3, 14'd0};     // ADD R5, R2, R3      ; R5 = R2 + R3
	    memory[7]  = {6'b000010, 4'd6, 4'd5, 4'd4, 14'd0};     // SUB R6, R5, R4      ; R6 = R5 - R4
	    memory[8]  = {6'b000000, 4'd7, 4'd3, 4'd4, 14'd0};     // OR R7, R3, R4       ; R7 = R3 | R4
	    memory[9]  = {6'b000011, 4'd8, 4'd3, 4'd4, 14'd0};     // CMP R8, R3, R4      ; R8 = R3 & R4  
		 */
		
		// Example program with memory load/store and ALU ops and jump 
		// Sample program  3  
		// LWD , SWD, Jump , Branch (precict false (true predict)) , Forwording 
		/*
		memory[0] = {6'd14, 4'd0, 4'd0, 4'd0, 14'd3};      // J + 3                 <- unconditional jump	
		memory[1]  = {6'd5,  4'd1, 4'd0, 4'd0, 14'd20};     // ADDI R1, R0, 20
	    memory[2]  = {6'd5,  4'd2, 4'd0, 4'd0, 14'd40};     // ADDI R2, R0, 40
		memory[3]  = {6'd7,  4'd4, 4'd1, 4'd0, 14'd4};     // SW R4, [R1 + 4]     <- normal store
	    memory[4]  = {6'd6,  4'd3, 4'd1, 4'd0, 14'd4};      // LD R3, [R1 + 4]      <- load
	    memory[5]  = {6'd0,  4'd4, 4'd3, 4'd2, 14'd0};      // OR R4, R3, R2        <- causes stall (uses result of load)
		memory[6]  = {6'd10, 4'd0, 4'd6, 4'd0, 14'd3};      // BZ R6, 3          <- if R6 == 0, skip next 3
	    memory[7]  = {6'd8,  4'd6, 4'd1, 4'd0, 14'd4};      // LWD R6, [R1 + 4]     <- double load
	    memory[8]  = {6'd2,  4'd8, 4'd6, 4'd2, 14'd0};      // SUB R8, R6, R2       <- uses double load)
	    memory[9]  = {6'd9,  4'd6, 4'd1, 4'd0, 14'd8};      // SWD R6, [R1 + 8]     <- double store
	    memory[10]  = {6'd7,  4'd4, 4'd1, 4'd0, 14'd12};     // SW R4, [R1 + 12]     <- normal store
	    memory[11]  = {6'd5,  4'd7, 4'd0, 4'd0, 14'd100};    // ADDI R7, R0, 100     
	    memory[12] = {6'd4,  4'd8, 4'd0, 4'd0, 14'd200};    // ORI R8, R0, 200    
	    memory[13] = {6'd5,  4'd9, 4'd8, 4'd0, 14'd255};    // ADDI R9, R8, 255     
		 */
		



			
		
		
	 
	// Test 4: Control flow instructions (BLZ, BGZ, J, JR, CALL, 
	//forwaord from load double word(stall between rounds and another stall between Round2 and next instuction ))
	// Tests branch, jump, and subroutine handling with pipeline forwarding
	memory[0] = {6'b000101, 4'd1, 4'd0, 4'd0, 14'd5};     // ADDI R1, R0, 5    ; R1=5
	memory[1] = {6'b000101, 4'd2, 4'd0, 4'd0, 14'd10};    // ADDI R2, R0, 10   ; R2=10
	memory[2] = {6'b000101, 4'd3, 4'd0, 4'd0, 14'd16379}; // ADDI R3, R0, -5  ; R3=-5 (16379 = -5 in 14-bit 2's comp)
	
	// Test BLZ (branch if negative) - should branch
	memory[3] = {6'b001100, 4'd0, 4'd3, 4'd0, 14'd2};     // BLZ R3, 2         ; branch to 3+2=5 (R3<0)
	memory[4] = {6'b000101, 4'd4, 4'd0, 4'd0, 14'd1};     // ADDI R4, R0, 1    ; skipped (should not execute)
	
	// Test BGZ (branch if positive) - should branch
	memory[5] = {6'b000101, 4'd4, 4'd0, 4'd0, 14'd2};     // ADDI R4, R0, 2    ; R4=2 (branch target)
	memory[6] = {6'b001011, 4'd0, 4'd1, 4'd0, 14'd2};     // BGZ R1, 2         ; branch to 6+2=8 (R1>0)
	memory[7] = {6'b000101, 4'd5, 4'd0, 4'd0, 14'd1};     // ADDI R5, R0, 1    ; skipped
	
	// Test J (unconditional jump)
	memory[8] = {6'b000101, 4'd5, 4'd0, 4'd0, 14'd2};     // ADDI R5, R0, 2    ; R5=2 (branch target)
	memory[9] = {6'b001110, 4'd0, 4'd0, 4'd0, 14'd2};     // J 2               ; jump to 9+2=11
	memory[10] = {6'b000101, 4'd6, 4'd0, 4'd0, 14'd1};    // ADDI R6, R0, 1    ; skipped
	
	// Test CALL and JR (subroutine call/return)
	memory[11] = {6'b000101, 4'd6, 4'd0, 4'd0, 14'd3};    // ADDI R6, R0, 3    ; R6=3
	memory[12] = {6'b001111, 4'd0, 4'd0, 4'd0, 14'd4};    // CLL 4            ; call to 12+4=16 (saves 13 in R14)
	memory[13] = {6'b000101, 4'd10, 4'd0, 4'd0, 14'd100}; // ADDI R10, R0, 100 ; R10=100 (return point)
	
	// Test JR with R15 (PC read)
	memory[14] = {6'b000101, 4'd11, 4'd14, 4'd0, 14'd20};  // ADDI R11, R14, 16  ; R11=52+20=72/4 (18)
	memory[15] = {6'b001101, 4'd0, 4'd11, 4'd0, 14'd0};   // JR R11            ; jump to R11=18
	
	// Subroutine (address 16)
	memory[16] = {6'b000001, 4'd7, 4'd1, 4'd2, 14'd0};    // ADD R7, R1, R2    ; R7=5+10=15
	memory[17] = {6'b001101, 4'd0, 4'd14, 4'd0, 14'd0};   // JR R14            ; return to caller (address 13)
	
	memory[18] = {6'b001110, 4'd0, 4'd0, 4'd0, 14'd3};   // J 3      ; 3 * 4 + current PC = 72    (Address: 21 )           
	memory[19] = {6'b001110, 4'd0, 4'd0, 4'd0, 14'd16366};   // J -1              ; Skipped	
	memory[20] = {6'b001000, 4'd3, 4'd1, 4'd0, 14'd0};   // LDW 3(1, 0)             ; Skipped
	memory[21] = {6'b001001, 4'd6, 4'd8, 4'd0, 14'd0};   // SDW R6(R8, 0)  3 --> r1(add : 0 + 8)  ||  15 -->R2(add :1 + 8)         ;	
	memory[22] = {6'b000111, 4'd10, 4'd0, 4'd0, 14'd4};   //SW R10(R0, 4)  100 -->>	(0 + 4)							 //SW  	R10()
	memory[23] = {6'b001000, 4'd8, 4'd8, 4'd0, 14'd0};   //  LDW R8(R8, 0)   r1(Add : 8 + 0)(3)  >> R8   ||  r2(ADD : 3 + 1)(100) >> R9        
	memory[24] = {6'b000011, 4'd1, 4'd9, 4'd8, 14'd0};   // CMP R1,R9,R8 
	  
	
    end
 
endmodule
 
 
// instruction_memory_tb.v
module instruction_memory_tb;
 
    reg [31:0] address;
    wire [31:0] instruction;
 
    instruction_memory uut (
        .address(address),
        .instruction(instruction)
    );
 
     initial begin
        $display("Time\tAddress\t\tInstruction");
        $monitor("%0t\t%h\t%h", $time, address, instruction);
 
        address = 32'd0;
        repeat (6) begin
            #10;
            address = address + 4;
        end
 
        #10 $finish;
    end
 
endmodule
 
 