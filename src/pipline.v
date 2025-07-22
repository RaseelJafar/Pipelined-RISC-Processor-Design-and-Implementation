module pipline(    
    input clk,
    input reset
);
	reg stall;
    //==================================================================
    // ==========         FETCH STAGE: Instruction Fetch         =======
    //==================================================================
 
    reg [31:0] PC;
    wire [31:0] instruction;
 
    instruction_memory imem (
        .address(PC),
        .instruction(instruction)
    );
// ===================== IF/ID Pipeline Register =====================
 
	reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_instruction ;
 
	IF_ID if_id ( 
	.kill1(kill1),
	.clk(clk),
	.reset(reset),
	.stall(stall),
	.PC_in(PC),
	.instruction_in(instruction),
	.PC_out(IF_ID_PC),
	.instruction_out(IF_ID_instruction)
	);
 
    //==================================================================
    // ==========         DECODE STAGE: Instruction Decode       =======
    //==================================================================
 
    // Instruction fields
    wire [5:0] opcode = IF_ID_instruction[31:26];
    wire [3:0] Rd     = IF_ID_instruction[25:22];
    wire [3:0] Rs     = IF_ID_instruction[21:18];
    wire [3:0] Rt     = IF_ID_instruction[17:14];
    wire [13:0] imm   = IF_ID_instruction[13:0];
 
    // Immediate extender
    wire ExtOp;
    wire [31:0] imm_ext;
    immediate_extender IMEXT (
        .imm_input(imm),
        .Mostbit(ExtOp),
        .imm_output(imm_ext)
    );
 
    //==================== Control Signals =======================
    wire RegWr, MemRd, MemWr, WBdata, Data_write;
    wire [1:0] RegDst, ALUSrc, PCSrc,RegReadB;  
	wire kill1;
    wire [2:0] ALUOp;
 
    
 
    control_unit CU (  
		.stall(stall),
	    .ForwardA(ForwardA),
		.ForwardB(ForwardB),
        .opcode(opcode),
        .RegDst(RegDst),
        .RegWr(RegWr),
        .ExtOp(ExtOp),
        .ALUOp(ALUOp),
        .ALUSrc(ALUSrc),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .WBdata(WBdata),
        .Data_write(Data_write),
        .RegReadB(RegReadB),
		.Exception(Exception)
    );
 
    //==================== Register File ==========================
    wire [3:0] real_Rt;
    wire [3:0] write_reg;
 
	mux4x1_4bit my_muxRB (
        .in0(Rt),
        .in1(Rd),
        .in2(Rd + 1),
        .in3(Rt),
        .sel(RegReadB),
        .out(real_Rt)
    	);
 
    mux4x1_4bit my_mux (
        .in0(Rd),
        .in1(Rd + 1),
        .in2(uut.RF.Registers[14]),
        .in3(Rt),
        .sel(RegDst),
        .out(write_reg)
    );
 
    wire [31:0] BusA, BusB;
	wire signed [31:0] BusW, BusW_temp;
	wire [31:0] R14_debug;
 
    RegisterFile RF (
        .clk(clk),
        .RegWrite(MEM_WB_RegWr),
        .Rd(MEM_WB_dest_reg),
        .Rs(Rs),
        .Rt(real_Rt),
        .BusW(BusW),
        .BusA(BusA),
        .BusB(BusB),
		//.R14_debug(R14_debug),
		.BusW2(BusW2),
		.RegWrite2(MEM_WB_RegWr || 1)
    );
 
	wire [31:0] BTA_in ; 
	assign BTA_in = IF_ID_PC + (imm_ext<< 2);
	wire zero, positive , negative;
 
 
Comparator comparator (
    .A(outForwardA),
    .B(outForwardB),
    .zero(zero),
    .positive(positive),
    .negative(negative)
);
 
// ===================== ID/EX Pipeline Register =====================   
 
	reg [31:0] ID_EX_PC;
	reg [31:0] BTA_out;
    reg [31:0] ID_EX_BusA, ID_EX_BusB;
    reg [31:0] ID_EX_imm_ext;
    reg [3:0] ID_EX_Rd;
    reg [1:0] ID_EX_RegDst, ID_EX_ALUSrc;
    reg ID_EX_RegWr, ID_EX_MemRd, ID_EX_MemWr, ID_EX_WBdata;
    reg [2:0] ID_EX_ALUOp;
    reg ID_EX_Data_write ; 
 
 
 
ID_EX id_ex(
    .clk(clk),
    .reset(reset),
    .ALUOp_in(ALUOp),
    .ALUSrc_in(ALUSrc),
    .MemRead_in(MemRd),
    .MemWrite_in(MemWr),
    .WBdata_in(WBdata),
    .RegWrite_in(RegWr),
    .Data_write_in(Data_write),
.PC(IF_ID_PC),
.BTA_in(BTA_in),
 
    // Data
    .BusA_in(outForwardA),
    .BusB_in(outForwardB),
    .imm_ext_in(imm_ext),
    .Rd_in(write_reg),
.PC_out(ID_EX_PC),
.BTA_out(BTA_out),
 
    .RegDst_out(ID_EX_RegDst),
    .ALUOp_out(ID_EX_ALUOp),
    .ALUSrc_out(ID_EX_ALUSrc),
    .MemRead_out(ID_EX_MemRd),
    .MemWrite_out(ID_EX_MemWr),
    .WBdata_out(ID_EX_WBdata),
    .RegWrite_out(ID_EX_RegWr),
    .Data_write_out(ID_EX_Data_write),
 
    .BusA_out(ID_EX_BusA),
    .BusB_out(ID_EX_BusB),
    .imm_ext_out(ID_EX_imm_ext),
    .Rd_out(ID_EX_Rd)
);
 
    //==================================================================
    // ==========         EXECUTE STAGE: ALU & Operand Select    =======
    //==================================================================
 
    wire [31:0] ALU_inputB;
	wire [31:0] outForwardA;
	wire [31:0] outForwardB;
 
    mux4x1_32bit my_mux2 (
        .in0(ID_EX_BusB),
        .in1(ID_EX_imm_ext),
        .in2(ID_EX_imm_ext + 1),
        .in3(ID_EX_BusB),
        .sel(ID_EX_ALUSrc),
        .out(ALU_inputB)
    );
 
    wire signed [31:0] ALU_result;
    wire Zero, Negative, Positive;
 
mux4x1_32bit my_mux4 (
    .in0(BusA),
    .in1(ALU_result),
    .in2(BusW_temp),
    .in3(BusW),
    .sel(ForwardA),
    .out(outForwardA)
);
 
mux4x1_32bit my_mux3 (
    .in0(BusB),
    .in1(ALU_result),
    .in2(BusW_temp),
    .in3(BusW),
    .sel(ForwardB),
    .out(outForwardB)
);
 
 
    ALU alu (
        .A(ID_EX_BusA),
        .B(ALU_inputB),
        .ALUOp(ID_EX_ALUOp),
        .ALU_result(ALU_result),
        .Zero(Zero),   
        .Negative(Negative),
        .Positive(Positive)
    );
	wire [1:0] ForwardA, ForwardB;
	reg Exception;
 
	forwarding_unit forward_unit ( 
		.ID_EX_MemRd(ID_EX_MemRd), 
		.ID_EX_PC(ID_EX_PC),
		.IF_ID_PC(IF_ID_PC),
		.opcode(opcode),
		.ID_EX_Rs(Rs),
		.ID_EX_Rt(real_Rt),
		.ID_EX_RegRd(ID_EX_Rd),
		.EX_MEM_RegRd(EX_MEM_write_reg),
		.MEM_WB_RegRd(MEM_WB_dest_reg),
		.ID_EX_RegWrite(ID_EX_RegWr),
		.EX_MEM_RegWrite(EX_MEM_RegWr),
		.MEM_WB_RegWrite(MEM_WB_RegWr),
		.Datawrite(MEM_WB_Data_write),
		.ForwardA(ForwardA),
		.ForwardB(ForwardB),
		.stall(stall),
		.Exception(Exception)
		); 

	Exception try(
		.opcode(opcode),
		.IF_ID_RegDest(IF_ID_instruction[25:22]),
		.exception(Exception)
		);
 
 
// ===================== EX/MEM Pipeline Register =====================
    reg [31:0] EX_MEM_ALU_result;
    reg [31:0] EX_MEM_BusB;
    reg [3:0] EX_MEM_write_reg;
    reg EX_MEM_RegWr, EX_MEM_MemRd, EX_MEM_MemWr, EX_MEM_WBdata;
    reg EX_MEM_Data_write;
    reg [31:0] EX_MEM_PC ;
 
EX_MEM ex_mem(
    .clk(clk),
    .reset(reset),
    // Control
    .MemRead_in(ID_EX_MemRd),
    .MemWrite_in(ID_EX_MemWr),
    .WBdata_in(ID_EX_WBdata),
    .RegWrite_in(ID_EX_RegWr),
    .Data_write_in(ID_EX_Data_write),
 
    // Data
    .ALU_result_in(ALU_result),
    .BusB_in(ID_EX_BusB),
    .dest_reg_in(ID_EX_Rd),
.PC(ID_EX_PC),
 
    .MemRead_out(EX_MEM_MemRd),
    .MemWrite_out(EX_MEM_MemWr),
    .WBdata_out(EX_MEM_WBdata),
    .RegWrite_out(EX_MEM_RegWr),
    .Data_write_out(EX_MEM_Data_write),
 
    .ALU_result_out(EX_MEM_ALU_result),
    .BusB_out(EX_MEM_BusB),
    .dest_reg_out(EX_MEM_write_reg),
.PC_out(EX_MEM_PC)
);
 
    //==================================================================
    // ==========         MEMORY STAGE: Load/Store              =======
    //==================================================================
 
    wire [31:0] MemDataOut;
 
    data_memory DMEM (
        .clk(clk),
        .address(EX_MEM_ALU_result),
        .write_data(EX_MEM_BusB),
        .mem_write(EX_MEM_MemWr),
        .mem_read(EX_MEM_MemRd),
        .read_data(MemDataOut)
    ); 
	assign BusW_temp = (EX_MEM_WBdata) ? MemDataOut : EX_MEM_ALU_result;
// ===================== MEM/WB Pipeline Register =====================
    reg [31:0] MEM_WB_DataOut;
    reg [3:0] MEM_WB_dest_reg;
    reg MEM_WB_RegWr, MEM_WB_WBdata;
    reg MEM_WB_Data_write;
    reg [31:0] MEM_WB_PC ;
 
MEM_WB mem_wb (
    .clk(clk),
    .reset(reset),
 
    //input WBdata_in,
    .RegWrite_in(EX_MEM_RegWr),
    .Data_write_in(EX_MEM_Data_write),
 
    //input [31:0] ALU_result_in,
    .Data_in(BusW_temp),
    .dest_reg_in(EX_MEM_write_reg),
.PC(EX_MEM_PC),
 
    //output reg WBdata_out,
    .RegWrite_out(MEM_WB_RegWr),
    .Data_write_out(MEM_WB_Data_write),
 
    //output reg [31:0] ALU_result_out,
    .Data_out(MEM_WB_DataOut),
    .dest_reg_out(MEM_WB_dest_reg),
.PC_out(MEM_WB_PC)
);
 
    //==================================================================
    // ==========         WRITE BACK STAGE                       =======
    //==================================================================
// Data
	assign BusW  = (MEM_WB_Data_write) ? MEM_WB_PC + 4 : MEM_WB_DataOut;
	wire [31:0] BusW2; 
	assign BusW2  = MEM_WB_PC; 
 
    //==================================================================
    // ==========         PROGRAM COUNTER & CONTROL LOGIC       =======
    //==================================================================
 
    PC_ctrl pc_control (
        .OPcode(opcode),
        .zero(zero),
        .positive(positive),
        .negative(negative),
        .PCSrc(PCSrc),
		.kill1(kill1)
    );
 
    Program_Counter pc (
		.IF_ID_PC(IF_ID_PC),
		.BTA(BTA_in),
		.stall(stall),
        .clk(clk),
        .reset(reset),
        .PCSrc(PCSrc),
        .imm_ext(imm_ext),
        .reg_addr(outForwardA),
        .PC(PC));
 
endmodule
 	 
module pip_tb3;	 

 
    reg clk, reset;
 
    // Instantiate the processor
    pipline uut (
        .clk(clk),
        .reset(reset)
    ); 
	
	wire [31:0] PC = uut.PC; 
	// Instruction fields
    wire [5:0] opcode = uut.opcode;
    wire [3:0] Rd = uut.Rd;
    wire [3:0] Rs = uut.Rs;
    wire [3:0] Rt = uut.Rt;	

    // Pipeline registers
    wire [31:0] IF_ID_PC = uut.IF_ID_PC;
    //wire [31:0] IF_ID_instruction = uut.IF_ID_instruction;
    wire [31:0] ID_EX_PC = uut.ID_EX_PC;
    wire [31:0] EX_MEM_PC = uut.EX_MEM_PC;
    wire [31:0] MEM_WB_PC = uut.MEM_WB_PC;
    
    // Register file
    wire [31:0] BusA = uut.BusA;
    wire [31:0] BusB = uut.BusB;
    wire [31:0] BusW = uut.BusW;
    wire [3:0] write_reg = uut.write_reg;
    
    // ALU
    wire [31:0] ALU_result = uut.ALU_result;
    wire Zero = uut.Zero;
    wire Negative = uut.Negative;
    wire Positive = uut.Positive;
 
    // Control signals
    wire [1:0] RegDst = uut.RegDst;
    wire RegWr = uut.RegWr;
    wire [1:0] RegReadB = uut.RegReadB;
    wire MemWr = uut.MemWr;
    wire MemRd = uut.MemRd;
    wire [1:0] ALUSrc = uut.ALUSrc;
    wire [2:0] ALUOp = uut.ALUOp;
    wire [1:0] PCSrc = uut.PCSrc;
    wire WBdata = uut.WBdata;
    wire Data_write = uut.Data_write;
	wire ExtOp = uut.ExtOp;
	
	wire [31:0] BTA_out = uut.BTA_out;
    wire [31:0] ID_EX_BusA = uut.ID_EX_BusA;
	wire [31:0] ID_EX_BusB = uut.ID_EX_BusB;
    wire [31:0] ID_EX_imm_ext = uut.ID_EX_imm_ext;
    wire [3:0] ID_EX_Rd = uut.ID_EX_Rd;
    wire [1:0] ID_EX_RegDst = uut.ID_EX_RegDst;
	wire [1:0] ID_EX_ALUSrc= uut.ID_EX_ALUSrc;
    wire ID_EX_RegWr = uut.ID_EX_RegWr;
	wire ID_EX_MemRd = uut.ID_EX_MemRd;
	wire ID_EX_MemWr=uut.ID_EX_MemWr;
	wire ID_EX_WBdata = uut.ID_EX_WBdata;
    wire [2:0] ID_EX_ALUOp = uut.ID_EX_ALUOp;
    wire ID_EX_Data_write= uut.ID_EX_Data_write;  
	wire [31:0] BTA_in = uut.BTA_in ;;
    
    // Forwarding
    wire [1:0] ForwardA = uut.ForwardA;
    wire [1:0] ForwardB = uut.ForwardB;
    
    // Hazard detection
    wire stall = uut.stall;
    wire kill1 = uut.kill1;
    wire Exception = uut.Exception;
    
    // Memory
    wire [31:0] MemDataOut = uut.MemDataOut;
	
	wire [31:0] EX_MEM_ALU_result = uut.EX_MEM_ALU_result;
    wire [31:0] EX_MEM_BusB = uut.EX_MEM_BusB;
    wire [3:0] EX_MEM_write_reg = uut.EX_MEM_write_reg;
    wire EX_MEM_RegWr = uut.EX_MEM_RegWr	;
	wire EX_MEM_MemRd = uut.EX_MEM_MemRd; 
	wire EX_MEM_MemWr = uut.EX_MEM_MemWr;
	wire EX_MEM_WBdata = uut.EX_MEM_WBdata;
    wire EX_MEM_Data_write = uut.EX_MEM_Data_write;
    
    // =============================================
    // Clock generation and test sequence
    // =============================================
	
	integer stall_cycles = 0;
	integer executed_instructions = 0;
	integer cycle_count = 0;
	integer load_count = 0;
	integer store_count = 0;
	integer alu_count = 0;
	integer control_count = 0;
	integer store_count_Db =0 ;
	integer load_count_Db = 0;
	integer kill_cycles =0;
	

 
    // Initialize register file
    initial begin
        integer i;
        for (i = 0; i < 16; i = i + 1)
            uut.RF.Registers[i] = i;
 
        uut.RF.Registers[15] = 32'h00000000; // R15 = PC  
		
    end	 
	
	initial begin
    	$dumpfile("pipeline.vcd");
    	$dumpvars(0, pip_tb3); // Dump all signals in testbench
	end
 
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
 
    // Reset and simulation control
    initial begin
        reset = 1;
        #10 reset = 0;
    end 
	
	reg done_flag = 0;
	integer i;

	always @(posedge clk) begin
	    if (uut.IF_ID_instruction == 32'hFFFFFFFF && done_flag == 0) begin
	        done_flag = 1; 
	    end
	end
	
	initial begin
	    wait (done_flag == 1); 
	    #30;
		cycle_count=cycle_count-1;
		load_count_Db = load_count_Db / 2 ;
		store_count_Db = store_count_Db /2 ;
		executed_instructions = load_count + store_count + load_count_Db + store_count_Db + alu_count + control_count ;  
	    $display("===========================================");
	    $display("Total Number of Cycles = %0d", cycle_count);
	    $display("Total Executed Instructions = %0d", executed_instructions);
	    $display("Total Stall Cycles = %0d", stall_cycles);	 
		$display("Total kill Cycles = %0d", kill_cycles);
	    $display("Total Load Instructions  = %0d", load_count);
	    $display("Total Store Instructions = %0d", store_count);
	    $display("Total Load Instructions Double  = %0d", load_count_Db);
	    $display("Total Store Instructions Double  = %0d", store_count_Db);
	    $display("Total ALU Instructions   = %0d", alu_count);
	    $display("Total Control Instructions = %0d", control_count);
	    $display("===========================================");
		
	    for (i = 1; i <= 15; i = i + 1) begin
	        $display("R%0d = %0d", i, uut.RF.Registers[i]);
	    end
		
	    $finish;
	end


 
	always @(posedge uut.Exception) begin
		$display("Time = %0t  ERROR : Invalid destination register (odd Rd = %0d) during %s instruction at PC = %0d",$time, uut.Rd, (uut.opcode == 6'b001000 ? "LDW" : "SDW"), uut.PC);
	end
	
   initial begin
  	$display("Time | Control Signals");
  	$display("-------------------------------------------------------------");
  	$monitor(
    "Time=%0t PC=%h BusW = %0d \n>> Control:RegDst=%b|RegWr=%b|RegReadB=%b|MemWr=%b|MemRd=%b|ALUSrc=%b|ALUOp=%b|PCSrc=%b|WBdata=%b|Data_write=%b|ForwardA=%b|ForwardB=%b | zero=%b|positive=%b|negative=%b",
    $time, uut.PC,uut.BusW,
    uut.RegDst, uut.RegWr, uut.RegReadB, uut.MemWr, uut.MemRd, uut.ALUSrc, uut.ALUOp, uut.PCSrc, uut.WBdata, uut.Data_write,
    uut.ForwardA, uut.ForwardB,uut.zero,uut.positive,uut.negative); 
	$monitor("Time=%0t R1=%d | R2=%d | R3=%d | R4=%d | R5=%d | R6=%d | R7=%d | R8=%d | R9=%d |R10=%d | R11=%d | R12=%d | R13=%d |R14=%d | R15=%d |",
             $time,
            uut.RF.Registers[1], uut.RF.Registers[2], uut.RF.Registers[3], uut.RF.Registers[4],
            uut.RF.Registers[5], uut.RF.Registers[6], uut.RF.Registers[7], uut.RF.Registers[8],
            uut.RF.Registers[9], uut.RF.Registers[10], uut.RF.Registers[11], uut.RF.Registers[12],uut.RF.Registers[13],uut.RF.Registers[14], uut.RF.Registers[15]);
	end

 
    // Pipeline stages and buffer monitoring
    initial begin
        $display("=======================================================================");
        $display("================= Pipeline Buffer Values ==============================");
        $display("=======================================================================");
        $monitor("Time=%0t\n\
IF/ID:   PC=%h, Instr=%h\n\ Rs=%d, Rt=%d\n\
ID/EX:   PC=%h, BusA=%h, BusB=%h, Imm=%h, Rd=%h, BTA=%h\n\
EX/MEM:  PC=%h, ALU=%h, BusB=%h, DestReg=%h, EX_MEM_BusB=%d\n\
MEM/WB:  PC=%h, DataOut=%h, DestReg=%h\n\
ForwardA=%b, ForwardB=%b , stall=%b , Exception=%b\n\
----------------------------------------------------------------------",
            $time,
            uut.IF_ID_PC, uut.IF_ID_instruction, uut.IF_ID_instruction[21:18], uut.IF_ID_instruction[17:14],
            uut.ID_EX_PC, uut.ID_EX_BusA, uut.ID_EX_BusB, uut.ID_EX_imm_ext, uut.ID_EX_Rd, uut.BTA_out,
            uut.EX_MEM_PC, uut.EX_MEM_ALU_result, uut.EX_MEM_BusB, uut.EX_MEM_write_reg,  uut.EX_MEM_BusB,
            uut.MEM_WB_PC, uut.MEM_WB_DataOut, uut.MEM_WB_dest_reg,
            uut.ForwardA, uut.ForwardB, uut.stall,uut.Exception
        );
 
    end

	always @(posedge clk) begin
	  if (uut.stall == 1) begin
	    stall_cycles = stall_cycles + 1;
	  end
	end	  
	
	always @(posedge clk) begin
	  if (uut.kill1 == 1) begin
	    kill_cycles = kill_cycles + 1;
	  end
	end	
	
	
	always @(posedge clk) begin
	    cycle_count = cycle_count + 1;
	end	
	
	
	always @ (posedge clk)begin	   // to calculate store and Double  
			if ((uut.MemWr==1) && (uut.opcode == 6'd7))
				begin
					store_count = store_count +1 ;
				end
			else if (uut.MemWr && uut.opcode == 6'd9)
				begin
					store_count_Db = store_count_Db +1 ;
				end
	end
	
	
	always @ (posedge clk)begin // to calculate load and Double 
			if (uut.MemRd && uut.opcode == 6'd6)
				begin
					load_count = load_count +1 ;
				end
			else if (uut.MemRd && uut.opcode == 6'd8)
				begin
					load_count_Db = load_count_Db +1 ;
				end
	end
	
	always @(posedge clk) begin    // to calculate ALU  
		
	    if ((uut.RegWr==1) && (uut.opcode == 6'd0|| uut.opcode ==6'd1 ||
			uut.opcode == 6'd2 || uut.opcode ==6'd3 || uut.opcode ==6'd4 || uut.opcode ==6'd5) &&
			((uut.ID_EX_MemRd == 0) || (uut.ForwardA!=1 || uut.ForwardB!=1)) && ( uut.IF_ID_instruction != 32'b0 ))
	        begin
				alu_count = alu_count + 1;
	    end
	end	
	
	always @(posedge clk) begin   // to calculate Jump and Branch 
	    if ((uut.opcode == 6'd10|| uut.opcode ==6'd11 || uut.opcode == 6'd12 || uut.opcode ==6'd13) && ((uut.ID_EX_MemRd == 0) && (uut.ForwardA!=1 || uut.ForwardB!=1))) begin
	        control_count = control_count + 1;
	    end
	else if (uut.opcode == 6'd14|| uut.opcode ==6'd15)
		begin
			
			control_count = control_count + 1;
		end
	end	

endmodule 




