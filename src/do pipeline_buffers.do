add wave -divider "IF/ID Stage"
add wave uut.IF_ID_PC
add wave uut.IF_ID_instruction

add wave -divider "ID/EX Stage"
add wave uut.ID_EX_PC
add wave uut.ID_EX_BusA
add wave uut.ID_EX_BusB
add wave uut.ID_EX_imm_ext
add wave uut.ID_EX_Rd
add wave uut.BTA_out

add wave -divider "EX/MEM Stage"
add wave uut.EX_MEM_PC
add wave uut.EX_MEM_ALU_result
add wave uut.EX_MEM_BusB
add wave uut.EX_MEM_write_reg

add wave -divider "MEM/WB Stage"
add wave uut.MEM_WB_PC
add wave uut.MEM_WB_DataOut
add wave uut.MEM_WB_dest_reg

add wave -divider "Control Signals"
add wave uut.stall
add wave uut.ForwardA
add wave uut.ForwardB
add wave uut.Exception
add wave clk
