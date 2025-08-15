`include "src/pipeline-risc-v.v"
`timescale 1ns/100ps

module top_module;
    reg clk = 0;
    always #5 clk = ~clk;  // Clock de 10 unidades
       
    // Testbench
    reg reset;

    initial begin
        reset = 1;
        #10 reset = 0;
        #250 $finish;
    end


    // IF stage (Saídas)
    wire [63:0] if_pc;
    wire [31:0] if_instruction;

    // IF/ID Register (Saídas para ID)
    wire [63:0] if_id_pc;
    wire [31:0] if_id_instruction;

    // ID stage (Saídas)
    wire [4:0]  id_dec_rs1, id_dec_rs2, id_dec_rd;
    wire [6:0]  id_dec_opcode, id_dec_funct7;
    wire [2:0]  id_dec_funct3;
    wire [63:0] id_imm_generated;

    wire [63:0] id_rs1_data_from_rf;
    wire [63:0] id_rs2_data_from_rf;

    wire        id_ctrl_reg_write;
    wire        id_ctrl_mem_to_reg;
    wire        id_ctrl_mem_read;
    wire        id_ctrl_mem_write;
    wire [3:0]  id_ctrl_alu_op;
    wire        id_ctrl_alu_src;
    wire        id_ctrl_branch;

    //  Sinais de controle após o MUX (podem ser NOPs)
    wire        id_ctrl_reg_write_final;
    wire        id_ctrl_mem_to_reg_final;
    wire        id_ctrl_mem_read_final;
    wire        id_ctrl_mem_write_final;
    wire        id_ctrl_alu_src_final;
    wire        id_ctrl_branch_final;

    // ID/EX Register (Saídas para EX)
    wire [63:0] id_ex_pc;
    wire [63:0] id_ex_rs1_data;
    wire [63:0] id_ex_rs2_data;
    wire [4:0]  id_ex_rs1;
    wire [4:0]  id_ex_rs2;
    wire [4:0]  id_ex_rd;
    wire [63:0] id_ex_imm;
    wire [6:0]  id_ex_opcode;
    wire [2:0]  id_ex_funct3;
    wire [6:0]  id_ex_funct7;
    wire        id_ex_reg_write;
    wire        id_ex_mem_read;
    wire        id_ex_mem_write;
    wire        id_ex_branch;
    wire        id_ex_alu_src;
    wire        id_ex_mem_to_reg;
    wire [3:0]  id_ex_alu_op;

    // EX stage (Saídas)
    wire [63:0] ex_alu_result;
    wire [63:0] ex_branch_addr;
    wire        ex_branch_taken;

    // EX/MEM Register (Saídas para MEM)
    wire [63:0] ex_mem_alu_result;
    wire [63:0] ex_mem_write_data;
    wire [4:0]  ex_mem_rd;
    wire        ex_mem_reg_write;
    wire        ex_mem_to_reg;
    wire        ex_mem_read;
    wire        ex_mem_write;

    // MEM stage (Saídas)
    wire [63:0] mem_read_data_from_dmem;

    // MEM/WB Register (Saídas para WB)
    wire [63:0] mem_wb_read_data;
    wire [63:0] mem_wb_alu_result;
    wire [4:0]  mem_wb_rd;
    wire        mem_wb_reg_write;
    wire        mem_wb_mem_to_reg;

    // WB stage (Saídas)
    wire [63:0] wb_write_data;

    // Sinais de Forwarding
    wire [1:0] forward_a, forward_b;

    // NOVOS: Sinais de Hazard Detection
    wire stall;
    wire pc_write;
    wire if_id_write;
    wire control_mux_sel;

    // ========================================
    // HAZARD DETECTION UNIT
    // ========================================
    hazard_detection_unit hazard_unit (
        .if_id_rs1(id_dec_rs1),
        .if_id_rs2(id_dec_rs2),
        .id_ex_rd(id_ex_rd),
        .id_ex_mem_read(id_ex_mem_read),
        .stall(stall)
    );

    // PIPELINE CONTROL
    pipeline_control pipe_ctrl (
        .stall(stall),
        .branch_taken(ex_branch_taken),
        .pc_write(pc_write),
        .if_id_write(if_id_write),
        .control_mux(control_mux_sel)
    );

    // CONTROL MUX (seleciona entre controles normais ou NOP)
    control_mux ctrl_mux (
        .control_mux_sel(control_mux_sel),
        .reg_write_in(id_ctrl_reg_write),
        .mem_read_in(id_ctrl_mem_read),
        .mem_write_in(id_ctrl_mem_write),
        .branch_in(id_ctrl_branch),
        .alu_src_in(id_ctrl_alu_src),
        .mem_to_reg_in(id_ctrl_mem_to_reg),
        .reg_write_out(id_ctrl_reg_write_final),
        .mem_read_out(id_ctrl_mem_read_final),
        .mem_write_out(id_ctrl_mem_write_final),
        .branch_out(id_ctrl_branch_final),
        .alu_src_out(id_ctrl_alu_src_final),
        .mem_to_reg_out(id_ctrl_mem_to_reg_final)
    );

    // Forwarding Unit
    forwarding_unit forward_unit (
        .id_ex_rs1(id_ex_rs1),
        .id_ex_rs2(id_ex_rs2),
        .ex_mem_rd(ex_mem_rd),
        .mem_wb_rd(mem_wb_rd),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_reg_write(mem_wb_reg_write),
        .forward_a(forward_a),
        .forward_b(forward_b)
    );
    
    // ========================================
    // Instâncias do pipeline 
    // ========================================

    // Program Counter 
    program_counter pc (
        .clk(clk),
        .reset(reset),
        .pc_write(pc_write),        // NOVO
        .branch_taken(ex_branch_taken),
        .branch_addr(ex_branch_addr),
        .pc(if_pc)
    );

    // Instruction Memory
    instruction_memory imem (
        .inst_address(if_pc),       
        .instruction(if_instruction)
    );

    // IF/ID Register 
    if_id_register if_id (
        .clk(clk),
        .reset(reset),
        .if_id_write(if_id_write),  
        .flush(ex_branch_taken),    
        .pc_in(if_pc),
        .instruction_in(if_instruction),
        .pc_out(if_id_pc),
        .instruction_out(if_id_instruction)
    );

    // Decoder
    decoder dec_inst (
        .instruction(if_id_instruction),
        .rs1(id_dec_rs1),
        .rs2(id_dec_rs2),
        .rd(id_dec_rd),
        .opcode(id_dec_opcode),
        .funct3(id_dec_funct3),
        .funct7(id_dec_funct7)
    );

    // Immediate Generator
    imm_gen imm_gen_inst (
        .instruction(if_id_instruction),
        .imm_out(id_imm_generated)
    );

    // Register File
    register_file rf (
        .clk(clk),
        .rs1(id_dec_rs1),
        .rs2(id_dec_rs2),
        .rd(mem_wb_rd),
        .write_data(wb_write_data),
        .reg_write(mem_wb_reg_write),
        .rs1_data(id_rs1_data_from_rf), 
        .rs2_data(id_rs2_data_from_rf)  
    );

    // Control Unit
    control_unit cu (
        .opcode(id_dec_opcode),
        .reg_write(id_ctrl_reg_write),
        .mem_to_reg(id_ctrl_mem_to_reg),
        .mem_read(id_ctrl_mem_read),
        .mem_write(id_ctrl_mem_write),
        .alu_src(id_ctrl_alu_src),
        .branch(id_ctrl_branch)
    );

    // ALU Control Unit
    alu_control alu_ctrl_inst (
        .opcode(id_dec_opcode),
        .funct3(id_dec_funct3),
        .funct7(id_dec_funct7),
        .alu_op(id_ctrl_alu_op)
    );

    // ID/EX Register 
    id_ex_register id_ex (
        .clk(clk),
        .reset(reset),

        .pc_in(if_id_pc),
        .rs1_data_in(id_rs1_data_from_rf),
        .rs2_data_in(id_rs2_data_from_rf),
        .rs1_in(id_dec_rs1),       
        .rs2_in(id_dec_rs2),       
        .rd_in(id_dec_rd),         
        .imm_in(id_imm_generated), 
        .opcode_in(id_dec_opcode), 
        .funct3_in(id_dec_funct3), 
        .funct7_in(id_dec_funct7), 
        .reg_write_in(id_ctrl_reg_write_final),    // USA SINAL FINAL
        .mem_read_in(id_ctrl_mem_read_final),      // USA SINAL FINAL
        .mem_write_in(id_ctrl_mem_write_final),    // USA SINAL FINAL
        .branch_in(id_ctrl_branch_final),          // USA SINAL FINAL
        .alu_src_in(id_ctrl_alu_src_final),        // USA SINAL FINAL
        .mem_to_reg_in(id_ctrl_mem_to_reg_final),  // USA SINAL FINAL
        .alu_control_in(id_ctrl_alu_op), 

        // Outputs (para EX Stage)
        .pc_out(id_ex_pc),
        .rs1_data_out(id_ex_rs1_data),
        .rs2_data_out(id_ex_rs2_data),
        .rs1_out(id_ex_rs1),
        .rs2_out(id_ex_rs2),
        .rd_out(id_ex_rd),
        .imm_out(id_ex_imm),
        .opcode_out(id_ex_opcode),
        .funct3_out(id_ex_funct3),
        .funct7_out(id_ex_funct7),
        .reg_write_out(id_ex_reg_write),
        .mem_read_out(id_ex_mem_read),
        .mem_write_out(id_ex_mem_write),
        .branch_out(id_ex_branch),
        .alu_src_out(id_ex_alu_src),
        .mem_to_reg_out(id_ex_mem_to_reg),
        .alu_control_out(id_ex_alu_op) 
    );

    // Execute Stage Logic
    ex_stage ex_stg (
        .pc_in(id_ex_pc),
        .rs1_data(id_ex_rs1_data),
        .rs2_data(id_ex_rs2_data),
        .imm_data(id_ex_imm),
        .alu_src(id_ex_alu_src),
        .alu_op(id_ex_alu_op),
        .branch_control(id_ex_branch),
        .ex_mem_alu_result(ex_mem_alu_result),
        .wb_write_data(wb_write_data),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .alu_result(ex_alu_result), 
        .branch_addr(ex_branch_addr),
        .branch_taken(ex_branch_taken) 
    );

    wire [63:0] ex_forwarded_rs2_data;
    
    mux_forward_b store_forward_mux (
        .id_ex_rs2_data(id_ex_rs2_data),
        .ex_mem_alu_result(ex_mem_alu_result),
        .wb_write_data(wb_write_data),
        .forward_b(forward_b),
        .forwarded_rs2_data(ex_forwarded_rs2_data)
    );

    // EX/MEM Register
    ex_mem_register ex_mem_reg (
        .clk(clk),
        .reset(reset),
        .ex_alu_result(ex_alu_result),
        .ex_rs2_data(ex_forwarded_rs2_data), 
        .ex_rd(id_ex_rd),
        .ex_mem_read(id_ex_mem_read),
        .ex_mem_write(id_ex_mem_write),
        .ex_reg_write(id_ex_reg_write),
        .ex_mem_to_reg(id_ex_mem_to_reg),
        .mem_alu_result(ex_mem_alu_result),
        .mem_write_data(ex_mem_write_data),
        .mem_rd(ex_mem_rd),
        .mem_mem_read(ex_mem_read),
        .mem_mem_write(ex_mem_write),
        .mem_reg_write(ex_mem_reg_write),
        .mem_mem_to_reg(ex_mem_to_reg)
    );

    // Data Memory
    data_memory dmem (
        .clk(clk),
        .mem_read(ex_mem_read),
        .mem_write(ex_mem_write),
        .addr(ex_mem_alu_result), 
        .write_data(ex_mem_write_data),
        .read_data(mem_read_data_from_dmem)
    );

    // MEM/WB Register
    mem_wb_register mem_wb_reg (
        .clk(clk),
        .reset(reset),
        .mem_read_data_in(mem_read_data_from_dmem),
        .mem_alu_result_in(ex_mem_alu_result),
        .mem_rd_in(ex_mem_rd),
        .mem_reg_write_in(ex_mem_reg_write),
        .mem_mem_to_reg_in(ex_mem_to_reg),
        .wb_read_data_out(mem_wb_read_data),
        .wb_alu_result_out(mem_wb_alu_result),
        .wb_rd_out(mem_wb_rd),
        .wb_reg_write_out(mem_wb_reg_write),
        .wb_mem_to_reg_out(mem_wb_mem_to_reg)
    );

    // WB MUX (Write Back MUX)
    assign wb_write_data = mem_wb_mem_to_reg ? mem_wb_read_data : mem_wb_alu_result;

    // ========================================
    // DEBUG: Monitor de hazards
    // ========================================
    always @(posedge clk) begin
        if (stall) begin
            $display("Time %0t: HAZARD DETECTED! Stall inserted - rs1=%0d, rs2=%0d, ex_rd=%0d", 
                     $time, id_dec_rs1, id_dec_rs2, id_ex_rd);
        end
        if (ex_branch_taken) begin
            $display("Time %0t: BRANCH TAKEN! Flush inserted", $time);
        end
    end

endmodule