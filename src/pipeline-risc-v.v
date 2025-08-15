`timescale 1ns/100ps
// Forwarding Unit
module forwarding_unit(
    input [4:0]  id_ex_rs1,
    input [4:0]  id_ex_rs2, 
    input [4:0]  ex_mem_rd,
    input [4:0]  mem_wb_rd,
    input        ex_mem_reg_write,
    input        mem_wb_reg_write,
    output reg [1:0] forward_a,
    output reg [1:0] forward_b
);
    always @(*) begin
        // Forward A (rs1)
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs1)) begin
            forward_a = 2'b10; // Forward do EX/MEM
        end else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs1) && 
                     !((ex_mem_reg_write) && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs1))) begin
            forward_a = 2'b01; // Forward do MEM/WB
        end else begin
            forward_a = 2'b00; // Sem forwarding
        end

        // Forward B (rs2)
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs2)) begin
            forward_b = 2'b10; // Forward do EX/MEM
        end else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs2) && 
                     !((ex_mem_reg_write) && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs2))) begin
            forward_b = 2'b01; // Forward do MEM/WB
        end else begin
            forward_b = 2'b00; // Sem forwarding
        end
    end
endmodule

// MUX para Forwarding A
module mux_forward_a(
    input [63:0] id_ex_rs1_data,
    input [63:0] ex_mem_alu_result,
    input [63:0] wb_write_data,
    input [1:0]  forward_a,
    output reg [63:0] alu_input_a
);
    always @(*) begin
        case (forward_a)
            2'b00: alu_input_a = id_ex_rs1_data;      // Sem forwarding
            2'b01: alu_input_a = wb_write_data;       // Forward do MEM/WB
            2'b10: alu_input_a = ex_mem_alu_result;   // Forward do EX/MEM
            default: alu_input_a = id_ex_rs1_data;
        endcase
    end
endmodule

// MUX para Forwarding B
module mux_forward_b(
    input [63:0] id_ex_rs2_data,
    input [63:0] ex_mem_alu_result,
    input [63:0] wb_write_data,
    input [1:0]  forward_b,
    output reg [63:0] forwarded_rs2_data
);
    always @(*) begin
        case (forward_b)
            2'b00: forwarded_rs2_data = id_ex_rs2_data;      // Sem forwarding
            2'b01: forwarded_rs2_data = wb_write_data;       // Forward do MEM/WB
            2'b10: forwarded_rs2_data = ex_mem_alu_result;   // Forward do EX/MEM
            default: forwarded_rs2_data = id_ex_rs2_data;
        endcase
    end
endmodule

module program_counter(
    input clk,
    input reset,
    input pc_write,
    input branch_taken,
    input [63:0] branch_addr,
    output reg [63:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 0;
        else if (pc_write) begin
            if (branch_taken) begin
                pc <= branch_addr;
                $display("Time %0t: PC changed due to BRANCH_TAKEN. New PC = %0d (0x%h)", $realtime, branch_addr, branch_addr);
            end else begin
                pc <= pc + 4;
                $display("Time %0t: PC advanced to %0d (0x%h)", $realtime, pc + 4, pc + 4);
            end
        end else begin
            $display("Time %0t: PC STALLED at %0d (0x%h)", $realtime, pc, pc);
        end
    end
endmodule

module instruction_memory(
    input  [63:0] inst_address,
    output [31:0] instruction
);
    reg [31:0] memory [0:15]; // Reduzido de 20 para 15

    assign instruction = memory[inst_address[5:2]];

    initial begin
        // PROGRAMA OTIMIZADO SEM NOPs DESNECESSÁRIOS
        memory[0]  = 32'h00506093; // ori x1, x0, 5           (x1 = 5)
        memory[1]  = 32'h00100023; // sb  x1, 0(x0)          (MEM[0] = 5)
        memory[2]  = 32'h00000103; // lb  x2, 0(x0)          (x2 = MEM[0] = 5) ⚠️ HAZARD load-use detectado!
        memory[3]  = 32'h002081b3; // add x3, x1, x2         (x3 = x1 + x2 = 10) - usa x2 imediatamente
        memory[4]  = 32'h0011f233; // and x4, x3, x1         (x4 = x3 & x1 = 0) - usa x3 imediatamente (forwarding resolve)
        memory[5]  = 32'h001192b3; // sll x5, x3, x1         (x5 = x3 << x1 = 320) - usa x3 (forwarding resolve)
        memory[6]  = 32'h00028463; // bne x5, x0, +8         (se x5 != 0, pula para memory[8])
        memory[7]  = 32'h06306313; // ori x6, x0, 99         (x6 = 99) - só executa se branch falhar
        memory[8]  = 32'h02a06313; // ori x6, x0, 42         (x6 = 42) - destino do branch
        memory[9]  = 32'h00000013; // nop - fim do programa
        memory[10] = 32'h00000013; // nop
        memory[11] = 32'h00000013; // nop
        memory[12] = 32'h00000013; // nop
        memory[13] = 32'h00000013; // nop
        memory[14] = 32'h00000013; // nop
        memory[15] = 32'h00000013; // nop
    end
endmodule

module if_id_register (
  input clk,
    input reset,
    input if_id_write,     
    input flush,           
    input [63:0] pc_in,
    input [31:0] instruction_in,
    output reg [63:0] pc_out,
    output reg [31:0] instruction_out
);
    always @(posedge clk or posedge reset) begin
        if (reset || flush) begin  // MODIFICADO: reset OU flush
            pc_out <= 0;
            instruction_out <= 32'h00000013; // NOP instruction
        end else if (if_id_write) begin  
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
        
    end
endmodule

module decoder(
    input [31:0] instruction,
    output [4:0] rs1,
    output [4:0] rs2,
    output [4:0] rd,
    output [6:0] opcode,
    output [2:0] funct3,
    output [6:0] funct7
);
    assign opcode = instruction[6:0];
    assign rd     = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign rs1    = instruction[19:15];
    assign rs2    = instruction[24:20];
    assign funct7 = instruction[31:25];
endmodule

module imm_gen(
  input [31:0] instruction,
  output reg [63:0] imm_out
);
  always @(*) begin
    case (instruction[6:0])
      7'b0000011: // I-type (ex: lb)
        imm_out = {{52{instruction[31]}}, instruction[31:20]};
      7'b0010011: // I-type (ex: ori, addi)
        imm_out = {{52{instruction[31]}}, instruction[31:20]};
      7'b0100011: // S-type (ex: sb)
        imm_out = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};
      7'b1100011: // B-type (ex: bne)
        imm_out = {{51{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
      default:
        imm_out = 64'b0;
    endcase
  end
endmodule
 
module control_unit(
    input  [6:0] opcode,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg branch,
    output reg alu_src,
    output reg mem_to_reg
);
  always @(*) begin
    reg_write = 0;
    mem_read  = 0;
    mem_write = 0;
    branch    = 0;
    alu_src   = 0;
    mem_to_reg= 0;

    case (opcode)
      7'b0000011: begin // lb
        reg_write = 1;
        mem_read  = 1;
        alu_src   = 1;
        mem_to_reg= 1;
      end
      7'b0100011: begin // sb
        mem_write = 1;
        alu_src   = 1;
      end
      7'b0110011: begin // R-type: add, and, sll
        reg_write = 1;
        alu_src   = 0;
      end
      7'b0010011: begin // I-type: ori
        reg_write = 1;
        alu_src   = 1;
      end
      7'b1100011: begin // bne
        branch    = 1;
        alu_src   = 0;
      end
    endcase
  end
endmodule

module id_ex_register(
    input clk,
    input reset,
    input  [63:0] pc_in,
    input  [63:0] rs1_data_in,
    input  [63:0] rs2_data_in,
    input  [4:0]  rs1_in,
    input  [4:0]  rs2_in,
    input  [4:0]  rd_in,
    input  [63:0] imm_in,
    input  [6:0]  opcode_in,
    input  [2:0]  funct3_in,
    input  [6:0]  funct7_in,
    input         reg_write_in,
    input         mem_read_in,
    input         mem_write_in,
    input         branch_in,
    input         alu_src_in,
    input         mem_to_reg_in,
    input  [3:0]  alu_control_in,
    output reg [63:0] pc_out,
    output reg [63:0] rs1_data_out,
    output reg [63:0] rs2_data_out,
    output reg [4:0]  rs1_out,
    output reg [4:0]  rs2_out,
    output reg [4:0]  rd_out,
    output reg [63:0] imm_out,
    output reg [6:0]  opcode_out,
    output reg [2:0]  funct3_out,
    output reg [6:0]  funct7_out,
    output reg        reg_write_out,
    output reg        mem_read_out,
    output reg        mem_write_out,
    output reg        branch_out,
    output reg        alu_src_out,
    output reg        mem_to_reg_out,
    output reg [3:0]  alu_control_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out          <= 64'b0;
            rs1_data_out    <= 64'b0;
            rs2_data_out    <= 64'b0;
            rs1_out         <= 5'b0;
            rs2_out         <= 5'b0;
            rd_out          <= 5'b0;
            imm_out         <= 64'b0;
            opcode_out      <= 7'b0;
            funct3_out      <= 3'b0;
            funct7_out      <= 7'b0;
            reg_write_out   <= 1'b0;
            mem_read_out    <= 1'b0;
            mem_write_out   <= 1'b0;
            branch_out      <= 1'b0;
            alu_src_out     <= 1'b0;
            mem_to_reg_out  <= 1'b0;
            alu_control_out <= 4'b0;
        end else begin
            pc_out          <= pc_in;
            rs1_data_out    <= rs1_data_in;
            rs2_data_out    <= rs2_data_in;
            rs1_out         <= rs1_in;
            rs2_out         <= rs2_in;
            rd_out          <= rd_in;
            imm_out         <= imm_in;
            opcode_out      <= opcode_in;
            funct3_out      <= funct3_in;
            funct7_out      <= funct7_in;
            reg_write_out   <= reg_write_in;
            mem_read_out    <= mem_read_in;
            mem_write_out   <= mem_write_in;
            branch_out      <= branch_in;
            alu_src_out     <= alu_src_in;
            mem_to_reg_out  <= mem_to_reg_in;
            alu_control_out <= alu_control_in;
        end
    end
endmodule

module data_memory(
    input clk,
    input mem_read,
    input mem_write,
    input [63:0] addr,
    input [63:0] write_data,
    output reg [63:0] read_data
);
    reg [7:0] memory [0:255];
    integer i;
    
    initial begin
        for (i = 0; i < 256; i = i + 1)
            memory[i] = 8'b0;
    end

    always @(posedge clk) begin
        if(mem_write) begin
            memory[addr] <= write_data[7:0];
            $display("Time %0t: DataMem WRITE - MEM[%0d] = %0d (0x%h)", $realtime, addr, write_data[7:0], write_data[7:0]);
        end
    end

    always @(*) begin
        if(mem_read) begin
            read_data = {{56{memory[addr][7]}}, memory[addr]};
            $display("Time %0t: DataMem READ - MEM[%0d] = %0d (0x%h) -> sign-extended: %0d", 
                     $realtime, addr, memory[addr], memory[addr], $signed(read_data));
        end else begin
            read_data = 64'b0;
        end
    end
endmodule


module register_file(
    input clk,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [63:0] write_data,
    input reg_write,
    output [63:0] rs1_data,
    output [63:0] rs2_data
);
    reg [63:0] regs[0:31];
    integer i;

    initial begin
        for (i = 0; i < 32; i = i + 1)
            regs[i] = 64'b0;
    end

    assign rs1_data = regs[rs1];
    assign rs2_data = regs[rs2];

    always @(posedge clk) begin
        if (reg_write && rd != 0) begin
            regs[rd] <= write_data;
            $display("Time %0t: RegFile WRITE - x%0d = %0d (0x%h)", $realtime, rd, write_data, write_data);
            

            #0.1; // Pequeno delay para garantir que a escrita aconteceu
            $display("---------------------------------------------");
            $display("RegFile State: x1=%0d, x2=%0d, x3=%0d, x4=%0d, x5=%0d, x6=%0d, x7=%0d, x8=%0d, x9=%0d, x10=%0d, x11=%0d, x12=%0d", 
                     regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8], regs[9], regs[10], regs[11], regs[12]);
            $display("x13=%0d, x14=%0d, x15=%0d, x16=%0d, x17=%0d, x18=%0d, x19=%0d, x20=%0d, x21=%0d, x22=%0d, x23=%0d, x24=%0d", 
                     regs[13], regs[14], regs[15], regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23], regs[24]);
            $display("x25=%0d, x26=%0d, x27=%0d, x28=%0d, x29=%0d, x30=%0d, x31=%0d", 
                     regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31]);
            $display("---------------------------------------------");
        end
    end
endmodule

module alu (
    input  [63:0] a,
    input  [63:0] b,
    input  [3:0]  alu_op,
    output reg [63:0] result,
    output        zero
);
    always @(*) begin
        case (alu_op)
            4'b0000: result = a + b;           // ADD
            4'b0001: result = a & b;           // AND
            4'b0010: result = a | b;           // OR
            4'b0011: result = a << b[5:0];     // SLL (shift left logical)
            4'b0100: result = (a != b);        // BNE (compare for branch)
            default: result = 64'b0;
        endcase
        
        // Debug da ALU
        if (alu_op == 4'b0011) // SLL
            $display("Time %0t: ALU SLL - %0d << %0d = %0d", $realtime, a, b[5:0], result);
        else if (alu_op == 4'b0000) // ADD
            $display("Time %0t: ALU ADD - %0d + %0d = %0d", $realtime, a, b, result);
        else if (alu_op == 4'b0001) // AND
            $display("Time %0t: ALU AND - %0d & %0d = %0d", $realtime, a, b, result);
        else if (alu_op == 4'b0100) // BNE
            $display("Time %0t: ALU BNE - %0d != %0d = %0d", $realtime, a, b, result);
    end

    assign zero = (result == 0);
endmodule

module branch_addr_calc (
    input  [63:0] pc,
    input  [63:0] imm,
    output [63:0] branch_addr
);
    assign branch_addr = pc + imm;
endmodule

module mux_alu_src (
    input  [63:0] rs2_data,
    input  [63:0] imm_data,
    input         alu_src,
    output [63:0] alu_b
);
    assign alu_b = (alu_src) ? imm_data : rs2_data;
endmodule

module ex_stage (
    input  [63:0] pc_in,
    input  [63:0] rs1_data,
    input  [63:0] rs2_data,
    input  [63:0] imm_data,
    input         alu_src,
    input  [3:0]  alu_op,
    input         branch_control,
    input  [63:0] ex_mem_alu_result,
    input  [63:0] wb_write_data,
    input  [1:0]  forward_a,
    input  [1:0]  forward_b,
    output [63:0] alu_result,
    output [63:0] branch_addr,
    output        branch_taken
);

    wire [63:0] alu_a_input, alu_b_input;
    wire [63:0] forwarded_rs2_data;
    wire        zero_flag;

    // MUX para forwarding do input A
    mux_forward_a forward_mux_a (
        .id_ex_rs1_data(rs1_data),
        .ex_mem_alu_result(ex_mem_alu_result),
        .wb_write_data(wb_write_data),
        .forward_a(forward_a),
        .alu_input_a(alu_a_input)
    );

    // MUX para forwarding do input B
    mux_forward_b forward_mux_b (
        .id_ex_rs2_data(rs2_data),
        .ex_mem_alu_result(ex_mem_alu_result),
        .wb_write_data(wb_write_data),
        .forward_b(forward_b),
        .forwarded_rs2_data(forwarded_rs2_data)
    );

    // MUX para escolher entre rs2 forwarded e imediato
    mux_alu_src mux_b (
        .rs2_data(forwarded_rs2_data),  
        .imm_data(imm_data),
        .alu_src(alu_src),
        .alu_b(alu_b_input)
    );

    alu alu_unit (
        .a(alu_a_input),  
        .b(alu_b_input),
        .alu_op(alu_op),
        .result(alu_result),
        .zero(zero_flag)
    );

    branch_addr_calc branch_calc (
        .pc(pc_in),
        .imm(imm_data),
        .branch_addr(branch_addr)
    );

    assign branch_taken = branch_control && (alu_op == 4'b0100) && (zero_flag == 0);
endmodule

module alu_control (
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,
    output reg [3:0] alu_op
);
    always @(*) begin
        case(opcode)
            7'b0000011: alu_op = 4'b0000; // lb
            7'b0100011: alu_op = 4'b0000; // sb
            7'b0010011: alu_op = 4'b0010; // ori
            7'b0110011: begin             // R-type
                case(funct3)
                    3'b000: alu_op = 4'b0000; // add
                    3'b111: alu_op = 4'b0001; // and
                    3'b001: alu_op = 4'b0011; // sll
                    default: alu_op = 4'b0000;
                endcase
            end
            7'b1100011: alu_op = 4'b0100;   // bne
            default: alu_op = 4'b0000;
        endcase
    end
endmodule

module ex_mem_register(
    input clk,
    input reset,
    input [63:0] ex_alu_result,
    input [63:0] ex_rs2_data,     
    input [4:0]  ex_rd,
    input        ex_mem_read,
    input        ex_mem_write,
    input        ex_reg_write,
    input        ex_mem_to_reg,
    output reg [63:0] mem_alu_result,
    output reg [63:0] mem_write_data,
    output reg [4:0]  mem_rd,
    output reg        mem_mem_read,
    output reg        mem_mem_write,
    output reg        mem_reg_write,
    output reg        mem_mem_to_reg
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_alu_result  <= 64'b0;
            mem_write_data  <= 64'b0;
            mem_rd          <= 5'b0;
            mem_mem_read    <= 0;
            mem_mem_write   <= 0;
            mem_reg_write   <= 0;
            mem_mem_to_reg  <= 0;
        end else begin
            mem_alu_result  <= ex_alu_result;
            mem_write_data  <= ex_rs2_data;
            mem_rd          <= ex_rd;
            mem_mem_read    <= ex_mem_read;
            mem_mem_write   <= ex_mem_write;
            mem_reg_write   <= ex_reg_write;
            mem_mem_to_reg  <= ex_mem_to_reg;
        end
    end
endmodule

module mem_wb_register(
    input clk,
    input reset,
    input [63:0] mem_read_data_in,
    input [63:0] mem_alu_result_in,
    input [4:0]  mem_rd_in,
    input        mem_reg_write_in,
    input        mem_mem_to_reg_in,
    output reg [63:0] wb_read_data_out,
    output reg [63:0] wb_alu_result_out,
    output reg [4:0]  wb_rd_out,
    output reg        wb_reg_write_out,
    output reg        wb_mem_to_reg_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            wb_read_data_out   <= 64'b0;
            wb_alu_result_out  <= 64'b0;
            wb_rd_out          <= 5'b0;
            wb_reg_write_out   <= 0;
            wb_mem_to_reg_out  <= 0;
        end else begin
            wb_read_data_out   <= mem_read_data_in;
            wb_alu_result_out  <= mem_alu_result_in;
            wb_rd_out          <= mem_rd_in;
            wb_reg_write_out   <= mem_reg_write_in;
            wb_mem_to_reg_out  <= mem_mem_to_reg_in;
        end
    end
endmodule

module hazard_detection_unit(
    input [4:0]  if_id_rs1,
    input [4:0]  if_id_rs2,
    input [4:0]  id_ex_rd,
    input        id_ex_mem_read,
    output reg   stall
);
    always @(*) begin
        // Detecta hazard load-use
        if (id_ex_mem_read && (id_ex_rd != 0) && 
            ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2))) begin
            stall = 1'b1; // Precisa inserir stall
        end else begin
            stall = 1'b0; // Sem hazard
        end
    end
endmodule

// Módulo para inserir NOPs (bolhas) no pipeline
module pipeline_control(
    input        stall,
    input        branch_taken,
    output reg   pc_write,      
    output reg   if_id_write,   
    output reg   control_mux    
);
    always @(*) begin
        if (stall) begin
            pc_write     = 1'b0;  
            if_id_write  = 1'b0;  
            control_mux  = 1'b1;  
        end else if (branch_taken) begin
            pc_write     = 1'b1;  
            if_id_write  = 1'b1;  
            control_mux  = 1'b1;  
        end else begin
            pc_write     = 1'b1;  
            if_id_write  = 1'b1;  
            control_mux  = 1'b0;  
        end
    end
endmodule

// MUX para controles (seleciona entre controles normais ou NOP)
module control_mux(
    input        control_mux_sel,
    input        reg_write_in,
    input        mem_read_in,
    input        mem_write_in,
    input        branch_in,
    input        alu_src_in,
    input        mem_to_reg_in,
    output reg   reg_write_out,
    output reg   mem_read_out,
    output reg   mem_write_out,
    output reg   branch_out,
    output reg   alu_src_out,
    output reg   mem_to_reg_out
);
    always @(*) begin
        if (control_mux_sel) begin
            // Insere NOP (todos os controles em 0)
            reg_write_out  = 1'b0;
            mem_read_out   = 1'b0;
            mem_write_out  = 1'b0;
            branch_out     = 1'b0;
            alu_src_out    = 1'b0;
            mem_to_reg_out = 1'b0;
        end else begin
            // Passa os controles normais
            reg_write_out  = reg_write_in;
            mem_read_out   = mem_read_in;
            mem_write_out  = mem_write_in;
            branch_out     = branch_in;
            alu_src_out    = alu_src_in;
            mem_to_reg_out = mem_to_reg_in;
        end
    end
endmodule