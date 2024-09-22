module CPU_TOP(clk, rst);
    input clk, rst;
    wire [18:0] pc_in, pc_out, instruction;
    wire [5:0] opcode;
    wire [3:0] rs1, rs2, rd;
    wire [18:0] reg1_data, reg2_data, alu_result, mem_data;
    wire reg_write, mem_write, mem_read, zero, branch, jump;

    // INSTRUCTION FETCH
    InstructionFetch IF(clk, rst, pc_in, pc_out);

    // INSTRUCTION MEMORY (FETCH INSTRUCTION AT pc_out)
    Memory InstMem(
        .adress(pc_out), 
        .write_data(19'b0),  // No write in instruction memory
        .mem_write(1'b0), 
        .mem_read(1'b1), 
        .read_data(instruction)
    );

    // Decode instruction fields 
    assign opcode = instruction[18:13];
    assign rs1 = instruction[12:9];
    assign rs2 = instruction[8:5];
    assign rd = instruction[4:1];

    // REGISTER FILE
    RegisterFile RF (
        .clk(clk),
        .rst(rst),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .write_data(alu_result),
        .reg_write(reg_write),
        .reg1_data(reg1_data),
        .reg2_data(reg2_data)
    );

    // ALU
    ALU ALU(
        .r2(reg1_data),
        .r3(reg2_data),
        .opcode(opcode),
        .r1(alu_result),
        .zero(zero)
    );

    // CONTROL UNIT
    Control_Unit CU(
        .opcode(opcode),
        .reg_write(reg_write),
        .alu_src(), // alu_src is unused in this example
        .mem_write(mem_write),
        .mem_read(mem_read),
        .branch(branch),
        .jump(jump)
    );

    // PC UPDATE (Branching and Jumping)
    assign pc_in = (branch && zero) ? pc_out + 1 : (jump ? instruction[12:0] : pc_out + 1);

endmodule

////////////////////////////////////////////

module ALU(r2, r3, opcode, r1, zero);
    input [18:0] r2, r3;     // Source 1,2
    input [5:0] opcode;      // OPERATION CODE
    output reg [18:0] r1;    // Result
    output reg zero;         // Zero flag

    always @ (*) begin 
        case(opcode)
            6'b000000: r1 = r2 + r3;   // ADD
            6'b000001: r1 = r2 - r3;   // SUB
            6'b000010: r1 = r2 * r3;   // MUL
            6'b000011: r1 = r2 / r3;   // DIV
            6'b000100: r1 = r2 & r3;   // AND
            6'b000101: r1 = r2 | r3;   // OR
            6'b000110: r1 = r2 ^ r3;   // XOR
            6'b000111: r1 = ~r2;       // NOT

            // CUSTOM INSTRUCTIONS
            6'b001000: r1 = FFT(r2);   // FFT
            6'b001001: r1 = ENC(r2);   // ENC
            6'b001010: r1 = DEC(r2);   // DEC

            default : r1 = 19'b0;
        endcase

        // Set zero flag
        zero = (r1 == 19'b0) ? 1'b1 : 1'b0;
    end

    // FFT, ENC, DEC functions
    function [18:0] FFT(input [18:0] data);
        FFT = data;   // Placeholder for FFT
    endfunction

    function [18:0] ENC(input [18:0] data);
        ENC = data;   // Placeholder for ENC
    endfunction

    function [18:0] DEC(input [18:0] data);
        DEC = data;   // Placeholder for DEC
    endfunction
endmodule

////////////////////////////////////////////

module Control_Unit(opcode, reg_write, alu_src, mem_write, mem_read, branch, jump);
    input [5:0] opcode;
    output reg reg_write;
    output reg alu_src;
    output reg mem_write;
    output reg mem_read;
    output reg branch;
    output reg jump;

    always @(*) begin 
        // Default control signals
        reg_write = 1'b0;
        alu_src = 1'b0;
        mem_write = 1'b0;
        mem_read = 1'b0;
        branch = 1'b0;
        jump = 1'b0;

        case (opcode)
            6'b000000, 6'b000001, 6'b000010, 6'b000011, 6'b000100, 6'b000101, 6'b000110, 6'b000111: begin 
                reg_write = 1'b1;  // ALU operations
            end
            6'b001100: begin // LD
                reg_write = 1'b1;
                mem_read = 1'b1;
            end
            6'b001101: begin // ST
                mem_write = 1'b1;
            end
            6'b001110: begin // JMP
                jump = 1'b1;
            end
            6'b001111: begin // BEQ
                branch = 1'b1;
            end
        endcase
    end
endmodule

////////////////////////////////////////////

module InstructionFetch(clk, rst, pc_in, pc_out);
    input clk, rst;
    input [18:0] pc_in;      // Next instruction address
    output reg [18:0] pc_out; // Present instruction address

    always @ (posedge clk or posedge rst) begin
        if (rst)
            pc_out <= 19'b0;  // Reset PC
        else
            pc_out <= pc_in;  // Update PC
    end
endmodule

////////////////////////////////////////////

module Memory(adress, write_data, mem_write, mem_read, read_data);
    input [18:0] adress;
    input [18:0] write_data;
    input mem_write, mem_read;   // Control signals
    output reg [18:0] read_data;

    reg [18:0] memory [0:255]; // 256 words of 19-bit memory

    always @(*) begin
        if (mem_read)
            read_data = memory[adress];
        if (mem_write)
            memory[adress] = write_data;
    end
endmodule

////////////////////////////////////////////

module RegisterFile(clk, rst, rs1, rs2, rd, write_data, reg_write, reg1_data, reg2_data);
    input clk, rst;
    input [3:0] rs1, rs2, rd;
    input [18:0] write_data;
    input reg_write;
    output [18:0] reg1_data;  // rs1
    output [18:0] reg2_data;  // rs2
    reg [18:0] registers [0:15]; // 16 registers, 19 bits each

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 16; i = i + 1)
                registers[i] <= 19'b0;  // Reset registers
        end
        else if (reg_write) begin
            registers[rd] <= write_data;
        end
    end

    // Read data from source registers
    assign reg1_data = registers[rs1];
    assign reg2_data = registers[rs2];
endmodule
