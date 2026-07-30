// RISCV32I CPU top module
// Verilog-2001 behavioral core with byte-addressed memory interface.

module cpu(
  input  wire                 clk_in,
  input  wire                 rst_in,
  input  wire                 rdy_in,

  input  wire [7:0]           mem_din,
  output wire [7:0]           mem_dout,
  output wire [31:0]          mem_a,
  output wire                 mem_wr,

  input  wire                 io_buffer_full,
  output wire [31:0]          dbgreg_dout
);

localparam [3:0]
  ST_FETCH0 = 4'd0,
  ST_FETCH1 = 4'd1,
  ST_FETCH2 = 4'd2,
  ST_FETCH3 = 4'd3,
  ST_DECODE = 4'd4,
  ST_LOAD0  = 4'd5,
  ST_LOAD1  = 4'd6,
  ST_LOAD2  = 4'd7,
  ST_LOAD3  = 4'd8,
  ST_STORE0 = 4'd9,
  ST_STORE1 = 4'd10,
  ST_STORE2 = 4'd11,
  ST_STORE3 = 4'd12,
  ST_HALT   = 4'd13;

reg [3:0] state;
reg [31:0] pc;
reg [31:0] regs [0:31];
reg [31:0] instr;
reg [7:0]  ib0, ib1, ib2, ib3;
reg [31:0] mem_a_r;
reg [7:0]  mem_dout_r;
reg        mem_wr_r;
reg [31:0] cycle_cnt;

reg [31:0] rs1_val, rs2_val, next_pc, addr, result;
reg [31:0] load_data;
reg [31:0] store_data;
reg [4:0]  rd_r, rs1_r, rs2_r;
reg [6:0]  opcode_r, funct7_r;
reg [2:0]  funct3_r;
reg [31:0] imm_i_r, imm_s_r, imm_b_r, imm_u_r, imm_j_r;
reg [1:0]  byte_off;
reg [2:0]  load_kind;
reg        take_branch;
reg        load_signed;
reg        load_pending;
reg        store_pending;
reg        halt_seen;

assign mem_a = mem_a_r;
assign mem_dout = mem_dout_r;
assign mem_wr = mem_wr_r;
assign dbgreg_dout = regs[10];

integer i;

function [31:0] sext8;
  input [7:0] v;
  begin sext8 = {{24{v[7]}}, v}; end
endfunction

function [31:0] zext8;
  input [7:0] v;
  begin zext8 = {24'b0, v}; end
endfunction

function [31:0] sext16;
  input [15:0] v;
  begin sext16 = {{16{v[15]}}, v}; end
endfunction

function [31:0] zext16;
  input [15:0] v;
  begin zext16 = {16'b0, v}; end
endfunction

function [31:0] sra32;
  input [31:0] a;
  input [4:0] sh;
  begin sra32 = $signed(a) >>> sh; end
endfunction

always @(posedge clk_in) begin
  if (rst_in) begin
    state <= ST_FETCH0;
    pc <= 32'b0;
    instr <= 32'b0;
    ib0 <= 0; ib1 <= 0; ib2 <= 0; ib3 <= 0;
    mem_a_r <= 0;
    mem_dout_r <= 0;
    mem_wr_r <= 0;
    cycle_cnt <= 0;
    load_data <= 0;
    store_data <= 0;
    rd_r <= 0; rs1_r <= 0; rs2_r <= 0;
    opcode_r <= 0; funct7_r <= 0; funct3_r <= 0;
    imm_i_r <= 0; imm_s_r <= 0; imm_b_r <= 0; imm_u_r <= 0; imm_j_r <= 0;
    byte_off <= 0;
    load_kind <= 0;
    take_branch <= 0;
    load_signed <= 0;
    load_pending <= 0;
    store_pending <= 0;
    halt_seen <= 0;
    for (i = 0; i < 32; i = i + 1) begin
      regs[i] <= 32'b0;
    end
  end else if (!rdy_in) begin
    mem_wr_r <= 1'b0;
  end else begin
    mem_wr_r <= 1'b0;
    cycle_cnt <= cycle_cnt + 1'b1;
    regs[0] <= 32'b0;

    case (state)
      ST_FETCH0: begin
        mem_a_r <= pc;
        state <= ST_FETCH1;
      end
      ST_FETCH1: begin
        ib0 <= mem_din;
        mem_a_r <= pc + 32'd1;
        state <= ST_FETCH2;
      end
      ST_FETCH2: begin
        ib1 <= mem_din;
        mem_a_r <= pc + 32'd2;
        state <= ST_FETCH3;
      end
      ST_FETCH3: begin
        ib2 <= mem_din;
        mem_a_r <= pc + 32'd3;
        instr <= {mem_din, ib2, ib1, ib0};
        state <= ST_DECODE;
      end
      ST_DECODE: begin
        opcode_r = instr[6:0];
        funct3_r = instr[14:12];
        funct7_r = instr[31:25];
        rd_r = instr[11:7];
        rs1_r = instr[19:15];
        rs2_r = instr[24:20];
        rs1_val = regs[rs1_r];
        rs2_val = regs[rs2_r];
        imm_i_r = {{20{instr[31]}}, instr[31:20]};
        imm_s_r = {{20{instr[31]}}, instr[31:25], instr[11:7]};
        imm_b_r = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
        imm_u_r = {instr[31:12], 12'b0};
        imm_j_r = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
        next_pc = pc + 32'd4;
        take_branch = 1'b0;

        case (opcode_r)
          7'b0110111: begin // LUI
            if (rd_r != 0) regs[rd_r] <= imm_u_r;
            pc <= next_pc;
            state <= ST_FETCH0;
          end
          7'b0010111: begin // AUIPC
            if (rd_r != 0) regs[rd_r] <= pc + imm_u_r;
            pc <= next_pc;
            state <= ST_FETCH0;
          end
          7'b1101111: begin // JAL
            if (rd_r != 0) regs[rd_r] <= next_pc;
            pc <= pc + imm_j_r;
            state <= ST_FETCH0;
          end
          7'b1100111: begin // JALR
            if (rd_r != 0) regs[rd_r] <= next_pc;
            pc <= (rs1_val + imm_i_r) & 32'hfffffffe;
            state <= ST_FETCH0;
          end
          7'b1100011: begin // branches
            case (funct3_r)
              3'b000: take_branch = (rs1_val == rs2_val);
              3'b001: take_branch = (rs1_val != rs2_val);
              3'b100: take_branch = ($signed(rs1_val) < $signed(rs2_val));
              3'b101: take_branch = ($signed(rs1_val) >= $signed(rs2_val));
              3'b110: take_branch = (rs1_val < rs2_val);
              3'b111: take_branch = (rs1_val >= rs2_val);
              default: take_branch = 1'b0;
            endcase
            pc <= take_branch ? (pc + imm_b_r) : next_pc;
            state <= ST_FETCH0;
          end
          7'b0000011: begin // loads
            addr = rs1_val + imm_i_r;
            mem_a_r <= addr;
            byte_off <= addr[1:0];
            load_kind <= funct3_r;
            state <= ST_LOAD0;
          end
          7'b0100011: begin // stores
            addr = rs1_val + imm_s_r;
            mem_a_r <= addr;
            mem_dout_r <= rs2_val[7:0];
            byte_off <= addr[1:0];
            store_data <= rs2_val;
            state <= ST_STORE0;
          end
          7'b0010011: begin // immediate ALU
            case (funct3_r)
              3'b000: result = rs1_val + imm_i_r;
              3'b010: result = ($signed(rs1_val) < $signed(imm_i_r)) ? 32'd1 : 32'd0;
              3'b011: result = (rs1_val < imm_i_r) ? 32'd1 : 32'd0;
              3'b100: result = rs1_val ^ imm_i_r;
              3'b110: result = rs1_val | imm_i_r;
              3'b111: result = rs1_val & imm_i_r;
              3'b001: result = rs1_val << instr[24:20];
              3'b101: begin
                if (instr[30]) result = sra32(rs1_val, instr[24:20]);
                else result = rs1_val >> instr[24:20];
              end
              default: result = 32'b0;
            endcase
            if (rd_r != 0) regs[rd_r] <= result;
            pc <= next_pc;
            state <= ST_FETCH0;
          end
          7'b0110011: begin // register ALU
            case ({funct7_r, funct3_r})
              {7'b0000000,3'b000}: result = rs1_val + rs2_val;
              {7'b0100000,3'b000}: result = rs1_val - rs2_val;
              {7'b0000000,3'b001}: result = rs1_val << rs2_val[4:0];
              {7'b0000000,3'b010}: result = ($signed(rs1_val) < $signed(rs2_val)) ? 32'd1 : 32'd0;
              {7'b0000000,3'b011}: result = (rs1_val < rs2_val) ? 32'd1 : 32'd0;
              {7'b0000000,3'b100}: result = rs1_val ^ rs2_val;
              {7'b0000000,3'b101}: result = rs1_val >> rs2_val[4:0];
              {7'b0100000,3'b101}: result = sra32(rs1_val, rs2_val[4:0]);
              {7'b0000000,3'b110}: result = rs1_val | rs2_val;
              {7'b0000000,3'b111}: result = rs1_val & rs2_val;
              default: result = 32'b0;
            endcase
            if (rd_r != 0) regs[rd_r] <= result;
            pc <= next_pc;
            state <= ST_FETCH0;
          end
          7'b1110011: begin // ecall/ebreak as NOP
            pc <= next_pc;
            state <= ST_FETCH0;
          end
          default: begin
            pc <= next_pc;
            state <= ST_FETCH0;
          end
        endcase
      end
      ST_LOAD0: begin
        mem_a_r <= mem_a_r + 32'd1;
        load_data[7:0] <= mem_din;
        state <= ST_LOAD1;
      end
      ST_LOAD1: begin
        mem_a_r <= mem_a_r + 32'd1;
        load_data[15:8] <= mem_din;
        state <= ST_LOAD2;
      end
      ST_LOAD2: begin
        mem_a_r <= mem_a_r + 32'd1;
        load_data[23:16] <= mem_din;
        state <= ST_LOAD3;
      end
      ST_LOAD3: begin
        load_data[31:24] <= mem_din;
        case (load_kind)
          3'b000: begin // LB
            if (rd_r != 0) begin
              case (byte_off)
                2'd0: regs[rd_r] <= sext8(load_data[7:0]);
                2'd1: regs[rd_r] <= sext8(load_data[15:8]);
                2'd2: regs[rd_r] <= sext8(load_data[23:16]);
                default: regs[rd_r] <= sext8(load_data[31:24]);
              endcase
            end
          end
          3'b100: begin // LBU
            if (rd_r != 0) begin
              case (byte_off)
                2'd0: regs[rd_r] <= zext8(load_data[7:0]);
                2'd1: regs[rd_r] <= zext8(load_data[15:8]);
                2'd2: regs[rd_r] <= zext8(load_data[23:16]);
                default: regs[rd_r] <= zext8(load_data[31:24]);
              endcase
            end
          end
          3'b001: begin // LH
            if (rd_r != 0) begin
              case (byte_off[1])
                1'b0: regs[rd_r] <= sext16(load_data[15:0]);
                default: regs[rd_r] <= sext16(load_data[31:16]);
              endcase
            end
          end
          3'b101: begin // LHU
            if (rd_r != 0) begin
              case (byte_off[1])
                1'b0: regs[rd_r] <= zext16(load_data[15:0]);
                default: regs[rd_r] <= zext16(load_data[31:16]);
              endcase
            end
          end
          default: begin // LW
            if (rd_r != 0) regs[rd_r] <= load_data;
          end
        endcase
        pc <= pc + 32'd4;
        state <= ST_FETCH0;
      end
      ST_STORE0: begin
        mem_wr_r <= 1'b1;
        mem_dout_r <= store_data[7:0];
        mem_a_r <= mem_a_r;
        state <= ST_STORE1;
      end
      ST_STORE1: begin
        mem_wr_r <= 1'b1;
        mem_dout_r <= store_data[15:8];
        mem_a_r <= mem_a_r + 32'd1;
        state <= ST_STORE2;
      end
      ST_STORE2: begin
        mem_wr_r <= 1'b1;
        mem_dout_r <= store_data[23:16];
        mem_a_r <= mem_a_r + 32'd1;
        state <= ST_STORE3;
      end
      ST_STORE3: begin
        mem_wr_r <= 1'b1;
        mem_dout_r <= store_data[31:24];
        mem_a_r <= mem_a_r + 32'd1;
        pc <= pc + 32'd4;
        state <= ST_FETCH0;
      end
      default: begin
        state <= ST_FETCH0;
      end
    endcase
  end
end

endmodule
