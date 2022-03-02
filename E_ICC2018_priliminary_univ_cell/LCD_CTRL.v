`include "define.v"

module LCD_CTRL(clk, reset, cmd, cmd_valid, IROM_Q, IROM_rd, IROM_A, IRAM_valid, IRAM_D, IRAM_A, busy, done);
input clk;
input reset;
input [3:0] cmd;
input cmd_valid;
input [7:0] IROM_Q;
output IROM_rd;
output [5:0] IROM_A;
output IRAM_valid;
output [7:0] IRAM_D;
output [5:0] IRAM_A;
output busy;
output done;


// DO NOT FORGET TO DO THIS.
wire [`STATE_W-1:0] curr_state;
wire [`STATE_DONE_W-1:0] done_state;

ctrl ctrl(
    .clk(clk),
    .reset(reset),
    .cmd_valid(cmd_valid),
    .curr_state(curr_state),// in the () is not the default name
    .done_state(done_state)
);

dp dp(
    .clk(clk),
    .reset(reset),
    .cmd(cmd),
    .cmd_valid(cmd_valid),
    .IROM_Q(IROM_Q),
    .IROM_rd(IROM_rd),
    .IROM_A(IROM_A),
    .IRAM_valid(IRAM_valid),
    .IRAM_D(IRAM_D),
    .IRAM_A(IRAM_A),
    .busy(busy),
    .done(done),
    .curr_state(curr_state),
    .done_state(done_state)
);

endmodule

module ctrl (
    input                           clk,
    input                           reset, 
    input                           cmd_valid,
    input       [`STATE_DONE_W-1:0] done_state,
    output reg  [`STATE_W-1:0]      curr_state
);

reg [`STATE_W-1:0] next_state;

// State Register (S)
always @(posedge clk) begin
    curr_state <= next_state;
end

// Next State Logic (C)
always @(*) begin
    next_state = `S_ZVEC;
    if (reset) begin
        next_state[`S_READ] = 1'b1;
    end else begin
        case (1'b1)
            curr_state[`S_READ]: begin
                if (done_state[`DONE_READ]) begin
                    next_state[`S_WAIT] = 1'b1;
                end else begin
                    next_state[`S_READ] = 1'b1;
                end
            end

            curr_state[`S_WAIT]: begin
                if (done_state[`DONE_WAIT]) begin
                    next_state[`S_COMPUTE] = 1'b1;
                end else begin
                    next_state[`S_WAIT] = 1'b1;
                end
            end

            curr_state[`S_COMPUTE]: begin
                if (done_state[`DONE_COMPUTE]) begin
                    next_state[`S_WAIT] = 1'b1;
                end else if (done_state[`DONE_FINISH]) begin
                    next_state[`S_END] = 1'b1;
                end else begin
                    next_state[`S_COMPUTE] = 1'b1;
                end
            end
        endcase
    end
end
endmodule

module dp (
    input                           clk,
    input                           reset,
    input       [3:0]               cmd,
    input                           cmd_valid,
    input       [7:0]               IROM_Q,
    output reg                         IROM_rd,
    output reg     [5:0]               IROM_A,
    output reg                        IRAM_valid,
    output reg     [7:0]               IRAM_D,
    output reg     [5:0]               IRAM_A,
    output reg                         busy,
    output reg                         done,
    input       [`STATE_W-1:0]      curr_state,
    output reg  [`STATE_DONE_W-1:0] done_state
);

reg [6:0] i;
reg [9:0] reg_img_org[63:0];
wire [6:0] index_img[3:0];

// busy
always @(posedge clk or posedge reset) begin
    if (reset) begin
        busy <= 1;
    end else begin
        if (curr_state[`S_READ]) begin
            busy <= 1;
        end else if (curr_state[`S_COMPUTE]) begin
            busy <= 1;
        end else if (curr_state[`S_WAIT]) begin
            busy <= 0;
        end else begin
            busy <= busy;
        end
    end
end

// IROM_rd
always @(posedge clk or posedge reset) begin
    if (reset) begin
        IROM_rd <= 1;
    end else begin
        if (curr_state[`S_READ]) begin
            IROM_rd <= 1; 
        end else begin
            IROM_rd <= 0;
        end
    end
end

// IROM_A
always @(posedge clk or posedge reset) begin
    if (reset) begin
        IROM_A <= 0;
    end else begin
        if (curr_state[`S_READ]) begin
            IROM_A <= IROM_A + 6'b1;
        end else begin
            IROM_A <= IROM_A;
        end
    end
end

// MAX, MIN, AVERAGE
wire [9:0] MAX, MIN, AVERAGE;
maxPool_2x2 max_pool(
    .in0(reg_img_org[index_img[0]]),
    .in1(reg_img_org[index_img[1]]),
    .in2(reg_img_org[index_img[2]]),
    .in3(reg_img_org[index_img[3]]),
    .max(MAX)
);

minPool_2x2 min_pool(
    .in0(reg_img_org[index_img[0]]),
    .in1(reg_img_org[index_img[1]]),
    .in2(reg_img_org[index_img[2]]),
    .in3(reg_img_org[index_img[3]]),
    .min(MIN)
);

assign AVERAGE = (reg_img_org[index_img[0]] + reg_img_org[index_img[1]] + reg_img_org[index_img[2]] + reg_img_org[index_img[3]]) >> 2;

// reg_image_org
always @(posedge clk or posedge reset) begin
    if (reset) begin
        for (i = 0; i < 64; i = i + 1) begin
            reg_img_org[i] <= 0;
        end
    end else begin
        if (curr_state[`S_READ]) begin
            reg_img_org[IROM_A] <= IROM_Q;
        end else if (curr_state[`S_COMPUTE]) begin
            case (cmd)
                4'b0101: begin
                    for (i = 0; i <= 3; i = i + 1) begin
                        reg_img_org[index_img[i]] <= MAX;
                    end 
                end

                4'b0110: begin
                    for (i = 0; i <= 3; i = i + 1) begin
                        reg_img_org[index_img[i]] <= MIN;
                    end 
                end

                4'b0111: begin
                    for (i = 0; i <= 3; i = i + 1) begin
                        reg_img_org[index_img[i]] <= AVERAGE;
                    end 
                end

                4'b1000: begin
                    reg_img_org[index_img[0]] <= reg_img_org[index_img[1]];
                    reg_img_org[index_img[1]] <= reg_img_org[index_img[3]];
                    reg_img_org[index_img[2]] <= reg_img_org[index_img[0]];
                    reg_img_org[index_img[3]] <= reg_img_org[index_img[2]];
                end

                4'b1001: begin
                    reg_img_org[index_img[0]] <= reg_img_org[index_img[2]];
                    reg_img_org[index_img[1]] <= reg_img_org[index_img[0]];
                    reg_img_org[index_img[2]] <= reg_img_org[index_img[3]];
                    reg_img_org[index_img[3]] <= reg_img_org[index_img[1]];
                end

                4'b1010: begin
                    reg_img_org[index_img[0]] <= reg_img_org[index_img[2]];
                    reg_img_org[index_img[1]] <= reg_img_org[index_img[3]];
                    reg_img_org[index_img[2]] <= reg_img_org[index_img[0]];
                    reg_img_org[index_img[3]] <= reg_img_org[index_img[1]];
                end

                4'b1011: begin
                    reg_img_org[index_img[0]] <= reg_img_org[index_img[1]];
                    reg_img_org[index_img[1]] <= reg_img_org[index_img[0]];
                    reg_img_org[index_img[2]] <= reg_img_org[index_img[3]];
                    reg_img_org[index_img[3]] <= reg_img_org[index_img[2]];
                end
                
                default: begin
                    for (i = 0; i <= 3; i = i + 1) begin
                        reg_img_org[index_img[i]] <= reg_img_org[index_img[i]];
                    end 
                end
            endcase
        end
    end
end

// assign the img_index according to the op_point
reg [6:0] op_point;

operation_point operation_point(
    .op_point(op_point),
    .index_img_0(index_img[0]),
    .index_img_1(index_img[1]),
    .index_img_2(index_img[2]),
    .index_img_3(index_img[3])
);

// WRITE
// IRAM_valid
always @(*) begin
    if (reset) begin
        IRAM_valid = 0;
    end else begin
        if (curr_state[`S_COMPUTE]) begin
            if (cmd == 4'b0000) begin
                IRAM_valid = 1;
            end else begin
                IRAM_valid = 0;
            end
        end
    end
end

// IRAM_A
always @(posedge clk or posedge reset) begin
    if (reset) begin
        IRAM_A <= 0;
    end else begin
        if (curr_state[`S_COMPUTE]) begin
            if (cmd == 4'b0000) begin
                if (IRAM_A >= 64) begin
                    IRAM_A <= IRAM_A;
                end else begin
                    IRAM_A <= IRAM_A + 1;        
                end
            end else begin
                IRAM_A <= IRAM_A;
            end
        end
    end
end

// IRAM_D
always @(*) begin
    if (reset) begin
        IRAM_D = 0;
    end else begin
        if (curr_state[`S_COMPUTE]) begin
            if (cmd == 4'b0000) begin
                IRAM_D = reg_img_org[IRAM_A];
            end  
        end
    end
end

// cmd
always @(posedge clk or posedge reset) begin
    if (reset) begin
        op_point <= 24;
    end else begin
        if (curr_state[`S_COMPUTE]) begin
            case (cmd)
                4'b0001: begin //SHIFT UP
                    if (op_point <= 6) begin
                        op_point <= op_point;
                    end else begin
                        op_point <= op_point - 7;
                    end
                end

                4'b0010: begin //SHIFT DOWN
                    if (op_point >= 42) begin
                        op_point <= op_point;
                    end else begin
                        op_point <= op_point + 7;
                    end
                end

                4'b0011: begin //SHIFT LEFT
                    if (op_point % 7 == 0) begin
                        op_point <= op_point;
                    end else begin
                        op_point <= op_point - 1;
                    end
                end

                4'b0100: begin //SHIFT RIGHT
                    if (op_point % 7 == 6) begin
                        op_point <= op_point;
                    end else begin
                        op_point <= op_point + 1;
                    end
                end

                default:begin
                    op_point <= op_point;
                end
            endcase
        end
    end
end

// done
always @(posedge clk or posedge reset) begin
    if (reset) begin
        done <= 0;
    end else begin
        if (curr_state[`S_END]) begin
            done <= 1;
        end else begin
            done <= done;
        end
    end
end

// done_state
always @(*) begin
    done_state = `DONE_ZVEC;
    case (1'b1)
        curr_state[`S_READ]: begin
            if (IROM_A == 63) begin
                done_state[`DONE_READ] = 1'b1;
            end
        end

        curr_state[`S_WAIT]: begin
            done_state[`DONE_WAIT] = 1'b1;
        end

        curr_state[`S_COMPUTE]: begin
            if (cmd == 4'b0000) begin
                if (IRAM_A >= 63) begin
                    done_state[`DONE_FINISH] = 1'b1;
                end
            end else begin
                done_state[`DONE_COMPUTE] = 1'b1;
            end
        end
    endcase
end

endmodule

module operation_point (
    input       [6:0]   op_point,
    output wire [6:0]   index_img_0,
    output wire [6:0]   index_img_1,
    output wire [6:0]   index_img_2,
    output wire [6:0]   index_img_3
);

assign index_img_0 = op_point / 7 + op_point;
assign index_img_1 = op_point / 7 + op_point + 1;
assign index_img_2 = op_point / 7 + op_point + 8;
assign index_img_3 = op_point / 7 + op_point + 9;
    
endmodule

module maxPool_2x2 (
                       input   wire    [`DATA_W-1:0] in0,
                       input   wire    [`DATA_W-1:0] in1,
                       input   wire    [`DATA_W-1:0] in2,
                       input   wire    [`DATA_W-1:0] in3,
                       output  wire    [`DATA_W-1:0] max
                       );

   wire [`DATA_W-1:0]                               cmp_0 = (in0 > in1) ? in0 : in1;
   wire [`DATA_W-1:0]                               cmp_1 = (in2 > in3) ? in2 : in3;
   assign max = (cmp_0 > cmp_1) ? cmp_0 : cmp_1;

endmodule

module minPool_2x2 (
                       input   wire    [`DATA_W-1:0] in0,
                       input   wire    [`DATA_W-1:0] in1,
                       input   wire    [`DATA_W-1:0] in2,
                       input   wire    [`DATA_W-1:0] in3,
                       output  wire    [`DATA_W-1:0] min
                       );

   wire [`DATA_W-1:0]                               cmp_0 = (in0 < in1) ? in0 : in1;
   wire [`DATA_W-1:0]                               cmp_1 = (in2 < in3) ? in2 : in3;
   assign min = (cmp_0 < cmp_1) ? cmp_0 : cmp_1;

endmodule