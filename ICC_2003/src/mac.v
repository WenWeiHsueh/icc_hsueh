module mac (instruction, multiplier, multiplicand, stall, clk, reset_n, result, protect);
input signed [15:0] multiplier;
input signed [15:0] multiplicand; 
input  clk;
input  reset_n;
input  stall;
input  [2:0] instruction;
output [31:0] result;
output [7:0] protect;

//Add you design here

reg signed [15:0] mul_mp, mul_mc; // multiplier, multiplicand
reg [2:0] inst_1, inst_2; // two-stage pipeline architecture

// read
always @(posedge clk) begin
    if (!reset_n) begin
        mul_mp <= 0;
        mul_mc <= 0;
        inst_1 <= 0;
    end else begin
        if (!stall) begin
            mul_mp <= multiplier;
            mul_mc <= multiplicand;
            inst_1 <= instruction;
        end
    end
end

// passing the stage for pipeline
always @(posedge clk) begin
    if (!reset_n) begin
        inst_2 <= 0;
    end else begin
        if (!stall) begin
            inst_2 <= inst_1;
        end
    end
end

wire [7:0] mul_mph, mul_mpl; // high, low
wire [7:0] mul_mch, mul_mcl;
assign {mul_mph[7:0], mul_mpl[7:0]} = mul_mp[15:0];
assign {mul_mch[7:0], mul_mcl[7:0]} = mul_mc[15:0];

reg [39:0] temp_ans_16;
reg [19:0] temp_ans_8h, temp_ans_8l;

// compute_1, the main part to compute the ans
always @(posedge clk) begin
    if (!reset_n) begin
        temp_ans_16 <= 40'd0;
        temp_ans_8l <= 20'd0;
        temp_ans_8h <= 20'd0;
    end else begin
        if (!stall) begin
            case (inst_1)
                3'b000: begin
                    temp_ans_16 <= 40'd0;
                    temp_ans_8l <= 20'd0; // the 8-bit temp must be initialize according to the spec: {protect[7:0], result[31:0]} += sign_ext(multiplier * multiplicand);
                    temp_ans_8h <= 20'd0;
                end 

                3'b001: begin
                    temp_ans_16 <= {{24{mul_mp[15]}}, mul_mp} * {{24{mul_mc[15]}}, mul_mc};
                end
                
                3'b010: begin
                    temp_ans_16 <= temp_ans_16 + {{24{mul_mp[15]}}, mul_mp} * {{24{mul_mc[15]}}, mul_mc};
                end

                // 32-bit saturation
                3'b011: begin
                    if ($signed(temp_ans_16) > $signed(40'h007fffffff)) begin
                        temp_ans_16[31:0] <= 32'h7fffffff; 
                    end else if($signed(temp_ans_16) < $signed(40'hff80000000)) begin
                        temp_ans_16[31:0] <= 32'h80000000;  
                    end else begin
                        temp_ans_16 <= temp_ans_16;  
                    end
                end

                3'b100: begin
                    temp_ans_16 <= 40'd0; // the 16-bit temp must be initialize according to the spec: {protect[3:0], result[15:0]} += sign_ext(multiplier[7:0] * multiplicand[7:0]);
                    temp_ans_8l <= 20'd0;
                    temp_ans_8h <= 20'd0;
                end

                3'b101: begin
                    temp_ans_8l <= {{12{mul_mpl[7]}}, mul_mpl} * {{12{mul_mcl[7]}}, mul_mcl};
                    temp_ans_8h <= {{12{mul_mph[7]}}, mul_mph} * {{12{mul_mch[7]}}, mul_mch};
                end

                3'b110: begin
                    temp_ans_8l <= temp_ans_8l + {{12{mul_mpl[7]}}, mul_mpl} * {{12{mul_mcl[7]}}, mul_mcl};
                    temp_ans_8h <= temp_ans_8h + {{12{mul_mph[7]}}, mul_mph} * {{12{mul_mch[7]}}, mul_mch};
                end

                // two concurrent 16-bit saturation
                3'b111: begin
                    if($signed(temp_ans_8l) > $signed(20'h07fff)) begin
                        temp_ans_8l[15:0] <= 16'h7fff; 
                    end else if($signed(temp_ans_8l) < $signed(20'hf8000)) begin
                        temp_ans_8l[15:0] <= 16'h8000; 
                    end else begin
                        temp_ans_8l <= temp_ans_8l;
                    end
                        
                    if($signed(temp_ans_8h) > $signed(20'h07fff)) begin
                        temp_ans_8h[15:0] <= 16'h7fff;
                    end else if($signed(temp_ans_8h) < $signed(20'hf8000)) begin
                        temp_ans_8h[15:0] <= 16'h8000;
                    end else begin
                        temp_ans_8h <= temp_ans_8h;
                    end
                end
                
                default: begin
                    temp_ans_8l <= temp_ans_8l;
                    temp_ans_8h <= temp_ans_8h;
                    temp_ans_16 <= temp_ans_16; 
                end
            endcase 
        end
    end
end

reg [31:0] reg_result;
reg [7:0] reg_protect;
assign result = reg_result;
assign protect = reg_protect;

// compute_2, to output the ans from the last stage
always @(posedge clk) begin
    if (!reset_n) begin
        {reg_protect[7:0], reg_result[31:0]} <= 40'd0;
    end else begin
        if (!stall) begin
            case (inst_2)
                3'b000: begin
                    {reg_protect[7:0], reg_result[31:0]} <= temp_ans_16;
                end 

                3'b001: begin
                    {reg_protect[7:0], reg_result[31:0]} <= temp_ans_16;
                end

                3'b010: begin
                    {reg_protect[7:0], reg_result[31:0]} <= temp_ans_16;
                end

                3'b011: begin
                    reg_result <= temp_ans_16[31:0];
                end

                3'b100: begin
                    {reg_protect[3:0], reg_result[15:0]} <= temp_ans_8l;
                    {reg_protect[7:4], reg_result[31:16]} <= temp_ans_8h;
                end

                3'b101: begin
                    {reg_protect[3:0], reg_result[15:0]} <= temp_ans_8l;
                    {reg_protect[7:4], reg_result[31:16]} <= temp_ans_8h;
                end

                3'b110: begin
                    {reg_protect[3:0], reg_result[15:0]} <= temp_ans_8l;
                    {reg_protect[7:4], reg_result[31:16]} <= temp_ans_8h;
                end

                3'b111: begin
                    reg_result[15:0] <= temp_ans_8l[15:0];
                    reg_result[31:16] <= temp_ans_8h[15:0];
                end

                default: begin
                    {reg_protect[7:0], reg_result[31:0]} <= {reg_protect[7:0], reg_result[31:0]};                    
                end
            endcase
        end else begin
            {reg_protect[7:0], reg_result[31:0]} <= {reg_protect[7:0], reg_result[31:0]};            
        end
    end
end

endmodule