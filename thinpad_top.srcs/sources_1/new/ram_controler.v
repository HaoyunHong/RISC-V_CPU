`timescale 1ns / 1ps

module ram_controler(
    input wire clk,
    input wire rst,
    input wire[31:0] data_in,
    output reg[31:0] data_out,
    output wire done,
    
    inout wire[31:0] base_ram_data_wire,
 
    input wire base_oen,
    input wire base_wen,
    output reg base_ram_ce_n,
    output reg base_ram_oe_n,
    output reg base_ram_we_n
    );
    
    reg[31:0] ram_data;
    reg data_z;
    assign base_ram_data_wire = data_z ? 32'bz : { ram_data};
    
    localparam STATE_IDLE = 4'b0000;
    localparam STATE_READ_BASE_0 = 4'b0001;
    localparam STATE_READ_BASE_1 = 4'b0010;
    localparam STATE_WRITE_BASE_0 = 4'b0011;
    localparam STATE_WRITE_BASE_1 = 4'b0100;
    localparam STATE_WRITE_BASE_2 = 4'b0101;
    localparam STATE_WRITE_BASE_3 = 4'b0110;
    localparam STATE_WRITE_BASE_4 = 4'b0111;
    localparam STATE_WRITE_BASE_5 = 4'b1000;
    localparam STATE_DONE = 4'b1001;
    localparam STATE_EMPTY = 4'b1010;
    localparam STATE_READ_BASE_2 = 4'b1011;
    
    reg[3:0] state = STATE_EMPTY;
    assign done = state == STATE_DONE;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            {base_ram_ce_n,base_ram_oe_n,base_ram_we_n} <= 3'b111;
            data_z <= 1'b1;
            state <= STATE_IDLE;
        end
        else begin
            case (state)
                STATE_IDLE: begin
                    if(~base_oen) begin
                        data_z <= 1'b1;
                        state <= STATE_READ_BASE_0;
                    end
                    else if(~base_wen) begin
                        data_z<=1'b0;
                        ram_data<=data_in;
                        state<=STATE_WRITE_BASE_0;
                    end
                end
                STATE_READ_BASE_0: begin
                    base_ram_ce_n<=1'b0;
                    state<=STATE_READ_BASE_1;
                end
                STATE_READ_BASE_1: begin
                    base_ram_oe_n<=1'b0;
                    state<=STATE_READ_BASE_2;
                end
                STATE_READ_BASE_2: begin
                    data_out <= base_ram_data_wire;
                    base_ram_ce_n<=1'b1;
                    base_ram_oe_n<=1'b1;
                    state <= STATE_DONE;
                end
                STATE_WRITE_BASE_0: begin
                    base_ram_ce_n <= 1'b0;
                    state <= STATE_WRITE_BASE_1;
                end
                STATE_WRITE_BASE_1: begin
                    state <= STATE_WRITE_BASE_2;
                end
                STATE_WRITE_BASE_2: begin
                    base_ram_we_n <=1'b0;
                    state <= STATE_WRITE_BASE_3;
                end
                STATE_WRITE_BASE_3: begin
                     state <= STATE_WRITE_BASE_4;
                end
                STATE_WRITE_BASE_4: begin
                    base_ram_we_n <=1'b1;
                    state <= STATE_WRITE_BASE_5;
                end
                STATE_WRITE_BASE_5: begin
                    state <= STATE_DONE;
                end
                STATE_DONE: begin
                    if (base_oen&base_wen) begin
                        state <= STATE_IDLE;
                        data_z <= 1'b1;
                        {base_ram_ce_n,base_ram_oe_n,base_ram_we_n} <= 3'b111;
                    end
                end
            endcase
        end
    end
endmodule