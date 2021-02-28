`timescale 1ns / 1ps
module base_io(
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
    output reg base_ram_we_n,

    input wire uart_oen,
    input wire uart_wen,
    output reg uart_rdn,
    output reg uart_wrn,
    input wire uart_dataready,
    input wire uart_tbre,
    input wire uart_tsre
);
   
reg[31:0] ram_data;
reg data_z;
assign base_ram_data_wire = data_z ? 32'bz : { ram_data};
    
localparam STATE_IDLE = 5'b00000;
localparam STATE_READ_BASE_0 = 5'b00001;
localparam STATE_READ_BASE_1 = 5'b00010;
localparam STATE_WRITE_BASE_0 = 5'b00011;
localparam STATE_WRITE_BASE_1 = 5'b00100;
localparam STATE_WRITE_BASE_2 = 5'b00101;
localparam STATE_WRITE_BASE_3 = 5'b00110;
localparam STATE_WRITE_BASE_4 = 5'b00111;
localparam STATE_WRITE_BASE_5 = 5'b01000;
localparam STATE_DONE = 5'b01001;
localparam STATE_EMPTY = 5'b01010;
localparam STATE_READ_BASE_2 = 5'b01011;
localparam STATE_READ_UART_0 =5'b01100;
localparam STATE_READ_UART_1 =5'b01101;
localparam STATE_WRITE_UART_0 =5'b01110;
localparam STATE_WRITE_UART_1 =5'b01111;
localparam STATE_WRITE_UART_2 =5'b10000;
localparam STATE_WRITE_UART_3 =5'b10001;
    
reg[4:0] state;
assign done = state == STATE_DONE;
    
always @(posedge clk or posedge rst) begin
    if (rst) begin
        {base_ram_ce_n,base_ram_oe_n,base_ram_we_n} <= 3'b111;
        {uart_rdn,uart_wrn}<=2'b11;
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
                else if(~uart_oen) begin
                    data_z <= 1'b1;
                    state <= STATE_READ_UART_0;
                end
                else if(~uart_wen) begin
                    data_z <= 1'b0;
                    ram_data <= {24'h000000,data_in[7:0]};
                    state <= STATE_WRITE_UART_0;
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

            STATE_READ_UART_0: begin
                if (uart_dataready) begin
                    uart_rdn <= 1'b0;
                    state <= STATE_READ_UART_1;
                end 
            end
            STATE_READ_UART_1: begin
                data_out <= {24'h000000,base_ram_data_wire[7:0]};
                uart_rdn <= 1'b1;
                state <= STATE_DONE;
            end
            STATE_WRITE_UART_0: begin
                uart_wrn <= 1'b0;
                state <= STATE_WRITE_UART_1;
            end
            STATE_WRITE_UART_1: begin
                uart_wrn <= 1'b1;
                state <= STATE_WRITE_UART_2;
            end
            STATE_WRITE_UART_2: begin
                if (uart_tbre)
                    state <= STATE_WRITE_UART_3;
            end
            STATE_WRITE_UART_3: begin
                if (uart_tsre)
                    state <= STATE_DONE;
            end

            STATE_DONE: begin
                if (base_oen & base_wen & uart_oen & uart_wen) begin
                    state <= STATE_IDLE;
                    data_z <= 1'b1;
                    {base_ram_ce_n,base_ram_oe_n,base_ram_we_n} <= 3'b111;
                    {uart_rdn,uart_wrn}<=2'b11;
                end
            end
        endcase
    end
end
endmodule
