`timescale 1ns / 1ps
module ram_io(
    input wire clk,
    input wire rst,
    input wire[31:0] data_in,
    output reg[31:0] data_out,
    output wire done,
    
    input wire[31:0] addr,
    input wire oen,
    input wire wen,
    input wire ben,
    
    inout wire[31:0] base_ram_data_wire,
    output wire[19:0] base_ram_addr,
    output wire[3:0] base_ram_be_n,
    inout wire[31:0] ext_ram_data_wire,
    output wire[19:0] ext_ram_addr,
    output wire[3:0] ext_ram_be_n,
 
    output wire base_ram_ce_n,
    output wire base_ram_oe_n,
    output wire base_ram_we_n,

    output wire ext_ram_ce_n,
    output wire ext_ram_oe_n,
    output wire ext_ram_we_n,

    output wire uart_rdn,
    output wire uart_wrn,
    input wire uart_dataready,
    input wire uart_tbre,
    input wire uart_tsre,

    input wire mode,
    input wire[31:0] satp,
    
    output wire[1:0] wrong
);

wire[31:0] base_data_out,ext_data_out;
wire base_done,ext_done;
reg base_oen,base_wen,ext_oen,ext_wen,uart_oen,uart_wen;
reg[1:0] wrong_reg;

assign wrong = wrong_reg;

base_io _base_io(
    .clk(clk),
    .rst(rst),
    .data_in(data_in),
    .data_out(base_data_out),
    .done(base_done),

    .base_ram_data_wire(base_ram_data_wire),
    
    .base_oen(base_oen),
    .base_wen(base_wen),
    .base_ram_ce_n(base_ram_ce_n),
    .base_ram_oe_n(base_ram_oe_n),
    .base_ram_we_n(base_ram_we_n),

    .uart_oen(uart_oen),
    .uart_wen(uart_wen),
    .uart_rdn(uart_rdn),
    .uart_wrn(uart_wrn),
    .uart_dataready(uart_dataready),
    .uart_tbre(uart_tbre),
    .uart_tsre(uart_tsre)
);

ext_io _ext_io(
    .clk(clk),
    .rst(rst),
    .data_in(data_in),
    .data_out(ext_data_out),
    .done(ext_done),

    .ext_ram_data_wire(ext_ram_data_wire),
    
    .ext_oen(ext_oen),
    .ext_wen(ext_wen),
    .ext_ram_ce_n(ext_ram_ce_n),
    .ext_ram_oe_n(ext_ram_oe_n),
    .ext_ram_we_n(ext_ram_we_n)
);

reg[19:0] base_ram_addr_reg,ext_ram_addr_reg;
reg i;

assign base_ram_addr=base_ram_addr_reg;
assign ext_ram_addr=ext_ram_addr_reg;

localparam STATE_IDLE = 5'b00000;
localparam STATE_DECODE_ADDR = 5'b00001;
localparam BASE_0 = 5'b00010;
localparam BASE_1 = 5'b00011;
localparam EXT_0 = 5'b00100;
localparam EXT_1 = 5'b00101;
localparam STATE_DONE = 5'b00110;
localparam UART_0=5'b00111;
localparam UART_1=5'b01000;
localparam UART_STATE_0=5'b01001;
localparam STATE_MMU=5'b01010;
localparam STATE_GET_PTE_0=5'b01011;
localparam STATE_GET_PTE_1=5'b01100;
localparam PTE_BASE_0=5'b01101;
localparam PTE_BASE_1=5'b01110;
localparam PTE_EXT_0=5'b01111;
localparam PTE_EXT_1=5'b10000;
localparam STATE_CHECK=5'b10001;
localparam STATE_FOUND=5'b10010;

reg[31:0] pa_reg,va_reg,pte_reg,pte_addr,a,i;




reg[4:0] state;
assign done = state == STATE_DONE;

reg[3:0] base_ram_be_n_reg,ext_ram_be_n_reg;

assign base_ram_be_n = base_ram_be_n_reg;
assign ext_ram_be_n = ext_ram_be_n_reg;
    
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= STATE_IDLE;
        {base_oen,base_wen,ext_oen,ext_wen,uart_oen,uart_wen}<=6'b111111;
        {base_ram_be_n_reg,ext_ram_be_n_reg}<=8'b00000000;
        wrong_reg<=2'b00;
    end
    else begin
        case (state)
            STATE_IDLE: begin
                if(~oen | ~wen) begin
                    if(mode==1'b1) begin
                        pa_reg<=addr;
                        state<=STATE_DECODE_ADDR;
                    end
                    else begin
                        va_reg<=addr;
                        state<=STATE_MMU;
                    end
                       
                    wrong_reg<=2'b00;
                    base_ram_addr_reg<=20'h00000;
                    ext_ram_addr_reg<=20'h00000;
                    {base_ram_be_n_reg,ext_ram_be_n_reg}<=8'b00000000;
                end
            end
            STATE_DECODE_ADDR: begin
                if(pa_reg>=32'h80000000 & pa_reg<=32'h803FFFFF) begin
                    base_ram_addr_reg=pa_reg[21:2];
                    state<=BASE_0;
                end
                else if(pa_reg>=32'h80400000 & pa_reg<=32'h807FFFFF) begin
                    ext_ram_addr_reg=pa_reg[21:2];
                    state<=EXT_0;
                end
                else if(pa_reg==32'h10000000) begin
                    state<=UART_0;
                end
                else if(pa_reg==32'h10000005) begin
                    state<=UART_STATE_0;
                end

                if(~ben & pa_reg!=32'h10000005 & pa_reg!=32'h10000000) begin
                    if(pa_reg[1:0]==2'b00) begin
                        base_ram_be_n_reg<=4'b1110;
                        ext_ram_be_n_reg<=4'b1110;
                    end
                    else if(pa_reg[1:0]==2'b01) begin
                        base_ram_be_n_reg<=4'b1101;
                        ext_ram_be_n_reg<=4'b1101;
                    end
                    else if(pa_reg[1:0]==2'b10) begin
                        base_ram_be_n_reg<=4'b1011;
                        ext_ram_be_n_reg<=4'b1011;
                    end
                    else if(pa_reg[1:0]==2'b11) begin
                            base_ram_be_n_reg<=4'b0111;
                            ext_ram_be_n_reg<=4'b0111;
                    end
                end
            end
            BASE_0: begin
                base_oen<=oen;
                base_wen<=wen;
                state<=BASE_1;
            end
            BASE_1: begin
                if(base_done) begin
                    if(ben) data_out<=base_data_out;
                    else if(addr[1:0]==2'b00) data_out<={24'h000000,base_data_out[7:0]};
                    else if(addr[1:0]==2'b01) data_out<={24'h000000,base_data_out[15:8]};
                    else if(addr[1:0]==2'b10) data_out<={24'h000000,base_data_out[23:16]};
                    else if(addr[1:0]==2'b11) data_out<={24'h000000,base_data_out[31:24]};
                    
                    state<=STATE_DONE;
                    {base_oen,base_wen}<=2'b11;
                end
            end
            EXT_0: begin
                ext_oen<=oen;
                ext_wen<=wen;
                state<=EXT_1;
            end
            EXT_1: begin
                if(ext_done) begin
                    if(ben) data_out<=ext_data_out;
                    else if(addr[1:0]==2'b00) data_out<={24'h000000,ext_data_out[7:0]};
                    else if(addr[1:0]==2'b01) data_out<={24'h000000,ext_data_out[15:8]};
                    else if(addr[1:0]==2'b10) data_out<={24'h000000,ext_data_out[23:16]};
                    else if(addr[1:0]==2'b11) data_out<={24'h000000,ext_data_out[31:24]};
                    state<=STATE_DONE;
                    {ext_oen,ext_wen}<=2'b11;
                end
            end
            UART_0: begin
                uart_oen<=oen;
                uart_wen<=wen;
                state<=UART_1;
            end
            UART_1: begin
                if(base_done) begin
                    data_out<=base_data_out;
                    state<=STATE_DONE;
                    {uart_oen,uart_wen}<=2'b11;
                end
            end 
            UART_STATE_0: begin
                data_out<={24'h000000,2'b00,uart_tbre&uart_tsre,4'b0000,uart_dataready};
                state<=STATE_DONE;
            end




            STATE_MMU: begin//step 1
                i<=1'b1;
                a<=satp[21:0]*4096;
                state<=STATE_GET_PTE_0;
            end
            STATE_GET_PTE_0: begin//step 2
                if(i==1) pte_addr<=a+4*va_reg[31:22];
                else pte_addr<=a+4*va_reg[21:12];
                state<=STATE_GET_PTE_1;
            end
            STATE_GET_PTE_1: begin
                if(pte_addr>=32'h80000000 & pte_addr<=32'h803FFFFF) begin
                    base_ram_addr_reg=pte_addr[21:2];
                    state<=PTE_BASE_0;
                end
                else if(pte_addr>=32'h80400000 & pte_addr<=32'h807FFFFF) begin
                    ext_ram_addr_reg=pte_addr[21:2];
                    state<=PTE_EXT_0;
                end
            end
            PTE_BASE_0: begin
                {base_oen,base_wen}<=2'b01;
                state<=PTE_BASE_1;
            end
            PTE_BASE_1: begin
                if(base_done) begin
                    pte_reg<=base_data_out;
                    state<=STATE_CHECK;
                    {base_oen,base_wen}<=2'b11;
                end
            end
            PTE_EXT_0: begin
                {ext_oen,ext_wen}<=2'b01;
                state<=PTE_EXT_1;
            end
            PTE_EXT_1: begin
                if(ext_done) begin
                    pte_reg<=ext_data_out;
                    state<=STATE_CHECK;
                    {ext_oen,ext_wen}<=2'b11;
                end
            end
            STATE_CHECK: begin//step 3
                if((pte_reg[0]==1'b0) | (pte_reg[1]==1'b0 & pte_reg[2]==1'b1)) begin
                    //raise page-fault exception
                    wrong_reg<=2'b01;
                    state<=STATE_DONE;
                end
                else begin
                    if(pte_reg[1]==1'b1 | pte_reg[3]==1'b1) begin
                        state<=STATE_FOUND;
                    end
                    else begin
                        if(i==1'b0) begin
                            //raise page-fault exception
                            wrong_reg<=2'b01;
                            state<=STATE_DONE;
                        end
                        else begin
                            i<=i-1;
                            a<=pte_reg[29:10]*4096;
                            state<=STATE_GET_PTE_0;
                        end
                    end
                end
            end
            STATE_FOUND: begin//step 5
                if(~oen & ~pte_reg[1]) begin
                    //raise page-fault exception
                    wrong_reg<=2'b01;
                    state<=STATE_DONE;
                end
                else if(~wen & ~pte_reg[2]) begin
                    //raise page-fault exception
                    wrong_reg<=2'b01;
                    state<=STATE_DONE;
                end
                else if(~mode & ~pte_reg[4]) begin
                    //raise page-fault exception
                    wrong_reg<=2'b01;
                    state<=STATE_DONE;
                end
                else if(i==1'b1) begin
                    pa_reg[31:22]<=pte_reg[29:20];
                    pa_reg[21:12]<=va_reg[21:12];
                    pa_reg[11:0]<=va_reg[11:0];
                    state<=STATE_DECODE_ADDR;
                end
                else begin
                    pa_reg[31:12]<=pte_reg[29:10];
                    pa_reg[11:0]<=va_reg[11:0];
                    state<=STATE_DECODE_ADDR;
                end
            end

            





            STATE_DONE: begin
                if(oen & wen) begin
                    state<=STATE_IDLE;
                end
            end
        endcase
    end
end
endmodule
