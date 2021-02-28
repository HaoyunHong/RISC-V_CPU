`timescale 1ns / 1ps

module regs(
    input wire clk,
    input wire we,
    
    input wire[4:0] waddr,
    input wire[31:0] wdata,
    
    input wire[4:0] raddr1,
    output reg[31:0] rdata1,
    
    input wire[4:0] raddr2,
    output reg[31:0] rdata2
);

reg[31:0] registers[0:31];

always @(posedge clk) begin
    if(we) begin
        registers[waddr] <= wdata;
    end
end

always @(*) begin
    if(raddr1 == 32'b0)
        rdata1 <= 32'b0;
    else
        rdata1 <= registers[raddr1];
end

always @(*) begin
    if(raddr2 == 32'b0)
        rdata2 <= 32'b0;
    else
        rdata2 <= registers[raddr2];
end

endmodule

