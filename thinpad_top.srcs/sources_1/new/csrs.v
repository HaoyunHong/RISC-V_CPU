`timescale 1ns / 1ps

module csrs(
    input wire clk,
    input wire we,
    
    input wire[11:0] addr,
    input wire[31:0] wdata,
	output reg[31:0] rdata,
    output reg[31:0] mscratch,
	output reg[31:0] mepc,
	output reg[31:0] mcause,
	output reg[31:0] mstatus,
	output reg[31:0] mtvec,
	output reg[31:0] satp
);
always @(posedge clk) begin
    if(we) begin
	    case(addr)
		    12'h340:begin
                mscratch <= wdata;
			end
		    12'h341:begin
                mepc <= wdata;
			end
		    12'h342:begin
                mcause <= wdata;
			end
		    12'h300:begin
                mstatus <= wdata;
			end
		    12'h305:begin
                mtvec <= wdata;
			end
		    12'h180:begin
                satp <= wdata;
			end
		endcase
    end
end

always @(*) begin
	case(addr)
	    12'h340:begin
			rdata <= mscratch;
		end
	    12'h341:begin
			rdata <= mepc;
		end
	    12'h342:begin
			rdata <= mcause;
		end
	    12'h300:begin
			rdata <= mstatus;
		end
	    12'h305:begin
			rdata <= mtvec;
		end
		12'h180:begin
			rdata <= satp;
	    end
	endcase
end

endmodule

