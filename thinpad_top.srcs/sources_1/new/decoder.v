`timescale 1ns / 1ps

module decoder(
    input wire[31:0] IR,
	output reg[4:0] rs1,
	output reg[4:0] rs2,
	output reg[11:0] csr,
	output reg[4:0] rd,
	output reg[4:0] op,
	output reg[31:0] Imm
    );
	localparam _add =  5'b00000;
	localparam _and =  5'b00001;
	localparam _or =   5'b00010;
	localparam _xor =  5'b00011;
	localparam _addi = 5'b00100;
	localparam _andi = 5'b00101;
	localparam _ori =  5'b00110;
	localparam _slli = 5'b00111;
	localparam _srli = 5'b01000;
	localparam _lb =   5'b01001;
	localparam _lw =   5'b01010;
	localparam _sb =   5'b01011;
	localparam _sw =   5'b01100;
	localparam _beq =  5'b01101;
	localparam _bne =  5'b01110;
	localparam _jal =  5'b01111;
	localparam _auipc =5'b10000;
	localparam _lui =  5'b10001;
	localparam _jalr = 5'b10010;
	localparam _sbclr =5'b10011;
	localparam _sbset =5'b10100;
	localparam _pcnt = 5'b10101;
	localparam _csrrc =5'b10110;
	localparam _csrrs =5'b10111;
	localparam _csrrw =5'b11000;
	localparam _ebreak=5'b11001;
	localparam _ecall =5'b11010;
	localparam _mret = 5'b11011;
	localparam _SFENCE=5'b11100;
	localparam _miss=5'b11101;
	
    localparam R_type = 7'b0110011;  //add and or xor sbclr sbset
    localparam I_type_1 = 7'b0010011;  //addi andi ori slli srli pcnt
    localparam I_type_2 = 7'b0000011;  //lb lw
    localparam S_type = 7'b0100011;  //sb sw
    localparam B_type = 7'b1100011;  //beq bne
    localparam J_type = 7'b1101111;  //jal
    localparam AUIPC = 7'b0010111;  //auipc
    localparam LUI = 7'b0110111;  //lui
    localparam JALR = 7'b1100111;  //jalr
    always @* begin
	    rs1[4:0]<=IR[19:15];
	    rs2[4:0]<=IR[24:20];
	    rd[4:0]<=IR[11:7];
		csr[11:0]<=IR[31:20];
		case(IR[6:0])
		    R_type :begin //add and or xor sbclr sbset
			    Imm<=0;
		        case(IR[14:12])
				    3'b000 :begin
					    op<=_add;
					end
				    3'b111 :begin
					    op<=_and;
					end
				    3'b110 :begin
					    op<=_or;
					end
				    3'b100 :begin
					    op<=_xor;
				    end
				    3'b001 :begin
					    if(IR[31:25]==7'b0100100)begin
					        op<=_sbclr;
					    end
						else begin
					        op<=_sbset;
						end
					end
					default:
					    op<=_miss;
				endcase
			end
			I_type_1 :begin
			    if(IR[14:12]==3'b001||IR[14:12]==3'b101) begin //slli srli pcnt
			        Imm<={20'h00000,IR[31:20]};
				end
				else begin //addi andi ori
			        Imm<={{20{IR[31]}},IR[31:20]};
				end
				case(IR[14:12])
				    3'b000 :begin
					    op<=_addi;
					end
				    3'b111 :begin
					    op<=_andi;
					end
				    3'b110 :begin
					    op<=_ori;
					end
				    3'b001 :begin
					    if(IR[31:25]==7'b0000000)begin
					        op<=_slli;
					    end
						else begin
					        op<=_pcnt;
						end
					end
				    3'b101 :begin
					    op<=_srli;
					end
					default:
					    op<=_miss;
				endcase
			end
			I_type_2 :begin //lb lw
			    Imm<={{20{IR[31]}},IR[31:20]};
				case(IR[14:12])
				    3'b000 :begin
					    op<=_lb;
					end
				    3'b010 :begin
					    op<=_lw;
				    end
					default:
					    op<=_miss;
				endcase
			end
			S_type :begin //sb sw
			    Imm<={{20{IR[31]}},IR[31:25],IR[11:7]};
				case(IR[14:12])
				    3'b000 :begin
					    op<=_sb;
					end
				    3'b010 :begin
					    op<=_sw;
				    end
					default:
					    op<=_miss;
				endcase
			end
			B_type :begin //beq bne
			    Imm<={{19{IR[31]}},IR[31],IR[7],IR[30:25],IR[11:8],1'b0};
				case(IR[14:12])
				    3'b000 :begin
					    op<=_beq;
					end
				    3'b001 :begin
					    op<=_bne;
				    end
					default:
					    op<=_miss;
				endcase
			end
			J_type :begin //jal
			    Imm<={{11{IR[31]}},IR[31],IR[19:12],IR[20],IR[30:21],1'b0};
				op<=_jal;
			end
			AUIPC :begin //auipc
			    Imm<={IR[31:12],12'h000};
				op<=_auipc;
			end
			LUI :begin //lui
			    Imm<={IR[31:12],12'h000};
				op<=_lui;
			end
			JALR :begin //jalr
			    Imm<={{20{IR[31]}},IR[31:20]};
				op<=_jalr;
			end
			7'b1110011 :begin
			    case(IR[14:12])
				    3'b011:begin//csrrc
					    op<=_csrrc;
					end
					3'b010:begin//csrrs
					    op<=_csrrs;
					end
					3'b001:begin//csrrw
					    op<=_csrrw;
					end
					3'b000:begin
			            case(IR[31:20])
						    12'b000000000000:begin
					            op<=_ecall;
							end
						    12'b000000000001:begin
					            op<=_ebreak;
							end
						    12'b001100000010:begin
					            op<=_mret;
							end
							default:begin
							    if(IR[31:25]==7'b0001001)
					                op<=_SFENCE;
							end
						endcase
					end
					default:
					    op<=_miss;
				endcase
			end
		endcase
	end
endmodule
