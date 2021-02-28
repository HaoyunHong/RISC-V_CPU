`default_nettype none

module thinpad_top(
    input wire clk_50M,           //50MHz ʱ������
    input wire clk_11M0592,       //11.0592MHz ʱ�����루���ã��ɲ��ã�

    input wire clock_btn,         //BTN5�ֶ�ʱ�Ӱ�ť���أ���������·������ʱΪ1
    input wire reset_btn,         //BTN6�ֶ���λ��ť���أ���������·������ʱΪ1

    input  wire[3:0]  touch_btn,  //BTN1~BTN4����ť���أ�����ʱΪ1
    input  wire[31:0] dip_sw,     //32λ���뿪�أ�����"ON"ʱΪ1
    output wire[15:0] leds,       //16λLED�����ʱ1����
    output wire[7:0]  dpy0,       //����ܵ�λ�źţ�����С���㣬���1����
    output wire[7:0]  dpy1,       //����ܸ�λ�źţ�����С���㣬���1����

    //CPLD���ڿ������ź�
    output wire uart_rdn,         //�������źţ�����Ч
    output wire uart_wrn,         //д�����źţ�����Ч
    input wire uart_dataready,    //��������׼����
    input wire uart_tbre,         //�������ݱ�־
    input wire uart_tsre,         //���ݷ�����ϱ�־

    //BaseRAM�ź�
    inout wire[31:0] base_ram_data,  //BaseRAM���ݣ���8λ��CPLD���ڿ���������
    output wire[19:0] base_ram_addr, //BaseRAM��ַ
    output wire[3:0] base_ram_be_n,  //BaseRAM�ֽ�ʹ�ܣ�����Ч�������ʹ���ֽ�ʹ�ܣ��뱣��Ϊ0
    output wire base_ram_ce_n,       //BaseRAMƬѡ������Ч
    output wire base_ram_oe_n,       //BaseRAM��ʹ�ܣ�����Ч
    output wire base_ram_we_n,       //BaseRAMдʹ�ܣ�����Ч

    //ExtRAM�ź�
    inout wire[31:0] ext_ram_data,  //ExtRAM����
    output wire[19:0] ext_ram_addr, //ExtRAM��ַ
    output wire[3:0] ext_ram_be_n,  //ExtRAM�ֽ�ʹ�ܣ�����Ч�������ʹ���ֽ�ʹ�ܣ��뱣��Ϊ0
    output wire ext_ram_ce_n,       //ExtRAMƬѡ������Ч
    output wire ext_ram_oe_n,       //ExtRAM��ʹ�ܣ�����Ч
    output wire ext_ram_we_n,       //ExtRAMдʹ�ܣ�����Ч

    //ֱ�������ź�
    output wire txd,  //ֱ�����ڷ��Ͷ�
    input  wire rxd,  //ֱ�����ڽ��ն�

    //Flash�洢���źţ��ο� JS28F640 оƬ�ֲ�
    output wire [22:0]flash_a,      //Flash��ַ��a0����8bitģʽ��Ч��16bitģʽ������
    inout  wire [15:0]flash_d,      //Flash����
    output wire flash_rp_n,         //Flash��λ�źţ�����Ч
    output wire flash_vpen,         //Flashд�����źţ��͵�ƽʱ���ܲ�������д
    output wire flash_ce_n,         //FlashƬѡ�źţ�����Ч
    output wire flash_oe_n,         //Flash��ʹ���źţ�����Ч
    output wire flash_we_n,         //Flashдʹ���źţ�����Ч
    output wire flash_byte_n,       //Flash 8bitģʽѡ�񣬵���Ч����ʹ��flash��16λģʽʱ����Ϊ1

    //USB �������źţ��ο� SL811 оƬ�ֲ�
    output wire sl811_a0,
    //inout  wire[7:0] sl811_d,     //USB�������������������dm9k_sd[7:0]����
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    //����������źţ��ο� DM9000A оƬ�ֲ�
    output wire dm9k_cmd,
    inout  wire[15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input  wire dm9k_int,

    //ͼ������ź�
    output wire[2:0] video_red,    //��ɫ���أ�3λ
    output wire[2:0] video_green,  //��ɫ���أ�3λ
    output wire[1:0] video_blue,   //��ɫ���أ�2λ
    output wire video_hsync,       //��ͬ����ˮƽͬ�����ź�
    output wire video_vsync,       //��ͬ������ֱͬ�����ź�
    output wire video_clk,         //����ʱ�����
    output wire video_de           //��������Ч�źţ���������������
);

reg oen,wen,ben;
reg[31:0] data_in;
wire[31:0] data_out;
wire[7:0] data_uart_out;
wire done;
wire[1:0] page_fault;

localparam FETCH = 5'b00000;
localparam DECODE = 5'b00001;
localparam EX = 5'b00010;
localparam MEM = 5'b00011;
localparam WRITE = 5'b00100;
localparam FETCH1 = 5'b00101;
localparam WRITE1 = 5'b00110;
localparam MEM1 = 5'b00111;
localparam INITIAL = 5'b01000;
localparam ECALL1 = 5'b01001;
localparam ECALL2 = 5'b01010;
localparam ECALL3 = 5'b01011;
localparam ECALL4 = 5'b01100;
localparam ECALL5 = 5'b01101;
localparam ECALL6 = 5'b01110;
localparam ECALL7 = 5'b01111;
localparam ECALL8 = 5'b10000;
localparam MRET1 = 5'b10001;
localparam FETCH0 = 5'b10010;

reg[4:0] state = INITIAL;

reg[31:0] addr;

wire[4:0] rs1,rs2,rd;
wire[11:0] csr;
reg[31:0] instr;
reg[31:0] pc, pc_now;
wire[31:0] rs1data,rs2data,csrdata;
wire[31:0] mscratch,mepc,mcause,mstatus,mtvec,satp;
reg[31:0] rddata,wcsrdata;
wire[31:0] imm;
reg[31:0] des_addr;
wire[4:0] op;
reg reg_we,csr_we;

reg[4:0] rs1_reg, rs2_reg, rd_reg;
reg[11:0] csr_reg;
// reg[31:0] rs1data_reg, rs2data_reg,rddata_reg;

// specific operations
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


reg[31:0] regA;
reg[31:0] regB;
wire[31:0] regC;
wire[1:0] jump;

reg[3:0] alu_op;
// alu op
localparam Add = 4'b0001;
localparam Equal = 4'b0010;
localparam And = 4'b0011;
localparam Or = 4'b0100;
localparam Xor = 4'b0101;
localparam Sll = 4'b0111;
localparam Srl = 4'b1000;

wire ena;
wire wea;
wire[16:0] addra;
wire [31 : 0] dina;
wire enb;
wire [18 : 0] addrb;
wire [7 : 0] doutb;

// �����Ǹ�ʹ�ܵ�
assign ena=1'b0;
assign wea=1'b0;
assign enb=1'b1; 

blk_mem_gen_0 _blk (
  .clka(clk_11M0592),    // input wire clka ��Ҫ��� 1/4
  .ena(ena),      // input wire ena
  .wea(wea),      // input wire [0 : 0] wea
  .addra(addra),  // input wire [16 : 0] addra
  .dina(dina),    // input wire [31 : 0] dina
  .clkb(clk_50M),    // input wire clkb
  .enb(enb),      // input wire enb
  .addrb(addrb),  // input wire [18 : 0] addrb
  .doutb(doutb)  // output wire [7 : 0] doutb
);

// ��������
wire[11:0] hdata;
wire[11:0] vdata;

assign addrb=vdata*800+hdata;

vga #(12, 800, 856, 976, 1040, 600, 637, 643, 666, 1, 1) _vga (
    .clk(clk_50M), 
    .hdata(hdata),
    .vdata(vdata),
    .hsync(video_hsync),
    .vsync(video_vsync),
    .data_enable(video_de)
);

assign video_clk = clk_50M;

assign video_red = doutb[7:5];
assign video_green = doutb[4:2];
assign video_blue = doutb[1:0];


reg _status=1;
reg inter=0;

ram_io _ram_io(
    .clk(clk_50M),
    .rst(reset_btn),
    .data_in(data_in),
    .data_out(data_out),
    .done(done),
    .addr(addr),
    .oen(oen),
    .wen(wen),
	.ben(ben),
    
    .base_ram_data_wire(base_ram_data),
    .base_ram_addr(base_ram_addr),
    .base_ram_be_n(base_ram_be_n),
    .ext_ram_data_wire(ext_ram_data),
    .ext_ram_addr(ext_ram_addr),
    .ext_ram_be_n(ext_ram_be_n),
    
    .base_ram_ce_n(base_ram_ce_n),
    .base_ram_oe_n(base_ram_oe_n),
    .base_ram_we_n(base_ram_we_n),

    .ext_ram_ce_n(ext_ram_ce_n),
    .ext_ram_oe_n(ext_ram_oe_n),
    .ext_ram_we_n(ext_ram_we_n),

    .uart_rdn(uart_rdn),
    .uart_wrn(uart_wrn),
    .uart_dataready(uart_dataready),
    .uart_tbre(uart_tbre),
    .uart_tsre(uart_tsre),
	.mode(_status),
	.satp(satp),
	.wrong(page_fault)
);

regs _regs(
    .clk(clk_50M),
    .raddr1(rs1_reg),
    .raddr2(rs2_reg),
    .rdata1(rs1data),
    .rdata2(rs2data),
    .we    (reg_we),
    .waddr (rd),
    .wdata (rddata)
);

csrs _csrs(
    .clk(clk_50M),
    .addr(csr_reg),
    .rdata(csrdata),
    .we    (csr_we),
    .wdata (wcsrdata),
	.mscratch(mscratch),
	.mepc(mepc),
	.mcause(mcause),
	.mstatus(mstatus),
	.mtvec(mtvec),
	.satp(satp)
);

decoder _decoder(
    .IR(instr),
	.rs1(rs1),
	.rs2(rs2),
	.csr(csr),
	.rd(rd),
	.op(op),
	.Imm(imm)
);
//assign leds=pc_now[15:0];
//SEG7_LUT _SEG7_LUT_1(dpy0, state[3:0]);
//SEG7_LUT _SEG7_LUT_2(dpy1, {3'b000,state[4]});
/*
assign leds=instr[31:16];
SEG7_LUT _SEG7_LUT_1(dpy0, instr[3:0]);
SEG7_LUT _SEG7_LUT_2(dpy1, instr[7:4]);
*/

assign leds={6'b000000,state,op};
reg[3:0] gg=4'b0000;
SEG7_LUT _SEG7_LUT_1(dpy0, gg);
SEG7_LUT _SEG7_LUT_2(dpy1, {op,1'b0,state[4]});

always @(posedge clk_50M or posedge reset_btn or posedge touch_btn[0]) begin
	if (reset_btn) begin
        {oen,wen,ben} <= 3'b111;
        state <= FETCH;
        addr<=32'h80000000;
        pc<=32'h80000000;
        reg_we<=1'b0;
        csr_we<=1'b0;
        instr<=32'h00000000;
        rs1_reg<=5'b00000;
        rs2_reg<=5'b00000;
        rd_reg<=5'b00000;
		_status<=1;
    end
 
    else if(touch_btn[0])begin
		csr_reg<=12'h342;//mcause
		wcsrdata<={28'h8000000,4'b0001};//�����쳣����,�ж�
		inter<=1;
	    state<=ECALL1;
	end
	
    else begin
        case (state)
            FETCH: begin
			    // �����ʱPC�Ͳ���4�ı�������ֱ�Ӵ���
                if(pc[1:0] != 2'b00) begin
				    csr_reg<=12'h342;//mcause
				    wcsrdata<={28'h0000000, 4'b0000};//�����쳣����,instruction address misaligned
					inter<=0;
                    state<=ECALL1;
                end
                else begin
                    state<=FETCH0;
                    pc_now<=pc;
                    addr<=pc;
                end
            end
            FETCH0: begin
                oen<=1'b0;
                state<=FETCH1;
            end
            FETCH1: begin
                if(done) begin
                    oen<=1'b1;
                    if(page_fault!=2'b00)begin
				        gg<=4'b1111;
				        csr_reg<=12'h342;//mcause
				        wcsrdata<={28'h0000000,4'b1100};//�����쳣����,page_fault
					    inter<=0;
                        state<=ECALL1;
					end
					else begin
                        instr<=data_out;
                        state<=DECODE;               
                        pc<=pc+4;
                    end
                end
            end
            DECODE: begin
                rs1_reg<=rs1;
                rs2_reg<=rs2;
                csr_reg<=csr;
                rd_reg<=rd;
                state<=EX;
            end     
            EX: begin
                if(op==_add) begin
                    rddata<=rs1data+rs2data;
                    state<=WRITE;
                end
                else if(op==_and) begin
                    rddata<=rs1data&rs2data;
                    state<=WRITE;
                end
                else if(op==_or) begin
                    rddata<=rs1data|rs2data;
                    state<=WRITE;
                end
                else if(op==_xor) begin
                    rddata<=rs1data^rs2data;
                    state<=WRITE;
                end
                else if(op==_addi) begin
                    rddata<=rs1data + imm;
                    state<=WRITE;
                end
                else if(op==_andi) begin
                    rddata<=rs1data & imm;
                    state<=WRITE;
                end
                else if(op==_ori) begin
                    rddata<=rs1data | imm;
                    state<=WRITE;
                end
                else if(op==_slli) begin
                    rddata<=rs1data<<imm;
                    state<=WRITE;
                end
                else if(op==_srli) begin
                    rddata<=rs1data>>imm;
                    state<=WRITE;
                end
                else if(op==_auipc) begin
                    rddata<=imm+pc_now;
                    state<=WRITE;
                end
                else if(op==_lui) begin
                    rddata<=imm;
                    state<=WRITE;
                end
                else if(op==_jalr) begin
                    rddata<=pc_now+4;
					if(rs1data[0]^imm[0])begin
                        pc<=rs1data+imm-1;
					end
					else begin
					    pc<=rs1data+imm;
					end
                    state<=WRITE;
                end
				else if(op==_jal) begin
                    rddata<=pc_now+4;
                    pc<=pc_now+imm;
                    state<=WRITE;
                end 
                else if(op==_sbclr) begin
				    if(rs1data[rs2data&(5'b11111)]==1)
                        rddata<=rs1data-(1<<(rs2data&(5'b11111)));
				    else
                        rddata<=rs1data;
                    state<=WRITE;
                end
				else if(op==_sbset) begin
				    if(rs1data[rs2data&(5'b11111)]==0)
                        rddata<=rs1data+(1<<(rs2data&(5'b11111)));
				    else
                        rddata<=rs1data;
                    state<=WRITE;
                end
				else if(op==_pcnt) begin
                    rddata<=rs1data[0]+rs1data[1]+rs1data[2]+rs1data[3]+rs1data[4]+rs1data[5]+rs1data[6]+rs1data[7]+rs1data[8]+rs1data[9]+rs1data[10]+rs1data[11]+rs1data[12]+rs1data[13]+rs1data[14]+rs1data[15]+rs1data[16]+rs1data[17]+rs1data[18]+rs1data[19]+rs1data[20]+rs1data[21]+rs1data[22]+rs1data[23]+rs1data[24]+rs1data[25]+rs1data[26]+rs1data[27]+rs1data[28]+rs1data[29]+rs1data[30]+rs1data[31];
                    state<=WRITE;
                end
				else if(op==_csrrc) begin
				    rddata<=csrdata;
					wcsrdata<=csrdata&(~rs1data);
                    state<=WRITE;
                end 
				else if(op==_csrrs) begin
				    rddata<=csrdata;
					wcsrdata<=csrdata|rs1data;
                    state<=WRITE;
                end
				else if(op==_csrrw) begin
				    rddata<=csrdata;
					wcsrdata<=rs1data;
                    state<=WRITE;
                end 
				///
				else if(op==_ecall) begin
				    csr_reg<=12'h342;//mcause
				    wcsrdata<={28'h0000000,4'b1000};//�����쳣����,ecall
					inter<=1'b0;
                    state<=ECALL1;
                end
				else if(op==_ebreak) begin
				    csr_reg<=12'h342;//mcause
				    wcsrdata<={28'h0000000,4'b0011};//�����쳣����,ebreak
					inter<=1'b0;
                    state<=ECALL1;
                end 
				else if(op==_mret) begin
                    state<=MRET1;
                end 
				///
                else if(op==_sw || op==_lw || op==_sb || op==_lb) begin
                    des_addr<=rs1data+imm;
                    state<=MEM;
                end
                else if(op==_beq) begin
                    if(rs1data==rs2data) begin
                        pc<=pc_now+imm;
                    end
                    state<=FETCH;
                end
				else if(op==_bne) begin
                    if(rs1data!=rs2data) begin
                        pc<=pc_now+imm;
                    end
                    state<=FETCH;
                end
                else if(op==_SFENCE) begin
                    state<=FETCH;
                end
				else if(op==_miss) begin
				    csr_reg<=12'h342;//mcause
				    wcsrdata<={28'h0000000, 4'b0010};//�����쳣����,illegal instruction
					inter<=1'b0;
                    state<=ECALL1;
                end
            end
			//ecall,ebreak
			ECALL1:begin
                csr_we<=1'b1;
                state<=ECALL2;
			end
			ECALL2:begin
                csr_we<=1'b0;
                state<=ECALL3;
			end
			ECALL3:begin
				csr_reg<=12'h341;//mepc
				if(inter==1'b1)
				    wcsrdata<=pc_now+4;//�����ж�ָ���ַ
				else
				    wcsrdata<=pc_now;//�����쳣ָ���ַ
				state<=ECALL4;
			end
			ECALL4:begin
                csr_we<=1'b1;
                state<=ECALL5;
			end
			ECALL5:begin
                csr_we<=1'b0;
			    pc<={mtvec[31:2],2'b00};//ת����mtvec
				_status<=1;
                state<=ECALL6;
			end
			ECALL6:begin
				csr_reg<=12'h300;//mstatus
			    wcsrdata<={mstatus[31:13],2'b00,mstatus[10:0]};
                state<=ECALL7;
			end
			ECALL7:begin
                csr_we<=1'b1;
                state<=ECALL8;
			end
			ECALL8:begin
                csr_we<=1'b0;
                state<=FETCH;
			end
			//
			//mret
			MRET1:begin
			    _status<=mstatus[12];
				pc<=mepc;
                state<=FETCH;
			end
			//
            MEM:begin
                addr<=des_addr;
                if(op==_sw||op==_sb) begin
                    if(op==_sw) data_in<=rs2data;
                    else data_in<={rs2data[7:0],rs2data[7:0],rs2data[7:0],rs2data[7:0]};
                    wen=1'b0;
                    if(op==_sb) begin
					    ben<=0;
					end
                    else if(op==_sw) begin
					    ben<=1;
					end
                end
                else if(op==_lw||op==_lb) begin
                    oen=1'b0;
                    if(op==_lb) begin
					    ben<=0;
					end
                    else if(op==_lw) begin
					    ben<=1;
					end
				end
                state<=MEM1;
            end
            MEM1:begin
                if(done) begin
                    {wen,oen,ben}<=3'b111;
                    
					if(page_fault!=2'b00)begin
				        gg<=4'b1111;
				        csr_reg<=12'h342;//mcause
				        wcsrdata<={28'h0000000,4'b1100};//�����쳣����,page_fault
					    inter<=0;
                        state<=ECALL1;
					end
					
					else begin
                        if(op==_lw||op==_lb) begin
				    		if(op==_lb) begin
					            rddata<={{24{data_out[7]}},data_out[7:0]};
					        end
                            else if(op==_lw) begin
					            rddata<=data_out;
					        end
						    state<=WRITE;
                        end
                        else if(op==_sw||op==_sb) begin
                            state<=FETCH;
                        end
					end
                end
            end
            WRITE:begin
                reg_we<=1'b1;
				if(op==_csrrc||op==_csrrs||op==_csrrw)
                    csr_we<=1'b1;
                state<=WRITE1;
            end
            WRITE1:begin
                state<=FETCH;
                reg_we<=1'b0;
				if(op==_csrrc||op==_csrrs||op==_csrrw)
                    csr_we<=1'b0;
            end
        endcase
    end
end
endmodule