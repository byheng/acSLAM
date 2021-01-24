`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/18/2018 04:19:35 PM
// Design Name: 
// Module Name: sort_node
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define SIM
//`define _MAX_
module sort_node
	#(
	parameter DATA_WIDTH = 32,
	parameter KEY_WIDTH = 16,
	parameter ADDR_WIDTH = 5,
	parameter INIT_DATA = {2'b01,{(DATA_WIDTH-2-KEY_WIDTH){1'b0}},{KEY_WIDTH{1'b0}}},  //×îÐ¡
	parameter LEVEL = 1
	)
	(
	input clk,
	input rstn,
	input init,
//up memory ports
	input [DATA_WIDTH-1:0] um_in,    //never used      
	output [DATA_WIDTH-1:0] um_out,
	output [ADDR_WIDTH-1:0] um_addr,
	output um_we,
//left memory ports
	input [DATA_WIDTH-1:0] lm_in,
	output [DATA_WIDTH-1:0] lm_out,  //ÒªÐŽµœŽ¢ŽæÆ÷µÄÖµ
	output [ADDR_WIDTH-1:0] lm_addr,
	output reg lm_we,
//right memory ports
	input [DATA_WIDTH-1:0] rm_in,
	output [DATA_WIDTH-1:0] rm_out,
	output [ADDR_WIDTH-1:0] rm_addr,
	output reg rm_we,
//value and control from/to previous level
	input pl_update_in,						//ÊäÈëžüÐÂÊ¹ÄÜÐÅºÅ
	input [ADDR_WIDTH-1:0] pl_addr_in,  //ÊäÈëµØÖ·
	input pl_branch_in,                 //0: left 1: right
	input [DATA_WIDTH-1:0] pl_in,  //ÊäÈëÊýŸÝ
	output reg [DATA_WIDTH-1:0] pl_out,  //Êä³öÊýŸÝ
	output reg pl_update_out,   //Êä³öžüÐÂÊ¹ÄÜ
	output [ADDR_WIDTH-1:0] pl_addr_out,
	output reg pl_branch_out,
//by-pass value from/to next level
	input nl_update_in,     				//indicate nl_in is valid
	input [ADDR_WIDTH-1:0] nl_addr_in,
	input nl_branch_in,
	input [DATA_WIDTH-1:0] nl_in,
	output reg [DATA_WIDTH-1:0] nl_out,
	output reg nl_update_out,				//indicate nl_out is new
	output [ADDR_WIDTH-1:0] nl_addr_out,
	output reg nl_branch_out               //0:left   1:right

	
    );

localparam ADDR_MAX = 1<<LEVEL;   //µØÖ·×îŽóÖµ
	
localparam IDLE		= 2'b00;	//when pl_update_in is valid, write pl_in and nl_in(if valid) into register, and read DPRAM from next level(lm and rm)
localparam INIT		= 2'b01;	//write INIT data into DPRAM
localparam SWAP		= 2'b10;	//compare and swap the three data

reg [1:0] pstate;//µ±Ç°×ŽÌ¬
reg [1:0] nstate;//ÏÂÒ»×ŽÌ¬

reg nl_update_in_r;

reg [DATA_WIDTH-1:0] pl_in_r;	//cache input data from previous level ÀŽ×ÔÉÏÒ»Œ¶µÄÊýŸÝ
reg [DATA_WIDTH-1:0] nl_in_r;	//cache input data from next level  ÀŽ×ÔÏÂÒ»Œ¶µÄÊýŸÝ
reg [DATA_WIDTH-1:0] lm_in_r;  //select between lm_in and nl_in_r;  ÔÚ×óÊäÈëºÍÉÏÒ»Œ¶ÊýŸÝÑ¡Ôñ
reg [DATA_WIDTH-1:0] rm_in_r;  //select between rm_in and nl_in_r;   ÔÚÓÒÊäÈë
reg [DATA_WIDTH-1:0] lm_in_r_reg; //avoid latch
reg [DATA_WIDTH-1:0] rm_in_r_reg;
reg [DATA_WIDTH-1:0] pl_out_reg;
reg [DATA_WIDTH-1:0] nl_out_reg;
reg [ADDR_WIDTH-1:0] nl_addr_in_r;
reg [ADDR_WIDTH-1:0] pl_addr_in_r;

reg nl_branch_in_r;

reg [ADDR_WIDTH-1:0] lrm_addr;
reg [ADDR_WIDTH-1:0] lrm_addr_r;
	
reg [ADDR_WIDTH-1:0] addr;	 //address to write initial data to DPRAM

always@(posedge clk or negedge rstn) begin
	if (!rstn)
		pstate <= IDLE;
	else
		pstate <= nstate;
end

always@(*) begin   //×ŽÌ¬»ú×ª»»
	case (pstate)
	IDLE: nstate = init ? INIT : (pl_update_in ? SWAP : IDLE); //Èç¹û³õÊŒ»¯ÐÅºÅÊ¹ÄÜ£¬ŸÍÈ¥³õÊŒ»¯£¬Èç¹û²»ÊÇ£¬ÅÐ¶ÏÊÇ·ñÒªžøÉÏÒ»Œ¶ÊýŸÝžüÐÂ
	INIT: nstate = addr==ADDR_MAX-1 ? IDLE : INIT;  //ÊÇ·ñ³õÊŒ»¯ÍêËùÓÐµØÖ·ÊýŸÝ
	SWAP: nstate = IDLE;  //œ»»»ÍêœøÈëIDLE
	default: nstate = IDLE;
	endcase
end
	
assign lm_addr = lrm_addr;
assign lm_out = nl_out;
assign rm_addr = lrm_addr;	
assign rm_out = nl_out;
assign um_we = pl_update_out;
assign um_out = pl_out;
assign um_addr = pl_addr_in_r;	
assign nl_addr_out = lrm_addr; //ÍùÏÂžøÊýŸÝµØÖ·
assign pl_addr_out = pl_addr_in_r;
//reg lm_we_delay;
always@(*) begin
	case (pstate)
	IDLE: begin  //ÏÐÖÃ×ŽÌ¬
		pl_out = pl_out_reg;
		pl_update_out = 0;
		nl_out = nl_out_reg;
		nl_update_out = 0;
		lrm_addr = (pl_addr_in << 1) + pl_branch_in;// + pl_branch_in * LEVEL;//lrm_addr_r;  ŽÓÉÏÃæÊäÈëµÄµØÖ·×ª»¯µœÏÂÃæµÄµØÖ·£¬²¢ÇÒ×óÓÒ¹²ÓÃÒ»žöµØÖ·
		lm_we = 0;
		rm_we = 0;
		lm_in_r = lm_in_r_reg;  //ËøŽæ
		rm_in_r = rm_in_r_reg;
		nl_branch_out = 0;
	end
	INIT: begin
		//pl_out = pl_out_reg;
		pl_out = INIT_DATA;
		pl_update_out = 0;  //²»ÐèÒªÍùÉÏÃæÊä³öÊýŸÝ
		nl_out = INIT_DATA;
		nl_update_out = 1;
		lm_we = 1;
		rm_we = 1;
		lrm_addr = addr;
		lm_in_r = lm_in_r_reg;
		rm_in_r = rm_in_r_reg;
		nl_branch_out = 0;
	end
	SWAP: begin   //œ»»»×ŽÌ¬»ú
		lrm_addr = lrm_addr_r;// pl_addr_in + pl_branch_in * LEVEL;//2'd1;// pl_branch_in * 2;
		//get left or right data from by-pass channel
		if (nl_update_in_r && nl_addr_in_r==lrm_addr_r) begin
			if (!nl_branch_in_r) begin  //ÅÐ¶ÏbranchÖžÏòÄÄ±ß
				lm_in_r = nl_in_r;  //ÅÔÂ·ÊýŸÝžø×ó±ßr
				rm_in_r = rm_in;    //ÓÒ±ßžøÓÒ±ßr
			end
			else begin
				lm_in_r = lm_in;   //×ó±ßžø×ó±ßr
				rm_in_r = nl_in_r;   //ÓÒ±ßÒªÅÔÂ·ÊýŸÝ
			end
		end
		//get left or right data from DPRAM
		else begin   //Èç¹ûÅÔÂ·žüÐÂ²»¿ÉÓÃ
			lm_in_r = lm_in;
			rm_in_r = rm_in;
		end	
		//swap up data and left data
`ifdef _MAX_
        if (cmp_lt(pl_in_r, lm_in_r) && cmp_lte(rm_in_r, lm_in_r))// begin ÅÐ¶Ï×ó±ß±ÈÓÒ±ßºÍÉÏÃæ¶ŒŽóÄÇÃŽ
`else
		if (cmp_lt(lm_in_r, pl_in_r) && cmp_lte(lm_in_r, rm_in_r))// begin
`endif
        begin
			pl_out = lm_in_r;   //ÏÂÃæžøÉÏÃæŽ«×ó²àÖµ
			nl_out = pl_in_r;   //ÉÏÃæžøÏÂÃæŽ«ÉÏÃæµÄÖµ
			pl_update_out = 1;  //ÉÏÊýŸÝžüÐÂ¿ÉÓÃ
			nl_update_out = 1;  //ÏÂžüÐÂ¿ÉÓÃ
			//lm_we_delay = 1;   //×ó±ßÐŽÊ¹ÄÜŒ€»î
			lm_we = 1;
			rm_we = 0;   //ÓÒ±ß²»Œ€»î
			nl_branch_out = 0;  //Ñ¡×ó±ß
		end

		//swap up data and right data
`ifdef _MAX_
		else if (cmp_lt(pl_in_r, rm_in_r) && cmp_lt(lm_in_r, rm_in_r)) //ÓÒ±ß±ÈÉÏÃæŽó£¬²¢ÇÒ±È×ó±ßŽó£¬ÒªÉÏÃæºÍÓÒ±ß»»
`else	
        else if (cmp_lt(rm_in_r, pl_in_r) && cmp_lt(rm_in_r, lm_in_r))
`endif
		begin
			pl_out = rm_in_r;
			nl_out = pl_in_r;
			pl_update_out = 1;
			nl_update_out = 1;
			lm_we = 0;
			//lm_we_delay =0;
			rm_we = 1;
			nl_branch_out = 1;
		end
		//do nothing
		else begin
			pl_out = pl_in_r;
			nl_out = nl_in_r;
            pl_update_out = 0;
            nl_update_out = 1;

//			pl_update_out = 0;
//			nl_update_out = 0;
			lm_we = 0;
			//lm_we_delay = 0;
			rm_we = 0;
			nl_branch_out = 0;
		end
	end
	default: begin
		pl_out = pl_out_reg;
		pl_update_out = 0;
		nl_out = nl_out_reg;
		nl_update_out = 0;
		lrm_addr = lrm_addr_r;
		lm_we = 0;
		//lm_we_delay = 0;
		rm_we = 0;
		lm_in_r = lm_in_r_reg;
		rm_in_r = rm_in_r_reg;
		nl_branch_out = 0;
	end
	endcase
end
//always@(negedge clk) begin
  //  lm_we <= lm_we_delay?1:lm_we; 
//end
always@(posedge clk or negedge rstn) begin
	if (!rstn) begin
		pl_addr_in_r <= 0;
		nl_addr_in_r <= 0;
		lrm_addr_r <= 0;
		//pl_update_in_r <= 0;
		nl_update_in_r <= 0;
		nl_branch_in_r <= 0;
		addr <= 0;
		pl_in_r <= 0;
		nl_in_r <= 0;
		pl_branch_out <= 0;
		lm_in_r_reg <= 0;
		rm_in_r_reg <= 0;
		pl_out_reg <= 0;
		nl_out_reg <= 0;
	end
	else begin
		pl_addr_in_r <= pl_addr_in;
		nl_addr_in_r <= nl_addr_in;
		lrm_addr_r <= lrm_addr;
		//pl_update_in_r <= pl_update_in;
		nl_update_in_r <= nl_update_in;
		nl_branch_in_r <= nl_branch_in;
		pl_branch_out <= pl_branch_in;
		lm_in_r_reg <= lm_in_r;
		rm_in_r_reg <= rm_in_r;
		pl_out_reg <= pl_out;
		nl_out_reg <= nl_out;
		if (pstate==INIT)
			addr <= addr==ADDR_MAX-1 ? 0 : addr+1;	  //initial×ŽÌ¬ addrµÝÔö
		if (pl_update_in) begin
			pl_in_r <= pl_in;
			nl_in_r <= nl_in;
		end
	end
end
/*
module cmp_lt
#(
	parameter DATA_WIDTH = 32,
	parameter KEY_WIDTH = 16
)
(
	input [DATA_WIDTH-1:0] d1,
	input [DATA_WIDTH-1:0] d2,
	output 
*/
function cmp_lt;        //return true if d1 key < d2 key
	input [DATA_WIDTH-1:0] d1; 
	input [DATA_WIDTH-1:0] d2;
	reg [1:0] d1_flag;
	reg [1:0] d2_flag;
	reg [KEY_WIDTH-1:0] d1_key;
	reg [KEY_WIDTH-1:0] d2_key;	
	begin
		d1_flag = d1[DATA_WIDTH-1:DATA_WIDTH-2];
		d2_flag = d2[DATA_WIDTH-1:DATA_WIDTH-2];
		d1_key = d1[KEY_WIDTH-1:0];
		d2_key = d2[KEY_WIDTH-1:0];
		case (d1_flag)
		2'b01: begin    //d1 is min
		    case (d2_flag)
		    2'b11: cmp_lt = 1;  //d2 is max
		    2'b00: cmp_lt = 1;  //d2 is normal
		    2'b01: cmp_lt = 0;  //d2 is min
		    default: cmp_lt = 0;
		    endcase
        end
        2'b11: begin    //d1 is max
            cmp_lt = 0;
        end
        2'b00: begin    //d1 is normal
            case(d2_flag)
            2'b11: cmp_lt = 1;  //d2 is max
            2'b01: cmp_lt = 0;  //d2 is min
            2'b00: cmp_lt = d1_key < d2_key;  //d2 is normal
            default: cmp_lt = 0;
            endcase
        end
        default:
            cmp_lt = 0;
        endcase
    end
endfunction
            
            
		    		    
		
	/*	
		
		//d1 is initial data
		if (d1_flag==2'b01)
			cmp_lt = 1;
		//d1 is flush data
		else if (d1_flag==2'b11)
			cmp_lt = 0;
		//d1 is variable
		else if (d1_flag==2'b00) begin
			//d2 is initial data
			if (d2_flag==2'b01)
				cmp_lt = 0;
			//d2 is flush data
			else if (d2_flag==2'b11)
				cmp_lt = 1;
			//d2 is variable
			else if (d2_flag==2'b00)
				cmp_lt = d1_key<d2_key;
			else
				cmp_lt = 0;
		end
		else
			cmp_lt = 0;
	end
endfunction
*/
function cmp_lte;        //return true if d1 key <= d2 key
	input [DATA_WIDTH-1:0] d1; 
	input [DATA_WIDTH-1:0] d2;
	reg [1:0] d1_flag;
	reg [1:0] d2_flag;
	reg [KEY_WIDTH-1:0] d1_key;
	reg [KEY_WIDTH-1:0] d2_key;	
	begin
		d1_flag = d1[DATA_WIDTH-1:DATA_WIDTH-2];
		d2_flag = d2[DATA_WIDTH-1:DATA_WIDTH-2];
		d1_key = d1[KEY_WIDTH-1:0];
		d2_key = d2[KEY_WIDTH-1:0];
		d1_flag = d1[DATA_WIDTH-1:DATA_WIDTH-2];
        d2_flag = d2[DATA_WIDTH-1:DATA_WIDTH-2];
        d1_key = d1[KEY_WIDTH-1:0];
        d2_key = d2[KEY_WIDTH-1:0];
        case (d1_flag)
        2'b01: begin    //d1 is min
            case (d2_flag)
            2'b11: cmp_lte = 1;  //d2 is max
            2'b00: cmp_lte = 1;  //d2 is normal
            2'b01: cmp_lte = 1;  //d2 is min
            default: cmp_lte = 0;
            endcase
        end
        2'b11: begin    //d1 is max
            case (d2_flag)
            2'b11: cmp_lte = 1;  //d2 is max
            default: cmp_lte = 0;
            endcase
        end
        2'b00: begin    //d1 is normal
            case(d2_flag)
            2'b11: cmp_lte = 1;  //d2 is max
            2'b01: cmp_lte = 0;  //d2 is min
            2'b00: cmp_lte = d1_key < d2_key || d1_key == d2_key;  //d2 is normal
            default: cmp_lte = 0;
            endcase
        end
        default:
            cmp_lte = 0;
        endcase
    end
endfunction
		    
		
		
		
		
/*		
		//d1 is initial data
		if (d1_flag==2'b01)
			cmp_lte = 1;
		//d1 is flush data
		else if (d1_flag==2'b11)
			cmp_lte = 0;
		//d1 is variable
		else if (d1_flag==2'b00) begin
			//d2 is initial data
			if (d2_flag==2'b01)
				cmp_lte = 0;
			//d2 is flush data
			else if (d2_flag==2'b11)
				cmp_lte = 1;
			//d2 is variable
			else if (d2_flag==2'b00)
				cmp_lte = d1_key<d2_key || d1_key==d2_key;
			else
				cmp_lte = 0;
		end
		else
			cmp_lte = 0;
	end
endfunction
*/

`ifdef SIM
wire [KEY_WIDTH-1:0] lm_in_key, rm_in_key, pl_in_key;
wire [KEY_WIDTH-1:0] lm_out_key, rm_out_key, pl_out_key;
assign pl_in_key = pl_in_r[KEY_WIDTH-1:0];
assign lm_in_key = lm_in_r[KEY_WIDTH-1:0];
assign rm_in_key = rm_in_r[KEY_WIDTH-1:0];
assign pl_out_key = pl_out[KEY_WIDTH-1:0];
assign lm_out_key = lm_out[KEY_WIDTH-1:0];
assign rm_out_key = rm_out[KEY_WIDTH-1:0];
`endif	
	
endmodule
