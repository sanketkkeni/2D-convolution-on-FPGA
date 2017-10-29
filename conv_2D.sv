module conv_2D(s_ready, clk, reset, done, data_in, data_out, s_valid, m_ready, s_last, m_last);
	parameter X=5, H=3;
	input clk, reset, s_valid, m_ready, s_last;
	
	output logic done, s_ready, m_last;
	input [7:0] data_in;
	output logic [31:0] data_out;	
	logic wr_en_h ,wr_en_x, wr_en_y,clear_acc;
	logic [13:0] addr_x;
	logic [13:0] addr_y;
	logic [7:0] addr_h;
	logic done1;
	logic [3:0] state;
	logic [15:0] data_out1,f;	
	logic [7:0] data_in1;
	logic [7:0] data_out_x;
		
	always_comb begin
				
		if (s_ready && s_valid) data_in1 = data_in;//********//
			else data_in1 =0;
	
	end
	
	
	assign data_out[15:0] = data_out1[15:0];
	assign data_out[31:16] = 0;

	
	
// Instantation of Data and Conntrol Path 
	datapath d(clk, data_in1,addr_x,wr_en_x,addr_h,wr_en_h,addr_y,wr_en_y,clear_acc,data_out1,f, data_out_x, m_ready);
	ctrlpath #(5, 3) c(clk, m_last, reset, addr_x, wr_en_x, addr_h, wr_en_h, clear_acc, addr_y, wr_en_y, done, s_valid, state, m_ready, s_ready);

endmodule

module memory(clk, data_in, data_out, addr, wr_en, m_ready);
	parameter WIDTH=16, SIZE=64, LOGSIZE=6;
	input [WIDTH-1:0] data_in;
	output logic [WIDTH-1:0] data_out;
	input [LOGSIZE-1:0] addr;
	input clk, wr_en, m_ready;
	logic [WIDTH-1:0] mem [SIZE-1:0];
		always_ff @(posedge clk) begin
			if(m_ready)data_out <= mem[addr];
			else data_out <= data_out;
			
			if (wr_en)
			mem[addr] <= data_in;
		end
endmodule

module datapath(clk, data_in,addr_x,wr_en_x,addr_h,wr_en_h,addr_y,wr_en_y,clear_acc,data_out,f, data_out_x, m_ready);
	input clk, m_ready;
	input logic clear_acc , wr_en_h , wr_en_x, wr_en_y;
	input [7:0] data_in;
	input logic[7:0] addr_h;
	input logic[13:0] addr_x;
	input logic[13:0] addr_y;
	output logic[15:0] data_out, f;
	logic [15:0] mul_out,add_r;
	logic [7:0] data_out_h;
	output logic [7:0] data_out_x;
	/*Memory Instantion*/
	memory #(8, 16384, 14) mem_x(clk, data_in, data_out_x, addr_x, wr_en_x, m_ready); // memory Instantaion for x column vector and has k memory location each having bit word length of 8 bits.
	memory #(8, 256, 8) mem_h(clk, data_in, data_out_h, addr_h, wr_en_h, m_ready); // memory Instantation k*k matrix and has k*k memory location each having bit word length of 8 bits.
	memory #(16, 16384, 14) mem_y(clk, f, data_out, addr_y, wr_en_y, m_ready);  // memory instantation of y column vector and has k memory location each having bit word length of 16 bits.
	// Multiply and Accumulate Block
	always_ff @ (posedge clk) begin

		if(clear_acc == 1) begin
			f <= 0;
			end
		else begin
			f <= add_r;
			end
	end
	always_comb begin
		mul_out = data_out_h * data_out_x;
		add_r = f + mul_out;
	end

endmodule

module ctrlpath(clk, last, reset, addr_x, wr_en_x, addr_h, wr_en_h, clear_acc, addr_y, wr_en_y,done, s_valid, state,m_ready, s_ready);
		parameter X=10, H=3;
		input clk, reset, s_valid, m_ready;
		output logic [7:0] addr_h;
		logic load_h, load_x, start;
		output logic [13:0] addr_x;
		logic [13:0] addr_xjump;
		output logic [13:0] addr_y;
		output logic wr_en_x,wr_en_h,clear_acc,wr_en_y, last;
		output logic done, s_ready;
		logic [3:0] next_state;
		output logic [3:0] state;
		logic  state1_done, state3_isone, state2_done, state3_jump, state3_donefinal, state5_done, state53_done;

		always @(posedge clk) begin
			if (reset == 1)    state <= 0; 
            else begin        
                       state <= next_state;
                   
			end
		end

		always @(posedge clk) begin
		if (reset) done <=0;
		else begin
			if (state==6 || state==7)
				done<=1;
			else
				done<=0;
		end
		end

		always @(posedge clk) begin
		if (reset) addr_h <= 0;
		else begin 
			if (state3_donefinal == 0 )
				addr_h <= addr_h+1;
			else if (state3_isone==1 && state!=5)
				addr_h <= addr_h;
			else if (state1_done == 0)
				addr_h <= addr_h-1;
			else if (load_h==1)
				addr_h <= (H*H)-1;
			else addr_h <= 0;	
			
		end	
		end

		always @(posedge clk) begin
		if (reset || state==0) addr_x <=0;
		else begin//********//
		if (s_valid) begin //Sometimes when s_valid==0, count is same but addr_x increases. So to keep it same we do this//********//
			if (((((state2_done == 0) && (state==2) ) || state3_donefinal == 0) && state3_jump!=1) || (state==5 && H==1))
				addr_x <= addr_x+1;
					
			else if (H==1 && (state==3 || state==4)) addr_x <= addr_x;
				
			else if (state3_jump==1)
				addr_x <= addr_x+X-H+1;
			
			else 
				addr_x <=  addr_xjump;
		end
		else addr_x <=  addr_x;
		end
		end
		
		always @(posedge clk) begin
		if (reset || state==0)	addr_xjump <=0;
		else begin
			if (state3_jump==1 && state53_done!=1 && addr_h<(H+1))
				addr_xjump <= addr_xjump+1;
			
			else if (state3_jump==1 && state53_done==1 && addr_h<(H+1))
				addr_xjump <= addr_xjump+H;
			
			
		end		
		
		end
		


		always @(posedge clk) begin
		if (reset) begin 
				addr_y <= 0; end
		
			else begin
				if (((state==5)&& (state5_done!=1)))
				addr_y <= addr_y+1;
				else if ((state==7 && next_state==7) || state==6)	//if m_ready==0, then output is not shown. Hence the addr_y should be maintained
					begin
						if (m_ready)	addr_y <= addr_y+1;//********//
						else addr_y <= addr_y;				
					end
				else if (next_state==0 || state5_done==1)
					addr_y <= 0;
				else
					addr_y <= addr_y;
		
				end
		end
		

		
		always @(posedge clk) begin
		if (reset) clear_acc <=0;
		else begin
			if (state==5 || state==2 || state==9 )
				clear_acc <= 1;
			else
				clear_acc <= 0;
		end
		end
		
		always @(posedge clk) begin
		
			if (state==7 && addr_y == ((X-H+1)*(X-H+1)-1))	last =1;
			else last=0;
					
		end

		always_comb begin
			if((state==1 || state==2) && s_valid)	s_ready=1;
			else s_ready=0;
		end
		
		
		always_comb begin state5_done=1'b0; load_h=1'b0; start=1'b0; load_x=1'b0; state1_done=1'b1; state53_done=0; state2_done =1'b0; state3_jump =1'b0;  state3_donefinal=1'b1; state3_isone=0;state5_done=1'b0;
		/*Beginning State*/
			if (state == 0) begin
				if (s_valid==1)begin
					next_state = 1; load_h=1;
				end
				else begin
					next_state = 0; load_h=0;
				end
			end

		/*Writing in Memory A(Matrix Storage)*/
			else if (state == 1) begin
				if (addr_h<=(H*H)-1) begin/////////
					next_state = 1;
					state1_done = 0;
				end
				else begin
					next_state = 2;
					state1_done = 1;
					load_x=1;
				end
			end

			/*Writing in Memory x(Vector Storage)*/
			else if (state == 2) begin
				if (addr_x<(X*X)) begin//////////////
					next_state = 2;
					state2_done = 0;
				end
				else begin
					next_state = 3; start=1;
					state2_done = 1;
				end;
			end

			else if (state == 9) begin
				if (start==1)
					next_state=3;
				else if (load_h == 1)
					next_state=1;
				else if (load_x == 1)
					next_state=2;
				else
					next_state=9;
			end

			/*Multiply and Accumulate stage -- > This works along with Data path and generates output*/
			else if (state == 3) begin
							
					if (addr_h<(H*H)-1) begin
					next_state = 3;
					state3_donefinal=0;
					state2_done=0;
					
						if ((addr_h+1)%H==0)
							state3_jump=1;
						else state3_jump=0;
						
						if (((addr_y+1)%(X-H+1))==0) 
							state53_done=1;
						else state53_done=0;
					
					end				
					else begin
					next_state = 4;
					state3_isone=1;
					end
				
			end

			/*Enable writing in Memory Y and Clearing accumulator For next MAC Operation*/
			else if (state==4) begin
				next_state=5;
				state3_isone=1;
			end

			/*Writing in Memory Y ( Output Vector Storage)*/
			else if (state==5) begin
				state3_isone=1;
				if (addr_y<((X-H+1)*(X-H+1)-1)) begin
					
					next_state=3;
					state5_done = 0;
				end
				else begin
					next_state=6;
					
					state5_done =1;
				end
			end

			else if(state==6) begin
				next_state=7; 
			end

			/*Outputting Data Storage Stored in Memory Y*/
			else if (state==7) begin
				if (addr_y < ((X-H+1)*(X-H+1)-1))
					next_state = 7;
				else
					next_state=0;
				end
				else next_state=8;
			end

		assign wr_en_h = (state==1 && reset==0);
		
		assign wr_en_x = (state==2 && reset==0);

		assign wr_en_y = (state==5 && reset==0);

endmodule
