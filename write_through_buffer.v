module write_buffer (
    input wire clk,
    input wire rst_n,

    //write signals
    input wire wr,
    input wire [7:0] data_in,
    input wire [3:0] addr_in,
    output wire full,

    //read signals
    input wire rd,
    output wire [7:0] data_out,
    output wire [3:0] addr_out,
    output wire empty,

    //snooping signals
    input wire snoop_en,
    input wire snoop_addr,
    output wire snoop_data,
    output reg snoop_hit
  );

  //declaration of memory array and pointers
  reg [7:0] data_array [0:15];
  reg [3:0] addr_array [0:15];
  reg [4:0] rd_ptr, wr_ptr;

  //full and empty flags condition
  assign full = ((rd_ptr ^ wr_ptr) == 5'b1_0000 ); // same but msb are different
  assign empty = (rd_ptr == wr_ptr); // exact same

  // reading from FIFO as read is combinational
  assign data_out = data_array[rd_ptr[3:0]];
  assign addr_out = addr_array[rd_ptr[3:0]];

  reg rewrite_hit; // high when collap slot hit
  reg [3:0] rewrite_ptr; //points to that hit slot
  reg [3:0] snoop_ptr; //points to the snoop slot

  //task that check the repeated word
  task automatic double_write;
    input [3:0] addr;
    output found_hit;
    output [3:0] found_ptr;
    reg [4:0] current_ptr;

    begin
      found_hit = 1'b0;
      found_ptr = 4'h0;
      current_ptr = rd_ptr;

      while(current_ptr != wr_ptr)
      begin
        if(addr_array[current_ptr[3:0]] == addr)
        begin
          found_hit = 1'b1;
          found_ptr = current_ptr[3:0];
        end
        
          current_ptr = current_ptr + 1;
    end
  end
  endtask

  // write forwarding the data 
  always @(*)
  begin :snooping
    if(!empty && snoop_en)
    begin
      double_write (snoop_addr, snoop_hit, snoop_ptr);
    end
  end

  assign snoop_data = (snoop_hit && snoop_en) ? data_array[snoop_ptr] : 8'h00;

  //FIFO and pointer updatetion logic
  always @(posedge clk)
  begin

    //reset the pointers
    if(!rst_n)
    begin : reset
      rd_ptr <= 5'b00000;
      wr_ptr <= 5'b00000;
    end

    else
    begin
      // Enqueue execution check
      if(wr && !full)
      begin : enqueue
        //write merging 
        double_write(addr_in, rewrite_hit, rewrite_ptr);

        if(rewrite_hit) begin
          data_array[rewrite_ptr] <= data_in;
        end
        else begin
        data_array[wr_ptr[3:0]] <= data_in;
        addr_array[wr_ptr[3:0]] <= data_in;
        wr_ptr <= wr_ptr + 1'b1;
        end
      end

      // Dequeue execution check
      if(rd && !empty)
      begin : dequeue
        rd_ptr <= rd_ptr + 1'b1;
      end

    end
  end
endmodule
