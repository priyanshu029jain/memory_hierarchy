module write_buffer (
    input wire clk,
    input wire rst_n,

    //write signals
    input wire wr,
    input wire [15:0] data_in,
    input wire [15:0]addr_in,
    output wire full,

    //read signals
    input wire rd,
    output wire [63:0]data_out,
    output wire [13:0] addr_out,
    output wire [3:0] mask_out,
    output wire empty,

    //snooping signals
    input wire snoop_en,
    input wire [15:0]snoop_addr,
    output wire [15:0] snoop_data,
    output reg snoop_hit
  );

  //declaration of memory array and pointers
  reg [63:0] data_array [0:15]; //whole block
  reg [13:0] addr_array [0:15]; // block address
  reg [3:0] vaild_words [0:15];
  reg [4:0] rd_ptr, wr_ptr;

  //full and empty flags condition
  assign full = ((rd_ptr ^ wr_ptr) == 5'b1_0000 ); // same but msb are different
  assign empty = (rd_ptr == wr_ptr); // exact same

  // reading from FIFO as read is combinational
  assign data_out = data_array[rd_ptr[3:0]];
  assign addr_out = addr_array[rd_ptr[3:0]];
  assign mask_out = (!empty) ? vaild_words[rd_ptr[3:0]] : 4'h0;

  reg rewrite_hit; // high when collap slot hit
  reg [3:0] rewrite_ptr; //points to that hit slot
  reg [3:0] snoop_ptr; //points to the snoop slot

  //task that check the repeated word
  task automatic double_write;
    input [13:0] addr;
    output found_hit;
    output [3:0] found_ptr;

    reg [4:0] current_ptr;
    integer i;

    begin
      found_hit = 1'b0;
      found_ptr = 4'h0;
      current_ptr = rd_ptr;

      for (i = 0; i < 16; i = i + 1)
      begin
        if (current_ptr != wr_ptr)
        begin
          if ((addr_array[current_ptr[3:0]] == addr[15:2]) && (vaild_words[current_ptr[3:0]] != 4'b0000))
          begin
            found_hit = 1'b1;
            found_ptr = current_ptr[3:0];
          end
          current_ptr = current_ptr + 1'b1;
        end
      end
    end
  endtask

  // write forwarding the data
  always @(*)
  begin :snooping
    snoop_hit = 1'b0;
    snoop_ptr = 4'h0;
    if(!empty && snoop_en)
    begin
      double_write (snoop_addr[15:2], snoop_hit, snoop_ptr);
    end
  end

  wire [1:0] snoop_offset = snoop_addr[1:0];
  assign snoop_data = (snoop_hit && snoop_en && vaild_words[snoop_ptr][snoop_offset]) ? data_array[snoop_ptr][16*snoop_offset +: 16] : 16'h0000;

  //FIFO and pointer updatetion logic
  always @(posedge clk or negedge rst_n)
  begin

    //reset the pointers
    if(!rst_n)
    begin : reset
      integer i;
      rd_ptr <= 5'b00000;
      wr_ptr <= 5'b00000;
      for (i = 0; i < 16; i = i + 1)
      begin
        vaild_words[i] <= 4'b0000;
      end
    end

    else
    begin
      // Enqueue execution check
      if(wr && !full)
      begin : enqueue
        //write merging
        double_write(addr_in, rewrite_hit, rewrite_ptr);

        //only update the targeted word
        if(rewrite_hit)
        begin
          data_array[rewrite_ptr][16 * addr_in[1:0] +: 16] <= data_in;
          vaild_words[rewrite_ptr][addr_in[1:0]] <= 1'b1;
        end

        //push the whole block
        else
        begin
          data_array[wr_ptr[3:0]][16 * addr_in[1:0] +: 16] <= data_in;
          addr_array[wr_ptr[3:0]] <= addr_in[15:2];
          vaild_words[wr_ptr[3:0]] <= 4'b0000;
          vaild_words[wr_ptr[3:0]][addr_in[1:0]] <= 1'b1;
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
