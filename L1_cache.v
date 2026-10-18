//4-ways set assosiative mapping
//matrix least reacent used replacement
//write-through buffer for L2 write
//no-write allotmate when write miss
//3 state FSM controller idle -> lookup -> allocate

module L1_cache #(
    parameter DATA_WIDTH = 16, //bites in data
    parameter ADDR_WIDTH = 16,// bites in address
    parameter LINE_SIZE = 4, //no. of words per line
    parameter CACHE_LINES = 32, //no of lines in cache
    parameter K_ways = 4 // no of lines per set #K-way
  ) (
    input wire clk, // clock signal
    input wire rst_n, // reset signal
    input wire mem_req_cpu, // request for read/write from cpu
    output reg stall_cpu, // stall signal to freez the pipeline [wr_stall, rd_stall]
    output reg ready_cpu, // signalthat opration is down [wr_ready, rd_ready]

    input wire wr_cpu, // high for write and low for read
    input wire [ADDR_WIDTH -1:0] addr, //address of word write
    input wire [DATA_WIDTH -1:0] data_in, //data input of word length
    output reg [DATA_WIDTH -1:0] data_out, //data output of word length
    output reg hit,// high on cache hit

    input wire full_FIFO, // full signal from write buffer
    output reg push_FIFO, // push signal for write buffer
    output reg [ADDR_WIDTH -1:0] addr_FIFO, // address to push in buffer
    output reg [DATA_WIDTH -1:0] data_FIFO, // data to push in buffer

    output reg snoop_en, // snoopin from FIFO
    output reg [ADDR_WIDTH -1:0] snoop_addr,// address of data to be snooped
    input wire [DATA_WIDTH -1:0] snoop_data, //snoop data from FIFO
    input wire snoop_hit,

    input wire ready_L2, //acknowledge from L2
    output reg req_L2, //request of data from L2
    output reg [ADDR_WIDTH -1:0] addr_L2, // address of data requested
    input wire [line_bites -1:0] data_L2 //data allocated from L2
  );

  // -------------------------------
  // Derived constants
  // -------------------------------
  // Calculate the number of bits for various components based on the parameters
  localparam line_bites = LINE_SIZE * DATA_WIDTH; // Number of bits in a line
  localparam K_bites = $clog2(K_ways); // no of bites for line in a set
  localparam sets = CACHE_LINES / K_ways; // number of set in cache

  // Calculate the number of bits for the tag, index and offset based on the address breakdown
  localparam offset_bites = $clog2(LINE_SIZE); // Number of bits for the word offset within a block
  localparam index_bites = $clog2(sets); //number of bites for set index
  localparam tag_bites = ADDR_WIDTH - offset_bites - index_bites; // Number of bits for the tag

  // -------------------------------
  // Memory structures
  // -------------------------------
  reg [line_bites -1:0] cache [0:sets-1][0:K_ways-1]; // 4 lines of cache, each line is 8 bits (1 word)
  reg [sets-1:0] valid_bits [0:K_ways-1]; // valid bits for each line
  reg [tag_bites -1:0] tags [0:sets-1][0:K_ways-1]; // tags for each line
  reg [5:0] track[0: sets-1]; // B01 B02 B03 B12 B13 B23


  // -------------------------------
  // Task: Address breakdown
  // -------------------------------
  task address_breakdown;
    input [ADDR_WIDTH -1:0] address;
    output [tag_bites -1:0] tag;
    output [index_bites -1:0] index;
    output [offset_bites -1:0] offset;

    begin
      tag = address[ADDR_WIDTH-1 -: tag_bites];
      index = address[offset_bites +: index_bites];
      offset = address[offset_bites -1:0];
    end
  endtask

  // -------------------------------
  // Task: reset cache
  // -------------------------------
  // reset the cache, tags, valid array all to zero
  task cache_reset;
    integer i, j;
    begin
      //reseting the array
      for (i = 0; i < sets; i = i + 1)
      begin
        for (j = 0; j < K_ways; j = j + 1)
        begin
          cache[i][j] = {line_bites{1'b0}}; // clear cache line
          valid_bits[i][j] = 1'b0; // clear valid bit
          tags[i][j] = {tag_bites{1'b0}}; // clear tag
        end
      end

      //reset the outputs pin to ZERO
      stall_cpu = 1'b0;
      ready_cpu = 1'b0;
      hit = 1'b0;
      push_FIFO = 1'b0;
      snoop_en = 1'b0;
      req_L2 = 1'b0;
    end
  endtask

  // -------------------------------
  // Task: search cache
  // -------------------------------
  //search for the hit line in cache hit = 0 for hit and hit = 1 for miss
  task cache_search;
    input [tag_bites -1:0] tag;
    input [index_bites -1:0] set_index;
    output hit;
    output [K_bites -1:0] line_in_set;
    integer i;
    begin
      //deafault values (no hit)
      line_in_set = {K_bites{1'b0}};
      hit = 1'b0;

      //saerching for hit
      for (i = 0; i < K_ways; i = i + 1)
      begin :for_loop
        if (valid_bits[set_index][i] && tags[set_index][i] == tag)
        begin : hit_condition
          line_in_set = i[K_bites-1:0]; // return line index
          hit = 1'b1; // set hit flag
          track[set_index] <= track_update(set_index, i[1:0]);
        end
      end
    end
  endtask


  // -----------------------------------------------
  // Task: True Matrix LRU Replacement Policy
  // -----------------------------------------------
  //replace the line of cache with block from memory
  task cache_replacement;
    input [index_bites -1:0] set_index;
    input [DATA_WIDTH -1:0] data_from_L2;
    input [tag_bites -1:0] tag_address;

    integer i;
    reg allocated;
    integer victim_idx;

    begin
      //search for empty line in set (inValid line)
      for(i = 0; i < 4; i = i + 1)
      begin : search_empty
        if (!valid_bits[set_index][i])
        begin
          cache[set_index][i] = data_from_L2; // Update the cache line with the new data
          valid_bits[set_index][i] = 1'b1; // Set the valid bit for the line
          tags[set_index][i] = tag_address; // Store the tag
          track[set_index] = track_update(set_index, i[1:0]);
          allocated = 1'b1;
        end
      end

      // If all lines are valid, replace the first line (simple replacement policy)
      if(!allocated)
      begin : replace_line
        victim_idx = evict_line(set_index);

        cache[set_index][victim_idx] = data_from_L2; // Replace the first line with the new data
        valid_bits[set_index][victim_idx] = 1'b1; // Set the valid bit for the replaced line
        tags[set_index][victim_idx] = tag_address; // Update the tag for the replaced line
        track[set_index] = track_update(set_index, victim_idx);
      end
    end
  endtask

  // =================================================================
  // Function: Perfect Matrix Age Update
  // Bit Map Layout: [ B01, B02, B03, B12, B13, B23 ]
  // Rule: Clear old relationships, then assert new access dominance
  // =================================================================
  function [5:0] track_update;
    input [index_bites -1:0] set_index;
    input[K_bites -1:0] line_in_set;
    begin
      case(line_in_set)
        2'b00 :
          track_update = (track[set_index] & 6'b111111) | 6'b111000;
        2'b01 :
          track_update = (track[set_index] & 6'b011111 ) | 6'b000110;
        2'b10 :
          track_update = (track[set_index] & 6'b101011 ) | 6'b000001;
        2'b11 :
          track_update = (track[set_index] & 6'b110100 ) | 6'b000000;
        default  :
          track_update = (track[set_index] & 6'b111111 ) | 6'b000000;
      endcase
    end
  endfunction

  // =================================================================
  // Function: matrix LRU Eviction Decoder
  // Decodes the 6-bit grid to find which way is older than all others
  // =================================================================
  function [K_bites -1:0] evict_line;
    input [index_bites -1:0] set_index;
    begin
      casez(track[set_index])
        6'b000??? :
          evict_line = 2'b00;
        6'b1??00? :
          evict_line = 2'b01;
        6'b?1?1?0 :
          evict_line = 2'b10;
        6'b??1?11 :
          evict_line = 2'b11;
        default :
          evict_line = 2'b00;
      endcase
    end
  endfunction

  // -----------------------------------------------
  // FSM : l1 cache controller
  // -----------------------------------------------
  localparam IDLE_STATE = 2'b00,
             LOOKUP_STATE = 2'b01,
             ALLOCATE_STATE = 2'b10;

  reg [1:0] current_state, next_state;

  reg [tag_bites -1:0] tag_r;
  reg [index_bites -1:0] index_r;
  reg [offset_bites -1:0] offset_r;
  reg [K_bites-1:0] line_r; // line in a perticular set

  // -------------------------------------------
  // FSM : state transition and reset logic 
  // -------------------------------------------
  always @(posedge clk or negedge rst_n)
  begin : state_transition_logic
    if(!rst_n)
    begin
      current_state <= IDLE_STATE;
      cache_reset();
    end
    else
    begin
      current_state <= next_state;
    end
  end

  // ---------------------------------------------------------
  // FSM : sequential logic of read and write opration 
  // ---------------------------------------------------------
  always @(posedge clk)
  begin
    case(current_state)

      LOOKUP_STATE :
      begin
        if(wr_cpu && !full_FIFO)
        begin : write_opration
          address_breakdown(addr, tag_r, index_r, offset_r);
          cache_search(tag_r, index_r, hit, line_r);

          if(hit)
            cache[index_r][line_r][offset_r * DATA_WIDTH +: DATA_WIDTH] <= data_in;

          //NO-write-allotmate write to L2 always and no allotmate on write miss
          push_FIFO <= 1'b1;
          addr_FIFO <= addr;
          data_FIFO <= data_in;
        end

        else
        begin :read_opration
          address_breakdown(addr, tag_r, index_r, offset_r);
          cache_search(tag_r, index_r, hit, line_r);

          // data from cache
          if(hit)
          begin
            data_out <= cache[index_r][line_r][offset_r * DATA_WIDTH +: DATA_WIDTH];
          end
          // data from buffer
          else if (snoop_hit)
          begin
            data_out <= snoop_data;
          end
        end
      end

      ALLOCATE_STATE :
      begin : data_replace
        if(ready_L2)
        begin
          cache_replacement(index_r, data_L2, tag_r);

          data_out <= data_L2[offset_r * DATA_WIDTH +: DATA_WIDTH];
        end
      end

    endcase
  end

  // ------------------------------------------------------------
  // FSM : combinational logic of output signal and next state 
  // ------------------------------------------------------------
  always @(*)
  begin
    next_state = current_state;
    stall_cpu  = 1'b0;
    ready_cpu  = 1'b0;

    req_L2     = 1'b0;
    addr_L2    = addr;

    snoop_en = (current_state == LOOKUP_STATE) && !wr_cpu;
    snoop_addr = addr;

    case (current_state)
      IDLE_STATE :
        next_state = mem_req_cpu ? LOOKUP_STATE : IDLE_STATE;

      LOOKUP_STATE :
      begin
        if(wr_cpu)
        begin
          if (full_FIFO)
          begin
            stall_cpu  = 1'b1;
            next_state = LOOKUP_STATE;
          end

          else
          begin
            ready_cpu  = 1'b1;
            next_state = mem_req_cpu ? LOOKUP_STATE : IDLE_STATE;
          end
        end

        else
        begin
          if (hit || snoop_hit)
          begin
            ready_cpu  = 1'b1;
            next_state = mem_req_cpu ? LOOKUP_STATE : IDLE_STATE;
          end

          else
          begin
            stall_cpu  = 1'b1;
            next_state = ALLOCATE_STATE;
          end
        end
      end

      ALLOCATE_STATE :
      begin : stalling_cpu
        stall_cpu = 1'b1;
        req_L2    = 1'b1;
        addr_L2   = addr;

        if (ready_L2)
        begin
          stall_cpu  = 1'b0;
          ready_cpu  = 1'b1;
          next_state = mem_req_cpu ? LOOKUP_STATE : IDLE_STATE;
        end

        else
        begin
          next_state = ALLOCATE_STATE;
        end
      end

      default :
        next_state = IDLE_STATE;
    endcase
  end
endmodule
