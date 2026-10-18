module L2_cache #(
    parameter DATA_WIDTH = 16, //bites in data
    parameter ADDR_WIDTH = 16,// bites in address
    parameter LINE_SIZE = 4, //no. of words per line
    parameter CACHE_LINES = 128, //no of lines in cache
    parameter K_ways = 8 // no of lines per set #K-way
  ) (
    //signal from CPU
    input wire clk, // clock signal
    input wire rst_n, // reset signal
    output reg stall_cpu, // stall signal to freez the pipeline [wr_stall, rd_stall]

    //signal from L1 cache
    input wire mem_req_L1, // request for read from L1
    input wire [ADDR_WIDTH -1:0] addr_L1, //address of word write
    output reg [line_bites -1:0] data_out, //data output of word length
    output reg down_L1, // signal that opration is down [wr_ready, rd_ready]
    output reg hit_L1,// high on cache hit

    //signal from write-through buffer
    output reg pop_buffer,
    input wire [line_bites -1:0] data_buffer,
    input wire [block_addr_bites -1:0] addr_buffer,
    input wire [LINE_SIZE -1:0] mask_in,
    input wire empty_buffer,

    //signal to write-back victim
    input wire full_victim, // full signal from write buffer
    output reg push_victim, // push signal for write buffer
    output reg [block_addr_bites -1:0] addr_victim, // address to push in buffer
    output reg [line_bites -1:0] data_victim, // data to push in buffer

    output reg snoop_en, // snooping from victim
    output reg [block_addr_bites -1:0] snoop_addr,// address of data to be snooped
    input wire [line_bites -1:0] snoop_data, //snoop data from victim
    input wire snoop_hit,

    //signal for L3 cache
    output reg req_L3, //request of data from L2
    output reg [ADDR_WIDTH -1:0] addr_L3, // address of data requested
    input wire [line_bites -1:0] data_L3, //data allocated from L2
    input wire ready_L3 //acknowledge from L2
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
  localparam block_addr_bites = tag_bites + index_bites; //number of bites for block number

  // -------------------------------
  // Memory structures
  // -------------------------------
  reg [line_bites -1:0] cache [0:sets-1][0:K_ways-1]; // 4 lines of cache, each line is 8 bits (1 word)
  reg valid_bits [sets-1:0] [0:K_ways-1]; // valid bits for each line
  reg [tag_bites -1:0] tags [0:sets-1][0:K_ways-1]; // tags for each line
  reg [6:0] track_tree [0: sets-1]; // l0 l1 l2 l3 l4 l5 l6
  reg dirty_bits [sets-1:0][0:K_ways-1];

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

      down_L1 = 1'b0;
      hit_L1 = 1'b0;

      pop_buffer = 1'b0;

      push_victim = 1'b0;
      snoop_en = 1'b0;

      req_L3 = 1'b0;
    end
  endtask

  // -------------------------------
  // Task: search cache
  // -------------------------------
  //search for the hit line in cache, hit = 0 for hit and hit = 1 for miss
  task cache_search;
    input [tag_bites -1:0] search_tag;
    input [index_bites -1:0] set_index;
    output cache_hit;
    output [K_bites -1:0] set_way;
    integer i;
    begin
      //deafault values (no hit)
      set_way = {K_bites{1'b0}};
      cache_hit = 1'b0;

      //saerching for hit
      for (i = 0; i < K_ways; i = i + 1)
      begin
        if (valid_bits[set_index][i] && tags[set_index][i] == search_tag)
        begin : hit_condition
          set_way = i[K_bites-1:0]; // return line index
          cache_hit = 1'b1; // set hit flag
          track_tree[set_index] <= track_tree_update(set_index, i[1:0]);
        end
      end
    end
  endtask

  // -----------------------------------------------
  // Task: Pseudo LRU Replacement Policy
  // -----------------------------------------------
  //replace the line of cache with block from memory form L3
  task cache_replacement;
    input [index_bites -1:0] set_index;
    input [line_bites -1:0] data_from_L3;
    input [tag_bites -1:0] tag_address;

    integer i;
    reg allocated;
    integer victim_idx;

    begin
      //search for empty way in set (inValid line)
      for(i = 0; i < K_ways; i = i + 1)
      begin : search_empty
        if (!valid_bits[set_index][i])
        begin
          cache[set_index][i] = data_from_L3; // Update the cache line with the new data
          valid_bits[set_index][i] = 1'b1; // Set the valid bit for the line
          tags[set_index][i] = tag_address; // Store the tag
          track_tree[set_index] = track_tree_update(set_index, i[1:0]);//update the tree
          allocated = 1'b1;
        end
      end

      // If all lines are valid, replace the least recent way
      if(!allocated)
      begin : replace_line
        victim_idx = evict_way(set_index);

        cache[set_index][victim_idx] = data_from_L3; // Replace the evit way with the new data
        valid_bits[set_index][victim_idx] = 1'b1; // Set the valid bit for the replaced line
        tags[set_index][victim_idx] = tag_address; // Update the tag for the replaced line
        track_tree[set_index] = track_tree_update(set_index, victim_idx);//update the tree
      end
    end
  endtask

  // =================================================================
  // Function: True PLRU Eviction Decoder
  // Decodes the 7-bit tree starting form root node
  // to find which way is older than all others
  // based on binary search 1 for right and 0 for left
  // =================================================================
  function automatic [2:0] evict_way;
    input [index_bites -1:0] set_index;

    reg [6:0] plru_bits;
    reg evict_0, evict_1, evict_2;
    begin
      plru_bits = track_tree[set_index];
      evict_0 = plru_bits[0];

      if (!evict_0)
      begin
        evict_1 = plru_bits[2];
        evict_2 = evict_1 ? plru_bits[6] : plru_bits[5];
      end
      else
      begin
        evict_1 = plru_bits[1];
        evict_2 = evict_1 ? plru_bits[4] : plru_bits[3];
      end

      evict_way = {evict_0, evict_1, evict_2};
    end
  endfunction

  // =================================================================
  // Function: Tree-Based Pseudo-LRU
  // Bit Map Layout: [L0, L1, L2, L3, L4, L5, L6]
  // Rule: Every time a way is accessed the tracking bits must be
  //updated to point away from that way inverse them from root to leaf.
  // =================================================================
  function [6:0] track_tree_update;
    input [index_bites -1:0] set_index;
    input [K_bites -1:0] set_way ;
    begin
      track_tree_update = track_tree[set_index];
      case (set_way)
        3'b000:
        begin
          track_tree_update[0] = 1'b1;
          track_tree_update[1] = 1'b1;
          track_tree_update[3] = 1'b1;
        end
        3'b001:
        begin
          track_tree_update[0] = 1'b1;
          track_tree_update[1] = 1'b1;
          track_tree_update[3] = 1'b0;
        end
        3'b010:
        begin
          track_tree_update[0] = 1'b1;
          track_tree_update[1] = 1'b0;
          track_tree_update[4] = 1'b1;
        end
        3'b011:
        begin
          track_tree_update[0] = 1'b1;
          track_tree_update[1] = 1'b0;
          track_tree_update[4] = 1'b0;
        end
        3'b100:
        begin
          track_tree_update[0] = 1'b0;
          track_tree_update[2] = 1'b1;
          track_tree_update[5] = 1'b1;
        end
        3'b101:
        begin
          track_tree_update[0] = 1'b0;
          track_tree_update[2] = 1'b1;
          track_tree_update[5] = 1'b0;
        end
        3'b110:
        begin
          track_tree_update[0] = 1'b0;
          track_tree_update[2] = 1'b0;
          track_tree_update[6] = 1'b1;
        end
        3'b111:
        begin
          track_tree_update[0] = 1'b0;
          track_tree_update[2] = 1'b0;
          track_tree_update[6] = 1'b0;
        end
      endcase
    end
  endfunction

  // =================================================================
  // Function: for finding id evicted way is dirty
  // =================================================================
  function dirty_evict;
    input [index_bites -1:0] set_index;
    reg [2:0] evicted_way;
    reg allocated;
    integer i;
    begin
      dirty_evict = 1'b0;

      for(i = 0; i < K_ways; i = i + 1)
        if(valid_bits[set_index][i] == 1'b0)
          allocated = 1'b1;

      if(!allocated)
      begin
        evicted_way = evict_way(set_index);
        dirty_evict = dirty_bits[set_index][evicted_way];
      end

    end
  endfunction

  // ------------------------------------------------------------------
  // TASK: writing the vaild masked word from write-buffer
  // ------------------------------------------------------------------
  task write_from_buffer;
    input [line_bites -1:0] data_from_buffer;
    input [LINE_SIZE -1:0] vaild_mask;
    input [index_bites -1:0] set_index;
    input [K_bites -1:0] set_way;

    reg [line_bites -1:0] r1;
    reg [line_bites -1:0] r2;
    reg [line_bites -1:0] r3;
    integer i;
    begin
      for (i= LINE_SIZE-1; i>=0; i=i-1)
        r1 = (r1<<DATA_WIDTH) + {DATA_WIDTH{vaild_mask[i]}};

      r2 = data_from_buffer & r1;
      r3 = cache[set_index][set_way] & ~r1;
      cache[set_index][set_way] = r2 | r3;
    end
  endtask

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++
  // FSM : l2 cache controller state and helper variable
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++
  localparam IDLE_STATE = 3'b000,
             LOOKUP_STATE = 3'b001,
             WRITE_STATE = 3'b010,
             ALLOCATE_STATE = 3'b011;

  reg [1:0] current_state, next_state;

  reg [tag_bites -1:0] tag_L1;
  reg [index_bites -1:0] index_L1;
  reg [offset_bites -1:0] offset_L1;
  reg [K_bites-1:0] way_L1; // way in a perticular set
  reg Is_dirty; // the evict way is dirty

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++
  // FSM : state transition and reset logic
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++
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

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // FSM : combionational next state and output signal logic
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  always @(*)
  begin
    //deafault values
    next_state = current_state;
    stall_cpu  = 1'b0;
    down_L1  = 1'b0;

    pop_buffer = 1'b0;
    push_victim = 1'b0;

    req_L3     = 1'b0;
    addr_L3    = addr_L1;

    //snooping the Victim buffer in background
    snoop_en = (current_state == LOOKUP_STATE) && mem_req_L1;
    snoop_addr = addr_L1;

    case(current_state)
      IDLE_STATE :
        next_state = (mem_req_L1 && !empty_buffer) ? LOOKUP_STATE : IDLE_STATE;


      LOOKUP_STATE :
      begin
        if(mem_req_L1)
        begin :read_opration
          address_breakdown(addr_L1, tag_L1, index_L1, offset_L1);
          cache_search(tag_L1, index_L1, hit_L1, way_L1);

          //read form L2 cache
          if(hit_L1)
          begin :read_hit
            data_out <= cache[index_L1][way_L1];
            down_L1 = 1'b1;
          end

          // read from victim buffer
          else if(snoop_hit)
          begin :victim_hit
            data_out <= snoop_data;
            down_L1 = 1'b1;
          end

          //read miss stall the cpu and req_data from L3
          else
          begin :read_miss
            stall_cpu = 1'b1; //read miss
            Is_dirty = dirty_evict(index_L1);
          end

          next_state = (hit_L1 || snoop_hit) ? IDLE_STATE :
            Is_dirty ? WRITE_STATE : ALLOCATE_STATE;
        end

        else
        begin : write_opration
          //write buffer have slots to read
          if(!empty_buffer)
          begin
            //dequeue signal to buffer
            pop_buffer = 1'b1;
            address_breakdown({addr_buffer, 2'b00}, tag_L1, index_L1, offset_L1);
            cache_search(tag_L1, index_L1, hit_L1, way_L1);

            //write to the existing line in cache
            if(hit_L1)
            begin :write_hit
              write_from_buffer(addr_buffer, mask_in, index_L1, way_L1);
              dirty_bits[index_L1][way_L1] <= 1'b1;
            end

            //stall the cpu and follow the write-allotmate policy on write miss
            else
            begin :write_miss
              stall_cpu = 1'b1; //write miss
              Is_dirty = dirty_evict(index_L1); //checkin the evicted line is dirty or not
            end

            next_state = hit_L1 ? IDLE_STATE : Is_dirty ? WRITE_STATE : ALLOCATE_STATE;
          end

          // nothing to write in L2 cache from write buffer
          else
            next_state =  IDLE_STATE;

        end
      end


      WRITE_STATE :
      begin
        //victim write-back buffer have space to write
        if(!full_victim)
        begin:push_data
          push_victim = 1'b1;
          addr_victim  <= {tag_L1, index_L1};
          data_victim  <=  cache[index_L1][evict_way(index_L1)];
        end

        //no free sloat in victim stall the cpu to empty the victim first
        else
        begin
          stall_cpu = 1'b1; // victim full;
        end

        next_state = full_victim ? WRITE_STATE : ALLOCATE_STATE;
      end


      ALLOCATE_STATE :
      begin
        req_L3 = 1'b1;

        if(ready_L3)
        begin : replacement
          //replace the evict line with new data 
          cache_replacement(index_L1, data_L3, tag_L1);

          //pass the data to L1 cache
          if(mem_req_L1) begin
          data_out <= data_L3;
          down_L1 = 1'b1;
          end
        end

        next_state = ready_L3 ? IDLE_STATE : ALLOCATE_STATE;
      end


      default :
        next_state = IDLE_STATE;

    endcase
  end
endmodule