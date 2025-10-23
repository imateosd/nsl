library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_amba, nsl_i2c, nsl_data, nsl_simulation;
use nsl_i2c.cbor_transactor.all;
use nsl_i2c.master.all;
use nsl_i2c.i2c."+";
use nsl_data.cbor.all;
-- use nsl_data.bytestream.all;

entity controller is
    generic(
        system_clock_c : natural;
        axi_s_cfg_c    : nsl_amba.axi4_stream.config_t
    );
    port(
        clock_i     : in std_ulogic;
        reset_n_i   : in std_ulogic;

        i2c_o       : out nsl_i2c.i2c.i2c_o;
        i2c_i       : in  nsl_i2c.i2c.i2c_i;

        cmd_i       : in nsl_amba.axi4_stream.master_t;
        cmd_o       : out nsl_amba.axi4_stream.slave_t;
        rsp_o       : out nsl_amba.axi4_stream.master_t;
        rsp_i       : in nsl_amba.axi4_stream.slave_t
    );
end entity;

architecture beh of controller is

    type state_t is (
        ST_RESET,

        ST_ARRAY_GET,             -- get first item (type and ai). should be an array header
        ST_ARRAY_ENTER,           -- store data if needed, reset parser and go to next
        
        ST_CMD_GET,               -- get command item (type and complete ai)
        ST_CMD_EXEC,              -- store data if needed, reset parser and go to next
                                  -- if command item is of array kind, go to ST_ADDR_GET
                                  -- if command item is a simple value (null) go to ST_STOP
        ST_CMD_END,               -- will get here after the command has been executed. 
                                  -- if the number of commands is completed, go to
                                  -- ARRAY_GET_FIRST (via RSP_BREAK_PUT), otherwise go to CMD_GET_FIRST
                                  -- if the number of commands is completer, send break for indefine array 

        ST_ADDR_GET,              -- get address (type and complete ai)
        ST_ADDR_SET,              -- store address, reset parser and go to next

        ST_OP_GET,                -- get operation item - type and ai (type defines if the address is for write or for read)
        ST_ADDR_SET_W_R,          -- store word_count, set address B0 for W or R, reset parser and go to next
        
        ST_ADDR_RUN,              -- set I2C_BUS_RUN
        ST_ADDR_DATA,             -- write address
        ST_ADDR_ACK,              -- wait for ack
             
        ST_WRITE_GET,             -- get first byte of bytestream to send (directly store in rin.data)
        ST_WRITE_RUN,             -- set I2C_BUS_RUN
        ST_WRITE_DATA,            -- write byte (r.data) to shift register (decrement word_count)
        ST_WRITE_ACK,             -- wait for ack
        ST_WRITE_END,             -- if word_count = 0, go to ST_CMD_END, if not, go to ST_WRITE_GET
     
        ST_READ_RUN,              -- set I2C_BUS_RUN
        ST_READ_DATA,             -- read DATA
        ST_READ_ACK,              -- send ACK
        ST_READ_PUT,              -- put read byte in rsp bus
        ST_READ_END,              -- (may be merged in ST_READ_PUT) if word_count = 0, go to ST_CMD_END,
                                  -- if not, go to ST_READ_RUN/ST_READ_DATA

        ST_START,                 -- send command to start the bus
        ST_START_WAIT,            -- wait for bus to start
        ST_STOP,                  -- send command to stop the bus
        ST_STOP_WAIT,             -- wait for bus to stop

        ST_RSP_OK_PREP,
        ST_RSP_OK_PUT,            -- put null (1 byte)
        ST_RSP_ANACK_PREP,
        ST_RSP_ANACK_PUT,         -- put false (1 byte)
        ST_RSP_DNACK_PREP,
        ST_RSP_DNACK_PUT,         -- put #6.2(uint) (more than 1 byte)
        ST_RSP_ARRAY_HDR_PREP,
        ST_RSP_ARRAY_HDR_PUT, 
        ST_RSP_BSTR_HDR_PREP,
        ST_RSP_BSTR_HDR_PUT,      -- send header for byte string with len = r.word_count
        ST_RSP_BREAK_PREP,
        ST_RSP_BREAK_PUT,

        ST_IO_FLUSH_GET,
        ST_IO_FLUSH_PUT
        
    );

    type regs_t is record
        state         : state_t;
        owned         : std_ulogic;
        addr          : std_ulogic_vector(9 downto 0);
        data          : std_ulogic_vector(7 downto 0);
        word_count    : natural range 0 to 63;
        word_total    : natural range 0 to 63;
        command_count : unsigned(31 downto 0); -- this could be up to 63 downto 0
        divisor       : unsigned(5 downto 0);
        parser        : nsl_data.cbor.parser_t;
        indefinite    : boolean;
        last          : boolean; 
        encoded       : nsl_data.bytestream.byte_string(8 downto 0);
        encoded_len   : natural range 0 to 8;
        encoded_i     : natural range 0 to 8;
        cmd_cancelled : boolean;
    end record;

    signal r, rin : regs_t;

    signal i2c_filt_i : nsl_i2c.i2c.i2c_i;
    signal i2c_clocker_o, i2c_shifter_o : nsl_i2c.i2c.i2c_o;
    signal start_i, stop_i : std_ulogic;
    signal clocker_owned_i, clocker_ready_i : std_ulogic;
    signal clocker_cmd_o : i2c_bus_cmd_t;
    signal shift_enable_o, shift_send_data_o, shift_arb_ok_i : std_ulogic;
    signal shift_w_valid_o, shift_w_ready_i : std_ulogic;
    signal shift_r_valid_i, shift_r_ready_o : std_ulogic;
    signal shift_w_data_o, shift_r_data_i : std_ulogic_vector(7 downto 0);
   
    
    constant c_print_logs : boolean := false;

    function to_fixed(s : string; len : natural) return string is
      variable result : string(1 to len) := (others => ' ');
    begin
      if s'length <= len then
        result(1 to s'length) := s;
      else
        result := s(1 to len);  -- truncate if longer
      end if;
      return result;
    end;

    procedure log_state_change(r : regs_t; rin: regs_t) is
      constant string_len : integer := 16;
      variable c_state, n_state : string (1 to string_len);
    begin
      case r.state is
        when ST_RESET => c_state := to_fixed("ST_RESET", string_len);
        when ST_ARRAY_GET => c_state := to_fixed("ST_ARRAY_GET", string_len);
        when ST_ARRAY_ENTER => c_state := to_fixed("ST_ARRAY_ENTER", string_len);
        when ST_CMD_GET => c_state := to_fixed("ST_CMD_GET", string_len);
        when ST_CMD_EXEC => c_state := to_fixed("ST_CMD_EXEC", string_len);
        when ST_CMD_END => c_state := to_fixed("ST_CMD_END", string_len);
        when ST_ADDR_GET => c_state := to_fixed("ST_ADDR_GET", string_len);
        when ST_ADDR_SET => c_state := to_fixed("ST_ADDR_SET", string_len);
        when ST_OP_GET => c_state := to_fixed("ST_OP_GET", string_len);
        when ST_ADDR_SET_W_R => c_state := to_fixed("ST_ADDR_SET_W_R", string_len);
        when ST_ADDR_RUN => c_state := to_fixed("ST_ADDR_RUN", string_len);
        when ST_ADDR_DATA => c_state := to_fixed("ST_ADDR_DATA", string_len);
        when ST_ADDR_ACK => c_state := to_fixed("ST_ADDR_ACK", string_len);
        when ST_WRITE_GET => c_state := to_fixed("ST_WRITE_GET", string_len);
        when ST_WRITE_RUN => c_state := to_fixed("ST_WRITE_RUN", string_len);
        when ST_WRITE_DATA => c_state := to_fixed("ST_WRITE_DATA", string_len);
        when ST_WRITE_ACK => c_state := to_fixed("ST_WRITE_ACK", string_len);
        when ST_WRITE_END => c_state := to_fixed("ST_WRITE_END", string_len);
        when ST_READ_RUN => c_state := to_fixed("ST_READ_RUN", string_len);
        when ST_READ_DATA => c_state := to_fixed("ST_READ_DATA", string_len);
        when ST_READ_ACK => c_state := to_fixed("ST_READ_ACK", string_len);
        when ST_READ_PUT => c_state := to_fixed("ST_READ_PUT", string_len);
        when ST_READ_END => c_state := to_fixed("ST_READ_END", string_len);
        when ST_START => c_state := to_fixed("ST_START", string_len);
        when ST_START_WAIT => c_state := to_fixed("ST_START_WAIT", string_len);
        when ST_STOP => c_state := to_fixed("ST_STOP", string_len);
        when ST_STOP_WAIT => c_state := to_fixed("ST_STOP_WAIT", string_len);
        when ST_RSP_OK_PREP => c_state := to_fixed("ST_RSP_OK_PREP", string_len);
        when ST_RSP_OK_PUT => c_state := to_fixed("ST_RSP_OK_PUT", string_len);
        when ST_RSP_ANACK_PREP => c_state := to_fixed("ST_RSP_ANACK_PREP", string_len);
        when ST_RSP_ANACK_PUT => c_state := to_fixed("ST_RSP_ANACK_PUT", string_len);
        when ST_RSP_DNACK_PREP => c_state := to_fixed("ST_RSP_DNACK_PREP", string_len);
        when ST_RSP_DNACK_PUT => c_state := to_fixed("ST_RSP_DNACK_PUT", string_len);
        when ST_RSP_ARRAY_HDR_PREP => c_state := to_fixed("ST_RSP_ARRAY_HDR_PREP", string_len);
        when ST_RSP_ARRAY_HDR_PUT => c_state := to_fixed("ST_RSP_ARRAY_HDR_PUT", string_len);
        when ST_RSP_BSTR_HDR_PREP => c_state := to_fixed("ST_RSP_BSTR_HDR_PREP", string_len);
        when ST_RSP_BSTR_HDR_PUT => c_state := to_fixed("ST_RSP_BSTR_HDR_PUT", string_len);
        when ST_RSP_BREAK_PREP => c_state := to_fixed("ST_RSP_BREAK_PREP", string_len);
        when ST_RSP_BREAK_PUT => c_state := to_fixed("ST_RSP_BREAK_PUT", string_len);
        when ST_IO_FLUSH_GET => c_state := to_fixed("ST_IO_FLUSH_GET", string_len);
        when ST_IO_FLUSH_PUT => c_state := to_fixed("ST_IO_FLUSH_PUT", string_len);
        when others => c_state := to_fixed("UNKNOWN", string_len);
      end case;

      case rin.state is
        when ST_RESET => n_state := to_fixed("ST_RESET", string_len);
        when ST_ARRAY_GET => n_state := to_fixed("ST_ARRAY_GET", string_len);
        when ST_ARRAY_ENTER => n_state := to_fixed("ST_ARRAY_ENTER", string_len);
        when ST_CMD_GET => n_state := to_fixed("ST_CMD_GET", string_len);
        when ST_CMD_EXEC => n_state := to_fixed("ST_CMD_EXEC", string_len);
        when ST_CMD_END => n_state := to_fixed("ST_CMD_END", string_len);
        when ST_ADDR_GET => n_state := to_fixed("ST_ADDR_GET", string_len);
        when ST_ADDR_SET => n_state := to_fixed("ST_ADDR_SET", string_len);
        when ST_OP_GET => n_state := to_fixed("ST_OP_GET", string_len);
        when ST_ADDR_SET_W_R => n_state := to_fixed("ST_ADDR_SET_W_R", string_len);
        when ST_ADDR_RUN => n_state := to_fixed("ST_ADDR_RUN", string_len);
        when ST_ADDR_DATA => n_state := to_fixed("ST_ADDR_DATA", string_len);
        when ST_ADDR_ACK => n_state := to_fixed("ST_ADDR_ACK", string_len);
        when ST_WRITE_GET => n_state := to_fixed("ST_WRITE_GET", string_len);
        when ST_WRITE_RUN => n_state := to_fixed("ST_WRITE_RUN", string_len);
        when ST_WRITE_DATA => n_state := to_fixed("ST_WRITE_DATA", string_len);
        when ST_WRITE_ACK => n_state := to_fixed("ST_WRITE_ACK", string_len);
        when ST_WRITE_END => n_state := to_fixed("ST_WRITE_END", string_len);
        when ST_READ_RUN => n_state := to_fixed("ST_READ_RUN", string_len);
        when ST_READ_DATA => n_state := to_fixed("ST_READ_DATA", string_len);
        when ST_READ_ACK => n_state := to_fixed("ST_READ_ACK", string_len);
        when ST_READ_PUT => n_state := to_fixed("ST_READ_PUT", string_len);
        when ST_READ_END => n_state := to_fixed("ST_READ_END", string_len);
        when ST_START => n_state := to_fixed("ST_START", string_len);
        when ST_START_WAIT => n_state := to_fixed("ST_START_WAIT", string_len);
        when ST_STOP => n_state := to_fixed("ST_STOP", string_len);
        when ST_STOP_WAIT => n_state := to_fixed("ST_STOP_WAIT", string_len);
        when ST_RSP_OK_PREP => n_state := to_fixed("ST_RSP_OK_PREP", string_len);
        when ST_RSP_OK_PUT => n_state := to_fixed("ST_RSP_OK_PUT", string_len);
        when ST_RSP_ANACK_PREP => n_state := to_fixed("ST_RSP_ANACK_PREP", string_len);
        when ST_RSP_ANACK_PUT => n_state := to_fixed("ST_RSP_ANACK_PUT", string_len);
        when ST_RSP_DNACK_PREP => n_state := to_fixed("ST_RSP_DNACK_PREP", string_len);
        when ST_RSP_DNACK_PUT => n_state := to_fixed("ST_RSP_DNACK_PUT", string_len);
        when ST_RSP_ARRAY_HDR_PREP => n_state := to_fixed("ST_RSP_ARRAY_HDR_PREP", string_len);
        when ST_RSP_ARRAY_HDR_PUT => n_state := to_fixed("ST_RSP_ARRAY_HDR_PUT", string_len);
        when ST_RSP_BSTR_HDR_PREP => n_state := to_fixed("ST_RSP_BSTR_HDR_PREP", string_len);
        when ST_RSP_BSTR_HDR_PUT => n_state := to_fixed("ST_RSP_BSTR_HDR_PUT", string_len);
        when ST_RSP_BREAK_PREP => n_state := to_fixed("ST_RSP_BREAK_PREP", string_len);
        when ST_RSP_BREAK_PUT => n_state := to_fixed("ST_RSP_BREAK_PUT", string_len);
        when ST_IO_FLUSH_GET => n_state := to_fixed("ST_IO_FLUSH_GET", string_len);
        when ST_IO_FLUSH_PUT => n_state := to_fixed("ST_IO_FLUSH_PUT", string_len);
        when others => n_state := to_fixed("UNKNOWN", string_len);
      end case;

    if c_print_logs then
      nsl_simulation.logging.log_info("In " & c_state & " => " & n_state & character'val(10));
    end if;

    end procedure;

begin

    line_mon: nsl_i2c.i2c.i2c_line_monitor
    generic map(
      debounce_count_c => 2
      )
    port map(
      clock_i => clock_i,
      reset_n_i => reset_n_i,
      raw_i => i2c_i,
      filtered_o => i2c_filt_i,
      start_o => start_i,
      stop_o => stop_i
      );

    clock_driver: nsl_i2c.master.master_clock_driver
    port map(
      clock_i   => clock_i,
      reset_n_i => reset_n_i,

      half_cycle_clock_count_i => to_unsigned(10, 3),

      i2c_i => i2c_filt_i,
      i2c_o => i2c_clocker_o,

      cmd_i => clocker_cmd_o,

      ready_o => clocker_ready_i,
      owned_o => clocker_owned_i
      );


    shifter: nsl_i2c.master.master_shift_register
    port map(
      clock_i  => clock_i,
      reset_n_i => reset_n_i,

      i2c_o => i2c_shifter_o,
      i2c_i => i2c_filt_i,

      start_i => start_i,
      arb_ok_o  => shift_arb_ok_i,

      enable_i => shift_enable_o,
      send_mode_i => shift_send_data_o,

      send_valid_i => shift_w_valid_o,
      send_ready_o => shift_w_ready_i,
      send_data_i => shift_w_data_o,

      recv_valid_o => shift_r_valid_i,
      recv_ready_i => shift_r_ready_o,
      recv_data_o => shift_r_data_i
      );

    ck : process (clock_i, reset_n_i)
    begin
      if rising_edge(clock_i) then
        r <= rin;
        if rin.state = r.state then
        else
          log_state_change(r => r, rin => rin);
        end if;
      end if;
      if reset_n_i = '0' then
        r.state <= ST_RESET;
      end if;
    end process;

    transition : process (clocker_owned_i, clocker_ready_i,
                          cmd_i, r, rsp_i,
                          shift_r_data_i, shift_r_valid_i, shift_w_ready_i,
                          shift_arb_ok_i)
      variable cbr_encoded : nsl_data.bytestream.byte_stream;
    begin
      rin <= r;

      if clocker_ready_i = '1' then
        rin.owned <= clocker_owned_i;
      end if;
      if shift_arb_ok_i = '0' then
        rin.owned <= '0';
      end if;
      
      case r.state is
        when ST_RESET =>
          nsl_simulation.logging.log_info("In ST_RESET");
          rin.divisor       <= (others => '1');
          rin.state         <= ST_ARRAY_GET;
          rin.parser        <= nsl_data.cbor.reset;
          rin.word_count    <= 0;
          rin.command_count <= (others => '0');
          rin.addr          <= (others => '0');
          rin.data          <= (others => '-');
          rin.last          <= false;
          rin.encoded       <= (others => (others => '-') );
          rin.encoded_len   <= 0;
          rin.encoded_i     <= 0;
          rin.cmd_cancelled <= false;

        when ST_ARRAY_GET =>
            if cmd_i.valid = '1' then
              nsl_simulation.logging.log_info("In ST_ARRAY_GET, parsing a byte");
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                rin.state <= ST_ARRAY_ENTER;
              end if;
            end if;

        when ST_ARRAY_ENTER =>
          if nsl_data.cbor.kind(r.parser) = KIND_ARRAY then
            if not r.parser.indefinite then
              rin.command_count <= nsl_data.cbor.arg(r.parser, 32);
              rin.indefinite    <= false;
            else
              rin.indefinite    <= true;
            end if;
            rin.parser <= nsl_data.cbor.reset;
            rin.state  <= ST_RSP_ARRAY_HDR_PREP;
          else 
            -- rin.state <= ST_FLUSH; -- TODO ??
          end if;

        when ST_CMD_GET =>
            if cmd_i.valid = '1' then
              nsl_simulation.logging.log_info("In ST_CMD_GET, parsing a byte");
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                rin.cmd_cancelled <= false;
                rin.state <= ST_CMD_EXEC;
              end if;
            end if;

        when ST_CMD_EXEC =>
          if nsl_data.cbor.kind(r.parser) = KIND_ARRAY then
            rin.state  <= ST_ADDR_GET;
          elsif nsl_data.cbor.kind(r.parser) = KIND_NULL then
            rin.state  <= ST_STOP;
          elsif nsl_data.cbor.kind(r.parser) = KIND_BREAK then
            if r.indefinite then
              rin.state  <= ST_RSP_BREAK_PREP;
            else 
            end if;
          else
            rin.state <= ST_RSP_BREAK_PREP;
          end if;
          rin.parser <= nsl_data.cbor.reset;
          if not r.indefinite then
            rin.command_count <= (r.command_count - 1) mod 32;
          end if;

        when ST_ADDR_GET =>
            if cmd_i.valid = '1' then
              nsl_simulation.logging.log_info("In ST_ADDR_GET, parsing a byte");
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                rin.state <= ST_ADDR_SET;
              end if;
            end if;
          
        when ST_ADDR_SET =>
          if nsl_data.cbor.kind(r.parser) = KIND_POSITIVE then
            rin.addr <= std_ulogic_vector(nsl_data.cbor.arg(r.parser, 10));       
            rin.parser <= nsl_data.cbor.reset;
            rin.state  <= ST_OP_GET;
          else
            -- rin.state <= ST_IO_FLUSH_GET;
          end if;

        when ST_OP_GET =>
            if cmd_i.valid = '1' then
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                rin.state <= ST_ADDR_SET_W_R;
              end if;
            end if;
        
        when ST_ADDR_SET_W_R =>
          rin.state <= ST_START;
          if nsl_data.cbor.kind(r.parser) = KIND_POSITIVE then
            -- READ OPERATION
            rin.addr <= r.addr(8 downto 0) & '1';
            rin.word_count <= to_integer(nsl_data.cbor.arg(r.parser, 64));
            rin.word_total <= to_integer(nsl_data.cbor.arg(r.parser, 64));
          elsif nsl_data.cbor.kind(r.parser) = KIND_BSTR then
            -- WRITE OPERATION
            rin.addr <= r.addr(8 downto 0) & '0';
            rin.word_count <= to_integer(nsl_data.cbor.arg(r.parser, 64));
            rin.word_total <= to_integer(nsl_data.cbor.arg(r.parser, 64));
          else
            -- rin.state <= ST_START;
          end if;

        when ST_START =>
          if clocker_ready_i = '1' then
            rin.state <= ST_START_WAIT;
          end if;

        when ST_START_WAIT =>
          if clocker_ready_i = '1' then
            if clocker_owned_i = '1' then
              rin.state <= ST_ADDR_RUN;
            else 
              rin.state <= ST_IO_FLUSH_GET;
            end if;
          end if;

        when ST_ADDR_RUN =>
          if clocker_ready_i = '1' then
            rin.state <= ST_ADDR_DATA;
            rin.data  <= r.addr(7 downto 0);
          end if;

        when ST_ADDR_DATA =>
          if shift_w_ready_i = '1' then
            rin.state <= ST_ADDR_ACK;
          end if;

        when ST_ADDR_ACK =>
          if shift_r_valid_i = '1' then
            rin.data <= (0 => not shift_r_data_i(0), others => '0');
            if shift_r_data_i(0) = '0' then -- ACK OK
              if r.addr(0) = '1' then
                rin.state <= ST_RSP_BSTR_HDR_PREP;  -- before going to
                                                    -- ST_READ_RUN, write the
                                                    -- bstr header to hold the
                                                    -- read bytes
              else
                rin.state <= ST_WRITE_GET;
              end if;
            else
              rin.cmd_cancelled <= true;
              rin.state <= ST_RSP_ANACK_PREP;
            end if;
          end if;
        
        when ST_READ_RUN =>
          if clocker_ready_i = '1' then
            rin.state <= ST_READ_DATA;
          end if;
        
        when ST_READ_DATA =>
          if shift_r_valid_i = '1' then
            rin.state <= ST_READ_ACK;
            rin.data <= shift_r_data_i;
          end if;

        when ST_READ_ACK =>
          if shift_w_ready_i = '1' then
            rin.word_count <= (r.word_count - 1) mod 64;
            rin.state <= ST_READ_PUT;
            -- rin.last <= (r.word_count - 1) mod 64 = 0;
          end if;
       
        when ST_READ_PUT =>
          if rsp_i.ready = '1' then
            rin.state <= ST_READ_END;
          end if;
        
        when ST_READ_END =>
          if r.word_count = 0 then
            rin.state <= ST_CMD_END;
          else
            rin.state <= ST_READ_RUN; -- TODO or ST_READ_DATA??
          end if;
          
        when ST_WRITE_GET =>
          if cmd_i.valid = '1' then
            rin.data <= cmd_i.data(0);
            if not r.cmd_cancelled then
              rin.state <= ST_WRITE_RUN;
            else
              rin.word_count <= (r.word_count - 1) mod 64;
              rin.state <= ST_WRITE_END;
              nsl_simulation.logging.log_info("[Cancelled command] In ST_WRITE_GET, discarding a byte and going to ST_WRITE_END");
            end if;
          end if;

          when ST_WRITE_RUN =>
            if clocker_ready_i = '1' then
            rin.state <= ST_WRITE_DATA;
            end if;

        when ST_WRITE_DATA =>
          if shift_w_ready_i = '1' then
            rin.state <= ST_WRITE_ACK;
          end if;

        when ST_WRITE_ACK =>
          if shift_r_valid_i = '1' then
            rin.word_count <= (r.word_count - 1) mod 64;
            if shift_r_data_i(0) = '1' then -- NACK
              rin.state <= ST_RSP_DNACK_PREP;
              rin.cmd_cancelled <= true;
              rin.data <= (0 => not shift_r_data_i(0), others => '0');
            else
              rin.state <= ST_WRITE_END;
              rin.data <= (0 => not shift_r_data_i(0), others => '0');
            end if;
          end if;

        when ST_WRITE_END =>
          -- if rsp_i.ready = '1' then
            if r.word_count = 0 then
              if not r.cmd_cancelled then
                rin.state <= ST_RSP_OK_PREP;
              else
                nsl_simulation.logging.log_info("[Cancelled command] In ST_WRITE_END, going to ST_CMD_END (NOT via ST_RSP_OK_PREP)");
                rin.state <= ST_CMD_END;
              end if;
            else
              rin.state <= ST_WRITE_GET;
            end if;
          -- end if;

        when ST_CMD_END =>
          if not r.indefinite and r.command_count = 0 then
            rin.state <= ST_RSP_BREAK_PREP;
          else
            rin.state <= ST_CMD_GET;
          end if;
          rin.parser <= nsl_data.cbor.reset;


        when ST_IO_FLUSH_GET =>
          if cmd_i.valid = '1' then
            rin.state <= ST_IO_FLUSH_PUT;
          end if;

        when ST_IO_FLUSH_PUT =>
          if rsp_i.ready = '1' then
            rin.word_count <= (r.word_count - 1) mod 64;
            if r.word_count = 0 then
              rin.state <= ST_CMD_GET;
            end if;
          end if;
        
        when ST_STOP =>
          if clocker_ready_i = '1' then
            rin.state <= ST_STOP_WAIT;
          end if;

        when ST_STOP_WAIT => -- TODO probably I can remove it
        if clocker_ready_i = '1' then
          rin.state <= ST_CMD_END;
        end if;

        when ST_RSP_OK_PREP =>
          rin.data  <= nsl_data.cbor.cbor_null(0);
          rin.state <= ST_RSP_OK_PUT;
          
        when ST_RSP_OK_PUT =>
          if rsp_i.ready = '1' then
            rin.state <= ST_CMD_END;
          end if;
        
        when ST_RSP_ANACK_PREP =>
          rin.data  <= nsl_data.cbor.cbor_false(0);
          rin.state <= ST_RSP_ANACK_PUT;
        
        when ST_RSP_ANACK_PUT =>
          if rsp_i.ready = '1' then
            if r.addr(0) = '1' then
              rin.state <= ST_CMD_END;
            else
              -- In the case of write operations, must read all the bytes to
              -- read, even if they will not be written.
              rin.state <= ST_WRITE_GET;
            end if;
          end if;
        
        when ST_RSP_DNACK_PREP =>
          nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tagged(tag => 2, item => nsl_data.cbor.cbor_positive(value => to_integer(to_unsigned(r.word_total - r.word_count - 1, 6))) ) );
          nsl_simulation.logging.log_info("cbr_encoded " & nsl_data.text.to_string(cbr_encoded.all) );

          rin.encoded_len <= cbr_encoded.all'length;
          rin.encoded(cbr_encoded.all'length-1 downto 0) <= cbr_encoded.all;
          nsl_data.bytestream.clear(s => cbr_encoded);

          rin.state <= ST_RSP_DNACK_PUT;
          rin.last  <= false;
          
        when ST_RSP_DNACK_PUT  =>
          if rsp_i.ready = '1' then
            if r.encoded_len - 1 = r.encoded_i then
              rin.state <= ST_WRITE_END;
              rin.encoded_len <= 0;
              rin.encoded_i <= 0;
            else
              rin.encoded_i <= (r.encoded_i + 1) mod 9;
            end if;
          end if;

      when ST_RSP_ARRAY_HDR_PREP =>
          nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_array_hdr(length => -1) );
          rin.encoded_len <= cbr_encoded.all'length;
          rin.encoded(cbr_encoded.all'length-1 downto 0) <= cbr_encoded.all;
          rin.state <= ST_RSP_ARRAY_HDR_PUT;
          rin.last  <= false;
          nsl_data.bytestream.clear(s => cbr_encoded);
          
      when ST_RSP_ARRAY_HDR_PUT =>
          if rsp_i.ready = '1' then
            if r.encoded_len - 1 = r.encoded_i then
              rin.state <= ST_CMD_GET;
              rin.encoded_len <= 0;
              rin.encoded_i <= 0;
            else
              rin.encoded_i <= (r.encoded_i + 1) mod 9;
            end if;
          end if;

      when ST_RSP_BSTR_HDR_PREP =>
          nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_bstr_hdr(length => to_integer(to_unsigned(r.word_count, 6))));
          rin.encoded_len <= cbr_encoded.all'length;
          rin.encoded(cbr_encoded.all'length-1 downto 0) <= cbr_encoded.all;
          nsl_data.bytestream.clear(s => cbr_encoded);
          rin.state <= ST_RSP_BSTR_HDR_PUT;

      when ST_RSP_BSTR_HDR_PUT =>
          if rsp_i.ready = '1' then
            if r.encoded_len - 1 = r.encoded_i then
              rin.state <= ST_READ_RUN;
              rin.encoded_len <= 0;
              rin.encoded_i <= 0;
            else
              rin.encoded_i <= (r.encoded_i + 1) mod 9;
            end if;
          end if;
          
      when ST_RSP_BREAK_PREP=>
          rin.data  <= nsl_data.cbor.cbor_break(0);
          rin.last  <= true;
          rin.state <= ST_RSP_BREAK_PUT;
    
      when ST_RSP_BREAK_PUT =>
          if rsp_i.ready = '1' then
            rin.state <= ST_ARRAY_GET;
          end if;

          
      end case;
    end process;

    i2c_o <= i2c_clocker_o + i2c_shifter_o;

    moore: process (r)
    begin
      cmd_o.ready <= '0';
      rsp_o <= nsl_amba.axi4_stream.transfer_defaults(cfg => axi_s_cfg_c);

      shift_enable_o <= '0';
      shift_send_data_o <= '-';
      shift_w_valid_o <= '0';
      shift_r_ready_o <= '0';
      shift_w_data_o <= (others => '-');

      if r.owned = '1' then
        clocker_cmd_o <= I2C_BUS_HOLD;
      else
        clocker_cmd_o <= I2C_BUS_RELEASE;
      end if;

      case r.state is
        when ST_RESET =>

        when ST_ARRAY_GET | ST_CMD_GET | ST_ADDR_GET | ST_OP_GET  =>
          cmd_o.ready <= '1';

        when ST_WRITE_GET =>
          cmd_o.ready <= '1';

       when ST_READ_PUT =>
          rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(r.data) , last => r.last);
          
          
        when ST_ARRAY_ENTER | ST_CMD_EXEC | ST_CMD_END | ST_ADDR_SET | ST_ADDR_SET_W_R | ST_WRITE_END | ST_READ_END =>
        when ST_RSP_OK_PREP| ST_RSP_DNACK_PREP | ST_RSP_ANACK_PREP | ST_RSP_BSTR_HDR_PREP | ST_RSP_ARRAY_HDR_PREP | ST_RSP_BREAK_PREP =>

        when ST_ADDR_RUN | ST_WRITE_RUN | ST_READ_RUN =>
          clocker_cmd_o <= I2C_BUS_RUN;

        when ST_ADDR_DATA =>
          shift_w_valid_o <= '1';
          shift_w_data_o <= r.data;

        when ST_ADDR_ACK | ST_WRITE_ACK | ST_READ_DATA =>
          shift_r_ready_o <= '1';

        when ST_WRITE_DATA =>
          shift_w_valid_o <= '1';
          shift_w_data_o <= r.data;
        
        when ST_READ_ACK =>
          shift_w_valid_o <= '1';
          shift_w_data_o <= (0 => '1', others => '-');
        
        when ST_START_WAIT | ST_STOP_WAIT =>
          clocker_cmd_o <= I2C_BUS_HOLD;

        when ST_START =>
          clocker_cmd_o <= I2C_BUS_START;

        when ST_STOP =>
          clocker_cmd_o <= I2C_BUS_STOP;
        
        when ST_RSP_OK_PUT | ST_RSP_ANACK_PUT | ST_RSP_BREAK_PUT =>
          rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(r.data) , last => r.last);

        when ST_RSP_BSTR_HDR_PUT | ST_RSP_ARRAY_HDR_PUT | ST_RSP_DNACK_PUT =>
          -- nsl_simulation.logging.log_info("r.encoded_len " & nsl_data.text.to_string(r.encoded_len) & " sending r.encoded(" & nsl_data.text.to_string(r.encoded_len - r.encoded_i - 1) & " downto " & nsl_data.text.to_string(r.encoded_len - r.encoded_i - 1) & ")" );
          -- nsl_simulation.logging.log_info("r.encoded " & nsl_data.text.to_string(r.encoded));
          rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => r.encoded(r.encoded_len - r.encoded_i - 1 downto r.encoded_len - r.encoded_i - 1), last => r.last);
       
        when ST_IO_FLUSH_GET =>
        when ST_IO_FLUSH_PUT =>      
      end case;

    case r.state is

      when ST_ADDR_RUN | ST_ADDR_DATA | ST_ADDR_ACK | ST_WRITE_RUN | ST_WRITE_DATA | ST_WRITE_ACK =>
        shift_enable_o <= '1';
        shift_send_data_o <= '1';

      when ST_READ_RUN | ST_READ_DATA | ST_READ_ACK =>
        shift_enable_o <= '1';
        shift_send_data_o <= '0';   
        
      when others =>
        null;
    end case;
    end process;

end architecture;
