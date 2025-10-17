library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_amba, nsl_i2c, nsl_data;
use nsl_i2c.cbor_transactor.all;
use nsl_i2c.master.all;
use nsl_i2c.i2c."+";
use nsl_data.cbor.all;
use nsl_data.bytestream.all;

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
                                  -- TODO: remove all checks of is_done(). Check, but possibly not needed with is_last()
                                  -- TODO: use the actual functions from axi4_stream 
        ST_ARRAY_GET,             -- get first item (type and ai). should be an array header
        ST_ARRAY_ENTER,           -- store data if needed, reset parser and go to next
        
        ST_CMD_GET,               -- get command item (type and complete ai)
        ST_CMD_EXEC,              -- store data if needed, reset parser and go to next
                                  -- if command item is of array kind, go to ST_ADDR_GET
                                  -- if command item is a simple value (null) go to ST_STOP
        ST_CMD_END,               -- will get here after the command has been executed. 
                                  -- if the number of commands is completed, go to ARRAY_GET_FIRST, otherwise go to CMD_GET_FIRST

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
        ST_WRITE_END,             -- (may be merged in ST_WRITE_ACK) if word_count = 0, go to ST_CMD_END, if not, go to ST_WRITE_GET

        ST_READ_RUN,              -- set I2C_BUS_RUN
        ST_READ_DATA,             -- read DATA
        ST_READ_ACK,              -- send ACK
        ST_READ_PUT,              -- put read byte in rsp bus
        ST_READ_END,              -- (may be merged in ST_READ_PUT) if word_count = 0, go to ST_CMD_END, if not, go to ST_READ_RUN/ST_READ_DATA

        ST_START,                 -- send command to start the bus
        ST_START_WAIT,            -- wait for bus to start
        ST_STOP,                  -- send command to stop the bus
        ST_STOP_WAIT,             -- wait (?) for bus to stop

        ST_RSP_PUT_OK,            -- put null (1 byte)
        ST_RSP_PUT_ANACK,         -- put false (1 byte)
        ST_RSP_PUT_DNACK,         -- put #6.2(uint) (more than 1 byte)

        ST_IO_FLUSH_GET,
        ST_IO_FLUSH_PUT
        
    );
  
    type regs_t is record
        state         : state_t;
        owned         : std_ulogic;
        received      : std_ulogic_vector(7 downto 0); -- I think it's not needed
        addr          : std_ulogic_vector(9 downto 0);
        data          : std_ulogic_vector(7 downto 0);
        word_count    : natural range 0 to 63;
        command_count : unsigned(31 downto 0); -- this could be up to 63 downto 0
        divisor       : unsigned(5 downto 0);
        parser        : nsl_data.cbor.parser_t;
        indefinite    : boolean;
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

begin

    -- Typical use case in a streaming environment is to consume the
    -- bytestream one byte at a time by calling ``feed()`` on a ``parser_t``
    -- record.  ``is_last`` will tell whether item header is complete at this
    -- cycle.  On subsequent cycle, ``kind()``, ``arg()`` and ``arg_int()``
    -- will give out the details of the item.  For items containing data
    -- (BSTR, TSTR), it is caller's responsability to consume data length
    -- before returning to parsing an item.

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
      end if;
      if reset_n_i = '0' then
        r.state <= ST_RESET;
      end if;
    end process;

    transition : process (clocker_owned_i, clocker_ready_i,
                          cmd_i, r, rsp_i,
                          shift_r_data_i, shift_r_valid_i, shift_w_ready_i,
                          shift_arb_ok_i)
    -- transition : process (cmd_i, r, rsp_i)
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
          report "In ST_RESET" severity note;
          rin.divisor <= (others => '1');
          rin.state <= ST_ARRAY_GET;
          rin.parser <= nsl_data.cbor.reset;
          rin.word_count <= 0;
          rin.command_count <= (others => '0');
          rin.addr <= (others => '0');
          rin.data <= (others => '0');

        when ST_ARRAY_GET =>
          if not nsl_data.cbor.is_done(r.parser) then
            if cmd_i.valid = '1' then
              report "In ST_ARRAY_GET, parsing a byte" severity note;
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                report "In ST_ARRAY_GET, going to ST_ARRAY_ENTER" severity note;
                report "========================================" severity note;
                rin.state <= ST_ARRAY_ENTER;
              end if;
            end if;
          end if;

        when ST_ARRAY_ENTER =>
          if nsl_data.cbor.kind(r.parser) = KIND_ARRAY then
            -- if not nsl_data.cbor.is_done(r.parser) then
            --   if cmd_i.valid = '1' then
            --     report "In ST_ARRAY_ENTER, parsing another byte" severity note;
            --     rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
            --     report "cmd_i.data(0)" & nsl_data.text.to_string(cmd_i.data(0));
            --   end if;
            -- else
            if not r.parser.indefinite then
              report "In ST_ARRAY_ENTER, command count set to " & nsl_data.text.to_string(nsl_data.cbor.arg(r.parser, 32));
              rin.command_count <= nsl_data.cbor.arg(r.parser, 32);
              rin.indefinite    <= false;
            else
              rin.indefinite    <= true;
            end if;
            rin.parser <= nsl_data.cbor.reset;
            report "In ST_ARRAY_ENTER, going to ST_CMD_GET" severity note;
            report "========================================" severity note;
            rin.state  <= ST_CMD_GET;
           --  end if;
          else 
            -- rin.state <= ST_FLUSH;
          end if;

        when ST_CMD_GET =>
          if not nsl_data.cbor.is_done(r.parser) then
            if cmd_i.valid = '1' then
              report "In ST_CMD_GET, parsing a byte" severity note;
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              report "cmd_i.data(0) is " & nsl_data.text.to_hex_string(cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                report "In ST_CMD_GET, going to ST_CMD_EXEC" severity note;
                report "========================================" severity note;
                rin.state <= ST_CMD_EXEC;
              end if;
            end if;
          end if;

        when ST_CMD_EXEC =>
          if nsl_data.cbor.kind(r.parser) = KIND_ARRAY then
            report "In ST_CMD_EXEC, going to ST_ADDR_GET" severity note;
            rin.state  <= ST_ADDR_GET;
          elsif nsl_data.cbor.kind(r.parser) = KIND_NULL then
            report "In ST_CMD_EXEC, going to ST_STOP" severity note;
            rin.state  <= ST_STOP;
          elsif nsl_data.cbor.kind(r.parser) = KIND_BREAK then
            if r.indefinite then
              report "In ST_CMD_EXEC, found a break, going to ST_ARRAY_GET" severity note;
              rin.state  <= ST_ARRAY_GET;
            else 
              report "In ST_CMD_EXEC, found a break in an item of definite lenght" severity warning;
            end if;
          else
            -- report "in ST_CMD_EXEC, Type of item was not the expected one" severity note;
          end if;
          rin.parser <= nsl_data.cbor.reset;
          if not r.indefinite then
            rin.command_count <= (r.command_count - 1) mod 32;
          end if;
          report "========================================" severity note;

        when ST_ADDR_GET =>
          if not nsl_data.cbor.is_done(r.parser) then
            if cmd_i.valid = '1' then
              report "In ST_ADDR_GET, parsing a byte" severity note;
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              report "cmd_i.data(0) is " & nsl_data.text.to_hex_string(cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                report "In ST_ADDR_GET, going to ST_ADDR_SET" severity note;
                report "========================================" severity note;
                rin.state <= ST_ADDR_SET;
              end if;
            end if;
          end if;
          
        when ST_ADDR_SET =>
          if nsl_data.cbor.kind(r.parser) = KIND_POSITIVE then
            report "In ST_ADDR_SET, address set to " & nsl_data.text.to_string(nsl_data.cbor.arg(r.parser, 10));
            rin.addr <= std_ulogic_vector(nsl_data.cbor.arg(r.parser, 10));       
            rin.parser <= nsl_data.cbor.reset;
            rin.state  <= ST_OP_GET;
            report "========================================" severity note;
          else
            report "in ST_ADDR_SET, Type of item was not the expected one" severity note;
            -- rin.state <= ST_IO_FLUSH_GET;
          end if;

        when ST_OP_GET =>
          if not nsl_data.cbor.is_done(r.parser) then
            if cmd_i.valid = '1' then
              report "In ST_OP_GET, parsing a byte" severity note;
              rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
              if nsl_data.cbor.is_last( r.parser, cmd_i.data(0) ) then
                report "In ST_OP_GET, going to ST_ADDR_SET_W_R" severity note;
                report "========================================" severity note;
                rin.state <= ST_ADDR_SET_W_R;
              end if;
            end if;
          end if;
        
        when ST_ADDR_SET_W_R =>
          rin.state <= ST_START;
          if nsl_data.cbor.kind(r.parser) = KIND_POSITIVE then
            -- READ OPERATION
            report "In ST_ADDR_SET_W_R, going to ST_START for a READ operation" severity note;
            report "========================================" severity note;
            rin.addr <= r.addr(8 downto 0) & '1';
            rin.word_count <= to_integer(nsl_data.cbor.arg(r.parser, 64));
          elsif nsl_data.cbor.kind(r.parser) = KIND_BSTR then
            -- WRITE OPERATION
            report "In ST_ADDR_SET_W_R, going to ST_START for a WRITE operation" severity note;
            report "========================================" severity note;
            rin.addr <= r.addr(8 downto 0) & '0';
            rin.word_count <= to_integer(nsl_data.cbor.arg(r.parser, 64));
          else
            report "In ST_ADDR_SET_W_R, next operation type is not a write or a read!" severity note;
            report "========================================" severity note;
            -- rin.state <= ST_START;
          end if;

        when ST_START =>
          if clocker_ready_i = '1' then
            report "In ST_START, going to ST_START_WAIT" severity note;
            report "========================================" severity note;
            rin.state <= ST_START_WAIT;
          end if;

        when ST_START_WAIT =>
          if clocker_ready_i = '1' then
            if clocker_owned_i = '1' then
              report "In ST_START_WAIT, going to ST_ADDR_RUN" severity note;
              report "========================================" severity note;
              rin.state <= ST_ADDR_RUN;
            else 
              report "In ST_START_WAIT, clock not owned, going to ST_IO_FLUSH_GET" severity note;
              report "========================================" severity note;
              rin.state <= ST_IO_FLUSH_GET;
            end if;
          end if;

        when ST_ADDR_RUN =>
          if clocker_ready_i = '1' then
            report "In ST_ADDR_RUN, going to ST_ADDR_DATA" severity note;
            report "========================================" severity note;
            rin.state <= ST_ADDR_DATA;
            rin.data  <= r.addr(7 downto 0);
          end if;

        when ST_ADDR_DATA =>
          if shift_w_ready_i = '1' then
            report "In ST_ADDR_DATA, going to ST_ADDR_ACK" severity note;
            report "========================================" severity note;
            rin.state <= ST_ADDR_ACK;
          end if;

        when ST_ADDR_ACK =>
          if shift_r_valid_i = '1' then
            rin.data <= (0 => not shift_r_data_i(0), others => '0');
            if shift_r_data_i(0) = '0' then -- ACK OK
              if r.addr(0) = '1' then
                report "In ST_ADDR_ACK, going to ST_READ_RUN" severity note;
                report "========================================" severity note;
                rin.state <= ST_READ_RUN;
              else
                report "In ST_ADDR_ACK, going to ST_WRITE_GET" severity note;
                report "========================================" severity note;
                rin.state <= ST_WRITE_GET;
              end if;
            else
              report "In ST_ADDR_ACK, going to ST_RSP_PUT_ANACK" severity note;
              report "========================================" severity note;
              rin.state <= ST_RSP_PUT_ANACK;
            end if;
          end if;

        -- when ST_READ_GET =>
        --   if not nsl_data.cbor.is_done(r.parser) then
        --     if cmd_i.valid = '1' then
        --       report "In ST_READ_GET, going to parse another byte" severity note;
        --       rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
        --      end if;
        --   else
        --     rin.word_count <= nsl_data.cbor.arg_int(r.parser);
        --     rin.parser <= nsl_data.cbor.reset;
        --     report "In ST_READ_CNT_GET, going to ST_READ_RUN" severity note;
        --     report "========================================" severity note;
        --     rin.state <= ST_READ_RUN;
        --   end if;
          
        when ST_READ_RUN =>
          if clocker_ready_i = '1' then
            report "In ST_READ_RUN, going to ST_READ_DATA" severity note;
            report "========================================" severity note;
            rin.state <= ST_READ_DATA;
          end if;
        
        when ST_READ_DATA =>
          if shift_r_valid_i = '1' then
            report "In ST_READ_DATA, going to ST_READ_ACK" severity note;
            report "========================================" severity note;
            rin.state <= ST_READ_ACK;
            rin.data <= shift_r_data_i;
          end if;

        when ST_READ_ACK =>
          if shift_w_ready_i = '1' then
            report "In ST_READ_ACK, going to ST_READ_PUT" severity note;
            report "========================================" severity note;
            rin.word_count <= (r.word_count - 1) mod 64;
            rin.state <= ST_READ_PUT;
          end if;
       
        when ST_READ_PUT =>
          if rsp_i.ready = '1' then
            report "In ST_READ_PUT, going to ST_READ_END" severity note;
            report "========================================" severity note;
            rin.state <= ST_READ_END;
          end if;
        
        when ST_READ_END =>
          if r.word_count = 0 then
            report "In ST_READ_END, going to ST_CMD_END" severity note;
            rin.state <= ST_CMD_END;
          else
            report "In ST_READ_END, going to ST_READ_RUN" severity note;
            rin.state <= ST_READ_RUN; -- or ST_READ_DATA??
          end if;
          report "========================================" severity note;
          
        -- when ST_WRITE_GET =>
        --   if not nsl_data.cbor.is_done(r.parser) then
        --     if cmd_i.valid = '1' then
        --       report "In ST_WRITE_CNT_GET, going to parse another byte" severity note;
        --       rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
        --      end if;
        --   else
        --     rin.word_count <= nsl_data.cbor.arg_int(r.parser);
        --     rin.parser <= nsl_data.cbor.reset;
        --     report "In ST_WRITE_CNT_GET, going to ST_WRITE_RUN" severity note;
        --     report "========================================" severity note;
        --     rin.state <= ST_WRITE_RUN;
        --   end if;

        -- when ST_WRITE_HDR_GET =>
        --   if cmd_i.valid = '1' then
        --     report "In ST_WRITE_HDR_GET, going to ST_ADDR_GET" severity note;
        --     report "========================================" severity note;
        --     rin.state  <= ST_ADDR_GET;
        --     rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
        --   end if;

        when ST_WRITE_GET =>
          if cmd_i.valid = '1' then
            report "In ST_WRITE_GET, getting a byte and going to ST_WRITE_RUN" severity note;
            report "========================================" severity note;
            rin.data <= cmd_i.data(0);
            rin.state <= ST_WRITE_RUN;
          end if;

          when ST_WRITE_RUN =>
            if clocker_ready_i = '1' then
              report "In ST_WRITE_RUN, going to ST_WRITE_DATA" severity note;
              report "========================================" severity note;
            rin.state <= ST_WRITE_DATA;
            end if;

        when ST_WRITE_DATA =>
          if shift_w_ready_i = '1' then
            report "In ST_WRITE_DATA, going to ST_WRITE_ACK" severity note;
            report "========================================" severity note;
            rin.state <= ST_WRITE_ACK;
          end if;

        when ST_WRITE_ACK =>
          if shift_r_valid_i = '1' then
            rin.word_count <= (r.word_count - 1) mod 64;
            if shift_r_data_i(0) = '1' then -- NACK
              report "In ST_WRITE_ACK, going to ST_RSP_PUT_DNACK" severity note;
              report "========================================" severity note;
              rin.state <= ST_RSP_PUT_DNACK;
              rin.data <= (0 => not shift_r_data_i(0), others => '0');
            else
              report "In ST_WRITE_ACK, going to ST_WRITE_END" severity note;
              report "========================================" severity note;
              rin.state <= ST_WRITE_END;
              rin.data <= (0 => not shift_r_data_i(0), others => '0');
            end if;
          end if;

        when ST_WRITE_END =>
          if rsp_i.ready = '1' then
            if r.word_count = 0 then
              report "In ST_WRITE_END, going to ST_CMD_END" severity note;
              report "========================================" severity note;
              rin.state <= ST_CMD_END;
            else
              report "In ST_WRITE_END, going to ST_WRITE_GET" severity note;
              report "========================================" severity note;
              rin.state <= ST_WRITE_GET;
            end if;
          end if;

        when ST_CMD_END => -- to me this is clearer to read, should be optimized and synthesized to the same hardware, or not?
          if not r.indefinite then
            if r.command_count = 0 then
              rin.state <= ST_ARRAY_GET;
            else
              rin.state <= ST_CMD_GET;
            end if;
          else
            rin.state <= ST_CMD_GET;
          end if;
          rin.parser <= nsl_data.cbor.reset;


        when ST_IO_FLUSH_GET =>
          if cmd_i.valid = '1' then
            report "In ST_IO_FLUSH_GET, going to ST_IO_FLUSH_PUT" severity note;
            report "========================================" severity note;
            rin.state <= ST_IO_FLUSH_PUT;
          end if;

        when ST_IO_FLUSH_PUT =>
          if rsp_i.ready = '1' then
            rin.word_count <= (r.word_count - 1) mod 64;
            if r.word_count = 0 then
              report "In ST_IO_FLUSH_PUT, going to ST_CMD_GET" severity note;
              report "========================================" severity note;
              rin.state <= ST_CMD_GET;
            end if;
          end if;
        
        when ST_STOP =>
          if clocker_ready_i = '1' then
            report "In ST_STOP, going to ST_STOP_WAIT" severity note;
            report "========================================" severity note;
            rin.state <= ST_STOP_WAIT;
          end if;

        when ST_STOP_WAIT => -- TODO probably I can remove it
        if clocker_ready_i = '1' then
          report "In ST_STOP_WAIT, going to ST_CMD_END" severity note;
          report "========================================" severity note;
          rin.state <= ST_CMD_END;
        end if;
        
        when ST_RSP_PUT_OK | ST_RSP_PUT_ANACK | ST_RSP_PUT_DNACK  =>
          if rsp_i.ready = '1' then
            report "In ST_RSP_PUT_OK | ST_RSP_PUT_ANACK | ST_RSP_PUT_DNACK | ST_RSP_PUT_DATA going to ST_CMD_GET" severity note;
            report "========================================" severity note;
            rin.state <= ST_CMD_GET;
          end if;
      end case;
    end process;

    i2c_o <= i2c_clocker_o + i2c_shifter_o;

    moore: process (r)
    begin
      cmd_o.ready <= '0';
      rsp_o.valid <= '0';
      rsp_o.last  <= '1';

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
          if not nsl_data.cbor.is_done(r.parser) then
            cmd_o.ready <= '1';
          end if;

        when ST_WRITE_GET =>
          cmd_o.ready <= '1';

       when ST_READ_PUT =>
          rsp_o.valid <= '1';
          rsp_o.data(0) <= r.data;
          
        when ST_ARRAY_ENTER | ST_CMD_EXEC | ST_CMD_END | ST_ADDR_SET | ST_ADDR_SET_W_R | ST_WRITE_END | ST_READ_END=>

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
        
        when ST_RSP_PUT_OK | ST_RSP_PUT_ANACK | ST_RSP_PUT_DNACK =>
          rsp_o.valid <= '1';
          rsp_o.data(0) <= r.data;
       
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
