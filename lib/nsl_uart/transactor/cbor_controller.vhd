library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_uart, nsl_bnoc, nsl_amba, nsl_data, nsl_simulation;
use nsl_data.cbor.all;

entity cbor_controller is
  generic(
    system_clock_c     : natural;
    axi_s_cfg_c        : nsl_amba.axi4_stream.config_t;
    stop_count_c       : natural range 1 to 2 := 1;
    parity_c           : nsl_uart.serdes.parity_t := nsl_uart.serdes.PARITY_NONE;
    handshake_active_c : std_ulogic := '0';
    divisor_c          : unsigned(0 to 31);
    timeout_c          : unsigned(0 to 31);
    tstr_max_size_c    : natural
    );
  port (
    reset_n_i    : in std_ulogic;
    clock_i      : in std_ulogic;
    
    tx_o   : out std_ulogic;
    cts_i  : in std_ulogic := handshake_active_c;
    rx_i   : in  std_ulogic;
    rts_o  : out std_ulogic;

    cmd_i  : in  nsl_amba.axi4_stream.master_t;
    cmd_o  : out nsl_amba.axi4_stream.slave_t;
    rsp_i  : in  nsl_amba.axi4_stream.slave_t;
    rsp_o  : out nsl_amba.axi4_stream.master_t
    );
end entity;

architecture rtl of cbor_controller is

  type cmd_state_t is (
    ST_TX_RESET,
    ST_TX_CMD_GET,
    ST_TX_CONFIG_ITEM_GET,
    ST_TX_CONFIG_STR_GET,
    ST_TX_CONFIG_PREP,
    ST_TX_CONFIG_PUT,
    ST_TX_MESSAGE_ROUTE
    );

  type rsp_state_t is (
    ST_RX_RESET,
    ST_RX_IDLE,
    ST_RX_DATA_GET,
    ST_RX_TSTR_HDR_PREP,
    ST_RX_TSTR_HDR_PUT,
    ST_RX_CONFIG_PUT,
    ST_RX_DATA_PUT
    );
  
  type map_parsing_state_t is (
    MAP_NONE,
    MAP_KEY,
    MAP_VAL_FC,
    MAP_VAL_PAR,
    MAP_VAL_BR
    );
  
  type cmd_regs_t is
  record
    state       : cmd_state_t;
    
    map_state   : map_parsing_state_t;
    parser      : nsl_data.cbor.parser_t;
    count       : unsigned(0 to 63);
    len         : unsigned(0 to 63);
    str         : nsl_data.bytestream.byte_string(0 to 8);
    
    parity      : nsl_uart.serdes.parity_t;
    hs          : std_ulogic;
    stop_count  : natural range 1 to 2;
    divisor     : unsigned(0 to 31);
    
    encoded     : nsl_data.bytestream.byte_string(100 downto 0);
    last        : boolean;
  end record;

  signal r, rin: cmd_regs_t;

  type rsp_regs_t is
  record
    state       : rsp_state_t;
    count       : natural range 0 to tstr_max_size_c;
    timeout     : unsigned(0 to 63);
    fifo        : nsl_data.bytestream.byte_string(0 to tstr_max_size_c - 1);
    encoded     : nsl_data.bytestream.byte_string(100 downto 0);
    encoded_len : natural range 0 to 100;
    encoded_i   : natural range 0 to 100;
  end record;

  signal rr, rrin: rsp_regs_t;
  
  signal bnoc_tx_s, bnoc_rx_s: nsl_bnoc.pipe.pipe_bus_t;

  signal req_s, done_s : std_ulogic := '0';
  
begin
  
  reg: process(clock_i, reset_n_i)
  begin
    if rising_edge(clock_i) then
      r <= rin;
      rr <= rrin;
      -- if rin.state = r.state then
      -- else
      --   log_state_change(r => r, rin => rin);
      -- end if;
    end if;
    if reset_n_i = '0' then
      r.state <= ST_TX_RESET;
      rr.state <= ST_RX_RESET;
    end if;
  end process;

  transition_tx: process(clock_i, cmd_i, r)
      variable cbr_encoded : nsl_data.bytestream.byte_stream;
      variable cbr_len : natural;
  begin
    rin <= r;
    case r.state is
      when ST_TX_RESET =>
        rin.state      <= ST_TX_CMD_GET;
        rin.parser     <= nsl_data.cbor.reset;
        rin.parity     <= parity_c;
        rin.hs         <= handshake_active_c;
        rin.stop_count <= stop_count_c;
        rin.map_state  <= MAP_NONE;
        rin.str        <= (others => nsl_data.bytestream.dontcare_byte_c );
        rin.divisor    <= divisor_c;
        rin.last       <= false;
        rin.len        <= (others => '0');
 
      when ST_TX_CMD_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          if cmd_i.valid = '1' then
            rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
          end if;
        else
          if nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_MAP then
            rin.count <= nsl_data.cbor.arg(r.parser, 64);
            rin.map_state <= MAP_KEY;
            rin.state <= ST_TX_CONFIG_ITEM_GET;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_TSTR then
            rin.count <= nsl_data.cbor.arg(r.parser, 64);
            rin.state <= ST_TX_MESSAGE_ROUTE;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_NULL then
            rin.state <= ST_TX_CONFIG_PREP;
          end if;
          rin.parser <= nsl_data.cbor.reset;
        end if;

      when ST_TX_CONFIG_ITEM_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          if cmd_i.valid = '1' then
            rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
          end if;
        else
          if nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_TSTR then
            rin.len <= nsl_data.cbor.arg(r.parser, 64);
            rin.state <= ST_TX_CONFIG_STR_GET;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_POSITIVE then
            if r.map_state = MAP_VAL_BR then
              rin.divisor <= system_clock_c / nsl_data.cbor.arg(r.parser, rin.divisor'length);
              if r.count = 0 then
                rin.map_state <= MAP_NONE;
                rin.state <= ST_TX_CMD_GET;
              else
                rin.state <= ST_TX_CONFIG_ITEM_GET;
              end if;
            end if;
          end if;
          rin.parser <= nsl_data.cbor.reset;
        end if;

      when ST_TX_CONFIG_STR_GET =>
        if r.len /= 0 then
          if cmd_i.valid = '1' then
            rin.len <= r.len - 1;
            -- rin.str <= r.str & nsl_data.bytestream.to_character(to_integer(cmd_i.data(0)));
            rin.str <= nsl_data.bytestream.shift_left(r.str, cmd_i.data(0));
          end if;
        else
          nsl_simulation.logging.log_info("Parsed string is " & nsl_data.bytestream.to_character_string(r.str));
          if r.map_state = MAP_KEY then
            if nsl_data.bytestream.to_character_string(r.str) = "flow-ctrl" then
              rin.map_state <= MAP_VAL_FC;
            elsif nsl_data.bytestream.to_character_string(r.str(3 to 8)) = "parity" then
              rin.map_state <= MAP_VAL_PAR;
            elsif nsl_data.bytestream.to_character_string(r.str) = "baud-rate" then
              rin.map_state <= MAP_VAL_BR;
            end if;

            rin.count <= r.count - 1;

          else

            if r.map_state = MAP_VAL_FC then
              if nsl_data.bytestream.to_character_string(r.str(5 to 8)) = "none" then
                rin.hs <= '0';
              elsif nsl_data.bytestream.to_character_string(r.str(6 to 8)) = "cts" then
                rin.hs <= '1';
              elsif nsl_data.bytestream.to_character_string(r.str(6 to 8)) = "xon" then
                rin.hs <= '0';
              end if;
              
            elsif r.map_state = MAP_VAL_PAR then
              if nsl_data.bytestream.to_character_string(r.str(8 to 8)) = "n" then
                rin.parity <= nsl_uart.serdes.PARITY_NONE;
              elsif nsl_data.bytestream.to_character_string(r.str(8 to 8)) = "e" then
                rin.parity <= nsl_uart.serdes.PARITY_EVEN;
              elsif nsl_data.bytestream.to_character_string(r.str(8 to 8)) = "o" then
                rin.parity <= nsl_uart.serdes.PARITY_ODD ;
              end if;
              
            end if;

          end if;

          rin.str <= (others => nsl_data.bytestream.dontcare_byte_c );
          
          rin.state <= ST_TX_CONFIG_ITEM_GET;
          
          if r.map_state /= MAP_KEY then
            rin.map_state <= MAP_KEY;
            if r.count = 0 then
              rin.map_state <= MAP_NONE;
              rin.state <= ST_TX_CMD_GET;
            end if;
          end if;
          
        end if;

      when ST_TX_CONFIG_PREP =>
        nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_map_hdr(length => 3));
        nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("flow-ctrl"));
        case r.hs is
          when '0' => nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("none"));
          when '1' => nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("cts"));
          when others => null;
        end case;
        nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("parity"));
        case r.parity is
          when nsl_uart.serdes.PARITY_NONE => nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("n"));
          when nsl_uart.serdes.PARITY_EVEN => nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("e"));
          when nsl_uart.serdes.PARITY_ODD  => nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("o"));
        end case;
        nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr("baud-rate"));
        nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_number(to_integer(system_clock_c/r.divisor)) );

        rin.len <= to_unsigned(cbr_encoded.all'length, 64);
        rin.encoded(cbr_encoded.all'length-1 downto 0) <= cbr_encoded.all;
        nsl_data.bytestream.clear(s => cbr_encoded);
                                              
        rin.state <= ST_TX_CONFIG_PUT;

      when ST_TX_CONFIG_PUT =>
        if done_s = '1' then
          rin.state <= ST_TX_CMD_GET;
        end if;

      when ST_TX_MESSAGE_ROUTE =>
        if cmd_i.valid = '1' and bnoc_tx_s.ack.ready = '1' then
          rin.count <= r.count - 1;
          if r.count = 1 then
            rin.state <= ST_TX_CMD_GET;
          end if;
        end if;
    end case;
    
  end process;

  output_tx: process(r, bnoc_tx_s, cmd_i)
  begin
    cmd_o.ready <= '0';
    bnoc_tx_s.req.valid <= '0';
    bnoc_tx_s.req.data <= (others => '-');
    req_s <= '0';
    case r.state is
      when ST_TX_RESET | ST_TX_CONFIG_PREP =>
    
      when ST_TX_CMD_GET | ST_TX_CONFIG_ITEM_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          cmd_o.ready <= '1';
        end if;
      
      when ST_TX_CONFIG_STR_GET =>
        if r.len /= 0 then
          cmd_o.ready <= '1';
        end if;
      
      when ST_TX_CONFIG_PUT =>
        req_s <= '1';
  
      when ST_TX_MESSAGE_ROUTE =>
        cmd_o.ready <= bnoc_tx_s.ack.ready;
        bnoc_tx_s.req.valid <= cmd_i.valid;
        bnoc_tx_s.req.data <= cmd_i.data(0);
    end case;
  end process;

  transition_rx: process(clock_i, bnoc_tx_s, rr)
      variable cbr_encoded : nsl_data.bytestream.byte_stream;
  begin
    case rr.state is
      when ST_RX_RESET =>
        rrin.state <= ST_RX_IDLE;
        rrin.timeout <= 50*divisor_c;
        rrin.encoded_len <= 0;
        rrin.encoded_i <= 0;
        
      when ST_RX_IDLE =>
        rrin.timeout <= timeout_c*divisor_c;
        if bnoc_rx_s.req.valid = '1' then
          rrin.state <= ST_RX_DATA_GET;
          rrin.fifo <= nsl_data.fifo.fifo_shift_data(
            storage => rr.fifo,
            fillness => rr.count,
            min_fill => 0,
            valid => bnoc_rx_s.req.valid = '1', -- push
            data => bnoc_rx_s.req.data,
            ready => false -- no pop
            );
          rrin.count <=  nsl_data.fifo.fifo_shift_fillness(
            storage => rr.fifo,
            fillness => rr.count,
            min_fill => 0,
            valid => bnoc_rx_s.req.valid = '1', -- push
            data => bnoc_rx_s.req.data,
            ready => false -- no pop
            );
        elsif req_s = '1' then
          rrin.state <= ST_RX_CONFIG_PUT;
          rrin.encoded_i <= 0;
        end if;
        
      when ST_RX_DATA_GET =>
        -- nsl_simulation.logging.log_info("ST_RX_DATA_GET");
        rrin.timeout <= rr.timeout - 1;
        if rrin.timeout /= 0 and nsl_data.fifo.fifo_can_push(storage => rr.fifo, fillness => rr.count) then
          rrin.fifo <= nsl_data.fifo.fifo_shift_data(
            storage => rr.fifo,
            fillness => rr.count,
            valid => bnoc_rx_s.req.valid = '1', -- push
            data => bnoc_rx_s.req.data,
            ready => false -- no pop
            );
          rrin.count <=  nsl_data.fifo.fifo_shift_fillness(
            storage => rr.fifo,
            fillness => rr.count,
            min_fill => 0,
            valid => bnoc_rx_s.req.valid = '1', -- push
            data => bnoc_rx_s.req.data,
            ready => false -- no pop
            );
        else
          rrin.state <= ST_RX_TSTR_HDR_PREP;
        end if;
     
      when ST_RX_TSTR_HDR_PREP =>
        nsl_data.bytestream.write(s => cbr_encoded, d => nsl_data.cbor.cbor_tstr_hdr(length => rr.count));
        rrin.encoded_len <= cbr_encoded.all'length;
        rrin.encoded(cbr_encoded.all'length-1 downto 0) <= cbr_encoded.all;
        nsl_data.bytestream.clear(s => cbr_encoded);
        rrin.state <= ST_RX_TSTR_HDR_PUT;

      when ST_RX_TSTR_HDR_PUT =>
        if rsp_i.ready = '1' then
          if rr.encoded_len - 1 = rr.encoded_i then
            rrin.state <= ST_RX_DATA_PUT;
            rrin.encoded_len <= 0;
            rrin.encoded_i <= 0;
          else
            rrin.encoded_i <= (rr.encoded_i + 1);
          end if;
        end if;

      when ST_RX_CONFIG_PUT =>
        if rsp_i.ready = '1' then
          if to_integer(r.len) - 1 = rr.encoded_i then
            rrin.state <= ST_RX_IDLE;
            rrin.encoded_i <= 0;
          else
            rrin.encoded_i <= (rr.encoded_i + 1);
          end if;
        end if;
                  
      when ST_RX_DATA_PUT =>
        if nsl_data.fifo.fifo_can_pop(storage => rr.fifo, fillness => rr.count) then
          rrin.fifo <= nsl_data.fifo.fifo_shift_data(
            storage => rr.fifo,
            fillness => rr.count,
            valid => bnoc_rx_s.req.valid = '1', -- push if valid
            data => bnoc_rx_s.req.data,
            ready => rsp_i.ready = '1' -- pop if ready
            );
          rrin.count <=  nsl_data.fifo.fifo_shift_fillness(
            storage => rr.fifo,
            fillness => rr.count,
            min_fill => 0,
            valid => bnoc_rx_s.req.valid = '1', -- push if valid
            data => bnoc_rx_s.req.data,
            ready => rsp_i.ready = '1' -- pop if ready
            );
        else
          rrin.state <= ST_RX_IDLE;
        end if;
          
    end case;
  end process;

  output_rx: process(rr)
  begin
    bnoc_rx_s.ack.ready <= '0';
    rsp_o <= nsl_amba.axi4_stream.transfer_defaults( cfg => axi_s_cfg_c);
      done_s <= '0';
      
    case rr.state is
      when ST_RX_RESET =>
        
      when ST_RX_IDLE => 
        bnoc_rx_s.ack.ready <= '1';
        
      when ST_RX_DATA_GET =>
        bnoc_rx_s.ack.ready <= '1';
        
      when ST_RX_TSTR_HDR_PREP =>
        
      when ST_RX_TSTR_HDR_PUT =>
        rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => rr.encoded(rr.encoded_len - rr.encoded_i - 1 downto rr.encoded_len - rr.encoded_i - 1), last => false);

      when ST_RX_CONFIG_PUT =>
        done_s <= '0';
        rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => r.encoded(to_integer(r.len) - rr.encoded_i - 1 downto to_integer(r.len) - rr.encoded_i - 1), last => rr.encoded_i = to_integer(r.len) - 1);
        if to_integer(r.len) - 1 = rr.encoded_i then
          done_s <= '1';
        end if;

      when ST_RX_DATA_PUT =>
        if nsl_data.fifo.fifo_can_pop(storage => rr.fifo, fillness => rr.count) then
          bnoc_rx_s.ack.ready <= '1';
          rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(rr.fifo(0)), last => rr.count = 1);
        end if;
        
    end case;
  end process;
  
  uart8: nsl_uart.transactor.uart8
    generic map(
      stop_count_c => r.stop_count,
      parity_c => r.parity,
      handshake_active_c => r.hs
    )
    port map(
      reset_n_i => reset_n_i,
      clock_i => clock_i,

      divisor_i  => r.divisor,
            
      tx_o   => tx_o,
      cts_i  => cts_i,
      rx_i   => rx_i,
      rts_o  => rts_o,

      -- Resync/deglitched raw signals
      cts_o => open,
      rx_o  => open,

      tx_data_i => bnoc_tx_s.req,
      tx_data_o => bnoc_tx_s.ack,
      rx_data_i => bnoc_rx_s.ack,
      rx_data_o => bnoc_rx_s.req,

      parity_error_o => open,
      break_o     => open
    );

end architecture;
