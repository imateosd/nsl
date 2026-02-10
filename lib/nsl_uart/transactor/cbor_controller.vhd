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
    divisor_c          : unsigned(31 downto 0);
    timeout_c          : unsigned(31 downto 0);
    bstr_max_size_c    : natural range 0 to 511
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

  constant OP_SUCCESS : std_ulogic_vector(7 downto 0) := X"F5";
  constant OP_FAILURE : std_ulogic_vector(7 downto 0) := X"F4";
  
  constant cbr_hdr_max_size_c : natural := 9;
  constant buffer_cfg_c       : nsl_amba.axi4_stream.buffer_config_t := nsl_amba.axi4_stream.buffer_config(axi_s_cfg_c, 5*cbr_hdr_max_size_c);

  constant FLOW_CTRL_STR_C : string := "flow-ctrl";
  constant PARITY_STR_C    : string := "parity";
  constant BAUD_RATE_STR_C : string := "baud-rate";
  constant NONE_STR_C      : string := "none";
  constant CTS_STR_C       : string := "cts";
  constant XON_STR_C       : string := "xon";
  constant N_STR_C         : string := "n";
  constant E_STR_C         : string := "e";
  constant O_STR_C         : string := "o";
  
  constant DIV_300     : unsigned := to_unsigned(system_clock_c / 300,     32);
  constant DIV_1200    : unsigned := to_unsigned(system_clock_c / 1200,    32);
  constant DIV_2400    : unsigned := to_unsigned(system_clock_c / 2400,    32);
  constant DIV_4800    : unsigned := to_unsigned(system_clock_c / 4800,    32);
  constant DIV_9600    : unsigned := to_unsigned(system_clock_c / 9600,    32);
  constant DIV_19200   : unsigned := to_unsigned(system_clock_c / 19200,   32);
  constant DIV_38400   : unsigned := to_unsigned(system_clock_c / 38400,   32);
  constant DIV_57600   : unsigned := to_unsigned(system_clock_c / 57600,   32);
  constant DIV_115200  : unsigned := to_unsigned(system_clock_c / 115200,  32);
  constant DIV_230400  : unsigned := to_unsigned(system_clock_c / 230400,  32);
  constant DIV_460800  : unsigned := to_unsigned(system_clock_c / 460800,  32);
  constant DIV_921600  : unsigned := to_unsigned(system_clock_c / 921600,  32);
  constant DIV_1000000 : unsigned := to_unsigned(system_clock_c / 1000000, 32);
  constant DIV_2000000 : unsigned := to_unsigned(system_clock_c / 2000000, 32);
  constant DIV_3000000 : unsigned := to_unsigned(system_clock_c / 3000000, 32);

  -- Lookup table function to convert baud rate to divisor without runtime division
  -- Supports common baud rates; unknown rates default to 115200
  function baud_to_divisor(baud_rate : unsigned(31 downto 0)) return unsigned is
  begin
    case to_integer(baud_rate) is
      when 300     => return DIV_300;
      when 1200    => return DIV_1200;
      when 2400    => return DIV_2400;
      when 4800    => return DIV_4800;
      when 9600    => return DIV_9600;
      when 19200   => return DIV_19200;
      when 38400   => return DIV_38400;
      when 57600   => return DIV_57600;
      when 115200  => return DIV_115200;
      when 230400  => return DIV_230400;
      when 460800  => return DIV_460800;
      when 921600  => return DIV_921600;
      when 1000000 => return DIV_1000000;
      when 2000000 => return DIV_2000000;
      when 3000000 => return DIV_3000000;
      when others  => return DIV_115200;
    end case;
  end function;

  -- Lookup table function to convert baud rate to divisor without runtime division
  -- Supports common baud rates; unknown rates default to 115200
  function divisor_to_baud(divisor : unsigned(31 downto 0)) return unsigned is
  begin
    if    divisor = DIV_300     then return to_unsigned(300,     32);
    elsif divisor = DIV_1200    then return to_unsigned(1200,    32);
    elsif divisor = DIV_2400    then return to_unsigned(2400,    32);
    elsif divisor = DIV_4800    then return to_unsigned(4800,    32);
    elsif divisor = DIV_9600    then return to_unsigned(9600,    32);
    elsif divisor = DIV_19200   then return to_unsigned(19200,   32);
    elsif divisor = DIV_38400   then return to_unsigned(38400,   32);
    elsif divisor = DIV_57600   then return to_unsigned(57600,   32);
    elsif divisor = DIV_115200  then return to_unsigned(115200,  32);
    elsif divisor = DIV_230400  then return to_unsigned(230400,  32);
    elsif divisor = DIV_460800  then return to_unsigned(460800,  32);
    elsif divisor = DIV_921600  then return to_unsigned(921600,  32);
    elsif divisor = DIV_1000000 then return to_unsigned(1000000, 32);
    elsif divisor = DIV_2000000 then return to_unsigned(2000000, 32);
    elsif divisor = DIV_3000000 then return to_unsigned(3000000, 32);
    else
      return to_unsigned(115200, 32);
    end if;
  end function;

  type cmd_state_t is (
    ST_TX_RESET,
    ST_TX_CMD_GET,
    ST_TX_CONFIG_ITEM_GET,
    ST_TX_CONFIG_STR_GET,
    ST_TX_CONFIG_PREP,
    ST_TX_CONFIG_PUT,
    ST_TX_STATE_PUT,
    ST_TX_MESSAGE_ROUTE,
    ST_TX_MESSAGE_DONE
    );

  type rsp_state_t is (
    ST_RX_RESET,
    ST_RX_IDLE,
    ST_RX_DATA_GET,
    ST_RX_BSTR_HDR_PREP,
    ST_RX_BSTR_HDR_PUT,
    ST_RX_CONFIG_PUT,
    ST_RX_DATA_PUT,
    ST_RX_OK_PUT,
    ST_RX_FAIL_PUT
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
    count       : natural range 0 to 63;
    len         : natural range 0 to 1023;
    str         : nsl_data.bytestream.byte_string(0 to 8);
    
    parity      : nsl_uart.serdes.parity_t;
    hs          : std_ulogic;
    stop_count  : natural range 1 to 2;
    divisor     : unsigned(31 downto 0);
    
    encoded     : nsl_amba.axi4_stream.buffer_t;
    last        : boolean;
  end record;

  signal r, rin: cmd_regs_t;

  type rsp_regs_t is
  record
    state       : rsp_state_t;
    count       : natural range 0 to bstr_max_size_c;
    timeout     : unsigned(0 to 63);
    fifo        : nsl_data.bytestream.byte_string(0 to bstr_max_size_c - 1);
    encoded     : nsl_amba.axi4_stream.buffer_t;
    last        : boolean;
  end record;

  signal rr, rrin: rsp_regs_t;
  
  signal bnoc_tx_s, bnoc_rx_s: nsl_bnoc.pipe.pipe_bus_t;

  signal req_s, req_ok_s, req_fail_s, done_s : std_ulogic := '0';

  signal parity_s, stop_count_s : unsigned(1 downto 0);
begin
  
  reg: process(clock_i, reset_n_i)
  begin
    if rising_edge(clock_i) then
      r <= rin;
      rr <= rrin;
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
        rin.len        <= 0;
        rin.count      <= 0;
 
      when ST_TX_CMD_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          if cmd_i.valid = '1' then
            rin.parser <= nsl_data.cbor.feed(r.parser, cmd_i.data(0));
          end if;
        else
          if nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_MAP then
            rin.count <= nsl_data.cbor.arg_int(r.parser);
            rin.map_state <= MAP_KEY;
            rin.state <= ST_TX_CONFIG_ITEM_GET;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_BSTR then
            rin.count <= nsl_data.cbor.arg_int(r.parser);
            rin.state <= ST_TX_MESSAGE_ROUTE;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_NULL then
            rin.count <= 0;
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
            rin.len <= nsl_data.cbor.arg_int(r.parser);
            rin.state <= ST_TX_CONFIG_STR_GET;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_POSITIVE then
            if r.map_state = MAP_VAL_BR then
              rin.divisor <= baud_to_divisor(nsl_data.cbor.arg(r.parser, 32));
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
            if nsl_data.bytestream.to_character_string(r.str) = FLOW_CTRL_STR_C then
              rin.map_state <= MAP_VAL_FC;
            elsif nsl_data.bytestream.to_character_string(r.str(3 to 8)) = PARITY_STR_C then
              rin.map_state <= MAP_VAL_PAR;
            elsif nsl_data.bytestream.to_character_string(r.str) = BAUD_RATE_STR_C then
              rin.map_state <= MAP_VAL_BR;
            end if;

            rin.count <= r.count - 1;

          else

            if r.map_state = MAP_VAL_FC then
              if nsl_data.bytestream.to_character_string(r.str(5 to 8)) = NONE_STR_C then
                rin.hs <= '0';
              elsif nsl_data.bytestream.to_character_string(r.str(6 to 8)) = CTS_STR_C then
                rin.hs <= '1';
              elsif nsl_data.bytestream.to_character_string(r.str(6 to 8)) = XON_STR_C then
                rin.hs <= '0';
              end if;
              
            elsif r.map_state = MAP_VAL_PAR then
              if nsl_data.bytestream.to_character_string(r.str(8 to 8)) = N_STR_C then
                rin.parity <= nsl_uart.serdes.PARITY_NONE;
              elsif nsl_data.bytestream.to_character_string(r.str(8 to 8)) = E_STR_C then
                rin.parity <= nsl_uart.serdes.PARITY_EVEN;
              elsif nsl_data.bytestream.to_character_string(r.str(8 to 8)) = O_STR_C then
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
              rin.state <= ST_TX_STATE_PUT;
              -- rin.state <= ST_TX_CMD_GET;
            end if;
          end if;
          
        end if;

      when ST_TX_CONFIG_PREP =>
        rin.count <= r.count + 1;
        rin.last <= false;
        case r.count is
          when 0 =>
            rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_map_hdr(length => to_unsigned(3, 2)));
          when 1 =>
            rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(FLOW_CTRL_STR_C));
          when 2 =>
            case r.hs is
              when '0' => rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(NONE_STR_C));
              when '1' => rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(CTS_STR_C));
              when others => null;
            end case;
          when 3 =>
            rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(PARITY_STR_C));
          when 4 =>
            case r.parity is
              when nsl_uart.serdes.PARITY_NONE => rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(N_STR_C));
              when nsl_uart.serdes.PARITY_EVEN => rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(E_STR_C));
              when nsl_uart.serdes.PARITY_ODD  => rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(O_STR_C));
              when others => null;
            end case;
          when 5 =>
            rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(BAUD_RATE_STR_C));
          when 6 =>
            rin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_number(divisor_to_baud(r.divisor)));
            rin.last <= true;
          when others =>
            rin.count <= 0;
            -- rin.state <= ST_TX_STATE_PUT;
            rin.state <= ST_TX_CMD_GET;
        end case;        
        if r.count < 7 then
          rin.state <= ST_TX_CONFIG_PUT;
        end if;
        
      when ST_TX_CONFIG_PUT =>
        if done_s = '1' then
          rin.state <= ST_TX_CONFIG_PREP;
        end if;

      when ST_TX_STATE_PUT => 
        if done_s = '1' then
          rin.state <= ST_TX_CMD_GET;
        end if;

      when ST_TX_MESSAGE_ROUTE =>
        if cmd_i.valid = '1' and bnoc_tx_s.ack.ready = '1' then
          rin.count <= r.count - 1;
          if r.count = 1 then
            rin.state <= ST_TX_MESSAGE_DONE;
          end if;
        end if;

      when ST_TX_MESSAGE_DONE =>
        if done_s = '1' then
          rin.state <= ST_TX_CMD_GET;
        end if;
    end case;
    
  end process;

  output_tx: process(r, bnoc_tx_s, cmd_i, done_s)
  begin
    -- cmd_o <= nsl_amba.axi4_stream.accept(axi_s_cfg_c, false);
    cmd_o.ready <= '0';
    bnoc_tx_s.req.valid <= '0';
    bnoc_tx_s.req.data <= (others => '-');
    req_s <= '0';
    req_ok_s <= '0';
    req_fail_s <= '0';
    
    case r.state is
      when ST_TX_RESET =>

      when ST_TX_CONFIG_PREP =>
                   
      when ST_TX_CMD_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          cmd_o.ready <= '1';
        end if;
        
      when ST_TX_CONFIG_ITEM_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          cmd_o.ready <= '1';
        else
          if r.map_state /= MAP_KEY and r.count = 0 then
            req_ok_s <= '1';
          end if;
        end if;
        
      when ST_TX_CONFIG_STR_GET =>
        if r.len /= 0 then
          cmd_o.ready <= '1';
        end if;
      
      when ST_TX_CONFIG_PUT =>
        if done_s /= '1' then
          req_s <= '1';
        end if;
  
      when ST_TX_MESSAGE_ROUTE =>
        cmd_o.ready <= bnoc_tx_s.ack.ready;
        bnoc_tx_s.req.valid <= cmd_i.valid;
        bnoc_tx_s.req.data <= cmd_i.data(0);
        -- if r.count = 1 then
        --   req_ok_s <= '1';
        -- end if;

      when ST_TX_MESSAGE_DONE =>
        req_ok_s <= '1';

      when ST_TX_STATE_PUT =>
        null;
        
    end case;
  end process;

  transition_rx: process(clock_i, bnoc_tx_s, rr)
  begin
    rrin <= rr;
    
    case rr.state is
      when ST_RX_RESET =>
        rrin.state <= ST_RX_IDLE;
        rrin.timeout <= 50*divisor_c;
        
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
          rrin.encoded <= r.encoded;
          rrin.last  <= r.last;
          rrin.state <= ST_RX_CONFIG_PUT;
        elsif req_ok_s = '1' then
          rrin.state <= ST_RX_OK_PUT;
        elsif req_fail_s = '1' then
          rrin.state <= ST_RX_FAIL_PUT;
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
          rrin.state <= ST_RX_BSTR_HDR_PREP;
        end if;
     
      when ST_RX_BSTR_HDR_PREP =>
        rrin.encoded <= nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_bstr_hdr(length => to_unsigned(rr.count, 12)) );  
        rrin.state <= ST_RX_BSTR_HDR_PUT;

      when ST_RX_BSTR_HDR_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          if nsl_amba.axi4_stream.is_last(buffer_cfg_c, rr.encoded) then
            rrin.state <= ST_RX_DATA_PUT;
          end if;
          rrin.encoded <= nsl_amba.axi4_stream.shift(buffer_cfg_c, rr.encoded);
        end if;

      when ST_RX_CONFIG_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          if nsl_amba.axi4_stream.is_last(buffer_cfg_c, rr.encoded) then
            rrin.state <= ST_RX_IDLE;
          end if;
          rrin.encoded <= nsl_amba.axi4_stream.shift(buffer_cfg_c, rr.encoded);
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

      when ST_RX_OK_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          rrin.state <= ST_RX_IDLE;
        end if;

      when ST_RX_FAIL_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
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
        
      when ST_RX_BSTR_HDR_PREP =>
        
      when ST_RX_BSTR_HDR_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          rsp_o <= nsl_amba.axi4_stream.next_beat(cfg => buffer_cfg_c, b => rr.encoded, last => false);
        end if;

      when ST_RX_CONFIG_PUT =>
        rsp_o <= nsl_amba.axi4_stream.next_beat(cfg => buffer_cfg_c, b => rr.encoded, last => nsl_amba.axi4_stream.is_last(cfg => buffer_cfg_c, b => rr.encoded) and rr.last);
        if nsl_amba.axi4_stream.is_last(cfg => buffer_cfg_c, b => rr.encoded) and nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          done_s <= '1';
        end if;

      when ST_RX_DATA_PUT =>
        if nsl_data.fifo.fifo_can_pop(storage => rr.fifo, fillness => rr.count) then
          bnoc_rx_s.ack.ready <= '1';
          rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(rr.fifo(0)), last => rr.count = 1);
        end if;

      when ST_RX_OK_PUT =>
        rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(OP_SUCCESS), last => true);
        done_s <= '1';

      when ST_RX_FAIL_PUT =>
        rsp_o <= nsl_amba.axi4_stream.transfer( cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(OP_FAILURE), last => true);
        done_s <= '1';
        
    end case;
  end process;

  stop_count_s <= to_unsigned(r.stop_count, 2);
  parity_s <= to_unsigned(nsl_uart.serdes.parity_t'pos(r.parity), 2);
  
  uart8: nsl_uart.transactor.uart8_no_generics
    port map(
      reset_n_i => reset_n_i,
      clock_i => clock_i,

      divisor_i  => r.divisor,
            
      tx_o   => tx_o,
      -- cts_i  => cts_i,
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
      break_o     => open,

      stop_count => stop_count_s,
      parity => parity_s
    );

end architecture;
