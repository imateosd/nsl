library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_uart, nsl_clocking, nsl_amba, nsl_data, nsl_simulation;
use nsl_data.cbor.all;

entity cbor_controller_no_bnoc is
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

architecture rtl of cbor_controller_no_bnoc is

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

  type state_t is (
    ST_RESET,
    ST_IDLE,
    ST_CMD_GET,
    ST_CONFIG_ITEM_GET,
    ST_CONFIG_STR_GET,
    ST_CONFIG_PREP,
    ST_CONFIG_PUT,
    ST_CONFIG_OK_PUT,
    ST_DATA_TX,
    ST_DATA_TX_OK_PUT,
    ST_FLUSH_BSTR_HDR_PREP,
    ST_FLUSH_BSTR_HDR_PUT,
    ST_FLUSH_DATA_PUT
    );

  type map_parsing_state_t is (
    MAP_NONE,
    MAP_KEY,
    MAP_VAL_FC,
    MAP_VAL_PAR,
    MAP_VAL_BR
    );

  type regs_t is
  record
    state       : state_t;

    -- CBOR parser / map state
    map_state   : map_parsing_state_t;
    parser      : nsl_data.cbor.parser_t;
    count       : natural range 0 to 63;
    len         : natural range 0 to 1023;
    str         : nsl_data.bytestream.byte_string(0 to 8);

    -- UART config (persistent)
    parity      : nsl_uart.serdes.parity_t;
    hs          : std_ulogic;
    stop_count  : natural range 1 to 2;
    divisor     : unsigned(31 downto 0);

    -- Encoded buffer + last flag (shared, one use at a time)
    encoded     : nsl_amba.axi4_stream.buffer_t;
    last        : boolean;

    -- RX FIFO
    fifo        : nsl_data.bytestream.byte_string(0 to bstr_max_size_c - 1);
    fifo_count  : natural range 0 to bstr_max_size_c;
    timeout     : unsigned(63 downto 0);
    flush_needed : boolean;

    -- Flush drain counter
    flush_count : natural range 0 to bstr_max_size_c;

    -- Deferred OK: send F5 after FIFO flush completes
    pending_ok : boolean;
  end record;

  signal r, rin: regs_t;

  -- Direct UART TX/RX byte-level signals (replace BNOC pipes)
  signal uart_tx_data_s  : std_ulogic_vector(7 downto 0);
  signal uart_tx_valid_s : std_ulogic;
  signal uart_tx_ready_s : std_ulogic;
  signal uart_rx_data_s  : std_ulogic_vector(7 downto 0);
  signal uart_rx_valid_s : std_ulogic;
  signal uart_rx_ready_s : std_ulogic;

  -- Resynced external signals
  signal rx_s, cts_s : std_ulogic;

  signal parity_s, stop_count_s : unsigned(1 downto 0);
begin

  reg: process(clock_i, reset_n_i)
  begin
    if rising_edge(clock_i) then
      r <= rin;
    end if;
    if reset_n_i = '0' then
      r.state <= ST_RESET;
    end if;
  end process;

  transition: process(r, cmd_i, rsp_i, uart_rx_valid_s, uart_rx_data_s, uart_tx_ready_s)
    variable v : regs_t;
  begin
    v := r;

    -- Always-on FIFO push (disabled during ST_FLUSH_DATA_PUT which does
    -- combined push+pop)
    if r.state /= ST_FLUSH_DATA_PUT then
      if uart_rx_valid_s = '1'
        and nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count)
      then
        v.fifo := nsl_data.fifo.fifo_shift_data(
          storage => r.fifo,
          fillness => r.fifo_count,
          min_fill => 0,
          valid => true,
          data => uart_rx_data_s,
          ready => false
          );
        v.fifo_count := nsl_data.fifo.fifo_shift_fillness(
          storage => r.fifo,
          fillness => r.fifo_count,
          min_fill => 0,
          valid => true,
          data => uart_rx_data_s,
          ready => false
          );
        v.timeout := timeout_c * divisor_c;
      end if;
    end if;

    -- Timeout/flush_needed management (disabled during entire flush sequence)
    if r.state /= ST_FLUSH_DATA_PUT
      and r.state /= ST_FLUSH_BSTR_HDR_PREP
      and r.state /= ST_FLUSH_BSTR_HDR_PUT
    then
      if not (uart_rx_valid_s = '1'
              and nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count))
      then
        -- No push happened this cycle
        if r.fifo_count > 0 and not r.flush_needed then
          if r.timeout = 0 then
            v.flush_needed := true;
          else
            v.timeout := r.timeout - 1;
          end if;
        end if;

        -- FIFO full also triggers flush
        if r.fifo_count > 0
          and not nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count)
          and not r.flush_needed
        then
          v.flush_needed := true;
        end if;
      end if;
    end if;

    -- FSM case statement
    case r.state is
      when ST_RESET =>
        v.state      := ST_IDLE;
        v.parser     := nsl_data.cbor.reset;
        v.parity     := parity_c;
        v.hs         := handshake_active_c;
        v.stop_count := stop_count_c;
        v.map_state  := MAP_NONE;
        v.str        := (others => nsl_data.bytestream.dontcare_byte_c);
        v.divisor    := divisor_c;
        v.last       := false;
        v.len        := 0;
        v.count      := 0;
        v.fifo_count := 0;
        v.timeout    := timeout_c * divisor_c;
        v.flush_needed := false;
        v.flush_count := 0;
        v.pending_ok := false;

      when ST_IDLE =>
        if r.flush_needed then
          -- Snapshot fifo_count, clear flag, reset timeout, begin flush
          v.flush_count := r.fifo_count;
          v.flush_needed := false;
          v.timeout := timeout_c * divisor_c;
          v.state := ST_FLUSH_BSTR_HDR_PREP;
        elsif r.fifo_count > 0 then
          -- Wait for timeout to trigger flush (data accumulating)
          null;
        elsif r.pending_ok then
          v.pending_ok := false;
          v.state := ST_DATA_TX_OK_PUT;
        elsif cmd_i.valid = '1' then
          v.state := ST_CMD_GET;
        end if;

      when ST_CMD_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          if cmd_i.valid = '1' then
            v.parser := nsl_data.cbor.feed(r.parser, cmd_i.data(0));
          end if;
        else
          if nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_MAP then
            v.count := nsl_data.cbor.arg_int(r.parser);
            v.map_state := MAP_KEY;
            v.state := ST_CONFIG_ITEM_GET;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_BSTR then
            v.count := nsl_data.cbor.arg_int(r.parser);
            v.state := ST_DATA_TX;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_NULL then
            v.count := 0;
            v.state := ST_CONFIG_PREP;
          end if;
          v.parser := nsl_data.cbor.reset;
        end if;

      when ST_CONFIG_ITEM_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          if cmd_i.valid = '1' then
            v.parser := nsl_data.cbor.feed(r.parser, cmd_i.data(0));
          end if;
        else
          if nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_TSTR then
            v.len := nsl_data.cbor.arg_int(r.parser);
            v.state := ST_CONFIG_STR_GET;
          elsif nsl_data.cbor.kind(r.parser) = nsl_data.cbor.KIND_POSITIVE then
            if r.map_state = MAP_VAL_BR then
              v.divisor := baud_to_divisor(nsl_data.cbor.arg(r.parser, 32));
              if r.count = 0 then
                v.map_state := MAP_NONE;
                v.state := ST_CONFIG_OK_PUT;
              else
                v.state := ST_CONFIG_ITEM_GET;
              end if;
            end if;
          end if;
          v.parser := nsl_data.cbor.reset;
        end if;

      when ST_CONFIG_STR_GET =>
        if r.len /= 0 then
          if cmd_i.valid = '1' then
            v.len := r.len - 1;
            v.str := nsl_data.bytestream.shift_left(r.str, cmd_i.data(0));
          end if;
        else
          nsl_simulation.logging.log_info("Parsed string is " & nsl_data.bytestream.to_character_string(r.str));
          if r.map_state = MAP_KEY then
            if nsl_data.bytestream.to_character_string(r.str) = FLOW_CTRL_STR_C then
              v.map_state := MAP_VAL_FC;
            elsif nsl_data.bytestream.to_character_string(r.str(3 to 8)) = PARITY_STR_C then
              v.map_state := MAP_VAL_PAR;
            elsif nsl_data.bytestream.to_character_string(r.str) = BAUD_RATE_STR_C then
              v.map_state := MAP_VAL_BR;
            end if;

            v.count := r.count - 1;

          else

            if r.map_state = MAP_VAL_FC then
              if nsl_data.bytestream.to_character_string(r.str(5 to 8)) = NONE_STR_C then
                v.hs := '0';
              elsif nsl_data.bytestream.to_character_string(r.str(6 to 8)) = CTS_STR_C then
                v.hs := '1';
              elsif nsl_data.bytestream.to_character_string(r.str(6 to 8)) = XON_STR_C then
                v.hs := '0';
              end if;

            elsif r.map_state = MAP_VAL_PAR then
              if nsl_data.bytestream.to_character_string(r.str(8 to 8)) = N_STR_C then
                v.parity := nsl_uart.serdes.PARITY_NONE;
              elsif nsl_data.bytestream.to_character_string(r.str(8 to 8)) = E_STR_C then
                v.parity := nsl_uart.serdes.PARITY_EVEN;
              elsif nsl_data.bytestream.to_character_string(r.str(8 to 8)) = O_STR_C then
                v.parity := nsl_uart.serdes.PARITY_ODD;
              end if;

            end if;

          end if;

          v.str := (others => nsl_data.bytestream.dontcare_byte_c);

          v.state := ST_CONFIG_ITEM_GET;

          if r.map_state /= MAP_KEY then
            v.map_state := MAP_KEY;
            if r.count = 0 then
              v.map_state := MAP_NONE;
              v.state := ST_CONFIG_OK_PUT;
            end if;
          end if;

        end if;

      when ST_CONFIG_PREP =>
        v.count := r.count + 1;
        v.last := false;
        case r.count is
          when 0 =>
            v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_map_hdr(length => to_unsigned(3, 2)));
          when 1 =>
            v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(FLOW_CTRL_STR_C));
          when 2 =>
            case r.hs is
              when '0' => v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(NONE_STR_C));
              when '1' => v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(CTS_STR_C));
              when others => null;
            end case;
          when 3 =>
            v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(PARITY_STR_C));
          when 4 =>
            case r.parity is
              when nsl_uart.serdes.PARITY_NONE => v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(N_STR_C));
              when nsl_uart.serdes.PARITY_EVEN => v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(E_STR_C));
              when nsl_uart.serdes.PARITY_ODD  => v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(O_STR_C));
              when others => null;
            end case;
          when 5 =>
            v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_tstr(BAUD_RATE_STR_C));
          when 6 =>
            v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_number(divisor_to_baud(r.divisor)));
            v.last := true;
          when others =>
            v.count := 0;
            v.state := ST_IDLE;
        end case;
        if r.count < 7 then
          v.state := ST_CONFIG_PUT;
        end if;

      when ST_CONFIG_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          if nsl_amba.axi4_stream.is_last(buffer_cfg_c, r.encoded) then
            v.state := ST_CONFIG_PREP;
          end if;
          v.encoded := nsl_amba.axi4_stream.shift(buffer_cfg_c, r.encoded);
        end if;

      when ST_CONFIG_OK_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          v.state := ST_IDLE;
        end if;

      when ST_DATA_TX =>
        if cmd_i.valid = '1' and uart_tx_ready_s = '1' then
          v.count := r.count - 1;
          if r.count = 1 then
            v.pending_ok := true;
            v.state := ST_IDLE;
          end if;
        end if;

      when ST_DATA_TX_OK_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          v.state := ST_IDLE;
        end if;

      when ST_FLUSH_BSTR_HDR_PREP =>
        v.encoded := nsl_amba.axi4_stream.reset(buffer_cfg_c, nsl_data.cbor.cbor_bstr_hdr(length => to_unsigned(r.flush_count, 12)));
        v.state := ST_FLUSH_BSTR_HDR_PUT;

      when ST_FLUSH_BSTR_HDR_PUT =>
        if nsl_amba.axi4_stream.is_ready(axi_s_cfg_c, rsp_i) then
          if nsl_amba.axi4_stream.is_last(buffer_cfg_c, r.encoded) then
            v.state := ST_FLUSH_DATA_PUT;
          end if;
          v.encoded := nsl_amba.axi4_stream.shift(buffer_cfg_c, r.encoded);
        end if;

      when ST_FLUSH_DATA_PUT =>
        -- Combined push+pop: accept new RX bytes while draining FIFO
        if r.flush_count > 0 then
          if rsp_i.ready = '1' then
            v.fifo := nsl_data.fifo.fifo_shift_data(
              storage => r.fifo,
              fillness => r.fifo_count,
              min_fill => 0,
              valid => uart_rx_valid_s = '1',
              data => uart_rx_data_s,
              ready => true
              );
            v.fifo_count := nsl_data.fifo.fifo_shift_fillness(
              storage => r.fifo,
              fillness => r.fifo_count,
              min_fill => 0,
              valid => uart_rx_valid_s = '1',
              data => uart_rx_data_s,
              ready => true
              );
            v.flush_count := r.flush_count - 1;
          end if;
        else
          v.state := ST_IDLE;
        end if;

    end case;

    rin <= v;
  end process;

  output: process(r, cmd_i, rsp_i, uart_tx_ready_s)
  begin
    cmd_o.ready <= '0';
    rsp_o <= nsl_amba.axi4_stream.transfer_defaults(cfg => axi_s_cfg_c);
    uart_tx_data_s <= (others => '-');
    uart_tx_valid_s <= '0';
    uart_rx_ready_s <= '0';

    case r.state is
      when ST_RESET =>
        null;

      when ST_IDLE =>
        -- Accept RX bytes into FIFO (always-on fill)
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_CMD_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          cmd_o.ready <= '1';
        end if;
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_CONFIG_ITEM_GET =>
        if not nsl_data.cbor.is_done(r.parser) then
          cmd_o.ready <= '1';
        end if;
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_CONFIG_STR_GET =>
        if r.len /= 0 then
          cmd_o.ready <= '1';
        end if;
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_CONFIG_PREP =>
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_CONFIG_PUT =>
        rsp_o <= nsl_amba.axi4_stream.next_beat(cfg => buffer_cfg_c, b => r.encoded, last => nsl_amba.axi4_stream.is_last(cfg => buffer_cfg_c, b => r.encoded) and r.last);
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_CONFIG_OK_PUT =>
        rsp_o <= nsl_amba.axi4_stream.transfer(cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(OP_SUCCESS), last => true);
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_DATA_TX =>
        cmd_o.ready <= uart_tx_ready_s;
        uart_tx_valid_s <= cmd_i.valid;
        uart_tx_data_s <= cmd_i.data(0);
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_DATA_TX_OK_PUT =>
        rsp_o <= nsl_amba.axi4_stream.transfer(cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(OP_SUCCESS), last => true);
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_FLUSH_BSTR_HDR_PREP =>
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_FLUSH_BSTR_HDR_PUT =>
        rsp_o <= nsl_amba.axi4_stream.next_beat(cfg => buffer_cfg_c, b => r.encoded, last => false);
        if nsl_data.fifo.fifo_can_push(storage => r.fifo, fillness => r.fifo_count) then
          uart_rx_ready_s <= '1';
        end if;

      when ST_FLUSH_DATA_PUT =>
        if r.flush_count > 0 then
          -- Drive FIFO head byte onto rsp_o; concurrent push is handled in transition
          uart_rx_ready_s <= '1';
          rsp_o <= nsl_amba.axi4_stream.transfer(cfg => axi_s_cfg_c, bytes => nsl_data.bytestream.from_suv(r.fifo(0)), last => r.flush_count = 1);
        end if;

    end case;
  end process;

  stop_count_s <= to_unsigned(r.stop_count, 2);
  parity_s <= to_unsigned(nsl_uart.serdes.parity_t'pos(r.parity), 2);

  -- Async resynchronizer for RX and CTS inputs
  rx_resync: nsl_clocking.async.async_sampler
    generic map(
      cycle_count_c => 2,
      data_width_c => 2
      )
    port map(
      clock_i => clock_i,
      data_i(0) => rx_i,
      data_i(1) => cts_i,
      data_o(0) => rx_s,
      data_o(1) => cts_s
      );

  -- UART TX (byte-level interface)
  tx_inst: nsl_uart.serdes.uart_tx_no_generics
    generic map(
      bit_count_c => 8
      )
    port map(
      reset_n_i => reset_n_i,
      clock_i => clock_i,

      divisor_i => r.divisor,

      uart_o => tx_o,
      rtr_i => cts_s,

      data_i => uart_tx_data_s,
      ready_o => uart_tx_ready_s,
      valid_i => uart_tx_valid_s,

      stop_count_i => stop_count_s,
      parity_i => parity_s
      );

  -- UART RX (byte-level interface)
  rx_inst: nsl_uart.serdes.uart_rx_no_generics
    generic map(
      bit_count_c => 8
      )
    port map(
      reset_n_i => reset_n_i,
      clock_i => clock_i,

      divisor_i => r.divisor,

      uart_i => rx_s,
      rts_o => rts_o,

      data_o => uart_rx_data_s,
      valid_o => uart_rx_valid_s,
      ready_i => uart_rx_ready_s,

      parity_error_o => open,
      break_o => open,

      stop_count_i => stop_count_s,
      parity_i => parity_s
      );

end architecture;
