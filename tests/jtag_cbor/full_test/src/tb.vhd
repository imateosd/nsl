library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb is
end tb;

library nsl_jtag, nsl_amba, nsl_simulation, nsl_data, nsl_event;


architecture arch of tb is

  constant clock_period : time := 10 ns;
  constant ir_len_c : natural := 8;

  constant cfg_c: nsl_amba.axi4_stream.config_t
    := nsl_amba.axi4_stream.config(1, last => true);

  signal s_cmd           : nsl_amba.axi4_stream.bus_t;
  signal s_rsp           : nsl_amba.axi4_stream.bus_t;
  signal s_rsp_pre       : nsl_amba.axi4_stream.bus_t;

  signal s_tap_ir : std_ulogic_vector(ir_len_c-1 downto 0);
  signal s_tap_reset, s_tap_run, s_tap_dr_capture,
    s_tap_dr_shift, s_tap_dr_update, s_tap_dr_in, s_tap_dr_out : std_ulogic;
  signal ate_o : nsl_jtag.jtag.jtag_ate_o;
  signal ate_i : nsl_jtag.jtag.jtag_ate_i;
  signal tap_o : nsl_jtag.jtag.jtag_tap_o;
  signal tap_i : nsl_jtag.jtag.jtag_tap_i;

  signal s_idcode_out, s_idcode_selected : std_ulogic;
  signal s_bypass_out, s_bypass_selected : std_ulogic;

  signal s_clk, s_resetn : std_ulogic;
  signal s_done : std_ulogic_vector(0 to 0);

  signal   tick_s      : std_ulogic;
  constant tick_divisor: unsigned(7 downto 0) := (others => '1');

  shared variable cmd_q, rsp_q: nsl_amba.axi4_stream.frame_queue_root_t;
  
begin

  ate_i <= nsl_jtag.jtag.to_ate(tap_o);
  tap_i <= nsl_jtag.jtag.to_tap(ate_o);
  
  tap: nsl_jtag.tap.tap_port
  generic map(
    ir_len => ir_len_c
    )
  port map(
    jtag_i => tap_i,
    jtag_o => tap_o,

    default_instruction_i => "00000010",

    ir_o => s_tap_ir,
    ir_out_i => "000000",
    reset_o => s_tap_reset,
    run_o => s_tap_run,
    dr_capture_o => s_tap_dr_capture,
    dr_shift_o => s_tap_dr_shift,
    dr_update_o => s_tap_dr_update,
    dr_tdi_o => s_tap_dr_in,
    dr_tdo_i => s_tap_dr_out
    );

  idcode: nsl_jtag.tap.tap_dr
  generic map(
    ir_len => ir_len_c,
    dr_len => 32
    )
  port map(
    tck_i => tap_i.tck,
    tdi_i => s_tap_dr_in,
    tdo_o => s_idcode_out,

    match_ir_i => "00010001",
    current_ir_i => s_tap_ir,
    active_o => s_idcode_selected,

    dr_capture_i => s_tap_dr_capture,
    dr_shift_i => s_tap_dr_shift,
    value_o => open,
    value_i => x"87654321"
    );

  bypass: nsl_jtag.tap.tap_dr
  generic map(
    ir_len => ir_len_c,
    dr_len => 1
    )
  port map(
    tck_i => tap_i.tck,
    tdi_i => s_tap_dr_in,
    tdo_o => s_bypass_out,

    match_ir_i => "00001111",
    current_ir_i => s_tap_ir,
    active_o => s_bypass_selected,

    dr_capture_i => s_tap_dr_capture,
    dr_shift_i => s_tap_dr_shift,
    value_o => open,
    value_i => "0"
    );

  tdo_gen: process(s_idcode_selected, s_idcode_out,
                   s_bypass_selected, s_bypass_out)
  begin
    s_tap_dr_out <= '-';

    if s_idcode_selected = '1' then
      s_tap_dr_out <= s_idcode_out;
    elsif s_bypass_selected = '1' then
      s_tap_dr_out <= s_bypass_out;
    end if;
  end process;

  dut: nsl_jtag.cbor_transactor.controller
  generic map(
    clock_i_hz_c => 10e7,
    -- tick_i_hz_c  => 10e7/to_integer(tick_divisor),
    axi_s_cfg_c  => cfg_c
    )
  port map(
    clock_i   =>  s_clk,
    reset_n_i => s_resetn,

    tick_i_hz  => 10e7/to_integer(tick_divisor),
    tick_i => tick_s,
    
    cmd_i  => s_cmd.m,
    cmd_o  => s_cmd.s,

    rsp_o  => s_rsp_pre.m,
    rsp_i  => s_rsp_pre.s,
    
    jtag_o => ate_o,
    jtag_i => ate_i
    );

  rsp_pacer : nsl_amba.stream_traffic.axi4_stream_pacer
    generic map(
      config_c => cfg_c,
      probability_c => 0.1
      )
  port map(
    clock_i => s_clk,
    reset_n_i => s_resetn,

    in_i => s_rsp_pre.m,
    in_o => s_rsp_pre.s,

    out_o => s_rsp.m,
    out_i => s_rsp.s
    ); 
  
  driver: nsl_simulation.driver.simulation_driver
  generic map(
    clock_count => 1,
    reset_count => 1,
    done_count => 1
    )
  port map(
    clock_period(0) => 10 ns,
    reset_duration(0) => 30 ns,
    reset_n_o(0) => s_resetn,
    clock_o(0) => s_clk,
    done_i => s_done
    );

  tick_gen: nsl_event.tick.tick_generator_integer
    port map(
      clock_i => s_clk,
      reset_n_i => s_resetn,
      period_m1_i => tick_divisor,
      tick_o => tick_s
      );
 
  stim: process
    variable check_status : boolean := false;
    variable pass_count, fail_count : integer := 0;
  begin

    nsl_amba.axi4_stream.frame_queue_init(cmd_q);
    nsl_amba.axi4_stream.frame_queue_init(rsp_q);

    -- Let FSM reach IDLE
    wait for 50 ns;

    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "======================================",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );
    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "JTAG CBOR TRANSACTOR TEST SUITE",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );
    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "======================================",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"87ca0602e2c9411103e1c81820"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000421436587ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("IDCODE read", check_status, pass_count, fail_count);

    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"87ca0602e2c9411103e1c3c81820"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000421436507ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Minus with no TDI", check_status, pass_count, fail_count);

    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"87ca0602e2c3c9411103e1c81820"),
                                              data2 => nsl_data.bytestream.from_hex("9f590004--------ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Minus with no TDO", check_status, pass_count, fail_count);

    -- Test 4: Run for 3ms (response is empty indefinite array 9fff)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"81CB03"),
                                              data2 => nsl_data.bytestream.from_suv(x"9fff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000 + 4 ms,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Run for 3ms", check_status, pass_count, fail_count);
   
    -- nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Testing Chain enum", color => nsl_simulation.logging.LOG_COLOR_BLUE);
    -- nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
    --                                           root_slave  => rsp_q,
    --                                           data1 => nsl_data.bytestream.from_suv(x"8ae3ca183201e1440355ffc0c8190200e2440355ffc0c819020001"),
    --                                           -- data2 => nsl_data.bytestream.from_suv(x"9f4487654321ff"),
    --                                           data2 => nsl_data.bytestream.from_suv(x"9f4400000000ff"),
    --                                          dt      => clock_period,
    --                                           timeout => clock_period*2000000);
    -- nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "==================================================================================== #1 CMD-RSP CHECK PASSED" & LF, color => nsl_simulation.logging.LOG_COLOR_GREEN);

    -- Test 5 (Large shift)
    -- nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
    --                                           root_slave  => rsp_q,
    --                                           data1 => nsl_data.bytestream.from_suv(x"9fc95900ffE8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8E8ff"),
    --                                           data2 => nsl_data.bytestream.from_suv(x"9fff"),
    --                                           check_status => check_status,
    --                                           dt      => clock_period,
    --                                           timeout => clock_period*2000000 + 4 ms,
    --                                           sev     => warning);
    -- nsl_simulation.logging.log_test_result("Large shift operation", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 6: Read 16 bits of IDCODE (similar to test 1 but fewer bits)
    -- Command: reset(6), run(2), ir-capture, shift-no-tdo(0x11=IDCODE), run(3), dr-capture, shift_cycles(16)
    -- 87 = array(7), ca06 = reset(6), 02 = run(2), e2 = ir-capture, c94111 = shift-no-tdo(0x11),
    -- 03 = run(3), e1 = dr-capture, c810 = shift_cycles(16)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"87ca0602e2c9411103e1c810"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f5900022143ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("IDCODE read (16 bits)", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 7: BYPASS mode
    -- Command: reset(6), run(2), ir-capture, shift-no-tdo(0x0F=BYPASS), run(3), dr-capture, shift_cycles(8)
    -- BYPASS is a 1-bit register, shift 8 bits of zeros (shift_cycles with no TDI data)
    -- 87 = array(7), ca06 = reset(6), 02 = run(2), e2 = ir-capture, c9410f = shift-no-tdo(0x0F),
    -- 03 = run(3), e1 = dr-capture, c808 = shift_cycles(8)
    -- Response: bstr with 1 byte = 0x00 (shifted zeros through BYPASS)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"87ca0602e2c9410f03e1c808"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000100ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("BYPASS mode (8 cycles)", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 8: Shift minus-7 (#6.7) - shift only 1 bit from a byte
    -- After BYPASS is still loaded, do dr-capture, then shift 1 bit
    -- c7 = tag7 (minus-7), 41aa = bstr(1 byte: 0xaa), only 1 bit shifted (8-7=1)
    -- 83 = array(3), e1 = dr-capture, c741aa = minus-7(bstr(0xaa)), 00 = run(0)
    -- Response: bstr with 1 byte, TDO depends on BYPASS state
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83e1c741aa00"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000154ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-7 (1 bit)", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 9: Shift minus-4 (#6.4) - shift 4 bits from a byte
    -- c4 = tag4 (minus-4), 41aa = bstr(1 byte: 0xaa), shifts 4 bits (8-4=4)
    -- 83 = array(3), e1 = dr-capture, c441aa = minus-4(bstr(0xaa)), 00 = run(0)
    -- Response: bstr with 1 byte, TDO depends on BYPASS state
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83e1c441aa00"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000104ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-4 (4 bits)", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 10: IR scan with TDO capture (not using shift-no-tdo)
    -- Reset, run, ir-capture, shift IR with TDO capture (not c9 tagged)
    -- 85 = array(5), ca06 = reset(6), 02 = run(2), e2 = ir-capture, 4111 = bstr(0x11), 00 = run(0)
    -- Response: bstr with captured IR output = 0x01 (default IR value after reset is 0x02,
    -- shifted out LSB first during IR scan, giving captured value 0x01)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"85ca0602e2411100"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000101ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("IR scan with TDO capture", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 11: Extended reset cycles (larger count)
    -- 82 = array(2), ca1864 = reset(100 cycles), 00 = run(0)
    -- Response: empty (9fff)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"82ca186400"),
                                              data2 => nsl_data.bytestream.from_suv(x"9fff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Extended reset (100 cycles)", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 12: Back-to-back DR shifts (multiple shifts in one command)
    -- Reset, run, load IDCODE, run, dr-capture, shift 8 bits x3, run
    -- 8a = array(10), ca06 = reset(6), 02 = run(2), e2 = ir-capture, c94111 = shift-no-tdo(0x11),
    -- 03 = run(3), e1 = dr-capture, c808 = shift_cycles(8), c808, c808, 00 = run(0)
    -- Response: 3 bstr entries (each 1 byte with 2-byte length encoding)
    -- IDCODE = 0x87654321, shifted LSB first: 0x21, 0x43, 0x65
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"8aca0602e2c9411103e1c808c808c80800"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f590001215900014359000165ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Back-to-back DR shifts", check_status, pass_count, fail_count);

    wait for 5 ms;

    nsl_simulation.logging.log_test_suite_summary("JTAG CBOR TRANSACTOR TESTS", pass_count, fail_count);

    if fail_count > 0 then
      nsl_simulation.control.terminate(1);
    else
      nsl_simulation.control.terminate(0);
    end if;
  end process;

  cmd_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Going to run frame_queue_master", color => nsl_simulation.logging.LOG_COLOR_MAGENTA);
    nsl_amba.axi4_stream.frame_queue_master(cfg => cfg_c, root => cmd_q, clock => s_clk,
                                            stream_i => s_cmd.s, stream_o => s_cmd.m, dt => clock_period); --, timeout => 200000 ms);
  end process;
  
  rsp_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Going to run frame_queue_slave", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_slave(cfg => cfg_c, root => rsp_q, clock => s_clk,
                                           stream_i => s_rsp.m, stream_o => s_rsp.s, dt => clock_period);   
  end process;

end architecture;
