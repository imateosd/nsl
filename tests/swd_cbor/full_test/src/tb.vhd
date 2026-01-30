library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb is
end tb;

library nsl_coresight, nsl_amba, nsl_simulation, nsl_data, nsl_event;

architecture arch of tb is
  constant clock_period : time := 10 ns;
  constant cfg_c: nsl_amba.axi4_stream.config_t
    := nsl_amba.axi4_stream.config(1, last => true);
  constant tick_divisor: unsigned(7 downto 0) := (others => '1');

  signal s_clk, s_resetn : std_ulogic;
  
  signal s_cmd        : nsl_amba.axi4_stream.bus_t;
  signal s_rsp        : nsl_amba.axi4_stream.bus_t;
  signal s_rsp_pre    : nsl_amba.axi4_stream.bus_t;

  signal s_master_swd : nsl_coresight.swd.swd_master_bus;
  signal s_slave_swd  : nsl_coresight.swd.swd_slave_bus;

  signal s_done          : std_ulogic_vector(0 to 0);

  signal tick_i_hz_s : natural := 1;
  signal tick_s : std_ulogic;
 
  shared variable cmd_q, rsp_q: nsl_amba.axi4_stream.frame_queue_root_t;
  
begin
  
  s_slave_swd.i <= nsl_coresight.swd.to_slave(s_master_swd.o);
  s_master_swd.i <= nsl_coresight.swd.to_master(s_slave_swd.o);
  
  target: block is
    constant clock_period_c : time := 20 ns;
    signal clock_s : std_ulogic;
    signal reset_n_s : std_ulogic;

    constant dp_idr_c : unsigned := x"04567e11";

    signal dapbus_gen, dapbus_memap : nsl_coresight.dapbus.dapbus_bus;
    constant axi_cfg_c : nsl_amba.axi4_mm.config_t := nsl_amba.axi4_mm.config(address_width => 32, data_bus_width => 32);
    signal axi_s : nsl_amba.axi4_mm.bus_t;
    signal ctrl, ctrl_w, stat :std_ulogic_vector(31 downto 0);
  begin
    dp: nsl_coresight.dp.swdp_sync
      generic map(
        idr => dp_idr_c
        )
      port map(
        ref_clock_i => clock_s,
        ref_reset_n_i => reset_n_s,

        swd_i => s_slave_swd.i,
        swd_o => s_slave_swd.o,

        dap_o => dapbus_gen.ms,
        dap_i => dapbus_gen.sm,

        ctrl_o => ctrl,
        stat_i => stat,
        abort_o => open
        );

    stat_update: process(ctrl)
    begin
      stat <= ctrl;
      stat(27) <= ctrl(26);
      stat(29) <= ctrl(28);
      stat(31) <= ctrl(30);
    end process;
    
    interconnect: nsl_coresight.dapbus.dapbus_interconnect
      generic map(
        access_port_count => 1
        )
      port map(
        s_i => dapbus_gen.ms,
        s_o => dapbus_gen.sm,

        m_i(0) => dapbus_memap.sm,
        m_o(0) => dapbus_memap.ms
        );

    mem_ap: nsl_coresight.ap.ap_axi4_lite
      generic map(
        rom_base => x"00000000",
        config_c => axi_cfg_c,
        idr => x"01234e11"
        )
      port map(
        clk_i => clock_s,
        reset_n_i => reset_n_s,

        dbgen_i => ctrl(28),
        spiden_i => '1',

        dap_i => dapbus_memap.ms,
        dap_o => dapbus_memap.sm,

        axi_o => axi_s.m,
        axi_i => axi_s.s
        );

    mem: nsl_amba.ram.axi4_mm_lite_ram
      generic map (
        byte_size_l2_c => 12,
        config_c => axi_cfg_c
        )
      port map (
        clock_i => clock_s,
        reset_n_i => reset_n_s,

        axi_i => axi_s.m,
        axi_o => axi_s.s
        );

    driver_target: nsl_simulation.driver.simulation_driver
      generic map(
        clock_count => 1,
        reset_count => 1,
        done_count => s_done'length
        )
      port map(
        clock_period(0) => clock_period_c,
        reset_duration(0) => 42 ns,
        reset_n_o(0) => reset_n_s,
        clock_o(0) => clock_s,
        done_i => s_done
        );
  end block;

  tick_i_hz_s <= 10e7/to_integer(tick_divisor);
  
  dut: nsl_coresight.cbor_transactor.controller
    generic map(
      clock_i_hz_c => 10e7,
      axi_s_cfg_c  => cfg_c
      )
    port map(
      clock_i   =>  s_clk,
      reset_n_i => s_resetn,

      tick_i_hz => tick_i_hz_s,
      tick_i    => tick_s,
      
      cmd_i  => s_cmd.m,
      cmd_o  => s_cmd.s,

      -- rsp_o  => s_rsp_pre.m,
      -- rsp_i  => s_rsp_pre.s,

      rsp_o  => s_rsp.m,
      rsp_i  => s_rsp.s,
      
      swd_i  => s_master_swd.i,
      swd_o  => s_master_swd.o
      );

  -- rsp_pacer : nsl_amba.stream_traffic.axi4_stream_pacer
  --   generic map(
  --     config_c => cfg_c,
  --     probability_c => 0.99
  --     )
  --   port map(
  --     clock_i => s_clk,
  --     reset_n_i => s_resetn,

  --     in_i => s_rsp_pre.m,
  --     in_o => s_rsp_pre.s,

  --     out_o => s_rsp.m,
  --     out_i => s_rsp.s
  --     ); 
  
  driver: nsl_simulation.driver.simulation_driver
    generic map(
      clock_count => 1,
      reset_count => 1,
      done_count => s_done'length
      )
    port map(
      clock_period(0) => clock_period,
      reset_duration(0) => 42 ns,
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
    -- Let FSM reach IDLE
    wait for 50 ns;

    nsl_amba.axi4_stream.frame_queue_init(cmd_q);
    nsl_amba.axi4_stream.frame_queue_init(rsp_q);

    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "======================================",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );
    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "SWD CBOR TRANSACTOR TEST SUITE",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );
    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "======================================",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );

    -- Test 1: JTAG-to-SWD sequence (true = f5)
    -- Command: [true] = 81 f5
    -- Response: empty indefinite array (no response for protocol switch)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"81f5"),
                                              data2       => nsl_data.bytestream.from_suv(x"9fff"),
                                              check_status => check_status,
                                              dt          => clock_period,
                                              timeout     => clock_period*2000000,
                                              sev         => warning);
    nsl_simulation.logging.log_test_result("JTAG-to-SWD sequence", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 2: Run command (50 cycles) for line reset
    -- Command: [50] = 81 18 32
    -- Response: empty (no response for run)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"811832"),
                                              data2       => nsl_data.bytestream.from_suv(x"9fff"),
                                              check_status => check_status,
                                              dt          => clock_period,
                                              timeout     => clock_period*2000000,
                                              sev         => warning);
    nsl_simulation.logging.log_test_result("Run 50 cycles", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 3: Read DP IDR (reg0, 1 word)
    -- Command: [#6.0(1)] = 81 c0 01
    -- Response: [array(3): [indef_bstr[bstr(4 bytes IDR)], offset=0, status=1(OK)]]
    -- DP IDR = 0x04567e11
    -- Format: 9f 83 5f 44 <4 bytes> ff 18 00 01 ff
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"81c001"),
                                              data2       => nsl_data.bytestream.from_suv(x"9f835f4404567e11ff180001ff"),
                                              check_status => check_status,
                                              dt          => clock_period,
                                              timeout     => clock_period*2000000,
                                              sev         => warning);
    nsl_simulation.logging.log_test_result("Read DP IDR (reg0)", check_status, pass_count, fail_count);

    wait for 100 us;

    -- Test 4: Full SWD initialization sequence
    -- Combined: turnaround(1), run(10), write CTRL/STAT, run(8), write SELECT, run(8)
    -- 86 = array(6)
    -- c8 01 = turnaround(1)
    -- 0a = run(10)
    -- c1 44 00 00 00 50 = DP reg1 write (CTRL/STAT = 0x50000000 - enable debug power)
    -- 08 = run(8)
    -- c2 44 f0 00 00 00 = DP reg2 write (SELECT = 0x000000F0 - AP0 bank F)
    -- 08 = run(8)
    -- Responses: write response x2 = 82 18 01 01, 82 18 01 01 (offset=1, status=OK)
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"86c8010ac1440000005008c244f000000008"),
                                              data2       => nsl_data.bytestream.from_suv(x"9f8218010182180101ff"),
                                              check_status => check_status,
                                              dt          => clock_period,
                                              timeout     => clock_period*5000000,
                                              sev         => warning);
    nsl_simulation.logging.log_test_result("SWD init (CTRL/STAT + SELECT)", check_status, pass_count, fail_count);

    wait for 100 us;
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"8EC801C94700000000000000F5C947000000000000000AC001C1440000005008C244F000000008C70101C70101"),
                                              data2       => nsl_data.bytestream.from_suv(x"9faabbccddff"),
                                              check_status => check_status,
                                              dt          => clock_period,
                                              timeout     => clock_period*200000000,
                                              sev         => warning);
    nsl_simulation.logging.log_test_result("Read DP IDR", check_status, pass_count, fail_count);

    wait for 1000 ns;

    nsl_simulation.logging.log_test_suite_summary("SWD CBOR TRANSACTOR TESTS", pass_count, fail_count);

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
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "Going to run frame_queue_master", 
                               color => nsl_simulation.logging.LOG_COLOR_MAGENTA);
    nsl_amba.axi4_stream.frame_queue_master(cfg => cfg_c, root => cmd_q, clock => s_clk,
                                            stream_i => s_cmd.s, stream_o => s_cmd.m, dt => clock_period); --, timeout => 200000 ms);    
  end process;
  
  rsp_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "Going to run frame_queue_slave", 
                               color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_slave(cfg => cfg_c, root => rsp_q, clock => s_clk,
                                           stream_i => s_rsp.m, stream_o => s_rsp.s, dt => clock_period);   
  end process;
end architecture;
