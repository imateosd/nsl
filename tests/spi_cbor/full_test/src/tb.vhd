library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb is
end tb;

library nsl_spi, nsl_amba, nsl_simulation, nsl_data, nsl_event, nsl_io, nsl_memory;

architecture arch of tb is

  constant clock_period : time := 10 ns;
  
  constant cfg_c: nsl_amba.axi4_stream.config_t
    := nsl_amba.axi4_stream.config(1, last => true);
  constant addr_byte_cnt: integer := 2;
  constant data_byte_cnt: integer := 2;

  signal s_cmd     : nsl_amba.axi4_stream.bus_t;
  signal s_rsp     : nsl_amba.axi4_stream.bus_t;
  signal s_rsp_pre : nsl_amba.axi4_stream.bus_t;

  signal spi_master_s: nsl_spi.spi.spi_master_io;
  signal spi_slave_s: nsl_spi.spi.spi_slave_io;
  
  signal spi_m: nsl_spi.spi.spi_slave_i;
  signal spi_s: nsl_spi.spi.spi_slave_o;
  signal cs_s_n : nsl_io.io.opendrain_vector(0 to 1);
  signal mosi_s : nsl_io.io.tristated;
  signal miso_s : std_ulogic;
  
  signal s_clk : std_ulogic := '0';
  signal s_resetn : std_ulogic;
  signal s_done : std_ulogic_vector(0 to 0);
  signal s_write : std_ulogic;
  signal s_rdata, s_wdata : std_ulogic_vector(8*data_byte_cnt-1 downto 0);
  signal s_wdata_bytestream : nsl_data.bytestream.byte_string(0 to data_byte_cnt-1);
  signal s_address : unsigned(8*addr_byte_cnt-1 downto 0);

  signal   tick_s      : std_ulogic;
  constant tick_divisor: unsigned(7 downto 0) := (others => '1');

  shared variable cmd_q, rsp_q: nsl_amba.axi4_stream.frame_queue_root_t;  
begin

  
  dut: nsl_spi.cbor_transactor.controller
    generic map(
      clock_i_hz_c   => 10e7,
      tick_i_hz_c    => 10e7/to_integer(tick_divisor),
      axi_s_cfg_c    => cfg_c,
      slave_count_c  => 2
      )
    port map(
      clock_i        => s_clk,
      reset_n_i      => s_resetn,

      tick_i         => tick_s,
      
      sck_o          => spi_slave_s.i.sck,
      cs_n_o         => cs_s_n,
      mosi_o         => mosi_s,
      miso_i         => miso_s,
      
      cmd_i          => s_cmd.m,
      cmd_o          => s_cmd.s,
      rsp_o          => s_rsp_pre.m,
      rsp_i          => s_rsp_pre.s
      );
  
  spi_slave_s.i.cs_n <= cs_s_n(0).drain_n;
  spi_slave_s.i.mosi <= nsl_io.io.to_logic(mosi_s);

  miso_s <= nsl_io.io.to_logic(spi_slave_s.o.miso) when cs_s_n(0).drain_n = '0' else
            nsl_io.io.to_logic(mosi_s)             when cs_s_n(1).drain_n = '0' else
            'Z';
  
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
    
  slave: nsl_spi.slave.spi_memory_controller
    generic map(
      addr_bytes_c   => addr_byte_cnt,
      data_bytes_c   => data_byte_cnt,
      write_opcode_c => x"0b"
      )
    port map(
      clock_i    => s_clk,
      reset_n_i  => s_resetn,

      spi_i      => spi_slave_s.i,
      spi_o      => spi_slave_s.o,
          
      selected_o => open,

      addr_o     => s_address,

      cpol_i     => '0',
      cpha_i     => '0',

      rdata_i    => nsl_data.bytestream.from_suv(s_rdata),
      rready_o   => open,

      wdata_o    => s_wdata_bytestream,
      wvalid_o   => s_write
      );

  s_wdata <= s_wdata_bytestream(1) & s_wdata_bytestream(0);
  
  ram : nsl_memory.ram.ram_1p
    generic map (
      addr_size_c => 8*addr_byte_cnt,
      data_size_c => 8*data_byte_cnt
    )
    port map (
      clock_i      => s_clk,
      write_en_i   => s_write,
      address_i    => s_address,
      write_data_i => s_wdata,
      read_data_o  => s_rdata
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
      message => "SPI CBOR TRANSACTOR TEST SUITE",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );
    nsl_simulation.logging.log(
      level => nsl_simulation.logging.LOG_LEVEL_INFO,
      message => "======================================",
      color => nsl_simulation.logging.LOG_COLOR_CYAN
    );

    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"84820000c9430b0000c942aa33f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9fff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Write to RAM: address 0x00, data 0xaa33", check_status, pass_count, fail_count);
    
    wait for 10*clock_period;
    
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"84820000c943030000c810f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000233aaff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Read from RAM: address 0x00, expected data 0xaa33", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- now do the same thing in one payload. Must use the 'pause' command
    
    -- nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Testing WRITE and READ from RAM using 'pause' command", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    -- nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
    --                                           root_slave  => rsp_q,
    --                                           data1 => nsl_data.bytestream.from_suv(x"8a820000c9430b0000c942aaaaf6ca1832820000c943030000ca15c810f6"),
    --                                           data2 => nsl_data.bytestream.from_suv(x"9f42aa33ff"),
    --                                           dt      => clock_period,
    --                                           timeout => clock_period*2000000);
    -- nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "=========== #2 READ and WRITE from RAM with 'pause' successfull" & LF, color => nsl_simulation.logging.LOG_COLOR_GREEN );


    wait for 10*clock_period;
    -- test minus with bstr
    
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c54155f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000103ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Test 'minus' command", check_status, pass_count, fail_count);

    wait for 10*clock_period;
    -- test shift_no_miso with minus 
    
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c9c241fff6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9fff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Test 'minus' command with no MISO", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 5: Full byte shift with MISO capture on loopback (CS1)
    -- Command: select CS1 mode 0, shift 0xaa, unselect
    -- 83 = array(3), 82 01 00 = [1, 0], 41 aa = bstr(0xaa), f6 = null
    -- Loopback returns same data
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"8382010041aaf6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f590001aaff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Full byte shift with MISO on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 6: SPI Mode 1 (CPOL=0, CPHA=1) on loopback
    -- 83 = array(3), 82 01 01 = [1, 1], 41 55 = bstr(0x55), f6 = null
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"838201014155f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000155ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("SPI Mode 1 shift on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 7: SPI Mode 2 (CPOL=1, CPHA=0) on loopback
    -- 83 = array(3), 82 01 02 = [1, 2], 41 33 = bstr(0x33), f6 = null
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"838201024133f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000133ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("SPI Mode 2 shift on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 8: SPI Mode 3 (CPOL=1, CPHA=1) on loopback
    -- 83 = array(3), 82 01 03 = [1, 3], 41 cc = bstr(0xcc), f6 = null
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"8382010341ccf6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f590001ccff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("SPI Mode 3 shift on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 9: Shift minus-1 (#6.1 = c1) on loopback - shift 7 bits
    -- 83 = array(3), 82 01 00 = [1, 0], c1 41 aa = tag1(bstr(0xaa)), f6 = null
    -- 0xaa = 10101010, shift 7 bits = 1010101 = 0x55
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c141aaf6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000155ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-1 on loopback (7 bits)", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 10: Shift minus-2 (#6.2 = c2) on loopback - shift 6 bits
    -- 83 = array(3), 82 01 00 = [1, 0], c2 41 ff = tag2(bstr(0xff)), f6 = null
    -- 0xff = 11111111, shift 6 bits = 111111 = 0x3f
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c241fff6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f5900013fff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-2 on loopback (6 bits)", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 11: Shift minus-3 (#6.3 = c3) on loopback - shift 5 bits
    -- 0xf0 = 11110000, shift 5 bits = 11110 = 0x1e
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c341f0f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f5900011eff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-3 on loopback (5 bits)", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 12: Shift minus-4 (#6.4 = c4) on loopback - shift 4 bits
    -- 0xab = 10101011, shift 4 bits = 1010 = 0x0a
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c441abf6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f5900010aff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-4 on loopback (4 bits)", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 13: Shift minus-6 (#6.6 = c6) on loopback - shift 2 bits
    -- 0xc0 = 11000000, shift 2 bits = 11 = 0x03
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c641c0f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000103ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-6 on loopback (2 bits)", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 14: Shift minus-7 (#6.7 = c7) on loopback - shift 1 bit
    -- 0x80 = 10000000, shift 1 bit = 1 = 0x01
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c74180f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000101ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Shift minus-7 on loopback (1 bit)", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 15: Pause command (#6.10 = ca) between operations
    -- 85 = array(5): select, shift, pause(50 ticks), shift, unselect
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"858201004112ca18324134f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f5900011259000134ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Pause between shifts on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 16: Multi-byte shift on loopback
    -- 83 = array(3), 82 01 00 = [1, 0], 43 aabbcc = bstr(3 bytes), f6 = null
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"8382010043aabbccf6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f590003aabbccff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Multi-byte shift on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 17: Back-to-back shifts without unselect
    -- 84 = array(4), select, shift1, shift2, unselect
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"8482010041114122f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f5900011159000122ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Back-to-back shifts on loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    -- Test 18: Multi-CS operation: CS0 write then CS1 loopback
    -- 86 = array(6): select CS0, write no-miso, unselect, select CS1, shift, unselect
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"86820000c9430b0001f68201004177f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f59000177ff"),
                                              check_status => check_status,
                                              dt      => clock_period,
                                              timeout => clock_period*2000000,
                                              sev     => warning);
    nsl_simulation.logging.log_test_result("Multi-CS: write to RAM then loopback", check_status, pass_count, fail_count);

    wait for 10*clock_period;

    nsl_simulation.logging.log_test_suite_summary("SPI CBOR TRANSACTOR TESTS", pass_count, fail_count);

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
                                            stream_i => s_cmd.s, stream_o => s_cmd.m, dt => clock_period);    
  end process;
  
  rsp_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Going to run frame_queue_slave", color => nsl_simulation.logging.LOG_COLOR_CYAN);
    nsl_amba.axi4_stream.frame_queue_slave(cfg => cfg_c, root => rsp_q, clock => s_clk,
                                           stream_i => s_rsp.m, stream_o => s_rsp.s, dt => clock_period);   
  end process;

end architecture;
