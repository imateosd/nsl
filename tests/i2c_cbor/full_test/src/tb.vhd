library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb is
end tb;

library nsl_i2c, nsl_amba, nsl_simulation, nsl_data;


architecture arch of tb is
  constant clock_period : time := 10 ns;
  constant cfg_c: nsl_amba.axi4_stream.config_t
    := nsl_amba.axi4_stream.config(1, last => true);

  signal s_cmd           : nsl_amba.axi4_stream.bus_t;
  signal s_rsp           : nsl_amba.axi4_stream.bus_t;
  
  signal s_i2c           : nsl_i2c.i2c.i2c_i;
  signal s_i2c_slave1, s_i2c_slave2, s_i2c_slave3, s_i2c_slave4, s_i2c_master : nsl_i2c.i2c.i2c_o;

  signal s_clk, s_resetn : std_ulogic;
  signal s_done : std_ulogic_vector(0 to 0);

  constant MAX_COUNT     : natural := 500; -- 5 us
  signal counter         : natural range 0 to MAX_COUNT := 0;
  signal s_enable_slave4 : std_ulogic;
  signal s_resetn_slave4 : std_ulogic;
  
  shared variable cmd_q, rsp_q: nsl_amba.axi4_stream.frame_queue_root_t;
  
  -- Operations
  -- Write to 0x50, 0x12 and 0x34          -> [[0x50, h'1234'], null]
  -- -> 82821850421234f6
    
  -- Read from 0x50, 2 bytes               -> [[0x50, 2], null]
  -- -> 8282185002f6
    
  -- Write 0x12 to memory address 0x00 and 0x34 to memory address 0x01
  -- -> [[0x40, h'00001234'], null]
  -- -> 828218404400001234f6
    
  -- Read 1 byte from memory address 0x00 and 1 from 0x01
  -- -> [[0x40, h'0000'], [0x40, 1], null, [0x40, h'0001'], [64, 1], null]
  -- -> 8682184042000082184001f682184042000182184001f6
    
  -- Read 3 bytes from memory address 0x00 -> [[64, h'0000'], [64, 3], null]
  -- -> 8382184042000082184003f6
    
  -- Write to not present address 0x60     -> [[0x60, h'12'], null]
  -- -> 828218604112f6
  -- -> Should return i2c-addr-nack
    
  -- Write to slave_nack at address 0x30   -> [[0x30, h'1234'], null]
  -- -> 82821830421234f6
  -- -> Won't acknowledge data bytes. Should return i2c-data-nack
    
  -- Try to read from address 0x70 (not present) with a timeout of 10 us -> [1([10, 0x70, 2]), null]
  -- -> 0x82c1830a187002f6
  -- -> Should return i2c-addr-nack
    
  -- Try to read from address 0x20 with a timeout of 10 us -> [1([10, 0x20, 1]), null]
  -- -> 0x82c1830a182002f6
  
begin

  resolver: nsl_i2c.i2c.i2c_resolver
    generic map(
      port_count => 5
      )
    port map(
      bus_i(0) => s_i2c_slave1,
      bus_i(1) => s_i2c_slave2,
      bus_i(2) => s_i2c_slave3,
      bus_i(3) => s_i2c_slave4,
      bus_i(4) => s_i2c_master,
      bus_o => s_i2c
      );

  
  i2c_slave: nsl_i2c.clocked.clocked_slave
    generic map(
      clock_freq_c => 100000000
    )
    port map(
      reset_n_i => s_resetn,
      clock_i   => s_clk,

      address_i => "1010000", -- 0x50

      i2c_i    => s_i2c,
      i2c_o    => s_i2c_slave1,

      start_o  => open,
      stop_o   => open,
      selected_o => open,
      
      r_data_i  => X"AA",
      r_ready_o => open,
      r_valid_i => '1',

      w_data_o  => open,
      w_valid_o => open,
      w_ready_i => '1'
    );

  
  i2c_mem: nsl_i2c.clocked.clocked_memory
    generic map(
      address => "1000000", -- 0x40
      addr_width => 16
      )
    port map(
      clock_i  => s_clk,
      reset_n_i => s_resetn,

      i2c_i => s_i2c,
      i2c_o => s_i2c_slave2
      );  

  
  i2c_slave_nack: entity work.clocked_slave_nack
    generic map(
      clock_freq_c => 100000000
    )
    port map(
      reset_n_i => s_resetn,
      clock_i   => s_clk,

      address_i => "0110000", -- 0x30

      i2c_i    => s_i2c,
      i2c_o    => s_i2c_slave3,

      start_o  => open,
      stop_o   => open,
      selected_o => open,
      
      r_data_i  => X"AA",
      r_ready_o => open,
      r_valid_i => '1',

      w_data_o  => open,
      w_valid_o => open,
      w_ready_i => '1'
    );

  
  i2c_slave_delayed: nsl_i2c.clocked.clocked_slave
    generic map(
      clock_freq_c => 100000000
    )
    port map(
      reset_n_i => s_resetn_slave4,
      clock_i   => s_clk,

      address_i => "0100000", -- 0x20

      i2c_i    => s_i2c,
      i2c_o    => s_i2c_slave4,

      start_o  => open,
      stop_o   => open,
      selected_o => open,
      
      r_data_i  => X"BB",
      r_ready_o => open,
      r_valid_i => '1',

      w_data_o  => open,
      w_valid_o => open,
      w_ready_i => '1'
    );

  
  dut: nsl_i2c.cbor_transactor.controller
    generic map(
      system_clock_c => 10e7,
      axi_s_cfg_c    => cfg_c
      )
    port map(
      clock_i  =>  s_clk,
      reset_n_i => s_resetn,
      
      cmd_i => s_cmd.m,
      cmd_o => s_cmd.s,

      rsp_o => s_rsp.m,
      rsp_i => s_rsp.s,
      
      i2c_i => s_i2c,
      i2c_o => s_i2c_master
      );

  
  driver: nsl_simulation.driver.simulation_driver
    generic map(
      clock_count => 1,
      reset_count => 1,
      done_count => 1
      )
    port map(
      clock_period(0) => clock_period,
      reset_duration(0) => 3*clock_period,
      reset_n_o(0) => s_resetn,
      clock_o(0) => s_clk,
      done_i => s_done
      );

  
  -- Keep slave4 in reset state until s_enable_slave4 is set to '1'
  -- Then wait 50 us and release the reset
  process(s_clk)
  begin
    if rising_edge(s_clk) then
      s_resetn_slave4 <= '0';
      if s_enable_slave4 = '0' then
        counter         <= 0;
      elsif counter < MAX_COUNT then -- 5 us
        counter  <= counter + 1;
      else
        s_resetn_slave4 <= '1'; -- release reset
      end if;
    end if;
  end process;
  
  
  stim: process
  begin
    -- Let FSM reach IDLE
    wait for 50 ns;

    s_enable_slave4 <= '0';
    
    nsl_amba.axi4_stream.frame_queue_init(cmd_q);
    nsl_amba.axi4_stream.frame_queue_init(rsp_q);

    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing write to clocked slave", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"8182185043123456"),
                                              data2       => nsl_data.bytestream.from_suv(x"9ff6ff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #0 CMD-RSP CHECK PASSED" & LF,
                   color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing read from clocked slave", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"8282185002f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9f42aaffff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #1 CMD-RSP CHECK PASSED" & LF, 
                   color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing write to memory", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"828218404400001234f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9ff6ff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #2 CMD-RSP CHECK PASSED" & LF, 
                   color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing read from memory (2x i2c read and write with restart)", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"8682184042000082184001f682184042000182184001f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9ff64112f64134ff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #3 CMD-RSP CHECK PASSED" & LF, 
                   color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing read from memory (reading 3 bytes from address 0x00)", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"8382184042000082184003f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9ff64312ffffff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #4 CMD-RSP CHECK PASSED" & LF, 
                   color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing access to non existant address", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"828218604112f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9ff4ff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #5 CMD-RSP CHECK PASSED" & LF, 
                   color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing NACK for data bytes", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"82821830421234f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9fc200ff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #6 CMD-RSP CHECK PASSED" & LF,
                   color => nsl_simulation.logging.LOG_COLOR_GREEN); 

    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing poll-read for a non-existant slave", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"82c1830a187002f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9ff4ff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #7 CMD-RSP CHECK PASSED" & LF,
                   color => nsl_simulation.logging.LOG_COLOR_GREEN); 


    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "Testing poll-read for a slave that will be enabled in 5 us (timeout set to 10 us)", 
                   color => nsl_simulation.logging.LOG_COLOR_BLUE);
    s_enable_slave4 <= '1';
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1       => nsl_data.bytestream.from_suv(x"82c1830a182001f6"),
                                              data2       => nsl_data.bytestream.from_suv(x"9f41bbff"),
                                              dt          => clock_period,
                                              timeout     => clock_period*2000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                   message => "============================================================= #8 CMD-RSP CHECK PASSED" & LF,
                   color => nsl_simulation.logging.LOG_COLOR_GREEN); 
  
    wait for 1000 ns;

    nsl_simulation.control.terminate(0);
  end process;

  cmd_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "Going to run frame_queue_master", 
                               color => nsl_simulation.logging.LOG_COLOR_MAGENTA);
    nsl_amba.axi4_stream.frame_queue_master(cfg => cfg_c, root => cmd_q, clock => s_clk,
                                            stream_i => s_cmd.s, stream_o => s_cmd.m, dt => clock_period);    
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
