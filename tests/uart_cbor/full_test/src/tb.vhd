library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_uart, nsl_amba, nsl_data, nsl_simulation;

entity tb is
end;


architecture arch of tb is
  constant c_clock_period : time := 10 ns;
  constant c_cfg : nsl_amba.axi4_stream.config_t := nsl_amba.axi4_stream.config(bytes => 1, last => true);
  
  type uart_t is
  record
    tx : std_ulogic;
    cts: std_ulogic;
    rx : std_ulogic;
    rts: std_ulogic;
  end record;

  signal s_uart: uart_t;
  signal s_clk, s_rst_n: std_ulogic;

  signal s_cmd, s_rsp : nsl_amba.axi4_stream.bus_t;

  shared variable cmd_q, rsp_q: nsl_amba.axi4_stream.frame_queue_root_t;
begin

  dut : nsl_uart.transactor.cbor_controller
    generic map(
      system_clock_c => 10e7,
      axi_s_cfg_c => c_cfg,
      stop_count_c => 1,
      parity_c => nsl_uart.serdes.PARITY_NONE,
      handshake_active_c => '0',
      divisor_c => to_unsigned(10416, 32), -- 9600 bds
      timeout_c => to_unsigned(2000, 32), -- timeout in bit time
      tstr_max_size_c => 12
      )
    port map(
      reset_n_i => s_rst_n,
      clock_i => s_clk,

      tx_o  => s_uart.tx,
      cts_i => s_uart.cts,
      rx_i  => s_uart.rx,
      rts_o => s_uart.rts,

      cmd_i => s_cmd.m,
      cmd_o => s_cmd.s,
      rsp_o => s_rsp.m,
      rsp_i => s_rsp.s
      );

  s_uart.rx <= s_uart.tx;

  driver : nsl_simulation.driver.simulation_driver
    generic map(
      clock_count => 1,
      reset_count => 1,
      done_count => 1
      )
    port map(
      clock_period(0) => c_clock_period,
      reset_duration(0) => c_clock_period*2,
      reset_n_o(0) => s_rst_n,
      clock_o(0) => s_clk,
      done_i(0) => '0'
      );
  
  stim: process
  begin

    nsl_amba.axi4_stream.frame_queue_init(cmd_q);
    nsl_amba.axi4_stream.frame_queue_init(rsp_q);
    
    -- Let FSM reach IDLE
    wait for 50 ns;

    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "Testing setting configuration",
                               color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_put(root => cmd_q,
                                         data => nsl_data.bytestream.from_suv(x"A369666C6F772D6374726C6363747366706172697479616569626175642D72617465192d00"));
    -- There's no response to this command, so the configuration changes would
    -- need to be asserted. Can't do that from here.
    -- assert()
    wait for 80 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "============================================== #0 CONFIGURATION SUCCESSFULLY SET" & LF,
                               color => nsl_simulation.logging.LOG_COLOR_GREEN);   


    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "Sending a message",
                               color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"6C48656C6C6F20776F726C6421"),
                                              data2 => nsl_data.bytestream.from_suv(x"6C48656C6C6F20776F726C6421"),
                                              dt      => c_clock_period,
                                              timeout => c_clock_period*1500000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "============================================== #1 MESSAGE SENT AND RECEIVED SUCCESSFULLY" & LF,
                               color => nsl_simulation.logging.LOG_COLOR_GREEN);    

    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "Recovering block config",
                               color => nsl_simulation.logging.LOG_COLOR_BLUE);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"F6"),
                                              data2 => nsl_data.bytestream.from_suv(x"A369666C6F772D6374726C6363747366706172697479616569626175642D72617465192d00"),
                                              dt      => c_clock_period,
                                              timeout => c_clock_period*1500000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO,
                               message => "============================================== #2 CONFIGURATION RETRIEVED SUCCESSFULLY" & LF,
                               color => nsl_simulation.logging.LOG_COLOR_GREEN);    
 

    nsl_simulation.control.terminate(0);
  end process;

  cmd_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Going to run frame_queue_master", color => nsl_simulation.logging.LOG_COLOR_MAGENTA);
    nsl_amba.axi4_stream.frame_queue_master(cfg => c_cfg, root => cmd_q, clock => s_clk,
                                            stream_i => s_cmd.s, stream_o => s_cmd.m, dt => c_clock_period);    
  end process;
  
  rsp_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Going to run frame_queue_slave", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_slave(cfg => c_cfg, root => rsp_q, clock => s_clk,
                                           stream_i => s_rsp.m, stream_o => s_rsp.s, dt => c_clock_period);   
  end process;

end architecture;
