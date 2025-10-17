library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb is
end tb;

library nsl_i2c, nsl_amba, nsl_simulation, nsl_data;


architecture arch of tb is

    constant cfg_c: nsl_amba.axi4_stream.config_t
      := nsl_amba.axi4_stream.config(1, last => true);

    signal s_cmd           : nsl_amba.axi4_stream.bus_t;
    signal s_rsp           : nsl_amba.axi4_stream.bus_t;
    -- signal s_rsp_post_fifo : nsl_amba.axi4_stream.bus_t;

    signal s_i2c           : nsl_i2c.i2c.i2c_i;
    signal s_i2c_slave, s_i2c_master : nsl_i2c.i2c.i2c_o;

    signal s_clk, s_resetn : std_ulogic;
    signal s_done : std_ulogic_vector(0 to 0);

begin

  resolver: nsl_i2c.i2c.i2c_resolver
    generic map(
      port_count => 2
      )
    port map(
      bus_i(0) => s_i2c_slave,
      bus_i(1) => s_i2c_master,
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
      i2c_o    => s_i2c_slave,

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

  -- i2c_mem: nsl_i2c.clocked.clocked_memory
  --   generic map(
  --     address => "1010001", -- 0x50
  --     addr_width => 16
  --     )
  --   port map(
  --     clock_i  => s_clk,
  --     reset_n_i => s_resetn,

  --     i2c_i => s_i2c,
  --     i2c_o => s_i2c_slave
  --     );  


  dut: nsl_i2c.cbor_transactor.controller
  generic map(
    system_clock_c => 10e6,
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


  net: nsl_amba.stream_to_udp.axi4_stream_udp_gateway
  generic map(
    config_c => cfg_c,
    bind_port_c => 4242
    )
  port map(
    clock_i => s_clk,
    reset_n_i => s_resetn,

    -- tx_i => s_rsp_post_fifo.m,
    -- tx_o => s_rsp_post_fifo.s,

    tx_i => s_rsp.m,
    tx_o => s_rsp.s,

    rx_o => s_cmd.m,
    rx_i => s_cmd.s
    );
  
  -- axi_fifo: nsl_amba.stream_fifo.axi4_stream_fifo
  -- generic map(
  --   config_c => cfg_c,
  --   depth_c  => 4,
  --   clock_count_c => 1
  -- )
  -- port map(
  --   clock_i(0)=> s_clk,
  --   reset_n_i => s_resetn,

  --   in_i => s_rsp.m,
  --   in_o => s_rsp.s,
  --   in_free_o => open,

  --   out_o => s_rsp_post_fifo.m,
  --   out_i => s_rsp_post_fifo.s,
  --   out_available_o => open
  -- );
  
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


  rx_dumper: nsl_amba.axi4_stream.axi4_stream_dumper
    generic map(
      prefix_c => "UDP RX",
      config_c => cfg_c
      )
    port map(
      clock_i => s_clk,
      reset_n_i => s_resetn,
      bus_i => s_cmd
      );

  tx_dumper: nsl_amba.axi4_stream.axi4_stream_dumper
    generic map(
      prefix_c => "UDP TX",
      config_c => cfg_c
      )
    port map(
      clock_i => s_clk,
      reset_n_i => s_resetn,
      bus_i => s_rsp
      );

  -- stim: process
  -- begin
  --   -- Let FSM reach IDLE
  --   wait for 500 ns;

  --   s_rsp.s.ready <= '1';

  --   -- Going to send [[0x50, 32]]
  --   -- 81 82 18 50 18 20

  --   -- ?????? Dump this and do it using a fifo???

  --   s_cmd.m <= nsl_amba.axi4_stream.transfer_defaults(cfg => cfg_c);

  --   s_cmd.m <= nsl_amba.axi4_stream.transfer(cfg => cfg_c, bytes => nsl_data.bytestream.from_suv(X"81") );
  --   wait until s_cmd.s.ready = '1';
  --   s_cmd.m.valid <= '0';
  --   wait for 10 ns;
    
  --   s_cmd.m <= nsl_amba.axi4_stream.transfer(cfg => cfg_c, bytes => nsl_data.bytestream.from_suv(X"82") );
  --   wait until s_cmd.s.ready = '1';
  --   s_cmd.m.valid <= '0';
  --   wait for 10 ns;
    
  --   s_cmd.m <= nsl_amba.axi4_stream.transfer(cfg => cfg_c, bytes => nsl_data.bytestream.from_suv(X"18") );
  --   wait until s_cmd.s.ready = '1';
  --   s_cmd.m.valid <= '0';
  --   wait for 10 ns;
    
  --   s_cmd.m <= nsl_amba.axi4_stream.transfer(cfg => cfg_c, bytes => nsl_data.bytestream.from_suv(X"50") );
  --   wait until s_cmd.s.ready = '1';
  --   s_cmd.m.valid <= '0';
  --   wait for 10 ns;
    
  --   -- s_cmd.m <= nsl_amba.axi4_stream.transfer(cfg => cfg_c, bytes => nsl_data.bytestream.from_suv(X"18") );
  --   -- wait until s_cmd.s.ready = '1';
  --   -- s_cmd.m.valid <= '0';
  --   -- wait for 10 ns;

  --   -- s_cmd.m <= nsl_amba.axi4_stream.transfer(cfg => cfg_c, bytes => nsl_data.bytestream.from_suv(X"20") );
  --   -- wait until s_cmd.s.ready = '1';
  --   -- s_cmd.m.valid <= '0';
  --   -- wait for 10 ns;

  --   wait for 2000 ns;

  --   nsl_simulation.control.terminate(0);
  -- end process;
 
end architecture;
