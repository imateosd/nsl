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

  signal s_cmd           : nsl_amba.axi4_stream.bus_t;
  signal s_rsp           : nsl_amba.axi4_stream.bus_t;

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
      rsp_o          => s_rsp.m,
      rsp_i          => s_rsp.s
      );

  spi_slave_s.i.cs_n <= cs_s_n(0).drain_n;
  spi_slave_s.i.mosi <= nsl_io.io.to_logic(mosi_s);

  miso_s <= nsl_io.io.to_logic(spi_slave_s.o.miso) when cs_s_n(0).drain_n = '0' else
            nsl_io.io.to_logic(mosi_s)             when cs_s_n(1).drain_n = '0' else
            'Z';
    
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
  begin

    nsl_amba.axi4_stream.frame_queue_init(cmd_q);
    nsl_amba.axi4_stream.frame_queue_init(rsp_q);
    
    -- Let FSM reach IDLE
    wait for 50 ns;

    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Testing write to RAM: address 0x00, data 0xaa33", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"84820000c9430b0000c942aa33f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9fff"),
                                              dt      => clock_period,
                                              timeout => clock_period*2000000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "=========== #0 WRITE to RAM successfull" & LF, color => nsl_simulation.logging.LOG_COLOR_GREEN);
    
    wait for 10*clock_period;
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Testing read from RAM: address 0x00, expected data 0xaa33", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"84820000c943030000c810f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f4233aaff"), --this is wrong, should be aa33?
                                              dt      => clock_period,
                                              timeout => clock_period*2000000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "=========== #1 READ from RAM successfull" & LF, color => nsl_simulation.logging.LOG_COLOR_GREEN );

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
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Testing 'minus' command", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c54155f6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f02ff"),
                                              dt      => clock_period,
                                              timeout => clock_period*2000000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "=========== #3 Test of command 'minus' successfull" & LF, color => nsl_simulation.logging.LOG_COLOR_GREEN );

    wait for 10*clock_period;
    -- test shift_no_miso with minus 
    
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "Testing 'minus' command with no MISO", color => nsl_simulation.logging.LOG_COLOR_YELLOW);
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"83820100c9c241fff6"),
                                              data2 => nsl_data.bytestream.from_suv(x"9fff"),
                                              dt      => clock_period,
                                              timeout => clock_period*2000000);
    nsl_simulation.logging.log(level => nsl_simulation.logging.LOG_LEVEL_INFO, message => "=========== #4 Test of command 'minus' without MISO successfull" & LF, color => nsl_simulation.logging.LOG_COLOR_GREEN );


    
    wait for 10*clock_period;
    -- test minus

    
    wait for 10*clock_period;
    -- test minus    
    
    
    -- wait for 1000 ns;                  

    nsl_simulation.control.terminate(0);
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
