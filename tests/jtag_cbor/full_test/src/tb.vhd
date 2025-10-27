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
 
 -- type log_color_t is (
  --   LOG_COLOR_BLACK,   -- Black 	30
  --   LOG_COLOR_RED,     -- Red 	  31
  --   LOG_COLOR_GREEN,   -- Green 	32
  --   LOG_COLOR_YELLOW,  -- Yellow 	33
  --   LOG_COLOR_BLUE,    -- Blue 	  34
  --   LOG_COLOR_MAGENTA, -- Magenta 35
  --   LOG_COLOR_CYAN,    -- Cyan 	  36
  --   LOG_COLOR_WHITE    -- White 	37
  --   );                 -- Default 39

  procedure log_with_color(level : nsl_simulation.logging.log_level_t; color: nsl_simulation.logging.log_color_t; message : string) is
    begin
      case color is
        when nsl_simulation.logging.LOG_COLOR_BLACK =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[30m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_RED =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[31m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_GREEN =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[32m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_YELLOW =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[33m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_BLUE =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[34m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_MAGENTA =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[35m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_CYAN =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[36m" & message & character'val(27) & "[0m");
        when nsl_simulation.logging.LOG_COLOR_WHITE =>
          nsl_simulation.logging.log(level => level, message => character'val(27) & "[37m" & message & character'val(27) & "[0m");
      end case;
    end procedure;
  
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
    system_clock_c => 10e6,
    axi_s_cfg_c    => cfg_c
    )
  port map(
    clock_i  =>  s_clk,
    reset_n_i => s_resetn,

    tick_i => tick_s,
    
    cmd_i => s_cmd.m,
    cmd_o => s_cmd.s,

    rsp_o => s_rsp.m,
    rsp_i => s_rsp.s,
    
    jtag_o => ate_o,
    jtag_i => ate_i
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
    -- Let FSM reach IDLE
    wait for 50 ns;

    nsl_amba.axi4_stream.frame_queue_init(cmd_q);
    nsl_amba.axi4_stream.frame_queue_init(rsp_q);

    log_with_color(level => nsl_simulation.logging.LOG_LEVEL_INFO, color => nsl_simulation.logging.LOG_COLOR_BLUE, message => "Testing IDCODE read");
    nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
                                              root_slave  => rsp_q,
                                              data1 => nsl_data.bytestream.from_suv(x"87ca0602e2c9411103e1c804"),
                                              data2 => nsl_data.bytestream.from_suv(x"9f4421436587ff"),
                                              dt      => clock_period,
                                              timeout => clock_period*2000000);
    log_with_color(level => nsl_simulation.logging.LOG_LEVEL_INFO, color => nsl_simulation.logging.LOG_COLOR_GREEN, message => "==================================================================================== #0 IDCODE READ PASSED" & character'val(10));

    -- log_with_color(level => nsl_simulation.logging.LOG_LEVEL_INFO, color => nsl_simulation.logging.LOG_COLOR_BLUE, message => "Testing Chain enum");
    -- nsl_amba.axi4_stream.frame_queue_check_io(root_master => cmd_q,
    --                                           root_slave  => rsp_q,
    --                                           data1 => nsl_data.bytestream.from_suv(x"8ae3ca183201e1440355ffc0c8190200e2440355ffc0c819020001"),
    --                                           -- data2 => nsl_data.bytestream.from_suv(x"9f4487654321ff"),
    --                                           data2 => nsl_data.bytestream.from_suv(x"9f4400000000ff"),
    --                                          dt      => clock_period,
    --                                           timeout => clock_period*2000000);
    -- log_with_color(level => nsl_simulation.logging.LOG_LEVEL_INFO, color => nsl_simulation.logging.LOG_COLOR_GREEN, message => "==================================================================================== #1 CMD-RSP CHECK PASSED" & character'val(10));    

    -- wait for 1000 ns;                  

    nsl_simulation.control.terminate(0);
  end process;

  cmd_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    log_with_color(level => nsl_simulation.logging.LOG_LEVEL_INFO, color => nsl_simulation.logging.LOG_COLOR_MAGENTA, message => "Going to run frame_queue_master");
    nsl_amba.axi4_stream.frame_queue_master(cfg => cfg_c, root => cmd_q, clock => s_clk,
                                            stream_i => s_cmd.s, stream_o => s_cmd.m, dt => clock_period);    
  end process;
  
  rsp_queue: process
  begin
    -- Let FSM reach IDLE and queues be initialized
    wait for 70 ns;
    log_with_color(level => nsl_simulation.logging.LOG_LEVEL_INFO, color => nsl_simulation.logging.LOG_COLOR_YELLOW, message => "Going to run frame_queue_slave");
    nsl_amba.axi4_stream.frame_queue_slave(cfg => cfg_c, root => rsp_q, clock => s_clk,
                                           stream_i => s_rsp.m, stream_o => s_rsp.s, dt => clock_period);   
  end process;

end architecture;
