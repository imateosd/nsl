library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_amba, nsl_data, nsl_logic;
library nsl_cypress;

entity fx2_controller_fixed_fast is
  generic(
    axi_cfg_c       : nsl_amba.axi4_stream.config_t;
    rx_ep_c         : nsl_cypress.ez_usb_fx2.fx2_ep_t   := nsl_cypress.ez_usb_fx2.FX2_EP2;
    rx_empty_flag_c : nsl_cypress.ez_usb_fx2.fx2_flag_t := nsl_cypress.ez_usb_fx2.FX2_FLAGA;
    tx_ep_c         : nsl_cypress.ez_usb_fx2.fx2_ep_t   := nsl_cypress.ez_usb_fx2.FX2_EP6;
    tx_full_flag_c  : nsl_cypress.ez_usb_fx2.fx2_flag_t := nsl_cypress.ez_usb_fx2.FX2_FLAGB
    );
  port(
    clock_i    : in std_ulogic;
    reset_n_i  : in std_ulogic;
    
    tx_i       : in nsl_amba.axi4_stream.master_t;
    tx_o       : out nsl_amba.axi4_stream.slave_t;
    
    rx_o       : out nsl_amba.axi4_stream.master_t;
    rx_i       : in nsl_amba.axi4_stream.slave_t;

    to_fx2_o   : out nsl_cypress.ez_usb_fx2.fx2_i;
    from_fx2_i : in nsl_cypress.ez_usb_fx2.fx2_flags_o
    );
end entity;

architecture rtl of fx2_controller_fixed_fast is

  type state_t is (
    ST_RESET,
    ST_TX_OR_START_RX,
    ST_COMPLETE_RX
    );

  type regs_t is record
    state   : state_t;
    data    : std_ulogic_vector(7 downto 0);
    last    : boolean;
    -- empty_n : std_ulogic;
    -- full_n  : std_ulogic;
  end record;
  signal r, rin: regs_t;

  signal rx_empty_n_s : std_ulogic;
  signal tx_full_n_s : std_ulogic;
begin

  -- flag assignment (TX - IN endpoint)
  with tx_full_flag_c select
    tx_full_n_s  <= from_fx2_i.flag_a when nsl_cypress.ez_usb_fx2.FX2_FLAGA,
                  from_fx2_i.flag_b when nsl_cypress.ez_usb_fx2.FX2_FLAGB,
                  from_fx2_i.flag_c when nsl_cypress.ez_usb_fx2.FX2_FLAGC,
                  from_fx2_i.flag_d when nsl_cypress.ez_usb_fx2.FX2_FLAGD,
                  '1' when others;

  -- flag assignment (RX - OUT endpoint)
  with rx_empty_flag_c select
    rx_empty_n_s <= from_fx2_i.flag_a when nsl_cypress.ez_usb_fx2.FX2_FLAGA,
                  from_fx2_i.flag_b when nsl_cypress.ez_usb_fx2.FX2_FLAGB,
                  from_fx2_i.flag_c when nsl_cypress.ez_usb_fx2.FX2_FLAGC,
                  from_fx2_i.flag_d when nsl_cypress.ez_usb_fx2.FX2_FLAGD,
                  '1' when others;
  
  regs: process(reset_n_i, clock_i)
  begin
    if rising_edge(clock_i) then
      r <= rin;
    end if;
    if reset_n_i = '0' then
      r.state <= ST_RESET;
    end if;
  end process;  

  -- TODO make work for other axi stream configurations!!  
  transition: process(r, tx_i, rx_i, from_fx2_i, rx_empty_n_s, tx_full_n_s)
    variable received_bytes : nsl_data.bytestream.byte_string(0 downto 0);
  begin
    rin <= r;
    
    case r.state is
      when ST_RESET =>
        rin.state <= ST_TX_OR_START_RX;
        rin.data <= (others => '-');
        rin.last <= false;

      when ST_TX_OR_START_RX => -- Write has priority
        if tx_full_n_s = '1' and nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
          rin.state <= ST_TX_OR_START_RX;
        elsif rx_empty_n_s = '1' and nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          rin.data <= from_fx2_i.data;
          rin.state <= ST_COMPLETE_RX;
        end if;
        
      when ST_COMPLETE_RX => -- Set FIFO address to point to EP6
        if nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          rin.state <= ST_TX_OR_START_RX;
        end if;
      
    end case;
  end process;

  output: process (r)
    variable received_bytes : nsl_data.bytestream.byte_string(0 downto 0);
  begin
    to_fx2_o.addr   <= "--";
    to_fx2_o.data   <= (others => '-');
    to_fx2_o.wr_n   <= '1';
    to_fx2_o.rd_n   <= '1';
    to_fx2_o.oe_n   <= '1';
    to_fx2_o.pktend <= '1';
    
    tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, false);
    rx_o <= nsl_amba.axi4_stream.transfer_defaults(axi_cfg_c);
    
    case r.state is
      when ST_RESET =>
      
      when ST_TX_OR_START_RX => -- priority to write
        -- ready to write if EP FIFO not full        
        tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, tx_full_n_s = '1');

        if tx_full_n_s = '1' and nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
          -- write and increment fifo pointer
          to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(tx_ep_c);
          received_bytes := nsl_amba.axi4_stream.bytes(axi_cfg_c, tx_i);
          to_fx2_o.pktend <= nsl_logic.bool.to_logic(not nsl_amba.axi4_stream.is_last(axi_cfg_c, tx_i));
          to_fx2_o.data   <= received_bytes(0);          
          to_fx2_o.wr_n   <= '0';

        elsif rx_empty_n_s = '1' and nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          -- read and increment fifo pointer
          to_fx2_o.addr   <= nsl_cypress.ez_usb_fx2.get_fifoaddr(rx_ep_c);
          to_fx2_o.oe_n <= '0';
          to_fx2_o.rd_n <= '0'; -- increment FIFO pointer
        end if;

      when ST_COMPLETE_RX =>
        -- put received data in AXI with correct TLAST
        to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(rx_ep_c);
        rx_o <= nsl_amba.axi4_stream.transfer(cfg => axi_cfg_c,
                                              bytes => nsl_data.bytestream.from_suv(r.data),
                                              last => rx_empty_n_s = '0');
    end case;
  end process;
end architecture;
