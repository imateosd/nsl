library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_amba, nsl_data;
library nsl_cypress;

entity fx2_to_axi4_stream is
  generic (
    axi_cfg_c : nsl_amba.axi4_stream.config_t;
    ep_c : nsl_cypress.ez_usb_fx2.fx2_ep_t := nsl_cypress.ez_usb_fx2.FX2_EP2
    );
  port (
    reset_n_i : in std_ulogic;
    clock_i : in std_ulogic;

    rx_o : out nsl_amba.axi4_stream.master_t;
    rx_i : in nsl_amba.axi4_stream.slave_t;

    to_fx2_o   : out nsl_cypress.ez_usb_fx2.fx2_i;
    from_fx2_i : in nsl_cypress.ez_usb_fx2.fx2_o     
    );
end entity;

architecture rtl of fx2_to_axi4_stream is

  type state_t is (
    ST_RESET,
    ST_IDLE,
    ST_STATE1,
    ST_STATE2,
    ST_STATE3,
    ST_STATE4
    );
  
  signal st, stin: state_t;

begin
  
  regs: process(reset_n_i, clock_i)
  begin
    if rising_edge(clock_i) then
      st <= stin;
    end if;
    if reset_n_i = '0' then
      st   <= ST_RESET;
    end if;
  end process;

  transition: process(st, rx_i, from_fx2_i)
  begin
    stin <= st;

    to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(ep_c);
    to_fx2_o.data <= (others => '-');
    to_fx2_o.wr_n <= '1';
    to_fx2_o.rd_n <= '1';
    to_fx2_o.oe_n <= '1';
    to_fx2_o.pktend <= '1';

    rx_o <= nsl_amba.axi4_stream.transfer_defaults(axi_cfg_c);
        
    case st is
      when ST_RESET =>
        to_fx2_o.addr <= (others => '-');
        stin <= ST_IDLE;

      when ST_IDLE =>
        to_fx2_o.addr <= (others => '-');
        if nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          stin <= ST_STATE1;
        end if;
        
      when ST_STATE1 =>
        stin <= ST_STATE2;
        
      when ST_STATE2 =>
        to_fx2_o.oe_n <= '0';
        if from_fx2_i.empty_n = '1' then
          stin <= ST_STATE3;
        end if;
        
      when ST_STATE3 =>
        rx_o <= nsl_amba.axi4_stream.transfer(cfg => axi_cfg_c,
                                              bytes => nsl_data.bytestream.from_suv(from_fx2_i.data),
                                              last => from_fx2_i.empty_n = '0');
        to_fx2_o.rd_n <= '0';

        stin <= ST_STATE4;
        
      when ST_STATE4 =>
        if nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          stin <= ST_STATE2;
        else
          stin <= ST_IDLE;
        end if;
      
    end case;
  end process;
  
end architecture;
