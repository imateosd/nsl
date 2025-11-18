library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_amba, nsl_logic, nsl_data;
library nsl_cypress;

entity axi4_stream_to_fx2 is
  generic (
    axi_cfg_c : nsl_amba.axi4_stream.config_t;
    ep_c :  nsl_cypress.ez_usb_fx2.fx2_ep_t :=  nsl_cypress.ez_usb_fx2.FX2_EP6
    );
  port (
    reset_n_i : in std_ulogic;
    clock_i : in std_ulogic;

    tx_i : in nsl_amba.axi4_stream.master_t;
    tx_o : out nsl_amba.axi4_stream.slave_t;

    to_fx2_o   : out  nsl_cypress.ez_usb_fx2.fx2_i;
    from_fx2_i : in  nsl_cypress.ez_usb_fx2.fx2_o
    );
end entity;


architecture rtl of axi4_stream_to_fx2 is

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
      st <= ST_RESET;
    end if;
  end process;  

  transition: process(st, tx_i, from_fx2_i)
      variable received_bytes : nsl_data.bytestream.byte_string(nsl_amba.axi4_stream.byte_count(axi_cfg_c, tx_i)-1 downto 0);
  begin
    stin <= st;

    to_fx2_o.addr <=  nsl_cypress.ez_usb_fx2.get_fifoaddr(ep_c);
    to_fx2_o.data <= (others => '-');
    to_fx2_o.wr_n <= '1';
    to_fx2_o.rd_n <= '1';
    to_fx2_o.oe_n <= '1';
    to_fx2_o.pktend <= '1';
    
    tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, false);
    
    case st is
      when ST_RESET =>
        to_fx2_o.addr <= (others => '-');
        stin <= ST_IDLE;

      when ST_IDLE =>
        to_fx2_o.addr <= (others => '-');
        if nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
          stin <= ST_STATE1;
        end if;
        
      when ST_STATE1 =>
        stin <= ST_STATE2;
        
      when ST_STATE2 =>
        if from_fx2_i.full_n = '1' then
          stin <= ST_STATE3;
        end if;
        
      when ST_STATE3 =>
        received_bytes := nsl_amba.axi4_stream.bytes(axi_cfg_c, tx_i); 
        to_fx2_o.data <= received_bytes(0);
        to_fx2_o.pktend <= nsl_logic.bool.to_logic(nsl_amba.axi4_stream.is_last(axi_cfg_c, tx_i));
        to_fx2_o.wr_n <= '0';

        tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, true);

        stin <= ST_STATE4;
        
      when ST_STATE4 =>
        if nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
          stin <= ST_STATE2;
        else
          stin <= ST_IDLE;
        end if;
      
    end case;
  end process;
  
end architecture;
