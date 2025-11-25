library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_amba, nsl_data, nsl_logic;
library nsl_cypress;

entity fx2_controller_fixed is
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

architecture rtl of fx2_controller_fixed is

  type state_t is (
    ST_RESET,
    ST_IDLE,
    ST_W_STATE1,
    ST_W_STATE2,
    ST_W_STATE3,
    ST_W_STATE4,
    ST_R_STATE1,
    ST_R_STATE2,
    ST_R_STATE3,
    ST_R_STATE4,
    ST_R_STATE5

    );

  type regs_t is record
    state   : state_t;
    data    : std_ulogic_vector(7 downto 0);
    -- last    : boolean;
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
    -- variable received_bytes : nsl_data.bytestream.byte_string(nsl_amba.axi4_stream.byte_count(axi_cfg_c, tx_i)-1 downto 0);
    variable received_bytes : nsl_data.bytestream.byte_string(0 downto 0);
  begin
    rin <= r;

    to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(rx_ep_c);
    to_fx2_o.data <= (others => '-');
    to_fx2_o.wr_n <= '1';
    to_fx2_o.rd_n <= '1';
    to_fx2_o.oe_n <= '1';
    to_fx2_o.pktend <= '1';
    
    tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, false);
    -- tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, nsl_logic.bool.to_boolean(tx_full_n_s));
    rx_o <= nsl_amba.axi4_stream.transfer_defaults(axi_cfg_c);
    
    case r.state is
      when ST_RESET =>
        rin.state <= ST_IDLE;

      when ST_IDLE =>
        if nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
          rin.state <= ST_W_STATE1;
        elsif rx_empty_n_s = '1' and nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          rin.state <= ST_R_STATE1;
        end if;
        
      when ST_W_STATE1 =>
        to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(tx_ep_c);
        rin.state <= ST_W_STATE2;
        
      when ST_W_STATE2 =>
        to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(tx_ep_c);
        if tx_full_n_s = '1' then
          rin.state <= ST_W_STATE3;
        end if;
        
      when ST_W_STATE3 =>
        to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(tx_ep_c);

        received_bytes := nsl_amba.axi4_stream.bytes(axi_cfg_c, tx_i);
        to_fx2_o.data <= received_bytes(0);

        tx_o <= nsl_amba.axi4_stream.accept(axi_cfg_c, true);

        to_fx2_o.pktend <= nsl_logic.bool.to_logic(not nsl_amba.axi4_stream.is_last(axi_cfg_c, tx_i));
        to_fx2_o.wr_n <= '0';
        
        rin.state <= ST_W_STATE4;
        
      when ST_W_STATE4 =>
        to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(tx_ep_c);
        if nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
          rin.state <= ST_W_STATE2;
        else
          rin.state <= ST_IDLE;
        end if;
        
      when ST_R_STATE1 =>
        to_fx2_o.addr <= nsl_cypress.ez_usb_fx2.get_fifoaddr(rx_ep_c);
        rin.state <= ST_R_STATE2;
        
      when ST_R_STATE2 =>
        to_fx2_o.oe_n <= '0';
        if rx_empty_n_s = '1' then
          rin.state <= ST_R_STATE3;
        -- elsif nsl_amba.axi4_stream.is_valid(axi_cfg_c, tx_i) then
        --   rin.state <= ST_W_STATE1;
        end if;

      when ST_R_STATE3 =>
        to_fx2_o.oe_n <= '0';
        rin.data <= from_fx2_i.data;
        to_fx2_o.rd_n <= '0'; -- increment FIFO pointer
        --rx_empty_n_s does not yet represent if this is the last byte
        rin.state <= ST_R_STATE4;
                
      when ST_R_STATE4 =>
        to_fx2_o.oe_n <= '0';
        rx_o <= nsl_amba.axi4_stream.transfer(cfg => axi_cfg_c,
                                              bytes => nsl_data.bytestream.from_suv(r.data),
                                              last => rx_empty_n_s = '0');

        if nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) then
          rin.state <= ST_R_STATE5;
        end if;
        
      when ST_R_STATE5 =>
        to_fx2_o.oe_n <= '0';
        if nsl_amba.axi4_stream.is_ready(axi_cfg_c, rx_i) and rx_empty_n_s = '1' then
          rin.state <= ST_R_STATE2;
        else
          rin.state <= ST_IDLE;
        end if;
      
    end case;
  end process;
  
end architecture;
