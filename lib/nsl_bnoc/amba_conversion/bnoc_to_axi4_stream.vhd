library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_bnoc, nsl_amba;

entity bnoc_to_axi4_stream is
  generic(
    axi4s_cfg_c : nsl_amba.axi4_stream.config_t
    );
  port(
    bnoc_req_i  : in nsl_bnoc.framed.framed_req_t;
    bnoc_ack_o  : out nsl_bnoc.framed.framed_ack_t;
    
    axi4s_req_o : out nsl_amba.axi4_stream.master_t;
    axi4s_ack_i : in nsl_amba.axi4_stream.slave_t
    );
end entity;

architecture beh of bnoc_to_axi4_stream is

begin
  
  axi4s_req_o <= nsl_amba.axi4_stream.transfer(
    cfg => axi4s_cfg_c,
    bytes => (0 => bnoc_req_i.data),
    valid => bnoc_req_i.valid = '1',
    last => bnoc_req_i.last = '1');
  bnoc_ack_o.ready <= '1' when nsl_amba.axi4_stream.is_ready(axi4s_cfg_c, axi4s_ack_i) else '0';
  
end architecture;
