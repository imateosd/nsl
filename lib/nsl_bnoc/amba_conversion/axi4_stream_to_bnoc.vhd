library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_bnoc, nsl_amba, nsl_data;

entity axi4_stream_to_bnoc is
  generic(
    axi4s_cfg_c : nsl_amba.axi4_stream.config_t
    );
  port(
    axi4s_req_i : in  nsl_amba.axi4_stream.master_t;
    axi4s_ack_o : out nsl_amba.axi4_stream.slave_t;
      
    bnoc_req_o  : out nsl_bnoc.framed.framed_req_t;
    bnoc_ack_i  : in  nsl_bnoc.framed.framed_ack_t
    );
end entity;

architecture beh of axi4_stream_to_bnoc is
    signal received_bytes : nsl_data.bytestream.byte_string(nsl_amba.axi4_stream.byte_count(axi4s_cfg_c, axi4s_req_i)-1 downto 0);
begin
  received_bytes <= nsl_amba.axi4_stream.bytes(axi4s_cfg_c, axi4s_req_i); 
  
  bnoc_req_o <= nsl_bnoc.framed.framed_flit(
    data => received_bytes(0),
    last => nsl_amba.axi4_stream.is_last(axi4s_cfg_c, axi4s_req_i),
    valid => nsl_amba.axi4_stream.is_valid(axi4s_cfg_c, axi4s_req_i));
  axi4s_ack_o <= nsl_amba.axi4_stream.accept(axi4s_cfg_c, bnoc_ack_i.ready = '1');
  
end architecture;
