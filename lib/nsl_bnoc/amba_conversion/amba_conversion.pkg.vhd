library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all; 

library nsl_bnoc, nsl_amba;

package amba_conversion is

  component bnoc_to_axi4_stream
    generic(
      axi4s_cfg_c : nsl_amba.axi4_stream.config_t
      );
    port(
      bnoc_req_i  : in nsl_bnoc.framed.framed_req_t;
      bnoc_ack_o  : out nsl_bnoc.framed.framed_ack_t;

      axi4s_req_o : out nsl_amba.axi4_stream.master_t;
      axi4s_ack_i : in nsl_amba.axi4_stream.slave_t
      );
  end component;

  component axi4_stream_to_bnoc
    generic(
      axi4s_cfg_c : nsl_amba.axi4_stream.config_t
      );
    port(
      axi4s_req_i : in  nsl_amba.axi4_stream.master_t;
      axi4s_ack_o : out nsl_amba.axi4_stream.slave_t;
      
      bnoc_req_o  : out nsl_bnoc.framed.framed_req_t;
      bnoc_ack_i  : in  nsl_bnoc.framed.framed_ack_t
      );
  end component;
  
end package amba_conversion;
