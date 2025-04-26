library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mux_generic is
    generic (
        DATA_WIDTH : integer := 16;
        SEL_WIDTH : integer := 2
    );
    port (
        inputs : in std_logic_vector((2**SEL_WIDTH * DATA_WIDTH) - 1 downto 0);
        sel    : in std_logic_vector(SEL_WIDTH - 1 downto 0);
        output : out std_logic_vector(DATA_WIDTH - 1 downto 0)
    );
end entity mux_generic;

architecture rtl of mux_generic is
begin
    process(inputs, sel)
        variable start_index : integer;
        variable end_index : integer;
    begin
        start_index := to_integer(unsigned(sel)) * DATA_WIDTH;
        end_index := start_index + DATA_WIDTH - 1;
        
        output <= inputs(end_index downto start_index);
    end process;
end architecture rtl;