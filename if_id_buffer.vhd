library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity IF_ID_Buffer is
  port (
    clk         : in  std_logic;
    rst         : in  std_logic;
    instr_in    : in  std_logic_vector(15 downto 0);
    pc_in       : in  std_logic_vector(15 downto 0);
    instr_out   : out std_logic_vector(15 downto 0);
    pc_out      : out std_logic_vector(15 downto 0)
  );
end IF_ID_Buffer;

architecture rtl of IF_ID_Buffer is
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        instr_out <= (others => '0');
        pc_out <= (others => '0');
      else
        instr_out <= instr_in;
        pc_out <= pc_in;
      end if;
    end if;
  end process;
end rtl;
