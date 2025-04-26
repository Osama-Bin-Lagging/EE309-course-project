library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity EX_MEM_Buffer is
  port (
    clk             : in  std_logic;
    rst             : in  std_logic;
    alu_result_in   : in  std_logic_vector(15 downto 0);
    reg_data2_in    : in  std_logic_vector(15 downto 0);
    dest_reg_in     : in  std_logic_vector(2 downto 0);

    alu_result_out  : out std_logic_vector(15 downto 0);
    reg_data2_out   : out std_logic_vector(15 downto 0);
    dest_reg_out    : out std_logic_vector(2 downto 0)
  );
end EX_MEM_Buffer;

architecture rtl of EX_MEM_Buffer is
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        alu_result_out <= (others => '0');
        reg_data2_out <= (others => '0');
        dest_reg_out <= (others => '0');
      else
        alu_result_out <= alu_result_in;
        reg_data2_out <= reg_data2_in;
        dest_reg_out <= dest_reg_in;
      end if;
    end if;
  end process;
end rtl;
