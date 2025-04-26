library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MEM_WB_Buffer is
  port (
    clk             : in  std_logic;
    rst             : in  std_logic;
    mem_data_in     : in  std_logic_vector(15 downto 0);
    alu_result_in   : in  std_logic_vector(15 downto 0);
    dest_reg_in     : in  std_logic_vector(2 downto 0);

    mem_data_out    : out std_logic_vector(15 downto 0);
    alu_result_out  : out std_logic_vector(15 downto 0);
    dest_reg_out    : out std_logic_vector(2 downto 0)
  );
end MEM_WB_Buffer;

architecture rtl of MEM_WB_Buffer is
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        mem_data_out <= (others => '0');
        alu_result_out <= (others => '0');
        dest_reg_out <= (others => '0');
      else
        mem_data_out <= mem_data_in;
        alu_result_out <= alu_result_in;
        dest_reg_out <= dest_reg_in;
      end if;
    end if;
  end process;
end rtl;
