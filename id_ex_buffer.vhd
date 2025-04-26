library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ID_EX_Buffer is
  port (
    clk             : in  std_logic;
    rst             : in  std_logic;
    reg_data1_in    : in  std_logic_vector(15 downto 0);
    reg_data2_in    : in  std_logic_vector(15 downto 0);
    imm_in          : in  std_logic_vector(15 downto 0);
    alu_op_in       : in  std_logic_vector(2 downto 0);
    dest_reg_in     : in  std_logic_vector(2 downto 0);
    
    reg_data1_out   : out std_logic_vector(15 downto 0);
    reg_data2_out   : out std_logic_vector(15 downto 0);
    imm_out         : out std_logic_vector(15 downto 0);
    alu_op_out      : out std_logic_vector(2 downto 0);
    dest_reg_out    : out std_logic_vector(2 downto 0)
  );
end ID_EX_Buffer;

architecture rtl of ID_EX_Buffer is
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        reg_data1_out <= (others => '0');
        reg_data2_out <= (others => '0');
        imm_out <= (others => '0');
        alu_op_out <= (others => '0');
        dest_reg_out <= (others => '0');
      else
        reg_data1_out <= reg_data1_in;
        reg_data2_out <= reg_data2_in;
        imm_out <= imm_in;
        alu_op_out <= alu_op_in;
        dest_reg_out <= dest_reg_in;
      end if;
    end if;
  end process;
end rtl;
