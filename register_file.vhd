library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.constants_and_types.all;
entity register_file is
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        rs1_addr  : in  std_logic_vector(2 downto 0);
        rs2_addr  : in  std_logic_vector(2 downto 0);
        rs1_data  : out std_logic_vector(15 downto 0);
        rs2_data  : out std_logic_vector(15 downto 0);
        rd_wr_en  : in  std_logic;
        rd_addr   : in  std_logic_vector(2 downto 0);
        rd_data   : in  std_logic_vector(15 downto 0)
    );
end entity register_file;
architecture rtl of register_file is
    -- the actual register file storage
    signal regs : reg_array := (others => (others => '0'));
begin
    -- synchronous write and reset
    process(clk, rst)
    begin
        if rst = '1' then
            -- initialize all registers to zero
            regs <= (others => (others => '0'));
        elsif rising_edge(clk) then
            if rd_wr_en = '1' then
                regs(to_integer(unsigned(rd_addr))) <= rd_data;
            end if;
        end if;
    end process;
    -- asynchronous read ports (combinational)
    rs1_data <= regs(to_integer(unsigned(rs1_addr)));
    rs2_data <= regs(to_integer(unsigned(rs2_addr)));
end architecture rtl;