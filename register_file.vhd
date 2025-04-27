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
    -- Define initial values for registers (hardcoded)
    signal regs : reg_array := (
        0 => x"1234",   -- Register 0 initialized to 0x1234
        1 => x"5678",   -- Register 1 initialized to 0x5678
        2 => x"9ABC",   -- Register 2 initialized to 0x9ABC
        3 => x"DEF0",   -- Register 3 initialized to 0xDEF0
        4 => x"0000",   -- Register 4 initialized to 0x0000
        5 => x"1111",   -- Register 5 initialized to 0x1111
        6 => x"2222",   -- Register 6 initialized to 0x2222
        7 => x"3333"    -- Register 7 initialized to 0x3333
    );
begin
    -- Synchronous write and reset
    process(clk, rst)
    begin
        if rst = '1' then
            -- Reset registers to hardcoded values
            regs <= (
                0 => x"0001",
                1 => x"0002",
                2 => x"0003",
                3 => x"0004",
                4 => x"0005",
                5 => x"0006",
                6 => x"0007",
                7 => x"0008"
            );
        elsif rising_edge(clk) then
            -- Write to register if enabled
            if rd_wr_en = '1' then
                regs(to_integer(unsigned(rd_addr))) <= rd_data;
            end if;
        end if;
    end process;

    -- Asynchronous read ports
    rs1_data <= regs(to_integer(unsigned(rs1_addr)));
    rs2_data <= regs(to_integer(unsigned(rs2_addr)));
end architecture rtl;