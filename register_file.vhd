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
        rd_data   : in  std_logic_vector(15 downto 0);
        
        -- Adding register value outputs for monitoring
        reg0_val  : out std_logic_vector(15 downto 0);
        reg1_val  : out std_logic_vector(15 downto 0);
        reg2_val  : out std_logic_vector(15 downto 0);
        reg3_val  : out std_logic_vector(15 downto 0);
        reg4_val  : out std_logic_vector(15 downto 0);
        reg5_val  : out std_logic_vector(15 downto 0);
        reg6_val  : out std_logic_vector(15 downto 0);
        reg7_val  : out std_logic_vector(15 downto 0)
    );
end entity register_file;

architecture rtl of register_file is
    -- Define initial values for registers (hardcoded)
    type reg_file_type is array (0 to 7) of std_logic_vector(15 downto 0);
    signal regs : reg_file_type := (
        0 => x"0000",   -- Register 0 initialized to 0x0000
        1 => x"0000",   -- Register 1 initialized to 0x0000
        2 => x"0000",   -- Register 2 initialized to 0x0000
        3 => x"0000",   -- Register 3 initialized to 0x0000
        4 => x"0000",   -- Register 4 initialized to 0x0000
        5 => x"0000",   -- Register 5 initialized to 0x0000
        6 => x"0000",   -- Register 6 initialized to 0x0000
        7 => x"0000"    -- Register 7 initialized to 0x0000
    );
    
    -- Internal registers to store current state for combinational reads
    signal regs_internal : reg_file_type := (
        0 => x"0000",
        1 => x"0000",
        2 => x"0000",
        3 => x"0000",
        4 => x"0000",
        5 => x"0000",
        6 => x"0000",
        7 => x"0000"
    );
begin
    -- Synchronous write and reset
    process(clk, rst)
    begin
        if rst = '1' then
            -- Reset registers to hardcoded values
            regs <= (
                0 => x"0000",
                1 => x"0000",
                2 => x"0000",
                3 => x"0000",
                4 => x"0000",
                5 => x"0000",
                6 => x"0000",
                7 => x"0000"
            );
            regs_internal <= (
                0 => x"0000",
                1 => x"0000",
                2 => x"0003",
                3 => x"0000",
                4 => x"0000",
                5 => x"0000",
                6 => x"0000",
                7 => x"0000"
            );
        elsif rising_edge(clk) then
            -- Update internal state with registered values
            regs_internal <= regs;
            
            -- Write to register if enabled (update happens in same cycle)
            if rd_wr_en = '1' then
                regs(to_integer(unsigned(rd_addr))) <= rd_data;
            end if;
        end if;
    end process;
    
    -- Read from the most up-to-date values
    -- This process handles read-after-write hazards in the same cycle
    READ_PROCESS: process(regs, regs_internal, rs1_addr, rs2_addr, rd_addr, rd_data, rd_wr_en)
    begin
        -- Default: read from internal registered values
        rs1_data <= regs_internal(to_integer(unsigned(rs1_addr)));
        rs2_data <= regs_internal(to_integer(unsigned(rs2_addr)));
        
        -- If there's a write in this cycle and the read address matches the write address,
        -- bypass the register and return the write data directly
        if rd_wr_en = '1' then
            if rs1_addr = rd_addr then
                rs1_data <= rd_data;
            end if;
            
            if rs2_addr = rd_addr then
                rs2_data <= rd_data;
            end if;
        end if;
    end process READ_PROCESS;
    
    -- Expose register values for simulation monitoring
    -- These reflect the most up-to-date values
    MONITORING: process(regs, rd_addr, rd_data, rd_wr_en)
    begin
        -- Default values from registers
        reg0_val <= regs(0);
        reg1_val <= regs(1);
        reg2_val <= regs(2);
        reg3_val <= regs(3);
        reg4_val <= regs(4);
        reg5_val <= regs(5);
        reg6_val <= regs(6);
        reg7_val <= regs(7);
        
        -- If there's a write in this cycle, show the updated value for monitoring
        if rd_wr_en = '1' then
            case rd_addr is
                when "000" => reg0_val <= rd_data;
                when "001" => reg1_val <= rd_data;
                when "010" => reg2_val <= rd_data;
                when "011" => reg3_val <= rd_data;
                when "100" => reg4_val <= rd_data;
                when "101" => reg5_val <= rd_data;
                when "110" => reg6_val <= rd_data;
                when "111" => reg7_val <= rd_data;
                when others => null;
            end case;
        end if;
    end process MONITORING;
end architecture rtl;