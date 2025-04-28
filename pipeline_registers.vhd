library ieee;
use ieee.std_logic_1164.all;
use work.constants_and_types.all;

entity pipeline_registers is
    port (
        clk         : in std_logic;
        rst         : in std_logic;
        if_id_in    : in if_id_reg_type;
        if_id_out   : out if_id_reg_type;
        id_ex_in    : in id_ex_reg_type;
        id_ex_out   : out id_ex_reg_type;
        ex_mem_in   : in ex_mem_reg_type;
        ex_mem_out  : out ex_mem_reg_type;
        mem_wb_in   : in mem_wb_reg_type;
        mem_wb_out  : out mem_wb_reg_type
    );
end entity;

architecture rtl of pipeline_registers is
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                -- Reset all pipeline registers
                if_id_out <= (
                    instruction => (others => '0'),
                    pc          => (others => '0')
                );
                id_ex_out <= (
                    opcode      => (others => '0'),
                    rd_addr     => (others => '0'),
                    rs1_addr    => (others => '0'),
                    rs2_addr    => (others => '0'),
                    rs1_data    => (others => '0'),
                    rs2_data    => (others => '0'),
                    immediate   => (others => '0'),
                    reg_write   => '0'
                );
                ex_mem_out <= (
                    opcode      => (others => '0'),
                    alu_result  => (others => '0'),
                    rs2_data    => (others => '0'),
                    rd_addr     => (others => '0'),
                    reg_write   => '0'
                );
                mem_wb_out <= (
                    result_data => (others => '0'),
                    rd_addr     => (others => '0'),
                    rd_wr_en    => '0'
                );
            else
                -- Propagate signals through pipeline stages
                if_id_out <= if_id_in;   -- IF -> ID
                id_ex_out <= id_ex_in;   -- ID -> EX
                ex_mem_out <= ex_mem_in; -- EX -> MEM
                mem_wb_out <= mem_wb_in; -- MEM -> WB
            end if;
        end if;
    end process;
end architecture;