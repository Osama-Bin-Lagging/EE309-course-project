library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.constants_and_types.all;

entity alu is
    port (
        opcode  : in std_logic_vector(3 downto 0);
        operand1: in std_logic_vector(15 downto 0);
        operand2: in std_logic_vector(15 downto 0);
        result  : out std_logic_vector(15 downto 0)
    );
end entity alu;

architecture rtl of alu is
begin
    process(opcode, operand1, operand2)
    begin
        case opcode is
            when OPCODE_ADD | OPCODE_ADDI =>
                result <= std_logic_vector(unsigned(operand1) + unsigned(operand2));
            when OPCODE_SUB =>
                result <= std_logic_vector(unsigned(operand1) - unsigned(operand2));
            when OPCODE_MUL =>
                result <= std_logic_vector(unsigned(operand1(7 downto 0)) * unsigned(operand2(7 downto 0)));
            when OPCODE_SLL =>
                result <= std_logic_vector(shift_left(unsigned(operand1), to_integer(unsigned(operand2(3 downto 0)))));
            when others =>
                result <= operand1; -- Default pass-through for other opcodes
        end case;
    end process;
end architecture rtl;