library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.constants_and_types.all;

entity alu is
    port (
        opcode   : in  std_logic_vector(3 downto 0);
        operand1 : in  std_logic_vector(15 downto 0);
        operand2 : in  std_logic_vector(15 downto 0);
        result   : out std_logic_vector(15 downto 0)
    );
end entity alu;

architecture rtl of alu is
begin
    process(opcode, operand1, operand2)
    begin
        case opcode is
            when OPCODE_ADD | OPCODE_ADDI =>
                -- Regular addition for both ADD and ADDI
                result <= std_logic_vector(unsigned(operand1) + unsigned(operand2));
                
            when OPCODE_SUB =>
                -- Subtraction (assuming no borrow required as per ISA)
                result <= std_logic_vector(unsigned(operand1) - unsigned(operand2));
                
            when OPCODE_MUL =>
                -- Multiplication (only storing lowest 16 bits as per ISA)
                result <= std_logic_vector(resize(unsigned(operand1) * unsigned(operand2), 16));
                
            when OPCODE_SLL =>
                -- Shift left logical by amount in lowest 4 bits of operand2
                result <= std_logic_vector(shift_left(unsigned(operand1), to_integer(unsigned(operand2(3 downto 0)))));
                
            when OPCODE_JRI =>
                -- For JRI, ALU passes operand1 (will be used elsewhere for jump calculation)
                result <= operand1;
                
            when OPCODE_LW | OPCODE_SW =>
                -- For Load/Store, ALU passes operand1 (register value)
                result <= operand1;
                
            when others =>
                -- Default case
                result <= (others => '0');
        end case;
    end process;
end architecture rtl;