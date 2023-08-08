1) We have created 10 registers in our design of the program but this can easily be altered by adding extra registers in the module "register_field".
2) Values are stored in the register at runtime and these values are specified below -
(Where regfile is a 2D array of 10 registers each 32 bit long.)
regfile[0][31:0]<= 32'd0;
regfile[1][31:0]<= 10;
regfile[2][31:0]<= 0;
regfile[3][31:0]<= 5;
regfile[4][31:0]<= 0;
regfile[5][31:0]<= 0;
regfile[6][31:0]<= 3;
regfile[7][31:0]<= 0;
regfile[8][31:0]<= 0;
regfile[9][31:0]<= 0;

3) The required set of commands are specifed in a txt file named "Instn_mem.txt".
The instructions executed are -

sub $2, $1, $3
and $8, $2, $5
or $9, $6, $2
add $5, $5, $6
sub $4, $3, $6
beq $5,$6

4) After executing instructions using the values specified above we get our final output as- (output of each instruction after it crosses the WB stage and exits the pipeline)

5
0
7
3
2

5) Screenshot containing this information is also included in the zip file.
6) The output sequence is subject to change dependung on register values used.
7) Clock time period is 10 units.