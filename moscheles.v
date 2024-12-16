`default_nettype none
module processor( input         clk, reset,
                  output [31:0] pc,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
    
    wire [31:0]PCPlus4, plusimm, ImmOp, BranchTarget, tores, res, PCn, SrcA, SrcB, writedata;
    wire BranchBeq, BranchJal, BranchJalr, BranchJalx, zero, BranchOutcome, MemWrite;
    wire ALUSrc, MemToReg, RegWrite, reset;
    wire [2:0] ALUControl;
    wire [2:0] immControl; 
    //adders
    adder pc4(pc, 4, PCPlus4);
    adder pcimm(pc, ImmOp, plusimm);
    //multiplexers
    mux32_1 branchJalop(address_to_mem, plusimm, BranchJalr, BranchTarget);
    mux32_1 muxa(PCPlus4, address_to_mem, BranchJalx, tores );
    mux32_1 muxres(data_from_mem, tores, MemToReg, res);
    mux32_1 muxPC(BranchTarget, PCPlus4, BranchOutcome, PCn);
    mux32_1 muxALU(ImmOp, writedata, ALUSrc, SrcB );

    alu ALU(SrcA, SrcB, ALUControl, address_to_mem, zero);

    registry registry(instruction [19:15], instruction [24:20], instruction [11:7],
                clk, RegWrite, SrcA, data_to_mem, res);
//jiný WE

    pc ProgCount(PCn, clk, reset, pc);
    
    immDecode immdecoder(instruction, ImmOp, immControl);

    ControlUnit radic( instruction [31:0], BranchBeq, BranchJal, BranchJalr, RegWrite, MemToReg, MemWrite, ALUControl, ALUSrc, immControl [2:0]);

    branch branchout(BranchJalr, BranchJal, BranchBeq, zero, BranchJalx, BranchOutcome);

    assign writedata = data_to_mem;
    assign WE = MemWrite;
endmodule


module mux32_1(input [31:0] a, b, input select,
                 output reg [31:0] y);
    always @(*)
	if(select)
	y=a;
	else y=b;
endmodule

module adder(input [31:0] a,  b, output [31:0] sum);
    assign sum=a+b;
endmodule

module alu (
    input [31:0] SrcA, input [31:0] SrcB, input [2:0] ALUControl, 
    output reg [31:0] ALUResult, output reg zero );
    
    always @(*) begin
        casez (ALUControl)
        //add
        3'b000 :  ALUResult = SrcA + SrcB; 
        //sub
        3'b001 : ALUResult = SrcA - SrcB;
        //and
        3'b010 : ALUResult = SrcA & SrcB;
        //slr
        3'b011 :ALUResult = $signed(SrcA) >> $signed(SrcB[4:0]);
        //blt
        3'b100 : ALUResult = $signed(SrcA) >= $signed(SrcB);
        //lui
        3'b101 : ALUResult = SrcB;

        //floor
        3'b111 : ALUResult = SrcA[30:23] - 8'd127;

    endcase
                zero = (ALUResult == 0);

    end


endmodule

module pc( input [31:0] PCN, input clk, input reset, output reg [31:0] pc);
    always @(posedge clk or posedge reset) begin
        casez (reset)
            1'b0 : pc = PCN;
            1'b1 : pc = 32'b0;
        endcase
    end
endmodule

module immDecode(input [31:0] instruction, output reg [31:0] ImmOp, input [2:0] operand);

    always @(*) begin
        casez (operand)
            3'b000 : ImmOp = 32'b0; //R-type
            3'b001 : ImmOp = {{20{instruction[31]}},instruction[31:20]}; //I-type
            3'b010 : ImmOp = {{20{instruction[31]}},instruction[31:25],instruction[11:7]}; //S-type
            3'b011 : ImmOp = {{19{instruction[31]}},instruction[31],instruction[7],instruction[30:25],instruction[11:8], 1'b0}; //B-type
            3'b100 : ImmOp = { instruction[31:12], 12'b0}; //U-type
            3'b101 : ImmOp = {{11{instruction[31]}},instruction[31],instruction[19:12],instruction[20],instruction[30:21],1'b0}; //J-type
            default: ImmOp = 32'b0;

        endcase
    end

endmodule

module ControlUnit(input [31:0] instruction, output reg BranchBeq, output reg BranchJal, output reg BranchJalr, output reg RegWrite, output reg MemToReg, output reg MemWrite, output reg [2:0] ALUControl, output reg ALUSrc, output reg [2:0] immControl);

//operand
    always @(*) begin

        BranchBeq = 0;
        BranchJal = 0;
        BranchJalr = 0;
        //BranchVal = 0;
        RegWrite = 0;
        MemToReg = 0;
        MemWrite = 0;
        ALUControl = 3'b000;
        ALUSrc = 0;
        immControl = 3'b000;

        casez (instruction [6:0])
            7'b0110011 : begin
                
                //add,and,sub,srl
                RegWrite=1;
                immControl=3'bxxx;
                casez (instruction [14:12])
                    3'b000 : begin
                        casez (instruction [31:30] )
                        2'b00 : ALUControl = 3'b000;
                        2'b01 : ALUControl = 3'b001;
                        endcase
                    end
                    3'b111 : ALUControl = 3'b010;
                    3'b101 : ALUControl = 3'b011;
                endcase
            end
            7'b0010011 : begin
                ALUSrc = 1;
                RegWrite = 1;
                ALUControl = 3'b000; 
                immControl = 3'b001; //I-type
                //addi
            end
            7'b1100011 : begin    
                immControl = 3'b011; // B
                MemToReg = 0;
                BranchBeq = 1;              
                casez(instruction[14:12])
                    3'b000 : ALUControl = 3'b001; //-
                    3'b100 : begin 
                        ALUControl = 3'b100; //<
                        //BranchVal = 1;
                    end
                endcase
             //beq,blt // tady chyba
             // zkusit napsat program, co skáče nahoru
            end
            7'b0000011 : begin
               ALUControl = 3'b000; //+
               immControl = 3'b001; // I-type
               RegWrite = 1;
               ALUSrc = 1;
               MemToReg = 1;
             // lw
            end
            7'b0100011 : begin
                ALUControl = 3'b000; //+
                MemWrite = 1;
                ALUSrc = 1;
                immControl = 3'b010;//S-type
                 //sw
            end
            7'b0110111 : begin 
                RegWrite = 1; 
                immControl = 3'b100; //U-type
                ALUSrc = 1;
                ALUControl = 3'b101;
                //lui
            end
            7'b1101111 : begin
                immControl = 3'b101; //J-type
                RegWrite = 1;
                ALUSrc = 1'bx;
                ALUControl = 3'bxxx;
                BranchJal=1; //jal
            end
            7'b1100111 : begin
                immControl = 3'b001;
                RegWrite = 1;
                ALUSrc = 1;
                BranchJalr= 1;
                ALUControl = 3'b000; //+

             //jalr
            end
            7'b0001011 : begin
             RegWrite = 1;
             ALUControl = 3'b111;
             ALUSrc = 0;
             immControl = 3'b001; // I-type
             //floor_log
            end
            /*
            7'b1000101 : begin
            //KTI instrukce
            end
            */

        endcase
    
    end

endmodule

module branch(input BranchJalr, input BranchJal, input BranchBeq, input zero, output BranchJalx, output BranchOutcome );
    assign BranchJalx = BranchJalr | BranchJal;
    assign BranchOutcome = (BranchBeq & zero) | BranchJalx;
endmodule

module registry (
    input [4:0] A1, input [4:0] A2, input [4:0] A3, input CLK, input WE3,
    output reg [31:0] RD1, output reg [31:0] RD2, input [31:0] WD3 );
    
    reg [31:0] registry [31:0];
    
    //a1 - rs1
    //a2 - rs2
    //a3 - rd

    //čtení
    always@(*)
    begin
       //reg 0 má 0 konstantu
        registry[0] = 0;

        RD1 = registry[A1];
        RD2 = registry[A2];


    end

    //zápis
    always @(posedge CLK) begin
            if(A3!=0 && WE3 == 1)   
                registry[A3] = WD3; 
        end
    
endmodule

`default_nettype wire
