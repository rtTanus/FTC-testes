
/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="TesteTO", group="OpMode")
public class TeleopTeste extends OpMode{
    // cria????o das vari??veis para os motores de movimenta????o
    // do sistema linear e tamb??m do servo motor
    public Servo servoMotor = null;
    public DcMotor MEF, MET, MDF, MDT = null;
    public DcMotor Arm = null;;

    @Override
    public void init(){


        // Nesta parte do codigo nos classicamos as variaveis
        // dos motores de movimento
        MEF = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        MDF = hardwareMap.get(DcMotor.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        // E nesta parte classificamos o motor do sistema linear
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        // E aqui o motor servo
        servoMotor = hardwareMap.get(Servo.class, "Servo");


        // Em seguida nos indentificamos as dire????es
        // dos motores de movimento

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);

        // E tamb??m classificamos o motor do sistema linear

        Arm.setDirection(DcMotor.Direction.FORWARD);



    }

    // aqui criamos a fun????o loop
    public void loop(){
        // Uso das fun????es criadas no loop
        servo();
        mover();
        linear();
    }

    //Cria????o da fun????o da movimenta????o

    public void mover(){

        // Cria????o das v??riaveis de for??a para a movimenta????o
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Cria????o de v??riaveis para a movimenta????o em 8 dire????es
        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);

        // Calculos para a movimenta????o das rodas omnidirectionais mecanum
        double MEFf = (axial + lateral + yaw / denominador);
        double MDFf = (axial - lateral - yaw / denominador);
        double METf = (axial - lateral + yaw / denominador);
        double MDTf = (axial + lateral - yaw / denominador);

        // Fun????o que define que as For??as utilizadas em cada motor

        if(gamepad1.right_bumper){
        allMotorsPower(MEFf, MDFf, METf, MDTf);
        }
        else{
        allMotorsPower(MEFf * 0.8, MDFf * 0.8, METf * 0.8, MDTf * 0.8);
        }


        // Nessa sequencia de if nos selecionamos as setas dos controles como meios
        // de movimenta????o mais precisa, sendo mais lentas e tendo um controle em ambas as 8 dire????es

        // Seta de baixo
        if(gamepad1.dpad_down){
            allMotorsPower(-0.5,-0.5,-0.5,-0.5);
        }

        // Seta de cima
        if(gamepad1.dpad_up){
            allMotorsPower(0.5,0.5,0.5,0.5);
        }

        // Seta da direita
        if(gamepad1.dpad_right){
            allMotorsPower(0.5,-0.5,-0.5,0.5);
        }

        // Seta da esquerda
        if(gamepad1.dpad_left){
            allMotorsPower(-0.5,0.5,0.5,-0.5);
        }


        // Nestas linhas de c??digos n??s usamos o comando Telemetry para
        // Conseguirmos ver as for??as dos motores de movimenta????o no Drive Hub

        telemetry.addData("A potencia do motorEsquerdoF ?? de:", MEFf);
        telemetry.addData("A potencia do motorDireitoF ?? de:", MDFf);
        telemetry.addData("A potencia do motorEsquerdoT ?? de:", METf);
        telemetry.addData("A potencia do motorDireitoT ?? de:", MDTf);

    }

    // Cria????o da Fun????o que define o poder de todos
    // os motores utilizados na movimenta????o

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    //Cria????o da fun????o do sistema linear

    public void linear() {
        // Cria????o das vari??veis utilizadas para definir as for??as
        // utilizadas no uso do sistema linear
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean poderImovel = gamepad2.x;
        double pow;

        // Nessa sequencia de if, nos colocamos os poderes para elevar
        // o sistema linear, para abaixar o sistema linear e tamb??m
        // para o ele estar imovel no ar

        // Para subir o sistema linear

        if(poderCima){
            pow = -0.6;
            Arm.setPower(pow);
        }

        // Para descer o sistema linear

        if(poderBaixo){
            pow = 0.3;
            Arm.setPower(pow);
        }

        // Para deixar o sistema linear imovel no ar

        if(poderImovel){
            pow = -0.2;
            Arm.setPower(pow);
        }

        // Para deixar o sistema linear sem poder caso nenhum dos bot??es seja apertado

        else{
            pow = 0;
            Arm.setPower(pow);
        }

        // Fun????o que permite que o poder do motor do sistema linear
        // seja exibido no Drive Hub
        telemetry.addData("A potencia do motor do sistema linear ?? de", pow);

    }

    // Cria????o da v??riavel de poder do servo fora da fun????o servo
    // para assim n??o ser necess??rio pressionar o bot??o para definir a for??a
    // e sim s?? apenas apertar

    double powServo = 0;

    //Cria????o da fun????o do servo
    public void servo() {

        // Cria????o das var??aveis tanto de for??a quanto dos bot??es
        // utilizados para abrir e fechar o servo

        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double aberto = 0;
        double fechado = 1;

        // Fun????o que define o servo estar aberto

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);
        }

        // Fun????o que define o servo estar fechado

        else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);
        }

        // Fun????o que mostra o poder do servo no Drive Hub
        telemetry.addData("A potencia do motor do servo ?? de:", powServo);
    }
}
