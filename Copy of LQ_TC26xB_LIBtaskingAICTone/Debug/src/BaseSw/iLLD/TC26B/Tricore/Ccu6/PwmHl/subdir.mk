################################################################################
# Automatically-generated file. Do not edit!
################################################################################

C_FILES += "..\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.c"
OBJ_FILES += "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.o"
"src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.o" : "..\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.c" "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.opt"
	@echo Compiling ${<F}
	@"${PRODDIR}\bin\cctc" -f "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.opt"

"src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.opt" : .refresh
	@argfile "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.opt" -o "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.o" "..\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.c" -Ctc27xd --lsl-core=vtc -t -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone" -Wa-H"sfr/regtc27xd.def" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw\Tricore" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw\Tricore\APP" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw\Tricore\Main" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\CStart" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\Irq" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\Trap" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Build" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Impl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Lib" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_PinMap" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Asclin" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cif" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dma" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dsadc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dts" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Emem" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eray" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eth" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fce" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fft" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Flash" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gpt12" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Hssl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\I2c" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Iom" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Msc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Mtu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Multican" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Port" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5s" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Qspi" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Scu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Sent" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Smu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Src" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Stm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Vadc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Platform" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Platform\Tricore" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Platform\Tricore\Compilers" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Sfr" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Sfr\TC26B" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Sfr\TC26B\_Reg" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\_Utilities" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\If" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\If\Ccu6If" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\StdIf" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Bsp" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Comm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\General" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Math" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Time" -Wa-gAHLs --emit-locals=-equs,-symbols -Wa-Ogs -Wa--error-limit=42 -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw\Tricore\APP" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw\Tricore\Main" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\AppSw\Tricore\Driver" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Build" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Impl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Lib" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\_PinMap" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Asclin" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Asclin\Asc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Asclin\Lin" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Asclin\Spi" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Asclin\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\Icu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmBc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\Timer" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\TimerWithTrigger" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Ccu6\TPwm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cif" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cif\Cam" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cif\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\CStart" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\Irq" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Cpu\Trap" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dma" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dma\Dma" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dma\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dsadc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dsadc\Dsadc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dsadc\Rdc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dsadc\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dts" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dts\Dts" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Dts\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Emem" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Emem\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eray" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eray\Eray" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eray\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eth" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eth\Phy_Pef7071" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Eth\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fce" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fce\Crc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fce\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fft" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fft\Fft" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Fft\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Flash" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Flash\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gpt12" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gpt12\IncrEnc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gpt12\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Pwm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\PwmHl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Timer" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim\In" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Pwm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\PwmHl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Timer" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Gtm\Trig" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Hssl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Hssl\Hssl" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Hssl\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\I2c" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\I2c\I2c" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\I2c\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Iom" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Iom\Driver" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Iom\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Msc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Msc\Msc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Msc\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Mtu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Mtu\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Multican" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Multican\Can" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Multican\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Port" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Port\Io" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Port\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5\Psi5" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5s" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5s\Psi5s" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Psi5s\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Qspi" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiMaster" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiSlave" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Qspi\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Scu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Scu\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Sent" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Sent\Sent" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Sent\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Smu" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Smu\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Src" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Src\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Stm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Stm\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Stm\Timer" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Vadc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Vadc\Adc" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\iLLD\TC26B\Tricore\Vadc\Std" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Platform" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Platform\Tricore" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Platform\Tricore\Compilers" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Sfr" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Sfr\TC26B" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Infra\Sfr\TC26B\_Reg" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\_Utilities" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\If" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\If\Ccu6If" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\StdIf" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Bsp" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Comm" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\General" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Math" -I"E:\tasking workspace\Copy of LQ_TC26xB_LIBtaskingAICTone\src\BaseSw\Service\CpuGeneric\SysSe\Time" --iso=99 --language=-gcc,-volatile,+strings,-kanji --fp-model=3 --switch=auto --align=0 --default-near-size=0 --default-a0-size=0 --default-a1-size=0 -O2 --tradeoff=4 --compact-max-size=200 -g --error-limit=42 --source -c --dep-file="src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.d" -Wc--make-target="src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.o"
DEPENDENCY_FILES += "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.d"


GENERATED_FILES += "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.o" "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.opt" "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\.IfxCcu6_PwmHl.o.d" "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.src" "src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl\IfxCcu6_PwmHl.lst"
