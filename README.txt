  Arduino-Uno-FreeRtos-CodeBlocks


  This is a demonstration of use of FreeRTOS in form the PWM control system for SMPS (Switching Mode Power Supply). This steps can be used for configuration other projects using FreeRTOS in the atmega328p or simply programming the arduino in Code::Blocks, an IDE superior that the Arduino IDE.

  First, install the Code::Blocks, the Arduino IDE (we will only use avrdude) and the GNU-AVR compiler, in the links below. In Windows can be install the WinAvr.

http://www.codeblocks.org/
https://www.arduino.cc/en/main/software
http://winavr.sourceforge.net/


  After that, download and open this project and try to compile. Make sure that the GNU-AVR compiler has been recognized by Code::Blocks.

  To configure the recorder, you must open the Tools -> Configure Tools menu and enter the following:

    Name:		->	Anything

    Executable:	->	The path of the archive "avrdude.exe". In my case C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude

    Parameters:	->	-F -v -v -v -v -pm328p -carduino -PCOM5 -b57600 -Uflash:w:"${TARGET_OUTPUT_BASENAME}.hex":i -C"C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf"
    Substitute the com port for your and the path for your patch of the file "avrdude.conf". The baudrate varies, and you can search or use trial and error.

    Working directory:	->	${PROJECT_DIR}\bin\Release

  I prefer the option Launch tool visible detached, but anything works.

  And is it. For record, just connect the board and click in the recorder configured.

  A demo video for this project is in link follow. In this video i wore the Eclipse IDE, but the program is the same.
  https://youtu.be/TNB-DN34b9c

//==================================================================

Eric Pusiol

Engenharia Elétrica

Núcleo de Iluminação Moderna - Universidade Federal de Juiz de Fora
            __               |
         __/  \__            |               /\
       _/        \_          |             /   \     /\
      /  |\    |   \         |      |    |  ┌──┐\  /   ┌──┐
     /   | \   |    \        |      |  / |  |    /     |\
     |   |  \  |    |        |      |/   | ─┼─ /  |   ─┼─\
     \   |   \ |    /        |     /_\__/ __|/    |    |  \
      \_ |    \|  _/         |             /| ___ | __ | __\
        \__    __/           |                    |
           \__/              |                └───┘
