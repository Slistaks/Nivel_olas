pid:
	make -f makefile.pid

rfid:
	make -f makefile.rfid

temperatura:
	make -f makefile.temperatura

nivel:
	make -f makefile.nivel

presion:
	make -f makefile.presion

mpu:
	make -f makefile.mpu

.PHONY : clean
clean:
	rm -f Libs/*.o pid rfid temperatura nivel presion mpu