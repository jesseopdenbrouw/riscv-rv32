#
# General makefile that makes all targets
#
# First the srec2vhdl, srec2mif and upload
# are made. srec2vhdl is needed for the
# next build steps. Next is the library.
# Other targets depend on it.
#


all: 
	if [ ! -d bin ]; then mkdir bin; fi
	$(MAKE) -C srec2vhdl all
	cp srec2vhdl/srec2vhdl bin
	$(MAKE) -C srec2mif all
	cp srec2mif/srec2mif bin
	$(MAKE) -C upload all
	cp upload/upload bin
	$(MAKE) -C lib all
	$(MAKE) -C add64 all
	$(MAKE) -C assembler all
	$(MAKE) -C basel_problem all
	$(MAKE) -C bootloader all
	$(MAKE) -C clock all
	$(MAKE) -C coremark USER_FLAGS+=-DRUN_COREMARK EFFORT=-O3 clean_all exe
	$(MAKE) -C double all
	$(MAKE) -C exp all
	$(MAKE) -C fatfs all
	$(MAKE) -C flash all
	$(MAKE) -C float all
	$(MAKE) -C global all
	$(MAKE) -C hex_display all
	$(MAKE) -C i2c1findslaves all
	$(MAKE) -C i2c1tmp102 all
	$(MAKE) -C interrupt_direct all
	$(MAKE) -C interrupt_vectored all
	$(MAKE) -C interval all
	$(MAKE) -C ioadd all
	$(MAKE) -C linked_list all
	$(MAKE) -C malloc all
	$(MAKE) -C mcountinhibit all
	$(MAKE) -C monitor all
	$(MAKE) -C mult all
	$(MAKE) -C qsort all
	$(MAKE) -C riemann_left all
	$(MAKE) -C shift all
	$(MAKE) -C spi1readeeprom all
	$(MAKE) -C spi1softnss all
	$(MAKE) -C spi1speed all
	$(MAKE) -C spi1writeeeprom all
	$(MAKE) -C sprintf all
	$(MAKE) -C string all
	$(MAKE) -C structio all
	$(MAKE) -C testio all
	$(MAKE) -C timer2 all
	$(MAKE) -C trig all
	$(MAKE) -C uart1_cpp all
	$(MAKE) -C uart1_printf all
	$(MAKE) -C uart1_sprintf all

clean:
	$(MAKE) -C srec2vhdl clean
	$(MAKE) -C srec2mif clean
	$(MAKE) -C upload clean
	rm -rf bin
	$(MAKE) -C lib clean
	$(MAKE) -C add64 clean
	$(MAKE) -C assembler clean
	$(MAKE) -C basel_problem clean
	$(MAKE) -C bootloader clean
	$(MAKE) -C clock clean
	$(MAKE) -C coremark clean_all
	$(MAKE) -C double clean
	$(MAKE) -C exp clean
	$(MAKE) -C fatfs clean
	$(MAKE) -C flash clean
	$(MAKE) -C float clean
	$(MAKE) -C global clean
	$(MAKE) -C hex_display clean
	$(MAKE) -C i2c1findslaves clean
	$(MAKE) -C i2c1tmp102 clean
	$(MAKE) -C interrupt_direct clean
	$(MAKE) -C interrupt_vectored clean
	$(MAKE) -C interval clean
	$(MAKE) -C ioadd clean
	$(MAKE) -C linked_list clean
	$(MAKE) -C malloc clean
	$(MAKE) -C mcountinhibit clean
	$(MAKE) -C monitor clean
	$(MAKE) -C mult clean
	$(MAKE) -C qsort clean
	$(MAKE) -C riemann_left clean
	$(MAKE) -C shift clean
	$(MAKE) -C spi1readeeprom clean
	$(MAKE) -C spi1softnss clean
	$(MAKE) -C spi1speed clean
	$(MAKE) -C spi1writeeeprom clean
	$(MAKE) -C sprintf clean
	$(MAKE) -C string clean
	$(MAKE) -C structio clean
	$(MAKE) -C testio clean
	$(MAKE) -C timer2 clean
	$(MAKE) -C trig clean
	$(MAKE) -C uart1_cpp clean
	$(MAKE) -C uart1_printf clean
	$(MAKE) -C uart1_sprintf clean

