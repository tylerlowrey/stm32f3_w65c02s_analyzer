RUST_TARGET=thumbv7em-none-eabihf
BINARY_NAME=stm32f3_digital_analyzer
BINARY_PATH=target/$(RUST_TARGET)
BINARY=$(BINARY_PATH)/release/$(BINARY_NAME)
DEBUG_BINARY=$(BINARY_PATH)/debug/$(BINARY_NAME)

all: $(BINARY)

$(BINARY):
	cargo build --target $(RUST_TARGET) --release

$(DEBUG_BINARY):
	cargo build --target $(RUST_TARGET)

objdump: $(BINARY)
	cargo objdump --release -- --disassemble-all --disassemble-zeroes

objdump-dbg: $(DEBUG_BINARY)
	cargo objdump -- --disassemble-all --disassemble-zeroes

openocd:
	openocd -f interface/stlink.cfg -f target/stm32f3x.cfg

gdb: $(DEBUG_BINARY)
	arm-none-eabi-gdb -q -ex "target remote :3333" $(DEBUG_BINARY)

minicom:
	minicom -D /dev/tty.usbmodem1411303 -b 115200

clean:
	rm -rf target