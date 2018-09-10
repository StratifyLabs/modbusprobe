
#include <cstdio>
#include <sapi/chrono.hpp>
#include <sapi/sys.hpp>
#include <sapi/hal.hpp>
#include <sapi/var.hpp>
#include <modbus/mbus.hpp>

static void show_usage(Printer & p);

class ModbusUart : public ModbusRtu {
public:

	ModbusUart(UartAttributes & attributes, mcu_pin_t tx_enable) :
		m_tx_enable(tx_enable),
		m_uart(attributes.port()){

		m_bitrate = attributes.freq();
		if( m_uart.open(Uart::NONBLOCK | Uart::RDWR) < 0 ){
			set_error_message("Failed to open UART");
			return;
		}

		if( m_tx_enable.set_output() < 0 ){
			set_error_message("Failed to initialize tx enable");
			return;
		}

		if( m_uart.set_attributes(attributes) < 0 ){
			set_error_message("Failed to set UART attributes");
			return;
		}

	}

private:

	int write(const var::Data & data){
		m_tx_enable = true;
		Timer::wait_microseconds(500);
		int result = m_uart.write(data);
		packet_spacing().wait();
		m_tx_enable = false;
		Timer::wait_milliseconds(1);
		char c;
		m_uart.get(c);
		if( result != (int)data.size() ){
			set_error_message(String().format("failed to send %d bytes (%d, %d)", data.size(), result, m_uart.error_number()));
		}
		return result;
	}

	int read(var::Data & data){
		return m_uart.read(data);
	}

	int bitrate() const {
		return m_bitrate;
	}

	Pin m_tx_enable;
	Uart m_uart;
	u32 m_bitrate;

};


int main(int argc, char * argv[]){
	Cli cli(argc, argv);
	Printer p;
	cli.set_publisher("Stratify Labs, Inc");
	cli.handle_version();

	UartAttr uart_attr;

	if( cli.handle_uart(uart_attr) == false ){
		show_usage(p);
		exit(1);
	}

	ModbusUart modbus_uart(uart_attr, cli.get_option_pin("-tx_enable"));
	ModbusMaster modbus(modbus_uart);

	if( modbus_uart.error_message().is_empty() == false ){
		p.error("Failed to initialize modbus UART: %s", modbus_uart.error_message().to_char());
		exit(1);
	}

	if( modbus.initialize() < 0 ){
		p.error("Invalid Uart: %s", modbus.error_message().str());
		show_usage(p);
	}

	p.open_object("uart attributes");
	p.key("freq", "%ld", uart_attr.freq());
	if( uart_attr.o_flags() & Uart::IS_STOP1 ){
		p.key("stop bits", "1");
	} else if( uart_attr.o_flags() & Uart::IS_STOP2 ){
		p.key("stop bits", "2");
	}
	if( uart_attr.o_flags() & Uart::IS_PARITY_EVEN ){
		p.key("parity", "even");
	} else if( uart_attr.o_flags() & Uart::IS_PARITY_ODD ){
		p.key("parity", "odd");
	} else {
		p.key("parity", "none");
	}
	p.key("width", String().format("%d", uart_attr.width()));
	p.close_object();

	if( cli.is_option("-write") ){
		u16 device_address = cli.get_option_value("-device_address");
		u16 register_address = cli.get_option_value("-register_address");
		if( cli.is_option("-value") == false ){
			show_usage(p);
			exit(1);
		}
		u16 value = cli.get_option_value("-value");
		p.open_object("preset single register");
		p.key("device address", "%d", device_address);
		p.key("register address", "%d", register_address);
		p.key("value", "%d", value);
		if( modbus.preset_single_register(device_address, register_address, value) < 0 ){
			p.error("Failed to preset single register: %s", modbus.error_message().str());
		}


		p.close_object();
	}

	if( cli.is_option("-read") ){
		u16 device_address = cli.get_option_value("-device_address");
		u16 register_address = cli.get_option_value("-register_address");
		u16 number_of_points = cli.get_option_value("-number_of_points");

		var::Data result;
		result = modbus.read_holding_registers(device_address, register_address, number_of_points);
		p.open_object("read holding registers");
		p.key("device address", "%X", device_address);
		p.key("register address", "%X", register_address);
		p.key("number of points", "%X", number_of_points);

		if( number_of_points == 0 ){
			p.error("invalid number of points");
			p.key("result", "fail");

		} else {

			if( result.size() == 0 ){
				p.error("Failed to read holding registers: %s", modbus.error_message().str());
				p.key("result", "fail");
			} else {
				p.open_object("holding registers");
				for(u32 i = 0; i < number_of_points; i++){
					p.key(String().format("[%ld]", register_address + i*2), "%d", result.at_u16(i));
				}
				p.close_object();
				p.key("result", "success");
			}
		}

		p.close_object();
	}

	return 0;
}

void show_usage(Printer & p){

	p.open_object("modbusprobe");
	p.key("usage", "modbusuart -uart <port> -tx <x.y> -rx <x.y> -tx_enable <x.y> -f <bitrate>");
}


#if 0
void modbus_test(){
	Pin tx_enable(1,8);
	Data data(64);
	Printer p;
	int result;

	Uart uart(1); //usart 2

	Fifo fifo;

	tx_enable.set_output();
	tx_enable = false;

	UartPinAssignment pin_assignment;
	pin_assignment->rx = mcu_pin(3,6);
	pin_assignment->tx = mcu_pin(3,5);

	if( (result = uart.init(Uart::SET_LINE_CODING_DEFAULT, 9600, 8, pin_assignment)) < 0 ){
		p.error("Failed to initialize UART (%d,%d)", result, uart.error_number());
		exit(1);
	}

	//if anything is received send it back
	p.message("Start waiting for data on the UART");
	fflush(stdout);

	Timer t;

	fifo.set_fileno(uart);
	u32 last_used = 0;
	while(1){
		FifoInfo info = fifo.get_info();

		if( info.used() > 0 ){
			if( info.used() != last_used ){
				t.restart();
				last_used = info.used();
				p.message("%d bytes arrived (%ld)", last_used, t.microseconds());
			}
			if( t.milliseconds() > 250 ){
				data.set_size(info.used());
				data.fill(0);
				if( uart.read(data) > 0 ){
					p.message("Read %d bytes from UART: %s", data.size(), data.cdata());
					fflush(stdout);
					//echo the packet
					tx_enable = true;
					uart.write(data);
					Timer::wait_milliseconds(10);
					tx_enable = false;
				} else {
					p.error("Failed to read data from uart");
					fflush(stdout);
				}
			}
		}
		Timer::wait_milliseconds(10);
	}
}
#endif
