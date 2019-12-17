#include <drivers/device/spi.h>

class MPU6000_SPI : public device::SPI
{
public:
	MPU6000_SPI(int bus, uint32_t device, int device_type);
	~MPU6000_SPI() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int probe() override;

private:

	int _device_type;
	/* Helper to set the desired speed and isolate the register on return */

	int _max_frequency;
	void set_bus_frequency(unsigned &reg_speed_reg_out);
};

