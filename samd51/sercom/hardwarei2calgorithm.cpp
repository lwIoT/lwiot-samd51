/*
 * Hardware I2C algorithm implementation.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

/*
 * Hardware I2C algorithm definition.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include <stdlib.h>
#include <lwiot.h>

#include <lwiot/types.h>
#include <lwiot/log.h>
#include <lwiot/sharedpointer.h>

#include <lwiot/io/i2calgorithm.h>
#include <lwiot/io/gpiopin.h>
#include <lwiot/kernel/lock.h>

#include <lwiot/io/hardwarei2calgorithm.h>

#include <lwiot/samd51/sercom.h>
#include <lwiot/samd51/hardwarei2calgorithm.h>

#define CMD_STA 1U
#define CMD_STO 2U
#define CMD_REPSTA 4U
#define CMD_RD 8U
#define CMD_ACK 16U

struct i2c_msg {
	uint8_t *buff;
	uint8_t sla;
	size_t len;
	uint8_t flags;
};


namespace lwiot
{
	namespace samd51
	{
		static volatile uint8_t msg_index = 0;
		static volatile uint8_t msg_num = 0;
		static constexpr auto MAX_MESSAGES = 12;
		static struct i2c_msg xfer_msgs[MAX_MESSAGES];

		HardwareI2CAlgorithm::HardwareI2CAlgorithm(const lwiot::GpioPin &scl, const lwiot::GpioPin &sda,
				uint32_t frequency, Sercom* coms) : lwiot::HardwareI2CAlgorithm(scl, sda, frequency),
													sercom_(new SERCOM(coms))
		{
			this->setFrequency(frequency);
		}

		HardwareI2CAlgorithm::HardwareI2CAlgorithm(HardwareI2CAlgorithm &&other) noexcept :
				lwiot::HardwareI2CAlgorithm(stl::forward<HardwareI2CAlgorithm>(other)), sercom_(stl::move(other.sercom_))
		{
		}

		HardwareI2CAlgorithm::~HardwareI2CAlgorithm() = default;

		HardwareI2CAlgorithm& HardwareI2CAlgorithm::operator=(HardwareI2CAlgorithm &&rhs) noexcept
		{
			this->move(rhs);
			this->sercom_ = stl::move(rhs.sercom_);
			return *this;
		}

		void HardwareI2CAlgorithm::start(uint16_t sla, bool repeated)
		{
			struct i2c_msg *msg;

			msg = &xfer_msgs[msg_index];
			msg->sla = (uint8_t) sla;

			if(repeated)
				msg->flags |= CMD_REPSTA;
			else
				msg->flags |= CMD_STA;

			msg->buff = (uint8_t *) &msg->sla;
			msg->len = sizeof(uint8_t);

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::stop()
		{
			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index - 1];
			msg->flags |= CMD_STO;
		}

		void HardwareI2CAlgorithm::write(const uint8_t *byte, bool ack)
		{
			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index];
			msg->buff = (uint8_t*) byte;
			msg->len = sizeof(*byte);

			if(ack)
				msg->flags = CMD_ACK;

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::write(const uint8_t *bytes, size_t length, bool ack)
		{
			struct i2c_msg *msg ;

			UNUSED(ack);

			msg = &xfer_msgs[msg_index];
			msg->buff = (uint8_t*) bytes;
			msg->len = length;

			if(ack)
				msg->flags |= CMD_ACK;

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::read(uint8_t *byte, bool ack)
		{

			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index];
			msg->buff = byte;
			msg->len = sizeof(*byte);

			if(ack)
				msg->flags |= CMD_ACK;

			msg->flags |= CMD_RD;

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::read(uint8_t *bytes, size_t length, bool ack)
		{
			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index];
			msg->buff = bytes;
			msg->len = length;

			if(ack)
				msg->flags |= CMD_ACK;

			msg->flags |= CMD_RD;

			msg_index += 1;
			msg_num += 1;
		}

		int HardwareI2CAlgorithm::flush() const
		{
			struct i2c_msg *msg;
			bool start = false;

			msg_index = 0;

			for(int idx = 0; idx < msg_num; idx++) {
				msg = &xfer_msgs[idx];

				if(msg->flags & CMD_STA || msg->flags & CMD_REPSTA) {
					start = true;
					continue;
				}

				if(start) {
					auto flag = msg->flags & CMD_RD ? WIRE_READ_FLAG : WIRE_WRITE_FLAG;
					this->sercom_->startTransmissionWIRE(msg->sla, flag);
					start = false;
				}

				if((msg->flags & CMD_RD) != 0) {
					msg->buff[0] = this->sercom_->readDataWIRE();

					for(auto j = 1U; j < msg->len; j++) {
						this->sercom_->prepareAckBitWIRE();
						this->sercom_->prepareCommandBitsWire(WIRE_MASTER_ACT_READ);
						msg->buff[j] = this->sercom_->readDataWIRE();
					}
				} else {
					for(auto j = 0U; j < msg->len; j++) {
						auto result = this->sercom_->sendDataMasterWIRE(msg->buff[j]);


						if(!result) {
							print_dbg("Unable to write data!");
							return -EINVALID;
						}
					}
				}

				if(msg->flags & CMD_STO) {
					this->sercom_->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
				}
			}

			return -EOK;
		}

		void HardwareI2CAlgorithm::reset()
		{
			memset(xfer_msgs, 0, sizeof(xfer_msgs));
			msg_index = 0;
			msg_num = 0;
		}

		void HardwareI2CAlgorithm::setFrequency(const uint32_t &freq)
		{
		}
	}
}
