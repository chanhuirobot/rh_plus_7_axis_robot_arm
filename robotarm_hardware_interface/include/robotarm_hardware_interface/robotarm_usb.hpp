
#ifndef ROBOTARM_USB__H
#define ROBOTARM_USB__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "robotarm_drvr.hpp"

namespace robotarm
{
	class robotarm_usb: public robotarm_drvr
	{
		public:
			robotarm_usb();
			~robotarm_usb();

			bool open(const std::string &portname) override;
			void close() override;

			// fix - implement
			virtual bool getJointPosition(int id, uint16_t &pos) override;
			// remove
			bool readJointPositionAll(std::vector<uint16_t> &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			void printDeviceInformation();

			hid_device *handle_;
	};
}

#endif // ROBOTARM_USB__H
