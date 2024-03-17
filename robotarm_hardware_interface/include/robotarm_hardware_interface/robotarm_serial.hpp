
#ifndef ROBOTARM_SERIAL__H
#define ROBOTARM_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "robotarm_drvr.hpp"

namespace robotarm
{
	class robotarm_serial: public robotarm_drvr
	{
		public:
			robotarm_serial();
			~robotarm_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // ROBOTARM_SERIAL__H
