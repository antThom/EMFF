#ifndef VEHICLEPARAM_H
#define VEHICLEPARAM_H

class vehicle {
	public:
		float currentSensor();
	private:
		//System Constants
		const float VCC_Reg = 5.0;
		const float VCC_Teensy = 3.3;
		const int daySec = 24*60*60;
		const float = 9.81;

		//Pin Assignment
		const int h_bridge_1 = 23;
		const int h_bridge_2 = 22;
		const int h_bridge_3 = 21;
		const int h_bridge_4 = 20;

}


#endif
