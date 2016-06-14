#include "sensor.h"

bool sensorFront()
{
    return LPC_GPIO4->FIOPIN & PIN(29);
}
bool sensorRain()
{
    return LPC_GPIO1->FIOPIN & PIN(29);
}
bool sensorCover()
{
    return LPC_GPIO4->FIOPIN & PIN(28);
}
bool sensorLift()
{
    return LPC_GPIO1->FIOPIN & PIN(16);
}
bool sensorCharger()
{
    return LPC_GPIO1->FIOPIN & PIN(21);
}
uint8_t sensorDIP()
{
    uint8_t val = 0;
    val = ((LPC_GPIO2->FIOPIN >> 7) & 1) | (((LPC_GPIO2->FIOPIN >> 3) & 1) << 1) | (((LPC_GPIO0->FIOPIN >> 1) & 1) << 2);
    return val;
}
uint8_t sensorWireR()
{
    uint8_t val = 0;
    val = ((LPC_GPIO0->FIOPIN >> 9) & 1) | (((LPC_GPIO0->FIOPIN >> 10) & 1) << 1);
    return val;
}
uint8_t sensorWireL()
{
    uint8_t val = 0;
    val = ((LPC_GPIO0->FIOPIN >> 7) & 1) | (((LPC_GPIO0->FIOPIN >> 8) & 1) << 1);
    return val;
}

//sens right
//sens left
//in-charger / Charging
//a-Batt Temp
//a-Batt Volt
//a-Current R?
//a-Current L?
//a-Current S?


//motor?
//c-RPM spindle
//c-speed wheel R
//c-speed wheel L
//is spinning 1-2-3
//has brake 1-2-3
