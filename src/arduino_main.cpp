#include <Arduino.h>

extern "C" void lamp_init(void);

void setup(void)
{
    lamp_init();
}

void loop(void)
{
    delay(portMAX_DELAY);
}
