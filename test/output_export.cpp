
# include <Arduino.h>
# include <output_export.h>



void output_char(int a)
{
    Serial.write(a);
}

void output_flush(void)
{
    Serial.flush();
}

void output_start(unsigned int baudrate)
{
    Serial.begin(9600);
}

void output_complete(void)
{
   Serial.end();
}

