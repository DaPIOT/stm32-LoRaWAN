#ifndef DHT_H_
#define DHT_H_

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;

__weak void delay(volatile uint32_t microseconds);
void DHT_GetData (DHT_DataTypedef *DHT_Data);

#endif 
