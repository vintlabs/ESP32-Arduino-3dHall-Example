/*
 *  Example code for reading an ALS31313 3D Hall Effect Sensor on ESP32 in Arduino environment.
 *  VintLabs 3dHall Module available at https://www.amazon.ca/dp/B0896VD73G
 *  
 *  Compile and run - I recommend using the serial plotter rather than the serial console for testing!
 *  
 *  
 *  Much of the code is taken from the example code provided by Allegro Microsystems:
 *    Example source code for an Arduino to show
 *    how to communicate with an Allegro ALS31300
 *
 *    Written by K. Robert Bate, Allegro MicroSystems, LLC.
 *
 *    ALS31300Demo is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

// Redefine these if you're using non-standard pins
//#define SDA = 5
//#define SCL = 4
#include <Wire.h>
#include <math.h>


char xyzB[128] = "";
char hall_d[8] = "";

int xd = 0;
int yd = 0;
int zd = 0;
int rx = 0;
int ry = 0;
int rz = 0;

// Return values of endTransmission in the Wire library
#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4

//#define M_PI    3.14159265358979323846
//#define M_TWOPI         (M_PI * 2.0)

// led blinking support
bool ledState = false;
//int ledPin = 13;
unsigned long nextTime;
String ugh = String("");



int hall_mode = 0;


int deviceAddress = 0x60; // Address of the ALS31300
// SCL Pin = 19
// SDA Pin = 18

//
// setup
//
// Initializes the Wire library for I2C communications,
// Serial for displaying the results and error messages,
// the hardware and variables to blink the LED,
// and sets the ALS31300 into customer access mode.
//
void setup()
{
    // Initialize the I2C communication library
    Wire.begin(SDA, SCL);
    Wire.setClock(1000000);    // 1 MHz

    // Initialize the serial port
    Serial.begin(115200);
    // If using a Arduino with USB built in, uncomment the next line,
    // this allows the errors in Setup to be seen
    // while (!Serial);

    // Setup hardware and variables for code which blinks the LED
    nextTime = millis();
    //pinMode(ledPin, OUTPUT);
    //digitalWrite(ledPin, LOW);
    
    // Enter customer access mode on the ALS31300
    uint16_t error = write(deviceAddress, 0x24, 0x2C413534);
    
    if (error != kNOERROR)
    {
        Serial.print("Error while trying to enter customer access mode. error = ");
        Serial.println(error);
    }

    Wire.requestFrom(0x02, 4);
    uint32_t data = Wire.read() << 24;
    data += Wire.read() << 16;
    data += Wire.read() << 8;
    data += Wire.read();
    Serial.printf("%08x\n", data);

    char a = bitRead(data,20);
    char b = bitRead(data,19);
    hall_mode |= a << 0;
    hall_mode |= b << 1;

    // set up the legend for plotter
    Serial.println("x y z angleXY angleXZ angleYZ");

}

void loop()
{
        readALS31300ADC(deviceAddress);
        delay(100);
}

//
// readALS31300ADC
//
// Read the X, Y, Z values from Register 0x28 and 0x29
// eight times. No loop mode is used.
//
void readALS31300ADC(int busAddress)
{    
    uint32_t value0x27;

    // Read the register the I2C loop mode is in
    uint16_t error = read(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }

    // I2C loop mode is in bits 2 and 3 so mask them out
    // and set them to the no loop mode
    value0x27 = (value0x27 & 0xFFFFFFF3) | (0x0 << 2);

    // Write the new values to the register the I2C loop mode is in
    error = write(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }
    xd = 0;
    yd = 0;
    zd = 0;
    for (int count = 0; count < 1; ++count)
    {
        // Write the address that is going to be read from the ALS31300
        Wire.beginTransmission(busAddress);
        Wire.write(0x28);
        uint16_t error = Wire.endTransmission(false);

        // The ALS31300 accepted the address
        if (error == kNOERROR)
        {
            // Start the read and request 8 bytes
            // which are the contents of register 0x28 and 0x29
            Wire.requestFrom(busAddress, 8);
            
            // Read the first 4 bytes which are the contents of register 0x28
            uint32_t value0x28 = Wire.read() << 24;
            value0x28 += Wire.read() << 16;
            value0x28 += Wire.read() << 8;
            value0x28 += Wire.read();

            // Read the next 4 bytes which are the contents of register 0x29
            uint32_t value0x29 = Wire.read() << 24;
            value0x29 += Wire.read() << 16;
            value0x29 += Wire.read() << 8;
            value0x29 += Wire.read();

            // Take the most significant byte of each axis from register 0x28 and combine it with the least
            // significant 4 bits of each axis from register 0x29, then sign extend the 12th bit.
            int x = SignExtendBitfield(((value0x28 >> 20) & 0x0FF0) | ((value0x29 >> 16) & 0x0F), 12);
            int y = SignExtendBitfield(((value0x28 >> 12) & 0x0FF0) | ((value0x29 >> 12) & 0x0F), 12);
            int z = SignExtendBitfield(((value0x28 >> 4) & 0x0FF0) | ((value0x29 >> 8) & 0x0F), 12);

            xd += x;
            yd += y;
            zd += z;

            // Display the values of x, y and z
            //sprintf(xyzB,"%d x: %d y: %d z: %d", count, x,y,z);
            //sprintf(xyzB,"%d ", count);
            
            // Look at the datasheet for the sensitivity of the part used.
            // In this case, full scale range is 500 gauss, other sensitivities
            // are 1000 gauss and 2000 gauss. 
            // Sensitivity of 500 gauss = 4.0 lsb/g
            // Sensitivity of 1000 gauss = 2.0 lsb/g
            // Sensitivity of 2000 gauss = 1.0 lsb/g
            
            float mx = (float)x / 4.0;
            float my = (float)y / 4.0;
            float mz = (float)z / 4.0;

            /*
             * 
            Serial.print("MX, MY, MZ = ");
            Serial.print(mx);
            Serial.print(", ");
            Serial.print(my);
            Serial.print(", ");
            Serial.print(mz);
            Serial.println(" Gauss");
            */
            
            // Convert the X, Y and Z values into radians
            float rx = (float)x / 4096.0 * M_TWOPI;
            float ry = (float)y / 4096.0 * M_TWOPI;
            float rz = (float)z / 4096.0 * M_TWOPI;

            // Use a four quadrant Arc Tan to convert 2
            // axis to an angle (which is in radians) then
            // convert the angle from radians to degrees
            // for display.
            float angleXY = atan2f(ry, rx) * 180.0 / M_PI;
            float angleXZ = atan2f(rz, rx) * 180.0 / M_PI;
            float angleYZ = atan2f(rz, ry) * 180.0 / M_PI;

            /*
             * 
            Serial.print("angleXY, angleXZ, angleYZ = ");
            Serial.print(angleXY);
            Serial.print(", ");
            Serial.print(angleXZ);
            Serial.print(", ");
            Serial.print(angleYZ);
            Serial.println(" Degrees");
            */
            //Serial.println("x   y   z   mx   my   mz  axy   axz   yz");
            //Serial.printf("%3d  %3d  %3d  %5.2f  %5.2f  %5.2f  %11.6f  %11.6f  %11.6f\n", x, y, z, mx, my, mz, angleXY, angleXZ, angleYZ);
            Serial.printf("%3d  %3d  %3d  %11.6f  %11.6f  %11.6f\n", x, y, z, angleXY, angleXZ, angleYZ);
        }
        else
        {
            Serial.print("Unable to read the ALS31300. error = ");
            Serial.println(error);
            break;
        }
    }

}

//
// readALS31300ADC_FastLoop
//
// Read the X, Y, Z 8 bit values from Register 0x28
// eight times quickly using the fast loop mode.
//
void readALS31300ADC_FastLoop(int busAddress)
{
    uint32_t value0x27;

    // Read the register the I2C loop mode is in
    uint16_t error = read(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }

    // I2C loop mode is in bits 2 and 3 so mask them out
    // and set them to the fast loop mode
    value0x27 = (value0x27 & 0xFFFFFFF3) | (0x1 << 2);

    // Write the new values to the register the I2C loop mode is in
    error = write(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }

    // Write the register address that is going to be read from the ALS31300 (0x28)
    Wire.beginTransmission(busAddress);
    Wire.write(0x28);
    error = Wire.endTransmission(false);

    // The ALS31300 accepted the address
    if (error == kNOERROR)
    {
        int x;
        int y;
        int z;

        // Eight times is arbitrary, there is no limit. What is being demonstrated
        // is that once the address is set to 0x28, all reads will be from 0x28 until the
        // register address is changed or the loop mode is changed.
        xd = 0;
        yd = 0;
        zd = 0;
        for (int count = 0; count < 8; ++count)
        {
            // Start the read and request 4 bytes
            // which is the contents of register 0x28
            Wire.requestFrom(busAddress, 4);
            
            // Read the first 4 bytes which are the contents of register 0x28
            // and sign extend the 8th bit
            x = SignExtendBitfield(Wire.read(), 8);
            y = SignExtendBitfield(Wire.read(), 8);
            z = SignExtendBitfield(Wire.read(), 8);
            Wire.read();    // Temperature and flags not used

            xd += x;
            yd += y;
            zd += z;
            // Display the values of x, y and z
            /*
            Serial.print("Count, X, Y, Z = ");
            Serial.print(count);
            Serial.print(", ");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.print(", ");
            Serial.println(z);
            */
            // Convert the X, Y and Z values into radians
            float rx = (float)x / 256.0 * M_TWOPI;
            float ry = (float)y / 256.0 * M_TWOPI;
            float rz = (float)z / 256.0 * M_TWOPI;

            
            // Use a four quadrant Arc Tan to convert 2
            // axis to an angle (which is in radians) then
            // convert the angle from radians to degrees
            // for display.
            float angleXY = atan2f(ry, rx) * 180.0 / M_PI;
            float angleXZ = atan2f(rz, rx) * 180.0 / M_PI;
            float angleYZ = atan2f(rz, ry) * 180.0 / M_PI;

            /*
             * 
            Serial.print("angleXY, angleXZ, angleYZ = ");
            Serial.print(angleXY);
            Serial.print(", ");
            Serial.print(angleXZ);
            Serial.print(", ");
            Serial.print(angleYZ);
            Serial.println(" Degrees");
            */
        }

    }
    else
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }
}

//
// readALS31300ADC_FullLoop
//
// Read the X, Y, Z 12 bit values from Register 0x28 and 0x29
// eight times quickly using the full loop mode.
//
void readALS31300ADC_FullLoop(int busAddress)
{
    uint32_t value0x27;

    // Read the register the I2C loop mode is in
    uint16_t error = read(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }

    // I2C loop mode is in bits 2 and 3 so mask them out
    // and set them to the full loop mode
    value0x27 = (value0x27 & 0xFFFFFFF3) | (0x2 << 2);

    // Write the new values to the register the I2C loop mode is in
    error = write(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }

    // Write the address that is going to be read from the ALS31300
    Wire.beginTransmission(busAddress);
    Wire.write(0x28);
    error = Wire.endTransmission(false);

    // The ALS31300 accepted the address
    if (error == kNOERROR)
    {
        int x;
        int y;
        int z;

        // Eight times is arbitrary, there is no limit. What is being demonstrated
        // is that once the address is set to 0x28, the first four bytes read will be from 0x28
        // and the next four will be from 0x29 after that it starts all over at 0x28
        // until the register address is changed or the loop mode is changed.
        xd = 0;
        yd = 0;
        zd = 0;
        for (int count = 0; count < 8; ++count)
        {
            // Start the read and request 8 bytes
            // which is the contents of register 0x28 and 0x29
            Wire.requestFrom(busAddress, 8);
            
            // Read the first 4 bytes which are the contents of register 0x28
            x = Wire.read() << 4;
            y = Wire.read() << 4;
            z = Wire.read() << 4;
            Wire.read();    // Temperature and flags not used

            // Read the next 4 bytes which are the contents of register 0x29
            Wire.read();    // Upper byte not used
            x |= Wire.read() & 0x0F;
            byte d = Wire.read();
            y |= (d >> 4) & 0x0F;
            z |= d & 0x0F;
            Wire.read();    // Temperature not used

            // Sign extend the 12th bit for x, y and z.
            x = SignExtendBitfield((uint32_t)x, 12);
            y = SignExtendBitfield((uint32_t)y, 12);
            z = SignExtendBitfield((uint32_t)z, 12);

            xd += x;
            yd += y;
            zd += z;
            /*
            // Display the values of x, y and z
            Serial.print("Count, X, Y, Z = ");
            Serial.print(count);
            Serial.print(", ");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.print(", ");
            Serial.println(z);
            */
            // Convert the X, Y and Z values into radians
            float rx = (float)x / 4096.0 * M_TWOPI;
            float ry = (float)y / 4096.0 * M_TWOPI;
            float rz = (float)z / 4096.0 * M_TWOPI;

            // Use a four quadrant Arc Tan to convert 2
            // axis to an angle (which is in radians) then
            // convert the angle from radians to degrees
            // for display.
            float angleXY = atan2f(ry, rx) * 180.0 / M_PI;
            float angleXZ = atan2f(rz, rx) * 180.0 / M_PI;
            float angleYZ = atan2f(rz, ry) * 180.0 / M_PI;

            
            Serial.print("angleXY, angleXZ, angleYZ = ");
            Serial.print(angleXY);
            Serial.print(", ");
            Serial.print(angleXZ);
            Serial.print(", ");
            Serial.print(angleYZ);
            Serial.println(" Degrees");
            
        }

    }
    else
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }
}

//
// read
//
// Using I2C, read 32 bits of data from the address on the device at the bus address
//
uint16_t read(int busAddress, uint8_t address, uint32_t& value)
{
    // Write the address that is to be read to the device
    Wire.beginTransmission(busAddress);
    Wire.write(address);
    int error = Wire.endTransmission(false);

    // if the device accepted the address,
    // request 4 bytes from the device
    // and then read them, MSB first
    if (error == kNOERROR)
    {
        Wire.requestFrom(busAddress, 4);
        value = Wire.read() << 24;
        value += Wire.read() << 16;
        value += Wire.read() << 8;
        value += Wire.read();
    }

    return error;
}

//
// write
//
// Using I2C, write 32 bit data to an address to the device at the bus address
//
uint16_t write(int busAddress, uint8_t address, uint32_t value)
{
    // Write the address that is to be written to the device
    // and then the 4 bytes of data, MSB first
    Wire.beginTransmission(busAddress);
    Wire.write(address);
    Wire.write((byte)(value >> 24));
    Wire.write((byte)(value >> 16));
    Wire.write((byte)(value >> 8));
    Wire.write((byte)(value));    
    return Wire.endTransmission();
}

//
// SignExtendBitfield
//
// Sign extend a right justified value
//
long SignExtendBitfield(uint32_t data, int width)
{
    long x = (long)data;
    long mask = 1L << (width - 1);

    if (width < 32)
    {
        x = x & ((1 << width) - 1); // make sure the upper bits are zero
    }

    return (long)((x ^ mask) - mask);
}
