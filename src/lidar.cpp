#include "lidar.h"

void Lidar::begin(const int lidarBaudRate, const int rxPin, const int txPin)
{
    SerialPort.begin(lidarBaudRate, SERIAL_8N1, rxPin, txPin);
}

size_t Lidar::scan()
{
    if (SerialPort.available())
    {
        // Serial.println("Data available");
        // Read the start sign and check if it matches the protocol
        byte startSign1 = SerialPort.read(); // PH1
        byte startSign2 = SerialPort.read(); // PH2
        // Serial.println(startSign1);
        // Serial.println(startSign2);
        if (startSign1 == 0xAA && startSign2 == 0x55)
        {

            byte ct = SerialPort.read();
            byte lsn = SerialPort.read();

            byte fsaLow = SerialPort.read();
            byte fsaHigh = SerialPort.read();

            byte lsaLow = SerialPort.read();
            byte lsaHigh = SerialPort.read();

            byte csLow = SerialPort.read();
            byte csHigh = SerialPort.read();

            int16_t fsa = (fsaHigh << 8) | fsaLow;
            int16_t lsa = (lsaHigh << 8) | lsaLow;

            int16_t cs = (csHigh << 8) | csLow;

            // Calculate the checksum
            // in the order (ph, fsa, s1, s2 ..., [ct, lsn], lsa)
            int16_t xor_res = (startSign2 << 8) | startSign1; // ph
            xor_res ^= fsa;

            float angleFSA = static_cast<float>(fsa >> 1) / 64.0f;
            float angleLSA = static_cast<float>(lsa >> 1) / 64.0f;

            // Calculate the number of samples
            int numSamples = static_cast<int>(lsn);

            Point2d points[numSamples];

            // Read and process the sample data
            for (int i = 0; i < numSamples; ++i)
            {
                byte sampleLow = SerialPort.read();
                byte sampleHigh = SerialPort.read();

                // Calculate the distance and angle for each sample
                int16_t sample = (sampleHigh << 8) | sampleLow;

                // add to xor check
                xor_res ^= sample;

                float distance = static_cast<float>(sample) / 4.0f;
                float angle;

                if (i == 0)
                {
                    angle = angleFSA;
                }
                else if (i == numSamples - 1)
                {
                    angle = angleLSA;
                }
                else
                {
                    angle = angleFSA + ((angleLSA - angleFSA) / (numSamples - 1)) * (i);
                }

                // this should go from 0 to numSamples
                //  second lever analysis
                float finalAngle;
                // if distance is zero
                if (distance == 0)
                {
                    finalAngle = angle;
                }
                else
                {
                    // calculate the angle of the second lever
                    float angCorrection = atan(21.8f * ((155.3f - distance) / (155.3f * distance)));
                    // calculate the final angle
                    finalAngle = angle + angCorrection;
                }

                // Calculate the x and y coordinates of the point
                // Point2d point = calculate_point(distance, angle);
                // To keep the computations as low as possible, we will just use the distance and angle
                Point2d point = {distance, finalAngle};
                points_buffer[buffer_index + i] = point;
            }
            // Check the xor (, [ct, lsn], lsa)
            xor_res ^= (lsn << 8) | ct;
            xor_res ^= lsa;

            // // get the firs bit of the ct
            // bool isStartPacket = ct & 0x01;
            // // Serial.print("isStartPacket: ");
            // // Serial.println(isStartPacket);
            // if (isStartPacket) {
            //   Serial.println("Start packet");
            //   return;
            // }

            if (xor_res == cs)
            {
                Serial.println("Checksum OK");
                const size_t maxSamples = min(size_t(numSamples), max_buffer_size);

                buffer_index += maxSamples;
                return maxSamples;
            }
            else
            {
                Serial.println("Checksum failed");
                return 0;
            }
        }
    }
    return 0;
}