#include "LoadCell.h"
#include <HX711.h>

extern boolean DEBUG; // from main.cpp

HX711 loadcell;

long zero_factor = -1;
float calibration_factor = 45999; // 45999.00 in testing

void loadcellSetup()
{
    loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    loadcell.set_scale(calibration_factor);
    loadcell.tare();
}

long loadcellDetermineZero()
{
    zero_factor = loadcell.read_average();
    if (DEBUG)
    {
        Serial.print("Zero Factor: "); Serial.println(zero_factor); 
    }
    return zero_factor;
}

// may delete this function if loadcellDetermineCalibrationFactor() works
// but may update this function to read the calibration factor out of the
// EPROM upon startup
void loadcellSetCalibrationFactor(float cal)
{
    if (DEBUG)
    {
        Serial.print("Reading BEFORE set cal factor: "); 
        Serial.print(loadcell.get_units(), 1); 
        Serial.println(" lbs");
    }
    
    
    loadcell.set_scale(cal);

    if (DEBUG)
    {
        Serial.print("Reading AFTER set cal factor: "); 
        Serial.print(loadcell.get_units(), 1); 
        Serial.println(" lbs");
    }
}

// The function below allows the user to set the calibration factor manually
float loadcellDetermineCalibrationFactor()
{
    Serial.println(" **Starting Calibration Factor Determination** ");
    Serial.println(" ! - Send 'x' or 'X' to stop/finish calibration procedure");
    Serial.println(" ! - Send '+' to increment calibration factor +10");
    Serial.println(" ! - Send '-' to decrement calibration factor -10");
    Serial.println(" ! - Send 'a' to increment calibration factor +100");
    Serial.println(" ! - Send 'z' to decrement calibration factor -100");
    Serial.println(" ! - Send 'd' to increment calibration factor +1000");
    Serial.println(" ! - Send 'f' to decrement calibration factor -1000");
    
    loadcell.set_scale(calibration_factor);

    Serial.print("Initial scale reading before calibration: ");
    Serial.print(loadcell.get_units(), 1);
    Serial.println(" lbs");

    loadcellDetermineZero();
    
    char cal_cmd;
    float old_calib = calibration_factor;
    unsigned long timeout = millis();
    
    // exit the while loop when commanded by the user 
    // (received 'x') or a 180 second (3 min) timeout    
    while ( millis() - timeout <= 180000 )
    {       
        if ( Serial.available() )
        {
            cal_cmd = Serial.read();
            if (cal_cmd == '+')
                calibration_factor += 10;
            else if(cal_cmd == '-')
                calibration_factor -= 10;
            else if(cal_cmd == 'a')
                calibration_factor += 100;
            else if(cal_cmd == 'z')
                calibration_factor -= 100;
            else if(cal_cmd == 'd')
                calibration_factor += 1000;
            else if(cal_cmd == 'f')
                calibration_factor -= 1000;
            else if(cal_cmd == 'x')
                break;
        }

        // if the calibration factor has changed print the feedback out to serial
        if ( old_calib != calibration_factor )
        {
            loadcell.set_scale(calibration_factor);
            Serial.print("updated calibration_factor: "); 
            Serial.println(calibration_factor);
            Serial.print("Scale reading: "); 
            Serial.print(loadcell.get_units(), 1); 
            Serial.println(" lbs");
            old_calib = calibration_factor;
        }
    }

    Serial.println(" *Completed Load Cell Calibration* ");
    Serial.print("Final calibration value: "); 
    Serial.println(calibration_factor);
    Serial.print("Final scale reading with updated cal factor: "); 
    Serial.print(loadcell.get_units(), 1); 
    Serial.println(" lbs");

    loadcellDetermineZero();

    return calibration_factor;
}

void loadcellTare()
{
    if (DEBUG)
    {
        Serial.print("Load cell reading before tare: ");
        Serial.print(loadcell.get_units(), 1); Serial.println(" lbs");
    }
    
    loadcell.tare();
    
    if (DEBUG)
    {
        Serial.print("Load cell reading after tare: ");
        Serial.print(loadcell.get_units(), 1); Serial.println(" lbs");
    } 
}

float loadcellReadCurrentValue()
{
    float reading = loadcell.get_units();
    
    if (DEBUG)
    {
        Serial.print("Scale Reading: "); Serial.print(reading); Serial.println(" lbs");
    }
    
    return reading;
}