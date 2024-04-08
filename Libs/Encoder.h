#ifndef Encoder_H
#define Encoder_H
 
/**
 * Includes
 */
#include "mbed.h"
 
/**
 * Defines
 */
#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.
 
/**
 * Quadrature Encoder Interface.
 */
class Encoder {
 
public:
 
    typedef enum Encoding {
 
        X2_ENCODING,
        X4_ENCODING
 
    } Encoding;
 
    /**
     * Constructor.
     *
     * Reads the current values on channel A and channel B to determine the
     * initial state.
     *
     * Attaches the encode function to the rise/fall interrupt edges of
     * channels A and B to perform X4 encoding.
     *
     * Attaches the index function to the rise interrupt edge of channel index
     * (if it is used) to count revolutions.
     *
     * @param channelA mbed pin for channel A input.
     * @param channelB mbed pin for channel B input.
     * @param pulsesPerRev Number of pulses in one revolution.
     * @param encoding The encoding to use. Uses X2 encoding by default. X2
     *                 encoding uses interrupts on the rising and falling edges
     *                 of only channel A where as X4 uses them on both
     *                 channels.
     */
    Encoder(PinName channelA, PinName channelB, int reductionRatio, int pulsesPerRev, Encoding encoding = X2_ENCODING);
 
    /**
     * Reset the encoder.
     *
     * Sets the pulses and revolutions count to zero.
     */
    void reset(void);
 
    /**
     * Read the state of the encoder.
     *
     * @return The current state of the encoder as a 2-bit number, where:
     *         bit 1 = The reading from channel B
     *         bit 2 = The reading from channel A
     */
    int getCurrentState(void);
 
    /**
     * Read the number of pulses recorded by the encoder.
     *
     * @return Number of pulses which have occured.
     */
    int getPulses(void);
 
    /**
     * Считает текущий угол поворота вала двигателя по формуле  
     * m_pulses * 360 / (m_reductionRatio * m_pulsesPerRev * m_pulseRepeats);
     * В режиме X2_ENCODING m_pulseRepeats = 2; 
     * В режиме X4_ENCODING m_pulseRepeats = 4; 
     * @return Текущий угол поворота вала редуктора ДПТ
    */
    float getCurrentAngle(void); 


    void setZeroAngleRad(float zero_angle);
    float getCurrentAngleRad(void); 
    /**
     * Считает текущую скорость вала двигателя по формуле 
     *
     *
     * @return Текущую скорость поворота вала редуктора ДПТ
    */
    float getCurrentSpeed(void); 

     /** 
     * Расчет скорости вала двигателя
     * Вызывается по тикеру   
    */
    void speedCalculation(void); 
    
private:
 
    /**
     * Update the pulse count.
     *
     * Called on every rising/falling edge of channels A/B.
     *
     * Reads the state of the channels and determines whether a pulse forward
     * or backward has occured, updating the count appropriately.
     */
    void encode(void);
 
   
    /**
     * Called on every rising edge of channel index to update revolution
     * count by one.
     */
    void index(void);
 
    Encoding m_encoding;
 
    InterruptIn channelA_;
    InterruptIn channelB_;
 
    int          m_reductionRatio; 
    int          m_pulsesPerRev;
    int          m_prevState;
    int          m_currState;
    int          m_pulseRepeats; 

    volatile int m_pulses;
    volatile int m_revolutions;
    volatile float m_currSpeed; 

    // Timer speedCacl; 

    float m_prevAngle; 
    float m_dt; 
    // int m_prevPulses; 
};
 
#endif /* Encoder_H */