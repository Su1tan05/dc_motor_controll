#include "mbed.h"
#include "encoder.h"

Encoder::Encoder(PinName ChA, PinName ChB, int reductioRatio, int encImpulseRotate, bool twoChannelMode) : _ChA(ChA), _ChB(ChB){


    m_reductioRatio = reductioRatio;
    m_encImpulseRotate = encImpulseRotate;
    m_twoChannelMode = twoChannelMode;

    int chanA = _ChA.read();
    int chanB = _ChB.read();
 
    //2-bit state.
    m_curState = (chanA << 1) | (chanB);
    m_prevState = m_curState;

    _ChA.rise(callback(this, &Encoder::incrementEncTicks));
    _ChA.fall(callback(this, &Encoder::incrementEncTicks));

    if(m_twoChannelMode == true){
        _ChB.rise(callback(this, &Encoder::incrementEncTicks));
        _ChB.fall(callback(this, &Encoder::incrementEncTicks));        
        m_pulseRepeats = 4;    
    } else{
        m_pulseRepeats = 2;  
    }
}

void Encoder::incrementEncTicks(){
//m_encTicks++;

    int change = 0;
    int chanA  = _ChA.read();
    int chanB  = _ChB.read();
 
    //2-bit state.
    m_curState = (chanA << 1) | (chanB);
 
    //11->00->11->00 is counter clockwise rotation or "forward".
    if(m_twoChannelMode == false){
        if ((m_prevState == 0x3 && m_curState == 0x0) || (m_prevState == 0x0 && m_curState == 0x3)) {
            m_encTicks++;
            m_clockwise = false;
        }
        //10->01->10->01 is clockwise rotation or "backward".
        else if ((m_prevState == 0x2 && m_curState == 0x1) || (m_prevState == 0x1 && m_curState == 0x2)) {
            m_encTicks--;
            m_clockwise = true;
        }
    } else {
        //Entered a new valid state.
        if (((m_curState ^ m_prevState) != INVALID) && (m_curState != m_prevState)) {
            //2 bit state. Right hand bit of prev XOR left hand bit of current
            //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
            change = (m_prevState & PREV_MASK) ^ ((m_curState & CURR_MASK) >> 1);
    
            if (change == 0) {
                change = -1;
                m_clockwise = true;
            } else{
                m_clockwise = false;
            }
    
            m_encTicks -= change;
        }   
    }
    
    m_prevState = m_curState;
   
}

int Encoder::getEncTicks(){

    return m_encTicks;

}

bool Encoder::getCurDirection(){

    return m_clockwise;
    
}

float Encoder::getCurAngle(){

    float curAngle;
    curAngle = m_encTicks * (float)360 / (m_reductioRatio * m_encImpulseRotate * m_pulseRepeats);
    return curAngle;

}

float Encoder::getCurSpeed(){
    float curSpeed = 12.0;
    return curSpeed;
}


void Encoder::reset(){
    m_encTicks = 0;
}
