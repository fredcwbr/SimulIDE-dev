/***************************************************************************
 *   Copyright (C) 2020 by Santiago González                               *
 *                                                                         *
 ***( see copyright.txt file at root folder )*******************************/
/*
 *   Based on simavr decoder
 *
 *   Copyright 2008, 2010 Michel Pollet <buserror@gmail.com>
 *
 */

#pragma once

#include "mcu8bits.h"
#include "mcutypes.h"
#include "e_mcu.h"
#include "mcu.h"
#include "mcuport.h"
#include "rammemoryproxy.h"
#include <QDebug>

class AvrCore : public Mcu8bits {
public:
    AvrCore( eMcu* mcu );
    ~AvrCore();

    virtual void reset() override;
    virtual void runStep() override;

    void setRamProxy( RamMemoryProxy* proxy );
    RamMemoryProxy* ramProxy() const { return m_ramProxy; }

    inline uint8_t readDataMem( uint16_t addr );
    inline void    writeDataMem( uint16_t addr, uint8_t val );

    // Intercept data space macros/functions used by LDS, STS, LD, ST, etc.
    // Intercept data space accesses: delegate internal RAM/Registers/IO to Mcu8bits, route XMEM (>= 0x2200) to proxy
    // Intercept data space accesses: delegate internal RAM/Registers/IO to Mcu8bits, route XMEM (>= 0x2200) to proxy
    inline uint8_t GET_RAM( uint16_t addr ) {
        if ( addr >= 0x2200 ) {
            return m_ramProxy->read( addr );
        }
        return Mcu8bits::GET_RAM( addr );
    }

    inline void SET_RAM( uint16_t addr, uint8_t val ) {
        // Catch XMCRA (0x74) and XMCRB (0x75) configuration writes
        if ( addr == 0x74 || addr == 0x75 || addr == 0x0074 || addr == 0x0075 ) {
            if ( m_ramProxy ) {
                m_ramProxy->write( addr, val );
            }
        }

        if ( addr >= 0x2200 ) {
            m_ramProxy->write( addr, val );
            return;
        }

        // Catch port writes (PORTA, PORTC, PORTG) and trigger their outChanged callbacks
        if ( addr == 0x22 || addr == 0x28 || addr == 0x34 || 
             addr == 0x0022 || addr == 0x0028 || addr == 0x0034 ) {
            writePortReg( addr, val );
            return;
        }

        Mcu8bits::SET_RAM( addr, val );
    }

    // Direct subscript access via proxy: mem[addr] = val; / val = mem[addr];
    inline RamProxyRef mem(uint16_t addr) {
        return (*m_ramProxy)[addr];
    }

    // Safely write to I/O registers so SimulIDE port watchers and logic analyzers update
    inline void writePortReg(uint16_t regAddr, uint8_t val) {
        qDebug() << "[AVRCORE-PORT] writePortReg invoked for register addr:" << Qt::hex << regAddr << "with value:" << val;
        m_mcu->writeReg(regAddr, val);
        
        // Forcefully update MCU ports when control registers change
        if (m_mcu) {
            if (regAddr == 0x34) { // PORTG
                McuPort* portG = m_mcu->getMcuPort("PORTG");
		qDebug() << "[PORTG-DEBUG] portG pointer:" << portG;
                if (portG){
		   portG->outChanged(val);
        	   qDebug() << "[AVRCORE-PORT-PORTG-CHANGED] outChanged(portG) addr:" << Qt::hex << regAddr << "with value:" << val;
		}
            } else if (regAddr == 0x22) { // PORTA
                McuPort* portA = m_mcu->getMcuPort("PORTA");
                if (portA) {
		   portA->outChanged(val);
        	   qDebug() << "[AVRCORE-PORT-PORTA-CHANGED] outChanged(portA) addr:" << Qt::hex << regAddr << "with value:" << val;
		}
            } else if (regAddr == 0x28) { // PORTC
                McuPort* portC = m_mcu->getMcuPort("PORTC");
                if (portC) {
		   portC->outChanged(val);
        	   qDebug() << "[AVRCORE-PORT-PORTC-CHANGED] outChanged(portC) addr:" << Qt::hex << regAddr << "with value:" << val;
		}
            }
        }
    }

private:
    void writeFlash();

    uint16_t m_bootStart;

    int m_pageSize;
    std::vector<uint8_t> m_tmpUsed;
    std::vector<uint16_t> m_tmpPage;

    regBits_t m_SELFPRGEN;
    regBits_t m_PGERS;
    regBits_t m_PGWRT;

    uint16_t m_rampzAddr;
    uint8_t* RAMPZ; // optional, only for ELPM/SPM on >64Kb cores
    uint8_t* EIND; // optional, only for EIJMP/EICALL on >64Kb cores

    RamMemoryProxy* m_ramProxy;
    DirectRamProxy  m_defaultProxy;

    void flags_ns( uint8_t res );
    void flags_zns( uint8_t res );
    void flags_Rzns( uint8_t res );
    void flags_sub( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_sub_Rzns( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_add_zns( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_sub_zns( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_znv0s( uint8_t res );
    void flags_zcnvs( uint8_t res, uint8_t vr );
    void flags_zcvs( uint8_t res, uint8_t vr );
    void flags_zns16( uint16_t res );
    int is_instr_32b( uint32_t pc );
   
    // Local lambda capturing 'this' to route all subscript lookups through our proxy
    inline RamProxyRef RAM(uint16_t addr) {
        return (*m_ramProxy)[addr];
    } 
};

inline uint8_t AvrCore::readDataMem( uint16_t addr )
{
    return m_ramProxy->read( addr );
}

inline void AvrCore::writeDataMem( uint16_t addr, uint8_t val )
{
    m_ramProxy->write( addr, val );
}

